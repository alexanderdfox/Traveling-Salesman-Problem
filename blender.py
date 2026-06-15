import bpy
import collections
import itertools
import random
import time

# Clear existing objects in the scene to prevent overlay clutter
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)

# --- 1. PROCEDURAL GRID & STOPS GENERATOR ---
def generate_dynamic_maze(rows, cols, num_stops):
    maze = [[1 for _ in range(cols)] for _ in range(rows)]
    start_r, start_c = 1, 1
    maze[start_r][start_c] = 0
    walls = [(start_r + dr, start_c + dc, start_r, start_c) 
             for dr, dc in [(-1,0), (1,0), (0,-1), (0,1)]]
    
    while walls:
        wr, wc, pr, pc = random.choice(walls)
        walls.remove((wr, wc, pr, pc))
        if 0 < wr < rows - 1 and 0 < wc < cols - 1:
            if maze[wr][wc] == 1:
                opp_r, opp_c = wr + (wr - pr), wc + (wc - pc)
                if 0 < opp_r < rows - 1 and 0 < opp_c < cols - 1:
                    if maze[opp_r][opp_c] == 1:
                        maze[wr][wc] = 0
                        maze[opp_r][opp_c] = 0
                        for dr, dc in [(-1,0), (1,0), (0,-1), (0,1)]:
                            walls.append((opp_r + dr, opp_c + dc, opp_r, opp_c))

    # Clean up isolated walls
    for r in range(1, rows - 1):
        for c in range(1, cols - 1):
            if maze[r][c] == 1 and random.random() < 0.15:
                maze[r][c] = 0

    corridors = [(r, c) for r in range(1, rows - 1) for c in range(1, cols - 1) if maze[r][c] == 0]
    num_stops = min(num_stops, len(corridors))
    chosen_coords = random.sample(corridors, num_stops)
    
    tsp_points = {f"P{i+1}": coord for i, coord in enumerate(chosen_coords)}
    return maze, tsp_points

# --- 2. THE FLUID NAVIGATION ENGINE & BACKTRACER ---
def fluid_distance_solver(maze, start_coord, end_coord, return_path=False):
    rows, cols = len(maze), len(maze[0])
    queue = collections.deque([(start_coord[0], start_coord[1], 0)])
    visited = {start_coord: None}

    while queue:
        r, c, dist = queue.popleft()
        if (r, c) == end_coord:
            if return_path:
                path = []
                curr = end_coord
                while curr is not None:
                    path.append(curr)
                    curr = visited[curr]
                return path[::-1]
            return dist

        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols:
                if maze[nr][nc] == 0 and (nr, nc) not in visited:
                    visited[(nr, nc)] = (r, c)
                    queue.append((nr, nc, dist + 1))
                    
    return [] if return_path else float('inf')

# --- 3. BLENDER MATERIAL GENERATOR ---
def create_material(name, color):
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    principled = nodes.get("Principled BSDF")
    if principled:
        principled.inputs['Base Color'].default_value = color
    return mat

# --- 4. BLENDER 3D ENGINE BUILDER ---
def build_3d_maze(maze, tsp_points, best_route_list):
    rows, cols = len(maze), len(maze[0])
    
    # Create distinct materials
    wall_mat = create_material("WallMat", (0.05, 0.05, 0.06, 1.0)) # Dark Slate
    floor_mat = create_material("FloorMat", (0.85, 0.85, 0.88, 1.0)) # Light Gray
    node_mat = create_material("NodeMat", (0.0, 0.65, 0.8, 1.0)) # Cyan
    path_mat = create_material("PathMat", (1.0, 0.1, 0.2, 1.0)) # Neon Red Outline
    
    # 1. Spawn Floor
    bpy.ops.mesh.primitive_plane_add(size=1, location=(cols/2 - 0.5, rows/2 - 0.5, 0))
    floor = bpy.context.object
    floor.scale = (cols, rows, 1)
    floor.data.materials.append(floor_mat)

    # 2. Spawn Walls (3D Extruded Blocks)
    for r in range(rows):
        for c in range(cols):
            if maze[r][c] == 1:
                # X = c, Y = rows - 1 - r (invert layout tracking)
                bpy.ops.mesh.primitive_cube_add(size=1, location=(c, rows - 1 - r, 0.5))
                cube = bpy.context.object
                cube.data.materials.append(wall_mat)

    # 3. Spawn Node Target Checkpoints
    for name, (r, c) in tsp_points.items():
        x, y = c, rows - 1 - r
        # Add 3D Spheres
        bpy.ops.mesh.primitive_uv_sphere_add(radius=0.3, location=(x, y, 0.3))
        sphere = bpy.context.object
        sphere.name = f"Sphere_{name}"
        sphere.data.materials.append(node_mat)
        
        # Add Text Tag above the nodes
        bpy.ops.object.text_add(location=(x - 0.2, y - 0.2, 0.8))
        text_obj = bpy.context.object
        text_obj.data.body = name
        text_obj.scale = (0.3, 0.3, 0.3)

    # 4. Draw In-Corridor Path (Using a 3D Poly-Curve Object)
    if best_route_list and len(best_route_list) > 1:
        # Build master list of sequential steps mapping out the entire path loop
        full_3d_coordinate_trail = []
        
        for i in range(len(best_route_list) - 1):
            p1_name = best_route_list[i]
            p2_name = best_route_list[i+1]
            leg_path = fluid_distance_solver(maze, tsp_points[p1_name], tsp_points[p2_name], return_path=True)
            
            for index, (r, c) in enumerate(leg_path):
                # Avoid adding duplicates if the end of leg 1 matches the start of leg 2
                if not full_3d_coordinate_trail or full_3d_coordinate_trail[-1] != (c, rows - 1 - r, 0.15):
                    full_3d_coordinate_trail.append((c, rows - 1 - r, 0.15))

        # Initialize Curve Data block structure
        curve_data = bpy.data.curves.new('RouteCurve', type='CURVE')
        curve_data.dimensions = '3D'
        curve_data.bevel_depth = 0.05 # Thickness of the path line pipe
        
        polyline = curve_data.splines.new('POLY')
        polyline.points.add(len(full_3d_coordinate_trail) - 1)
        
        for idx, (x, y, z) in enumerate(full_3d_coordinate_trail):
            polyline.points[idx].co = (x, y, z, 1.0) # (X, Y, Z, Weight)
            
        curve_obj = bpy.data.objects.new('SolvedTSPRoute', curve_data)
        bpy.context.collection.objects.link(curve_obj)
        curve_obj.data.materials.append(path_mat)

# --- 5. THE TSP COMBINATORIAL OPTIMIZER ---
def solve_all_tsp_routes(maze, tsp_points):
    nodes = list(tsp_points.keys())
    distance_matrix = {}
    
    for pair in itertools.combinations(nodes, 2):
        p1, p2 = pair
        dist = fluid_distance_solver(maze, tsp_points[p1], tsp_points[p2])
        distance_matrix[(p1, p2)] = dist
        distance_matrix[(p2, p1)] = dist

    best_route_list = []
    
    if len(nodes) <= 9:
        all_permutations = list(itertools.permutations(nodes))
        min_total_distance = float('inf')
        for route in all_permutations:
            current_route_distance = 0
            valid_route = True
            for i in range(len(route) - 1):
                leg_distance = distance_matrix.get((route[i], route[i+1]), float('inf'))
                if leg_distance == float('inf'):
                    valid_route = False
                    break
                current_route_distance += leg_distance
                
            if valid_route and current_route_distance < min_total_distance:
                min_total_distance = current_route_distance
                best_route_list = list(route)
    else:
        unvisited = set(nodes)
        current_node = random.choice(nodes)
        unvisited.remove(current_node)
        best_route_list = [current_node]
        
        while unvisited:
            next_node = min(unvisited, key=lambda node: distance_matrix.get((current_node, node), float('inf')))
            current_node = next_node
            unvisited.remove(current_node)
            best_route_list.append(current_node)
            
    return best_route_list

if __name__ == "__main__":
    # --- CONFIGURATION BAR ---
    GRID_ROWS = 15      # Maze size
    GRID_COLS = 21      
    TOTAL_STOPS = 10    # Set up to 100 stops
    # -------------------------
    
    # Execute pipelines inside Blender
    MAZE_GRID, TSP_POINTS = generate_dynamic_maze(GRID_ROWS, GRID_COLS, TOTAL_STOPS)
    best_route = solve_all_tsp_routes(MAZE_GRID, TSP_POINTS)
    build_3d_maze(MAZE_GRID, TSP_POINTS, best_route)
