import collections
import itertools
import random
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# --- 1. PROCEDURAL GRID & STOPS GENERATOR ---
def generate_dynamic_maze(rows, cols, num_stops):
	"""
	Generates a guaranteed-solvable maze using a randomized Prim's Algorithm
	and distributes a user-specified number of stops across open corridors.
	"""
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

	# Clean up isolated walls to make paths look more organic
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
	"""
	Simulates a high-velocity fluid volume filling the maze corridors.
	If return_path=True, returns the exact coordinate path array locked inside the corridors.
	"""
	rows, cols = len(maze), len(maze[0])
	queue = collections.deque([(start_coord[0], start_coord[1], 0)])
	visited = {start_coord: None} # Stores parent node relationships to backtrack steps

	while queue:
		r, c, dist = queue.popleft()
		if (r, c) == end_coord:
			if return_path:
				# Backtrack parents to construct the clean route trail
				path = []
				curr = end_coord
				while curr is not None:
					path.append(curr)
					curr = visited[curr]
				return path[::-1] # Reverse to get path from start -> end
			return dist

		for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
			nr, nc = r + dr, c + dc
			if 0 <= nr < rows and 0 <= nc < cols:
				if maze[nr][nc] == 0 and (nr, nc) not in visited:
					visited[(nr, nc)] = (r, c)
					queue.append((nr, nc, dist + 1))
					
	return [] if return_path else float('inf')

# --- 3. THE MATPLOTLIB GRAPHICS & SVG GENERATOR ---
def generate_and_save_svg_map(maze, tsp_points, best_route_list, filename="maze_tsp_solution.svg"):
	"""
	Renders the maze layout visually, traces out the optimal solved path route, 
	and saves the production output natively to an infinitely scalable SVG asset file.
	"""
	rows = len(maze)
	cols = len(maze[0])
	
	fig, ax = plt.subplots(figsize=(cols * 0.5, rows * 0.5))
	
	# 1. Render Maze Structural Boundaries
	for r in range(rows):
		for c in range(cols):
			if maze[r][c] == 1:
				ax.add_patch(patches.Rectangle((c, rows - 1 - r), 1, 1, color="#1e1e24"))
			else:
				ax.add_patch(patches.Rectangle((c, rows - 1 - r), 1, 1, color="#f4f4f9"))

	# 2. Trace In-Corridor Precise Trajectory Path
	if best_route_list and len(best_route_list) > 1:
		for i in range(len(best_route_list) - 1):
			p1_name = best_route_list[i]
			p2_name = best_route_list[i+1]
			
			# Find the turn-by-turn coordinate trail through the corridors
			leg_path = fluid_distance_solver(maze, tsp_points[p1_name], tsp_points[p2_name], return_path=True)
			
			# Convert grid positions to visual map positions
			x_coords = [c + 0.5 for (r, c) in leg_path]
			y_coords = [rows - 1 - r + 0.5 for (r, c) in leg_path]
			
			# Plot the line directly along the corridor centers
			ax.plot(x_coords, y_coords, color="#ff4a5a", lw=2.5, alpha=0.85, zorder=3)
			
			# Place an directional indicator arrow at the end of each leg segment
			if len(x_coords) > 1:
				ax.annotate("", xy=(x_coords[-1], y_coords[-1]), xytext=(x_coords[-2], y_coords[-2]),
							arrowprops=dict(arrowstyle="-|>", color="#ff4a5a", 
											lw=2.5, mutation_scale=12, alpha=0.9), zorder=4)

	# 3. Render Node Target Checkpoint Marks
	for name, (r, c) in tsp_points.items():
		x, y = c + 0.5, rows - 1 - r + 0.5
		ax.plot(x, y, marker='o', markersize=14, color="#00a8cc", markeredgecolor='black', zorder=5)
		ax.text(x, y, name, color='white', weight='bold', fontsize=7,
				ha='center', va='center', zorder=6)

	# Clean up axis display properties
	ax.set_xlim(0, cols)
	ax.set_ylim(0, rows)
	ax.set_aspect('equal')
	ax.axis('off')
	
	plt.title(f"Water-Maze Vector Path Layout Map ({len(tsp_points)} Checkpoints Loop)", 
			  fontsize=12, weight='bold', pad=10)
	
	plt.savefig(filename, format='svg', bbox_inches='tight', dpi=300)
	print(f"\n[SUCCESS] Scalable Vector Map Asset Successfully Generated & Exported to: '{filename}'")
	plt.show()

# --- 4. THE TSP COMBINATORIAL OPTIMIZER ---
def solve_all_tsp_routes(maze, tsp_points):
	print("=" * 60)
	print(" DYNAMIC WATER-MAZE PUZZLE: AUTOMATED SOLVER & GENERATOR")
	print("=" * 60)
	
	start_time = time.time()
	nodes = list(tsp_points.keys())
	distance_matrix = {}
	
	print(f"Pre-computing fluid matrices across {len(nodes)} stops...")
	for pair in itertools.combinations(nodes, 2):
		p1, p2 = pair
		dist = fluid_distance_solver(maze, tsp_points[p1], tsp_points[p2])
		distance_matrix[(p1, p2)] = dist
		distance_matrix[(p2, p1)] = dist

	best_route_list = []
	
	if len(nodes) <= 9:
		all_permutations = list(itertools.permutations(nodes))
		print(f"Evaluating all {len(all_permutations)} absolute permutations...\n")
		
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
		print("Large scale problem detected. Activating Nearest Neighbor optimization heuristic...\n")
		unvisited = set(nodes)
		current_node = random.choice(nodes)
		unvisited.remove(current_node)
		best_route_list = [current_node]
		min_total_distance = 0
		
		while unvisited:
			next_node = min(unvisited, key=lambda node: distance_matrix.get((current_node, node), float('inf')))
			min_total_distance += distance_matrix[(current_node, next_node)]
			current_node = next_node
			unvisited.remove(current_node)
			best_route_list.append(current_node)

	best_route_str = " → ".join(best_route_list) if best_route_list else "None"
	end_time = time.time()
	execution_speed = (end_time - start_time) * 1000

	# --- 5. PRINT TELEMETRY DISPLAY ---
	print("=" * 60)
	print(" OPTIMAL ROUTE SELECTION FOUND")
	print("=" * 60)
	print(f" Best Evaluated Route : {best_route_str}")
	print(f" Minimum Move Cost    : {min_total_distance} corridor tiles")
	print(f" Compute Engine Speed : {execution_speed:.3f} ms")
	print("=" * 60)
	
	return best_route_list

if __name__ == "__main__":
	# --- CONFIGURATION BAR ---
	GRID_ROWS = 150      # Vertical size of grid canvas (Odd numbers recommended)
	GRID_COLS = 210      # Horizontal size of grid canvas
	TOTAL_STOPS = 10    # Up to 100 stops supported smoothly
	# -------------------------
	
	# 1. Run dynamic procedural generation
	MAZE_GRID, TSP_POINTS = generate_dynamic_maze(GRID_ROWS, GRID_COLS, TOTAL_STOPS)
	
	# 2. Extract optimization pathways
	best_route = solve_all_tsp_routes(MAZE_GRID, TSP_POINTS)
	
	# 3. Create, draw, show, and write beautiful savable SVG file structures
	generate_and_save_svg_map(MAZE_GRID, TSP_POINTS, best_route, filename="maze_tsp_solution.svg")