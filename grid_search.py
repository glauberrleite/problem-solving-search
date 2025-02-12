import heapq
import time

# Directions for moving in the grid (row, col)
# Orthogonal moves have a cost of 1, diagonal moves have a cost of 1.4
DIRECTIONS = [
    (-1, 0, 1),  # Up (cost 1)
    (1, 0, 1),   # Down (cost 1)
    (0, -1, 1),  # Left (cost 1)
    (0, 1, 1),   # Right (cost 1)
    (-1, -1, 1.4), # Diagonal Top-Left (cost 1.4)
    (-1, 1, 1.4),  # Diagonal Top-Right (cost 1.4)
    (1, -1, 1.4),  # Diagonal Bottom-Left (cost 1.4)
    (1, 1, 1.4),   # Diagonal Bottom-Right (cost 1.4)
]

# Dijkstra's Algorithm with diagonal movement
def dijkstra(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    dist = [[float('inf')] * cols for _ in range(rows)]
    dist[start[0]][start[1]] = 0  # Start point has a distance of 0
    pq = [(0, start)]  # Priority Queue for the algorithm (min-heap)
    
    # Store the path for backtracking
    parent = {start: None}

    while pq:
        current_dist, current_pos = heapq.heappop(pq)
        x, y = current_pos

        # If we reached the goal, reconstruct the path
        if current_pos == goal:
            path = []
            while current_pos is not None:
                path.append(current_pos)
                current_pos = parent[current_pos]
            return path[::-1]  # Reverse the path to get it from start to goal

        # Process neighbors
        for dx, dy, cost in DIRECTIONS:
            nx, ny = x + dx, y + dy
            if 0 <= nx < rows and 0 <= ny < cols and grid[nx][ny] != 1:  # Within bounds and not an obstacle
                new_dist = current_dist + cost
                if new_dist < dist[nx][ny]:
                    dist[nx][ny] = new_dist
                    heapq.heappush(pq, (new_dist, (nx, ny)))
                    parent[(nx, ny)] = (x, y)  # Track the parent to reconstruct the path

    return []  # If no path found


# A* Algorithm with diagonal movement
def astar(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    g_cost = [[float('inf')] * cols for _ in range(rows)]
    f_cost = [[float('inf')] * cols for _ in range(rows)]
    g_cost[start[0]][start[1]] = 0
    f_cost[start[0]][start[1]] = heuristic(start, goal)
    
    pq = [(f_cost[start[0]][start[1]], start)]  # Priority Queue for the algorithm (min-heap)
    
    # Store the path for backtracking
    parent = {start: None}
    
    while pq:
        current_f_cost, current_pos = heapq.heappop(pq)
        x, y = current_pos
        
        # If we reached the goal, reconstruct the path
        if current_pos == goal:
            path = []
            while current_pos is not None:
                path.append(current_pos)
                current_pos = parent[current_pos]
            return path[::-1]  # Reverse the path to get it from start to goal
        
        # Process neighbors
        for dx, dy, cost in DIRECTIONS:
            nx, ny = x + dx, y + dy
            if 0 <= nx < rows and 0 <= ny < cols and grid[nx][ny] != 1:  # Within bounds and not an obstacle
                tentative_g_cost = g_cost[x][y] + cost
                if tentative_g_cost < g_cost[nx][ny]:
                    g_cost[nx][ny] = tentative_g_cost
                    f_cost[nx][ny] = g_cost[nx][ny] + heuristic((nx, ny), goal)
                    heapq.heappush(pq, (f_cost[nx][ny], (nx, ny)))
                    parent[(nx, ny)] = (x, y)  # Track the parent to reconstruct the path
    
    return []  # If no path found


# Heuristic function for A* (Manhattan Distance)
def heuristic(current, goal):
    return abs(current[0] - goal[0]) + abs(current[1] - goal[1])


# Function to print the state of the world with the robot's position
def print_grid(grid, path=[], current_position=None):
    # Create a copy of the grid to modify and print the robot's position
    grid_copy = [row[:] for row in grid]
    
    # If a current position is provided, mark it with 'X'
    if current_position:
        cx, cy = current_position
        grid_copy[cx][cy] = 'X'
    
    # Print the grid
    for row in grid_copy:
        print(" ".join(str(cell) if cell != 'X' else 'X' for cell in row))
    print("\n" + "-" * 20)  # Separator for each step

# Example Grid (5x7) with obstacles (1 represents an obstacle, 0 represents free space)
grid = [
    [0, 0, 0, 1, 0, 0, 0],
    [0, 0, 0, 1, 0, 0, 0],
    [0, 0, 0, 1, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0]
]

# Start and goal positions
start = (1, 1)  # Starting at top-left corner
goal = (1, 5)   # Goal at bottom-right corner

# Run Dijkstra or A* (Choose which one to use)
path_dijkstra = dijkstra(grid, start, goal)
path_astar = astar(grid, start, goal)

# Visualize the steps for Dijkstra
if path_dijkstra:
    print("Dijkstra path:")
    for step in path_dijkstra:
        print_grid(grid, path_dijkstra, current_position=step)
        time.sleep(0.5)  # Slow down to visualize each step
else:
    print("No path found by Dijkstra")

# Visualize the steps for A*
if path_astar:
    print("A* path:")
    for step in path_astar:
        print_grid(grid, path_astar, current_position=step)
        time.sleep(0.5)  # Slow down to visualize each step
else:
    print("No path found by A*")

