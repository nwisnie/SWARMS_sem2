import heapq
import numpy as np
import matplotlib.pyplot as plt

class Node:
    def __init__(self, position, parent=None, g=0, h=0):
        self.position = position
        self.parent = parent
        self.g = g  # Start to current node cost
        self.h = h  # Distance to goal cost
        self.f = g + h  # Total cost

    def __lt__(self, other):
        return self.f < other.f  # Needed for priority queue sorting

def get_dist(a, b):
    # Uncool but computationally nice way to do distance
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(grid, start, goal):
    open_set = []
    closed_set = set()
    start_node = Node(start, None, 0, get_dist(start, goal))
    
    heapq.heappush(open_set, start_node)
    
    while open_set:
        current_node = heapq.heappop(open_set)
        
        if current_node.position == goal:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]  # Reverse to get correct order
        
        closed_set.add(current_node.position)
        
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # Up, Down, Left, Right
            neighbor_pos = (current_node.position[0] + dx, current_node.position[1] + dy)
            
            if (0 <= neighbor_pos[0] < len(grid) and 
                0 <= neighbor_pos[1] < len(grid[0]) and 
                grid[neighbor_pos[0]][neighbor_pos[1]] == 0 and 
                neighbor_pos not in closed_set):
                
                g_cost = current_node.g + 1
                h_cost = get_dist(neighbor_pos, goal)
                neighbor_node = Node(neighbor_pos, current_node, g_cost, h_cost)

                if not any(node.position == neighbor_pos and node.f <= neighbor_node.f for node in open_set):
                    heapq.heappush(open_set, neighbor_node)

    return None  # No path found

def plot_path(grid, path, start, goal):
    grid_size = np.array(grid).shape

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_xlim(-0.5, grid_size[1] - 0.5)
    ax.set_ylim(-0.5, grid_size[0] - 0.5)
    ax.set_xticks(range(grid_size[1]))
    ax.set_yticks(range(grid_size[0]))
    ax.grid(True, which="both", linestyle="--", linewidth=0.5)

    # Obstacles
    for x in range(grid_size[0]):
        for y in range(grid_size[1]):
            if grid[x][y] == 1:
                ax.scatter(y, grid_size[0] - x - 1, color="red", s=1000, marker="s")

    # Start and Goal
    ax.scatter(start[1], grid_size[0] - start[0] - 1, color="green", s=200, label="Start", marker="o")
    ax.scatter(goal[1], grid_size[0] - goal[0] - 1, color="purple", s=200, label="Goal", marker="o")

    # Path
    if path:
        path_x = [p[1] for p in path]
        path_y = [grid_size[0] - p[0] - 1 for p in path]
        ax.plot(path_x, path_y, color="blue", linewidth=5, label="Path")

    ax.legend()
    plt.show()



# Example grid (0 = walkable, 1 = obstacle)
grid = [
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0, 0, 0, 0, 0]
]

start = (9, 0)
goal = (3, 9)

path = a_star(grid, start, goal)

if path:
    print("Path:", path)
    plot_path(grid, path, start, goal)
else:
    print("No path found.")