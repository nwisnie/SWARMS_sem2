import heapq
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Node:
    def __init__(self, position, parent=None, g=0, h=0):
        self.position = position
        self.parent = parent
        self.g = g  # Cost from start to current node
        self.h = h  # dist cost to goal
        self.f = g + h  # Total cost

    def __lt__(self, other):
        return self.f < other.f  # Needed for priority queue sorting

def get_dist(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def get_neighbors(position):
    x, y, z = position
    neighbors = [(x + dx, y + dy, z + dz) for dx in [-1, 0, 1] for dy in [-1, 0, 1] for dz in [-1, 0, 1]
                 if not (dx == 0 and dy == 0 and dz == 0)]
    return neighbors

def a_star_3d(grid, start, goal):
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
        
        for neighbor in get_neighbors(current_node.position):
            x, y, z = neighbor
            
            if (0 <= x < len(grid) and 0 <= y < len(grid[0]) and 0 <= z < len(grid[0][0]) and 
                grid[x][y][z] == 0 and neighbor not in closed_set):
                
                g_cost = current_node.g + 1
                h_cost = get_dist(neighbor, goal)
                neighbor_node = Node(neighbor, current_node, g_cost, h_cost)

                if not any(node.position == neighbor and node.f <= neighbor_node.f for node in open_set):
                    heapq.heappush(open_set, neighbor_node)

    return None  # No path found

def plot_path_3d(grid, path, start, goal):
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')

    label_flag = 0
    for x in range(len(grid)):
        for y in range(len(grid[0])):
            for z in range(len(grid[0][0])):
                if grid[x][y][z] == 1:
                    if label_flag:
                        ax.scatter(x, y, z, color='red', s=500, marker='s')
                    else:
                        ax.scatter(x, y, z, color='red', s=500, label="Obstacle", marker='s')
                        label_flag = 1

    # Plot start and goal
    ax.scatter(start[0], start[1], start[2], color='green', s=200, label="Start", marker="o")
    ax.scatter(goal[0], goal[1], goal[2], color='purple', s=200, label="Goal", marker="o")

    # Plot path
    if path:
        path_x = [p[0] for p in path]
        path_y = [p[1] for p in path]
        path_z = [p[2] for p in path]
        ax.plot(path_x, path_y, path_z, color="blue", linewidth=2, label="Path")

    ax.set_xlabel("X Axis")
    ax.set_ylabel("Y Axis")
    ax.set_zlabel("Z Axis")
    ax.legend()
    plt.show()



# Example 3D grid (0 = walkable, 1 = obstacle)
grid = np.zeros((5, 5, 5), dtype=int)

# Very tedius way to define obstacles
grid[2, 2, 1] = 1
grid[2, 2, 2] = 1
grid[2, 2, 3] = 1

grid[2, 1, 1] = 1
grid[2, 1, 2] = 1
grid[2, 1, 3] = 1

grid[1, 2, 1] = 1
grid[1, 2, 2] = 1
grid[1, 2, 3] = 1

grid[1, 3, 4] = 1
grid[2, 3, 4] = 1
grid[3, 3, 4] = 1

grid[1, 4, 4] = 1
grid[2, 4, 4] = 1
grid[3, 4, 4] = 1

start = (0, 0, 0)
goal = (4, 4, 4)

path = a_star_3d(grid, start, goal)

if path:
    print("Path:", path)
    plot_path_3d(grid, path, start, goal)
else:
    print("No path found.")