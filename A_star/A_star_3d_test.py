import heapq
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import CubicSpline, interp1d
from scipy.integrate import cumtrapz
import time


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
    # x, y, z = position
    # neighbors = [(x + dx, y + dy, z + dz) for dx in [-1, 0, 1] for dy in [-1, 0, 1] for dz in [-1, 0, 1]
    #              if not (dx == 0 and dy == 0 and dz == 0)]
    # return neighbors
    x, y, z = position
    neighbors = [
        (x+1, y, z), (x, y+1, z), (x, y, z+1),
        (x-1, y, z), (x, y-1, z), (x, y, z-1)
    ]
    # Filter neighbors to stay within grid bounds
    return neighbors


def a_star_3d(grid, start, goal, vel_bias):
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

                # I do velocity bias finding distance from each point to starting point
                # then scaling with the veloctiy amount
                x_bias = vel_bias[0] / max(abs(x - start[0]), 1)
                y_bias = vel_bias[1] / max(abs(y - start[1]), 1)
                z_bias = vel_bias[2] / max(abs(z - start[2]), 1)
                neighbor_weighted = (neighbor[0] + x_bias, neighbor[1] + y_bias, neighbor[2] + z_bias)

                # was just neighbor
                h_cost = get_dist(neighbor_weighted, goal)
                neighbor_node = Node(neighbor, current_node, g_cost, h_cost)

                if not any(node.position == neighbor and node.f <= neighbor_node.f for node in open_set):
                    heapq.heappush(open_set, neighbor_node)

    return None  # No path found


def plot_path_3d(grid, path, spline, start, goal):

    print(path)
    print(spline)

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

    spline_x = [p[0] for p in spline]
    spline_y = [p[1] for p in spline]
    spline_z = [p[2] for p in spline]
    ax.plot(spline_x, spline_y, spline_z, color="green", linewidth=2, label="Spline")

    ax.set_xlabel("X Axis")
    ax.set_ylabel("Y Axis")
    ax.set_zlabel("Z Axis")
    ax.legend()
    plt.show()

def gen_spline(waypoints, num_points):

    path_x = []
    path_y = []
    path_z = []
    for i in waypoints:
        path_x.append(i[0])
        path_y.append(i[1])
        path_z.append(i[2])

    t = np.linspace(0, 1, len(waypoints))

    spline_x = CubicSpline(t, path_x)
    spline_y = CubicSpline(t, path_y)
    spline_z = CubicSpline(t, path_z)

    dt = 0.01  # Small time step for numerical differentiation
    t_fine = np.arange(0, 1, dt)
    dx_dt = spline_x(t_fine, 1)
    dy_dt = spline_y(t_fine, 1)
    dz_dt = spline_z(t_fine, 1)

    # Compute the differential arc length
    d_length = np.sqrt(dx_dt**2 + dy_dt**2 + dz_dt**2)
    arc_length = cumtrapz(d_length, t_fine, initial=0)

    # Step 2: Interpolate to find evenly spaced points along arc length
    total_length = arc_length[-1]
    even_arc_lengths = np.linspace(0, total_length, num_points)
    t_even = interp1d(arc_length, t_fine)(even_arc_lengths)

    # Step 3: Evaluate splines at these new t values
    path_x = spline_x(t_even)
    path_y = spline_y(t_even)
    path_z = spline_z(t_even)
    path = np.stack((path_x, path_y, path_z), axis=-1)

    # print(path)

    return path


grid = np.zeros((11, 11, 11), dtype=int)
start = (0, 0, 0)
goal = (10, 10, 10)

# added: initial velocty at starting point
vel_bias = (0,0,0)

# start_time = time.time()
# a_star_3d(grid, start, goal)
# end_time = time.time()
# measured_time = end_time - start_time
# print(end_time-start_time)

path = a_star_3d(grid, start, goal, vel_bias)

print("Path:", path)

# spline test 1
# spline = gen_spline(path, 5)

# spline test 2
# take average of 3 points
spline = []
for i in range(len(path) - 4):
    tempX = (path[i][0] + path[i+1][0] + path[i+2][0] + path[i+3][0]) / 4
    tempY = (path[i][1] + path[i+1][1] + path[i+2][1] + path[i+3][1]) / 4
    tempZ = (path[i][2] + path[i+1][2] + path[i+2][2] + path[i+3][2]) / 4
    spline.append((tempX,tempY,tempZ))

spline[0] = path[0]
spline[-1] = path[-1]

plot_path_3d(grid, path, spline, start, goal)
