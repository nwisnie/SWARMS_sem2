from scipy.interpolate import CubicSpline, interp1d
from scipy.integrate import cumtrapz
import heapq
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# this file is for properly testing my 3d A* without having to run the sim
# I get the set of coords from the sim

## start A_star implementation ##

class Node:
    def __init__(self, position, parent=None, g=0, h=0):
        self.position = position
        self.parent = parent
        self.g = g  # Cost from start to current node
        self.h = h  # dist cost to goal
        self.f = g + h  # Total cost

    def __lt__(self, other):
        return self.f < other.f  # Needed for priority queue sorting


# helper functions for A*
def get_dist(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))
def get_neighbors(position):
    x, y, z = position
    neighbors = [(x + dx, y + dy, z + dz) for dx in [-1, 0, 1] for dy in [-1, 0, 1] for dz in [-1, 0, 1]
                 if not (dx == 0 and dy == 0 and dz == 0)]
    return neighbors


# grid: a 3d numpy array representing the 3d space that the path can be in
# start: a tuple containing the starting point in 3d
# end: a tuple containing the ending pount in 3d
def a_star_3d(grid, start, goal):
    # print(grid)
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


## for visualizing a generated path in a 3d space
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

## end A_star implementation 


# function to be called in main to get A* path based on two points (curPos and gatePos)
# curPos and gatePos are Airsim pose objects that will be the starting and ending points of A*
def get_path(curPos, gatePos):
    minX_at_drone = 0
    minY_at_drone = 0
    minZ_at_drone = 0

    minX = min(gatePos[0], curPos[0])
    minY = min(gatePos[1], curPos[1])
    minZ = min(gatePos[2], curPos[2])

    if (min(gatePos[0], curPos[0]) == curPos[0]):
        minX_at_drone = 1
    if (min(gatePos[1], curPos[1]) == curPos[1]):
        minY_at_drone = 1
    if (min(gatePos[2], curPos[2]) == curPos[2]):
        minZ_at_drone = 1

    maxX = max(gatePos[0], curPos[0])
    maxY = max(gatePos[1], curPos[1])
    maxZ = max(gatePos[2], curPos[2])

    # make grid with subdivision size of 0.25
    gridX = int((maxX-minX)//0.25)
    gridY = int((maxY-minY)//0.25)
    gridZ = int((maxZ-minZ)//0.25)

    print("dimensions:")
    print(gridX)
    print(gridY)
    print(gridZ)

    # really scuffed fix to grid generation issue
    if gridX == 0:
        gridX = 1
    if gridY == 0:
        gridY = 1
    if gridZ == 0:
        gridZ = 1

    a_star_grid = np.zeros((gridX, gridY, gridZ), dtype=int)

    # first run A_star on simplified grid, then get actual positions based on generate points
    # here is my scuffed way of getting start and end positions
    start = (0 if minX_at_drone else gridX-1, 0 if minY_at_drone else gridY-1, 0 if minZ_at_drone else gridZ-1)
    end = (0 if not minX_at_drone else gridX-1, 0 if not minY_at_drone else gridY-1, 0 if not minZ_at_drone else gridZ-1)
    # print(start, end)
    # print()

    a_star_path = a_star_3d(a_star_grid, start, end)
    # print(a_star_path)

    actual_path = []

    # in case a path isn't generated, so the run doesn't end
    if a_star_path == None:
        return None

    # convert back to airsim gridspace
    for i in a_star_path:
        actual_path.append((minX+i[0]*0.25, minY+i[1]*0.25, minZ+i[2]*0.25))
    # print(actual_path)

    return actual_path, a_star_grid


# gen_spline take A* path and make better
# waypoints: path generated by A* (array of 3d tuples)
# num_points: number of points in generated spline
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



### mainline code ###

# coords ripped from the sim
coord_array =  [((0.75,-2.0,0.7219569683074951),(0.0,2.0,2.0199999809265137)),
                ((-0.02349688671529293,0.8665379285812378,1.7400134801864624),(1.5999999046325684,10.800000190734863,2.0199999809265137)),
                ((1.174094319343567,9.966672897338867,1.9711052179336548),(8.887084007263184,18.478761672973633,2.0199999809265137)),
                ((7.8332414627075195,17.2027530670166,2.1105854511260986),(18.74375343322754,22.20650863647461,2.0199999809265137)),
                ((16.048688888549805,22.930774688720703,2.0449466705322266),(30.04375457763672,22.20648956298828,2.0199999809265137)),
                ((28.934246063232422,21.789390563964844,2.1449925899505615),(39.04375457763672,19.206478118896484,2.0199999809265137)),
                ((37.6785888671875,19.854711532592773,2.1753125190734863),(45.74375534057617,11.706478118896484,2.0199999809265137)),
                ((43.849464416503906,12.781919479370117,2.1517276763916016),(45.74375534057617,2.2064781188964844,2.0199999809265137)),
                ((44.349327087402344,4.253911972045898,1.9831748008728027),(40.343753814697266,-4.793521404266357,2.0199999809265137)),
                ((39.563621520996094,-2.7493977546691895,2.112156391143799),(30.74375343322754,-7.893521785736084,2.0199999809265137)),
                ((33.52824020385742,-6.758218288421631,1.9137357473373413),(18.54375457763672,-7.893521785736084,2.0199999809265137))]

for start, end in coord_array:
    print(start,end)
    path, grid = get_path(start,end)
    print(path)
    plot_path_3d(grid,path,start,end)
    # print(path)

    # generate a spline based on the A* result:
    # I do this because I want to keep A* low resolution for faster runtime
    # path = gen_spline(path, 50)
    # print(path)
