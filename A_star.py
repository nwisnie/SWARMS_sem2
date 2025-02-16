import airsim
import airsimneurips
# from scipy.interpolate import CubicSpline, interp1d
# from scipy.integrate import cumtrapz
import heapq
import numpy as np
import copy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


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

## for plotting the generated path in a 3d space
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


# mainline helper function to get A* path with drone and gate positions
def get_path(curPos, gatePos):
    minX_at_drone = 0
    minY_at_drone = 0
    minZ_at_drone = 0

    minX = min(gatePos.x_val, curPos.x_val)
    minY = min(gatePos.y_val, curPos.y_val)
    minZ = min(gatePos.z_val, curPos.z_val)

    if (min(gatePos.x_val, curPos.x_val) == curPos.x_val):
        minX_at_drone = 1
    if (min(gatePos.y_val, curPos.y_val) == curPos.y_val):
        minY_at_drone = 1
    if (min(gatePos.z_val, curPos.z_val) == curPos.z_val):
        minZ_at_drone = 1

    maxX = max(gatePos.x_val, curPos.x_val)
    maxY = max(gatePos.y_val, curPos.y_val)
    maxZ = max(gatePos.z_val, curPos.z_val)

    # make grid with subdivision size of 0.25
    gridX = int((maxX-minX)//0.25)
    gridY = int((maxY-minY)//0.25)
    gridZ = int((maxZ-minZ)//0.25)

    print("dimensions:")
    print(gridX)
    print(gridY)
    print(gridZ)
    a_star_grid = np.zeros((gridX, gridY, gridZ), dtype=int)

    # first run A_star on simplified grid, then get actual positions based on generate points
    # here is my scuffed way of getting start and end positions
    start = (0 if minX_at_drone else gridX-1, 0 if minY_at_drone else gridY-1, 0 if minZ_at_drone else gridZ-1)
    end = (0 if not minX_at_drone else gridX-1, 0 if not minY_at_drone else gridY-1, 0 if not minZ_at_drone else gridZ-1)
    print(start, end)
    print()

    a_star_path = a_star_3d(a_star_grid, start, end)
    # print(a_star_path)

    actual_path = []

    # in case a path isn't generated, the run doesn't end
    if a_star_path == None:
        return None

    # convert back to airsim gridspace
    for i in a_star_path:
        actual_path.append((minX+i[0]*0.25, minY+i[1]*0.25, minZ+i[2]*0.25))

    return actual_path


## start mainline code

#connect to server with drone controls
client = airsimneurips.MultirotorClient()
client.confirmConnection()

# Soccer_Field_Easy, ZhangJiaJie_Medium, Qualifier_Tier_1
client.simLoadLevel('Soccer_Field_Easy')

client.enableApiControl(vehicle_name="drone_1")
client.arm(vehicle_name="drone_1")

client.simStartRace()

# soccerfield_easy gates
gates = ['Gate00', 'Gate01', 'Gate02', 
         'Gate03', 'Gate04', 'Gate05', 
         'Gate06', 'Gate07', 'Gate08', 
         'Gate09', 'Gate10_21', 'Gate11_23', 
         'StartBlock']

# # Async methods returns Future. Call join() to wait for task to complete.
client.takeoffAsync(vehicle_name="drone_1").join()

for gate in gates:
    curPos = client.getMultirotorState().kinematics_estimated.position
    print(curPos)

    gatePos = client.simGetObjectPose(gate).position
    print(gatePos)

    path = get_path(curPos,gatePos)

    # print(path)
    if path != None:
        for i in path:
            client.moveToPositionAsync(i[0], i[1], i[2],5).join()