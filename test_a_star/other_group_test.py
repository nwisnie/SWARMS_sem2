import math
import copy
# import random
# import numpy as np
# import pickle
import heapq
# import re
# import time
import matplotlib.pyplot as plt

class Node:

    def __init__(self, position : tuple[int, int, int], parent : 'Node' = None) -> None:
        self.position = position
        self.parent = parent # The last node to update this node's distance value (With the shortest path)

        self.distance = 0 # The correct distance from the start position to this Node
        self.heuristic = 0 # An estimate of the distance from the goal to this node

    def __eq__(self, other: object) -> bool: # Are the positions the same
        return (self.position[0] == other.position[0] and
                self.position[1] == other.position[1] and
                self.position[2] == other.position[2])
    
    def __hash__(self) -> int: # Allows usage in a set
        return hash(self.position)
    
    def __lt__(self, other: object) -> bool: # Allows usage in a Priority queue
        return ((self.distance + self.heuristic) < (other.distance + other.heuristic))

    

def create_cost_grid(grid, check_range):
    new_grid = copy.deepcopy(grid)

    for l in range(len(grid)):
        for r in range(len(grid[0])):
            for c in range(len(grid[0][0])):
                if grid[l][r][c] == 1:
                    if c == 0 or grid[l][r][c - 1] == 1:
                        left_check = 0
                    else:
                        left_check = -check_range

                    if c == len(grid[0][0]) - 1 or grid[l][r][c + 1] == 1:
                        right_check = 0
                    else:
                        right_check = check_range

                    if r == len(grid[0]) - 1 or grid[l][r + 1][c] == 1:
                        up_check = 0
                    else:
                        up_check = check_range

                    if r == 0 or grid[l][r - 1][c] == 1:
                        down_check = 0
                    else:
                        down_check = -check_range

                    layer_down_chcek = -check_range
                    layer_up_check = check_range
                    for dx in range(left_check, right_check + 1):
                        for dy in range(down_check, up_check + 1):
                            for dz in range(layer_down_chcek, layer_up_check + 1):
                                x, y, z = c + dx, r + dy, l + dz
                                if 0 <= x < len(grid[0][0]) and 0 <= y < len(grid[0]) and 0 <= z < len(grid) and grid[z][y][x] != 1:
                                    cost = min((check_range**2), (check_range**2) / math.sqrt(dx**2 + dy**2 + dz**2))
                                    new_grid[z][y][x] = max(new_grid[z][y][x], cost)

    return new_grid


def get_directions():
    directions = []
    for x in [-1, 0, 1]:
        for y in [-1, 0, 1]:
            for z in [-1, 0, 1]:
                if (x, y, z) != (0, 0, 0):  # Exclude the zero vector
                    directions.append((x, y, z))
    return directions


def get_distance(start, end, scales):
    return math.hypot((start[0] - end[0]) * scales[0], (start[1] - end[1]) * scales[1], (start[2] - end[2]) * scales[2])


def astar(start, end, graph, scales, cost_grid=[]):
    """
    Returns the path traversing graph using a* search

    Parameters
    ----------
    start : tuple or list
        (x, y)
    end : tuple or list
        (x, y)
    graph : 3d list
        graph to traverse
    cost_grid: 3d list
        adds a penalty

    Returns
    -------
    list of (x, y, z) or False
        path from start to end if found
    """
    
    if graph[start[2]][start[1]][start[0]] == 1 or graph[end[2]][end[1]][end[0]] == 1:
        return []

    open_list = []
    closed_list = set()
    open_set = {}

    # f_score. g_score, pos
    start_node = (0, 0, start)
    index = 0
    open_list.append((0, index, start_node))
    heapq.heapify(open_list)

    parent = {}

    while open_list:
        start_f_score, start_g_score, start_pos = heapq.heappop(open_list)[-1]
        closed_list.add(start_pos)

        if start_pos[0] == end[0] and start_pos[1] == end[1] and start_pos[2] == end[2]:
            pos = start_pos
            path = []
            while pos in parent:
                path.append(pos)
                pos = parent[pos]

            path = path[::-1]
            return path, [node[2] for _, _, node in open_list], closed_list

#        for dir in get_directions():
        for dir in [(-1, -1, -1), (-1, -1, 0), (-1, -1, 1), (-1, 0, -1), (-1, 0, 0), (-1, 0, 1), (-1, 1, -1), (-1, 1, 0), (-1, 1, 1), (0, -1, -1), (0, -1, 0), (0, -1, 1), (0, 0, -1), (0, 0, 1), (0, 1, -1), (0, 1, 0), (0, 1, 1), (1, -1, -1), (1, -1, 0), (1, -1, 1), (1, 0, -1), (1, 0, 0), (1, 0, 1), (1, 1, -1), (1, 1, 0), (1, 1, 1)]:
            x = dir[0]
            y = dir[1]
            z = dir[2]

            pos = (start_pos[0] + x, start_pos[1] + y, start_pos[2] + z)

            if pos in closed_list:
                continue

            if pos[0] < 0:
                continue
            if pos[0] >= len(graph[0][0]):
                continue
            if pos[1] < 0:
                continue
            if pos[1] >= len(graph[0]):
                continue
            if pos[2] < 0:
                continue
            if pos[2] >= len(graph):
                continue

            if graph[pos[2]][pos[1]][pos[0]] == 1:
                continue

            g_score = start_g_score + get_distance(start_pos, pos, scales)
            h_score = get_distance(end, pos, scales)
            f_score = g_score + h_score

            if len(cost_grid) != 0:
                f_score += cost_grid[pos[2]][pos[1]][pos[0]]

            if pos in open_set:
                if open_set[pos] <= g_score:
                    continue

            open_set[pos] = g_score
            parent[pos] = start_pos
            index += 1
            heapq.heappush(open_list, (f_score, index, (f_score, g_score, pos)))

    return []


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


def get_grid(curPos, gatePos):
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

    # a_star_grid = np.zeros((gridX, gridY, gridZ), dtype=int)
    a_star_grid = [[[0 for x in range(gridX)] for y in range(gridY)] for z in range(gridZ)]

    # first run A_star on simplified grid, then get actual positions based on generate points
    # here is my scuffed way of getting start and end positions
    start = (0 if minX_at_drone else gridX-1, 0 if minY_at_drone else gridY-1, 0 if minZ_at_drone else gridZ-1)
    end = (0 if not minX_at_drone else gridX-1, 0 if not minY_at_drone else gridY-1, 0 if not minZ_at_drone else gridZ-1)

    return a_star_grid, start, end

    
# grid = [[[0 for x in range(10)] for y in range(10)] for z in range(10)]
# start = (0,0,0)
# end = (9,9,9)
# scales = (0,0,0) # uniform scaling

# print("here")

# start_time = time.time()
# astar(start,end,grid,scales)
# end_time = time.time()
# measured_time = end_time - start_time
# print(end_time-start_time)

# print(astar(start,end,grid,scales)[0])


### mainline tests ###
# comment out when running compare_astar.py

scales = (1,1,1)
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
    grid, start, end = get_grid(start, end)
    path = astar(start,end,grid,scales)
    print(path[0])
    plot_path_3d(grid,path[0],start,end)