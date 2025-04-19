import airsim
import airsimneurips
import heapq
import numpy as np


## This file is needed to run get_path_A_star.py


class Node:
    def __init__(self, position, parent=None, cost=0):
        self.position = position
        self.parent = parent
        self.cost = cost  # Total cost

    def __lt__(self, other):
        return self.cost < other.cost  # Needed for priority queue sorting

def get_dist(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))
    # return abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2])

def get_neighbors(position, grid_shape):
    x, y, z = position
    neighbors = [
        (x+1, y, z), (x, y+1, z), (x, y, z+1),
        (x-1, y, z), (x, y-1, z), (x, y, z-1)
    ]
    # Filter neighbors to stay within grid bounds
    return [(nx, ny, nz) for nx, ny, nz in neighbors
            if 0 <= nx < grid_shape[0] and 0 <= ny < grid_shape[1] and 0 <= nz < grid_shape[2]]

def a_star_3d(grid, start, goal):
    open_set = []
    closed_set = set()
    node_map = {}  # Stores best known cost to a position
    
    start_node = Node(start, None, get_dist(start, goal))
    heapq.heappush(open_set, start_node)
    node_map[start] = start_node.cost

    while open_set:
        # print(node_map)
        current_node = heapq.heappop(open_set)

        if current_node.position == goal:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]  # Reverse to get correct order
        
        closed_set.add(current_node.position)

        for neighbor in get_neighbors(current_node.position, grid.shape):
            if neighbor in closed_set or grid[neighbor] == 1:  # Skip obstacles
                continue

            cur_cost = current_node.cost + 1
            cur_cost += get_dist(neighbor, goal)
            # print(node_map)
            # if node_map != 0 and node_map[neighbor]:
            #     print(node_map[neighbor])
            #     cur_cost += node_map[neighbor]
            neighbor_node = Node(neighbor, current_node, cur_cost)

            if neighbor not in node_map or neighbor_node.cost < node_map[neighbor]:
                node_map[neighbor] = neighbor_node.cost
                heapq.heappush(open_set, neighbor_node)

    return []  # No path found