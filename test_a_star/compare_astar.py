import other_group_test
import A_star_3d_test
import time
import timeit
import numpy as np

# test file to compare both groups

start = (0,0,0)
goal = (9,9,0)
scales = (1,1,1) # uniform scaling

# print("previous group's A* (on empty grid):")

# grid1 =  np.zeros((10, 10, 1), dtype=int)
# grid1 = grid1.tolist()
# print(grid1)

# start_time = time.time()
# other_group_test.astar(start,goal,grid1,scales)
# end_time = time.time()
# measured_time = end_time - start_time
# print(end_time-start_time)

# print(other_group_test.astar(start,goal,grid1,scales)[0])

# print()

print("My A* (on empty grid):")

grid2 = np.zeros((10, 10, 1), dtype=int)

start_time = time.time()
A_star_3d_test.a_star_3d(grid2, start, goal)
end_time = time.time()
measured_time = end_time - start_time
print(end_time-start_time)

print(A_star_3d_test.a_star_3d(grid2, start, goal))

print()
print()

# now add obstacles:

# grid1 = np.array(grid1)
# print(grid1.shape)

# grid2[1, 0, 0] = 1 # wall one
# grid2[1, 1, 0] = 1
# grid2[1, 1, 0] = 1
# grid2[1, 2, 0] = 1
# grid2[1, 3, 0] = 1
# grid2[0, 5, 0] = 1 # wall two
# grid2[1, 5, 0] = 1
# grid2[2, 5, 0] = 1
# grid2[3, 5, 0] = 1
# grid2[3, 4, 0] = 1 # wall three
# grid2[3, 3, 0] = 1
# grid2[3, 2, 0] = 1
# grid2[7, 0, 0] = 1 # wall four
# grid2[7, 1, 0] = 1
# grid2[7, 2, 0] = 1
# grid2[7, 3, 0] = 1
# grid2[7, 4, 0] = 1
# grid2[7, 5, 0] = 1
# grid2[6, 5, 0] = 1 # wall five
# grid2[5, 5, 0] = 1
# grid2[5, 6, 0] = 1 # wall six
# grid2[5, 7, 0] = 1

# grid1 = grid1.tolist()

print(grid2.shape)

grid2[1, 0, 0] = 1 # wall one
grid2[1, 1, 0] = 1
grid2[1, 1, 0] = 1
grid2[1, 2, 0] = 1
grid2[1, 3, 0] = 1
grid2[0, 5, 0] = 1 # wall two
grid2[1, 5, 0] = 1
grid2[2, 5, 0] = 1
grid2[3, 5, 0] = 1
grid2[3, 4, 0] = 1 # wall three
grid2[3, 3, 0] = 1
grid2[3, 2, 0] = 1
grid2[7, 0, 0] = 1 # wall four
grid2[7, 1, 0] = 1
grid2[7, 2, 0] = 1
grid2[7, 3, 0] = 1
grid2[7, 4, 0] = 1
grid2[7, 5, 0] = 1
grid2[6, 5, 0] = 1 # wall five
grid2[5, 5, 0] = 1
grid2[5, 6, 0] = 1 # wall six
grid2[5, 7, 0] = 1



# print("previous group's A* (with obstacles):")

# start_time = time.time()
# other_group_test.astar(start,goal,grid1,scales)
# end_time = time.time()
# measured_time = end_time - start_time
# print(end_time-start_time)

# path1 = other_group_test.astar(start,goal,grid1,scales)[0]
# print(path1)

# print()

print("My A* (with obstacles):")

start_time = time.time()
A_star_3d_test.a_star_3d(grid2, start, goal)
end_time = time.time()
measured_time = end_time - start_time
print(end_time-start_time)

path2 = A_star_3d_test.a_star_3d(grid2, start, goal)
print(path2)


# plot paths
# grid1 = np.array(grid1)
# A_star_3d_test.plot_path_3d(grid1, path1, start, goal) # theirs
A_star_3d_test.plot_path_3d(grid2, path2, start, goal) # mine