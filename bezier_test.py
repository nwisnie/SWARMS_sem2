import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import interp1d

# start and end points:
P0 = np.array([0, 0, 0])
P1 = np.array([10, 10, 10])
# initial velocty (will be implicit to drone):
V0 = np.array([10, 0, 0])

# test values from the sim:
# P0 = np.array([1.41622841, 8.51588917, 2.46687388]) 
# P1 = np.array([8.88708401, 18.47876167, 2.01999998])
# V0 = np.array([0.43964687, 6.77013874, -0.75947905])

# Control points 
vel_bias = 0.75  # when we give this to controls team, they will need to find best value for this
P1a = P0 + V0 * vel_bias
P1b = P1
# P1b = P1 - (P1 - P0) * vel_bias  # Calculate second control point because we only have initial velocity

# Bezier equation
def bezier(t):
    return (1 - t)**3 * P0 + 3 * (1 - t)**2 * t * P1a + 3 * (1 - t) * t**2 * P1b + t**3 * P1

# get curve
ts = np.linspace(0, 1, 100)
curve = np.array([bezier(t) for t in ts])
dists = np.linalg.norm(np.diff(curve, axis=0), axis=1)
arc_lengths = np.concatenate([[0], np.cumsum(dists)])

# make points on curve evenly spaced
num_points = 50
even_arc_lens = np.linspace(0, arc_lengths[-1], num_points)
interp_func = interp1d(arc_lengths, curve, axis=0)
evenly_spaced_curve = interp_func(even_arc_lens)

# print(evenly_spaced_curve)

# For plotting
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(*evenly_spaced_curve.T, color='red', label='Evenly spaced points')
ax.scatter(*P0, color='green', s=100, label='Start')
ax.scatter(*P1, color='blue', s=100, label='End')
ax.quiver(*P0, *V0, length=0.5, color='black', label='Initial Velocity')
ax.legend(loc='upper left')
plt.show()