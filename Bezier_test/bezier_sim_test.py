import airsim
import airsimneurips
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import interp1d


# -- get curve --
# P0 -> drone's current position (numpy array with 3 entries)
# P1 -> next gate postion (numpy array with 3 entries)
# V0 -> drone's current velocity (numpy array with 3 entries)
# vel_bias -> effect of inital velocity (int)
# spacing_const -> increasing this number will increase how many points are in each path // generally keep this number pretty low
def get_curve(P0, P1, V0, vel_bias, spacing_const):

    # decrease effect of z velocity
    V0[2] *= 0.5

    P1a = P0 + V0 * vel_bias
    P1b = P1

    # Bezier equation
    def bezier(t):
        return (1 - t)**3 * P0 + 3 * (1 - t)**2 * t * P1a + 3 * (1 - t) * t**2 * P1b + t**3 * P1

    # get curve
    ts = np.linspace(0, 1, 25)
    curve = np.array([bezier(t) for t in ts])
    dists = np.linalg.norm(np.diff(curve, axis=0), axis=1)
    arc_lengths = np.concatenate([[0], np.cumsum(dists)])

    # determine number of points based on distance between
    num_points = spacing_const * int(np.linalg.norm(np.array(P0) - np.array(P1)))
    print(num_points)

    # make points on curve evenly spaced
    even_arc_lens = np.linspace(0, arc_lengths[-1], num_points)
    interp_func = interp1d(arc_lengths, curve, axis=0)
    evenly_spaced_curve = interp_func(even_arc_lens)

    return evenly_spaced_curve


## -- mainline code --

#connect to server with drone controls
client = airsimneurips.MultirotorClient()
client.confirmConnection()

# Soccer_Field_Easy, ZhangJiaJie_Medium, Qualifier_Tier_1
client.simLoadLevel('Soccer_Field_Easy')

client.enableApiControl(vehicle_name="drone_1")
client.arm(vehicle_name="drone_1")

# Async methods returns Future. Call join() to wait for task to complete.
client.takeoffAsync(vehicle_name="drone_1").join()

client.simStartRace()

# qualifier_tier_1
gates = ['Gate00', 'Gate01', 'Gate02', 
         'Gate03', 'Gate04', 'Gate05', 
         'Gate06', 'Gate07', 'Gate08', 
         'Gate09', 'Gate10_21', 'Gate11_23', 
         'StartBlock']

all_pts = []

for obj in gates:
    cur_pos = client.getMultirotorState().kinematics_estimated.position
    cur_vel = client.getMultirotorState().kinematics_estimated.linear_velocity
    gate_pos = client.simGetObjectPose(obj).position

    P0 = np.array([cur_pos.x_val, cur_pos.y_val, cur_pos.z_val])
    P1 = np.array([gate_pos.x_val, gate_pos.y_val, gate_pos.z_val])
    V0 = np.array([cur_vel.x_val, cur_vel.y_val, cur_vel.z_val])

    print("start/end/vel:")
    print(P0, P1, V0)

    points = get_curve(P0,P1,V0, 0.75,2)

    # print("curve:")
    # print(points)

    for i in points:
        client.moveToPositionAsync(i[0],i[1],i[2],8).join()
        all_pts.append((i[0],i[1],i[2]))
        


# -- graphing --

coords = np.array(all_pts)
x, y, z = coords[:, 0], coords[:, 1], coords[:, 2]

fig = plt.figure()
plt.figure()
plt.plot(x, y, marker='o', label='2D Path')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('2D Path Plot')
plt.grid(True)
plt.legend()
plt.show()
