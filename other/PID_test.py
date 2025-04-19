import airsim
import airsimneurips
import time
import numpy as np
from scipy.interpolate import CubicSpline, interp1d
from scipy.integrate import cumtrapz
import matplotlib.pyplot as plt

#connect to server with drone controls
client = airsimneurips.MultirotorClient()
client.confirmConnection()

# Soccer_Field_Easy, ZhangJiaJie_Medium, Qualifier_Tier_1
client.simLoadLevel('Qualifier_Tier_1')

client.enableApiControl(vehicle_name="drone_1")
client.arm(vehicle_name="drone_1")

# # Async methods returns Future. Call join() to wait for task to complete.
client.takeoffAsync(vehicle_name="drone_1").join()


# // gate coords //
# for testing PID

# qualifier_1
moveList = [airsim.Vector3r(10.388415336608887, 80.77406311035156, -43.57999801635742),
            airsim.Vector3r(18.11046600341797, 76.26078033447266, -43.57999801635742),
            airsim.Vector3r(25.433794021606445, 66.28687286376953, -43.57999801635742),
            airsim.Vector3r(30.065513610839844, 56.549530029296875, -43.57999801635742),
            airsim.Vector3r(32.30064392089844, 45.6310920715332, -43.87999725341797),
            airsim.Vector3r(26.503353118896484, 38.19984436035156, -43.37999725341797),
            airsim.Vector3r(3.264113664627075, 37.569061279296875, -43.57999801635742),
            airsim.Vector3r(-16.862957000732422, 45.41843795776367, -46.57999801635742),
            airsim.Vector3r(-15.493884086608887, 63.18687438964844, -52.07999801635742),
            airsim.Vector3r(-6.320737361907959, 78.21236419677734, -55.779998779296875),
            airsim.Vector3r(5.143640041351318, 82.38504791259766, -55.779998779296875),
            airsim.Vector3r(14.558510780334473, 84.4320297241211, -55.18000030517578),
            airsim.Vector3r(20.858510971069336, 87.83203125, -42.07999801635742), # added
            airsim.Vector3r(23.858510971069336, 82.83203125, -32.07999801635742),
            airsim.Vector3r(38.25851058959961, 78.13202667236328, -31.3799991607666),
            airsim.Vector3r(51.058509826660156, 52.13203048706055, -25.8799991607666),
            airsim.Vector3r(44.95851135253906, 38.932029724121094, -25.8799991607666),
            airsim.Vector3r(25.958515167236328, 26.33203125, -19.8799991607666),
            airsim.Vector3r(11.658514976501465, 26.33203125, -12.779999732971191),
            airsim.Vector3r(-10.141484260559082, 22.632030487060547, -6.37999963760376),
            airsim.Vector3r(-24.641483306884766, 9.132031440734863, 2.119999885559082),]


# // generate spline based on coords //

waypoints = []
for i in moveList:
    waypoints.append([i.x_val,i.y_val,i.z_val])

waypoints = np.array(waypoints)
t = np.linspace(0, 1, len(waypoints))

spline_x = CubicSpline(t, waypoints[:, 0])
spline_y = CubicSpline(t, waypoints[:, 1])
spline_z = CubicSpline(t, waypoints[:, 2])

# clamp z axis?

num_points = 300 
# t_new = np.linspace(0, 1, num_points)

# path_x = spline_x(t_new)
# path_y = spline_y(t_new)
# path_z = spline_z(t_new)
# path = np.stack((path_x, path_y, path_z), axis=-1)

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

moveList = []
for i in path:
    moveList.append(airsim.Vector3r(i[0],i[1],i[2]))
    # print([i[0],i[1],i[2]])

# print(path)
# print(moveList)


print("start here:")

client.simStartRace(2)

print(client.getMultirotorState().kinematics_estimated.position)

# values for PID graph
vel_num = 0
vel_list = []

# // PID VALUES //

kp = 8 # proportional gain was 8
ki = 6 # integral gain
kd = 0.07 # derivative gain
dt = 0.001 # sleep time for loop (ms)

prevError = np.array([0.0, 0.0, 0.0])
integral = np.array([0.0, 0.0, 0.0])

# // PID logic // 

for i in path:

    prevError = np.array([0.0, 0.0, 0.0])
    integral = np.array([0.0, 0.0, 0.0])

    while True:

        curVel = client.getMultirotorState().kinematics_estimated.linear_velocity
        vel_num += 1
        vel_list.append([vel_num, (curVel.x_val**2+curVel.y_val**2)**0.5])

        # get current position
        state = client.getMultirotorState()
        print(client.simGetVehiclePose("drone_1"))
        print()
        curPos = np.array([state.kinematics_estimated.position.x_val,
                           state.kinematics_estimated.position.y_val,
                           state.kinematics_estimated.position.z_val])

        # PID logic and calculations:
        error = np.array(i) - np.array(curPos)
        integral += error * dt # accumulation of error
        derivative = (error - prevError) / dt

        controlSignal = kp * error + ki * integral + kd * derivative
        prevError = error

        # print(f"error: {error}, integral: {integral}, derivative: {derivative}")
        # print(f"input: {controlSignal}, position: {curPos}, error: {error}")
        # print(f"vx: {controlSignal[0]}, vy: {controlSignal[1]}, vz: {controlSignal[2]}")
        # print("\n")

        # plug in PID values
        client.moveByVelocityAsync(controlSignal[0], controlSignal[1], controlSignal[2], dt) # .join()
        # print(controlSignal)

        # Check if drone is close enough to the waypoint (sphere detection)
        if np.linalg.norm(np.array(i) - np.array(curPos)) < 1:
            # print(f"here {error}")
            break
        
        time.sleep(dt)



plot_time, plot_vel = zip(*vel_list)
plot_time = np.array(plot_time)
plot_vel = np.array(plot_vel)
# ideal_vel = np.array([5]*len(plot_time))

plt.figure()
plt.plot(plot_time,plot_vel)
# plt.plot(plot_time,ideal_vel)
plt.xlabel('Time')
plt.ylabel('Vel')
plt.title('Vel Plot (only X and Y)')
plt.grid(True)
plt.show()
