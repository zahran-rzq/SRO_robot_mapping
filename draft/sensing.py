# %%
import time
import math
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np

# %%
# 1. Setup Connection
client = RemoteAPIClient()
sim = client.require('sim')
client.setStepping(True)

# %%
# 2. Start Simulation
sim.startSimulation()
print("Simulation Started")

# %%
# 3. Setup Robot dan Pose Awal
p3dx_RW = sim.getObject("/PioneerP3DX/rightMotor")
p3dx_LW = sim.getObject("/PioneerP3DX/leftMotor")
p3dx = sim.getObject("/PioneerP3DX")
# The ultrasonic sensors are children of "visible" in the PioneerP3DX model.
# CoppeliaSim object aliases are case-sensitive.
s3 = sim.getObject("/PioneerP3DX/visible/ultrasonicSensor[3]")

p3dx_pos_x = 0.0
p3dx_pos_y = 0.0
# theta_z = 0.0
theta_z = sim.getObjectOrientation(p3dx, sim.handle_world)[2]

rw = 0.195/2  # radius roda
rb = 0.381/2  # setengah jarak antar roda
wheel_vec = sim.getObjectPosition(p3dx_RW, p3dx_LW)
rb = math.sqrt(sum(c**2 for c in wheel_vec)) / 2  # setengah jarak antar roda dari model
print(f"rb = {rb:.4f} m")

s3_orientation = np.deg2rad([-90, 80, 180])  # orientation sensor 3 in radians
s3_position_homogeneous = np.array([0.19061, 0.08313, 0.0681, 1])  # position sensor 3 in m
s3_rotation_matrix_x = np.array([
    [1, 0, 0],
    [0, np.cos(s3_orientation[0]), -np.sin(s3_orientation[0])],
    [0, np.sin(s3_orientation[0]), np.cos(s3_orientation[0])],
])
s3_rotation_matrix_y = np.array([
    [np.cos(s3_orientation[1]), 0, np.sin(s3_orientation[1])],
    [0, 1, 0],
    [-np.sin(s3_orientation[1]), 0, np.cos(s3_orientation[1])],
])
s3_rotation_matrix_z = np.array([
    [np.cos(s3_orientation[2]), -np.sin(s3_orientation[2]), 0],
    [np.sin(s3_orientation[2]), np.cos(s3_orientation[2]), 0],
    [0, 0, 1],
])
s3_rotation_matrix = s3_rotation_matrix_z @ s3_rotation_matrix_y @ s3_rotation_matrix_x #combine rotation matrices

# print rotation matrix
print (s3_rotation_matrix.tolist())

dt = sim.getSimulationTimeStep()

x_history = [p3dx_pos_x]
y_history = [p3dx_pos_y]
time_history = [0.0]

# %%
try:
    # 4. Main Loop
    # start_time = time.time()
    # sim_start = sim.getSimulationTime()
    # last_time = sim_start
    sim_start = sim.getSimulationTime()

    # while (time.time() - start_time) < 10:
    while (sim.getSimulationTime() - sim_start) < 30:
        sim.step()

        # now = sim.getSimulationTime()
        # dt = now - last_time
        # last_time = now
        now = sim.getSimulationTime()

        wr_vel = sim.getJointTargetVelocity(p3dx_RW)
        wl_vel = sim.getJointTargetVelocity(p3dx_LW)

        v = (wr_vel + wl_vel) * rw / 2
        w = (wr_vel - wl_vel) * rw / (2 * rb)

        theta_z += w * dt
        p3dx_pos_x += v * math.cos(theta_z) * dt
        p3dx_pos_y += v * math.sin(theta_z) * dt

        t = now - sim_start
        sim.addLog(1, f"t: {t:.2f}s | x: {p3dx_pos_x:.3f} m, y: {p3dx_pos_y:.3f} m, theta: {theta_z:.3f} rad")
        print(f"t={t:.2f}s  x={p3dx_pos_x:.3f}  y={p3dx_pos_y:.3f}  theta={theta_z:.3f}", end="\r")

        x_history.append(p3dx_pos_x)
        y_history.append(p3dx_pos_y)
        time_history.append(t)

        # sensor transformation execercise
        res, detected_distance, point, obj, normal = sim.readProximitySensor(s3)
        if res:
            p3_position_homogeneous = np.append(np.asarray(point), 1)

            # Transform the point to the world coordinate system
            transformed_point = s3_transformation_matrix @ p3_position_homogeneous

        # concat horiizontaly the rotation matrix and the position vector
        s3_transformation_matrix = np.vstack((s3_rotation_matrix, np.array([[0,0,0]])))
        s3_transformation_matrix = np.hstack((s3_transformation_matrix, s3_position_homogeneous))

        

        # time.sleep(0.1)

finally:
    # 5. Stop Simulation
    sim.stopSimulation()
    client.setStepping(False)
    print("\nSimulation Stopped")

# %%
# 6. Plot Trajectory
plt.figure(figsize=(7, 7))
plt.plot(x_history, y_history, 'b-', label="Odometry")
plt.scatter(x_history[0], y_history[0], c='green', marker='o', label="Start")
plt.scatter(x_history[-1], y_history[-1], c='black', marker='x', label="End")
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.title("Trajektori Robot P3DX (Odometry)")
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.show()

plt.figure()
plt.plot

# %%
# 7. Plot x dan y terhadap waktu
# plt.figure(figsize=(8, 5))
# plt.plot(time_history, x_history, 'b-', label="x")
# plt.plot(time_history, y_history, 'r-', label="y")
# plt.xlabel("Waktu (s)")
# plt.ylabel("Posisi (m)")
# plt.title("Log Posisi x dan y terhadap Waktu (Odometry)")
# plt.legend()
# plt.grid(True)
# plt.show()