# %%
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# %%
# 1. Setup Connection
client = RemoteAPIClient()
sim = client.require('sim')
client.setStepping(True)  # waktu mengikuti waktu simulasi, bukan jam dinding

# %%
# 2. Start Simulation
sim.startSimulation()
print("Simulation Started")

# %%
# 3. Robot handles
p3dx_RW = sim.getObject("/PioneerP3DX/rightMotor")
p3dx_LW = sim.getObject("/PioneerP3DX/leftMotor")
p3dx = sim.getObject("/PioneerP3DX")

SENSOR_IDX = 1  # sesuaikan dengan sensor yang menghadap objek di scene


def get_sensor(idx):
    # path berbeda antar versi model: dengan/tanpa "visible"
    for path in (f"/PioneerP3DX/ultrasonicSensor[{idx}]",
                 f"/PioneerP3DX/visible/ultrasonicSensor[{idx}]"):
        try:
            return sim.getObject(path)
        except Exception:
            pass
    raise RuntimeError(f"ultrasonicSensor[{idx}] tidak ditemukan")


s3 = get_sensor(SENSOR_IDX)

# Posisi awal robot (dari simulator). Ganti ke 0.0 jika ingin frame odometry
p3dx_pos_x, p3dx_pos_y = sim.getObjectPosition(p3dx, sim.handle_world)[:2]

# Parameter robot
rw = 0.195 / 2   # radius roda
rb = 0.381 / 2   # setengah jarak antar roda

# Cek rb dari model
wheel_vec = sim.getObjectPosition(p3dx_RW, p3dx_LW)
print(f"rb model = {math.sqrt(sum(c**2 for c in wheel_vec)) / 2:.4f} m (dipakai: {rb:.4f})")

# %%
# 4. Pose sensor terhadap frame robot
USE_SIM_POSE = True
if USE_SIM_POSE:
    s3_position = np.array(sim.getObjectPosition(s3, p3dx))
    s3_orientation = np.array(sim.getObjectOrientation(s3, p3dx))  # rad
else:
    # nilai manual dari GUI CoppeliaSim (harus cocok dengan sensor yang dipilih!)
    s3_orientation = np.deg2rad([-90, 80, 180])
    s3_position = np.array([0.20919, 0.02727, 0.06809])

s3_rotation_matrix_x = np.array([
    [1, 0, 0],
    [0, np.cos(s3_orientation[0]), -np.sin(s3_orientation[0])],
    [0, np.sin(s3_orientation[0]),  np.cos(s3_orientation[0])],
])
s3_rotation_matrix_y = np.array([
    [ np.cos(s3_orientation[1]), 0, np.sin(s3_orientation[1])],
    [0, 1, 0],
    [-np.sin(s3_orientation[1]), 0, np.cos(s3_orientation[1])],
])
s3_rotation_matrix_z = np.array([
    [np.cos(s3_orientation[2]), -np.sin(s3_orientation[2]), 0],
    [np.sin(s3_orientation[2]),  np.cos(s3_orientation[2]), 0],
    [0, 0, 1],
])
# Konvensi Euler CoppeliaSim: R = Rx @ Ry @ Rz
s3_rotation_matrix = s3_rotation_matrix_x @ s3_rotation_matrix_y @ s3_rotation_matrix_z
print(s3_rotation_matrix.tolist())

# Validasi terhadap matriks asli dari CoppeliaSim
R_sim = np.array(sim.getObjectMatrix(s3, p3dx)).reshape(3, 4)[:, :3]
print("Rotasi cocok dengan CoppeliaSim:", np.allclose(R_sim, s3_rotation_matrix, atol=1e-3))


def make_T(R, p):
    """Matriks transformasi homogen 4x4 dari R (3x3) dan p (3,)"""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


# Transformasi sensor -> frame robot (konstan)
s3_transformation_matrix = make_T(s3_rotation_matrix, s3_position)

# Penyimpanan data untuk plot
position_data = []
t_data = []
d_data = []
s3_to_p3dx_data = []
s3_to_world_data = []

# %%
# 5. Main Loop
try:
    sim_start = sim.getSimulationTime()
    elapsed_before = 0.0
    while (sim.getSimulationTime() - sim_start) < 100:
        sim.step()  # maju satu langkah simulasi
        elapsed = sim.getSimulationTime() - sim_start  # waktu simulasi
        print(f"Running... {elapsed:.1f}s", end="\r")

        # Kecepatan sudut roda
        wr_vel = sim.getJointTargetVelocity(p3dx_RW)
        wl_vel = sim.getJointTargetVelocity(p3dx_LW)

        # Kecepatan linear dan sudut robot
        v = (wr_vel + wl_vel) * rw / 2
        w = (wr_vel - wl_vel) * rw / (2 * rb)

        # Orientasi robot di dunia
        theta = sim.getObjectOrientation(p3dx, sim.handle_world)

        # Kecepatan di frame dunia
        vx = v * math.cos(theta[2])
        vy = v * math.sin(theta[2])

        # dt untuk integrasi
        dt = elapsed - elapsed_before
        elapsed_before = elapsed

        # Posisi dari integrasi
        p3dx_pos_x = p3dx_pos_x + vx * dt
        p3dx_pos_y = p3dx_pos_y + vy * dt
        position_data.append((p3dx_pos_x, p3dx_pos_y))

        # ---------- Sensor transformation exercise ----------
        res, d, point, obj, n = sim.readProximitySensor(s3)
        if res == 1:
            # titik terdeteksi dalam frame sensor (homogen)
            point_h = np.array([point[0], point[1], point[2], 1.0])

            # sensor frame -> robot frame
            s3_position_in_p3dx_frame = s3_transformation_matrix @ point_h

            # robot frame -> world frame
            p3dx_rotation_matrix = np.array([
                [np.cos(theta[2]), -np.sin(theta[2]), 0],
                [np.sin(theta[2]),  np.cos(theta[2]), 0],
                [0, 0, 1],
            ])
            p3dx_transformation_matrix = make_T(
                p3dx_rotation_matrix, [p3dx_pos_x, p3dx_pos_y, 0.0])
            s3_position_in_world_frame = p3dx_transformation_matrix @ s3_position_in_p3dx_frame

            t_data.append(elapsed)
            d_data.append(d)
            s3_to_p3dx_data.append(s3_position_in_p3dx_frame[:3])
            s3_to_world_data.append(s3_position_in_world_frame[:3])

            sim.addLog(1, f"d: {d:.2f} m")

    # Cek odometri vs posisi sebenarnya di simulator
    p_true = sim.getObjectPosition(p3dx, sim.handle_world)
    print(f"\nOdometri: ({p3dx_pos_x:.3f}, {p3dx_pos_y:.3f}) | "
          f"Sebenarnya: ({p_true[0]:.3f}, {p_true[1]:.3f})")

finally:
    # 6. Stop Simulation safely
    sim.stopSimulation()
    client.setStepping(False)
    print("\nSimulation Stopped")

# %%
# 7. Plot
# Lintasan robot
position_data = np.array(position_data)
plt.figure()
plt.plot(position_data[:, 0], position_data[:, 1])
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Robot Path')
plt.grid(True)
plt.axis('equal')
plt.show()

if len(t_data) > 0:
    # Sensing terhadap waktu
    plt.figure()
    plt.plot(t_data, d_data)
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
    plt.title('Sensing Over Time')
    plt.grid(True)
    plt.show()

    # Posisi titik terdeteksi di frame robot
    s3_to_p3dx_data = np.array(s3_to_p3dx_data)
    plt.figure()
    plt.plot(s3_to_p3dx_data[:, 0], s3_to_p3dx_data[:, 1], 'o')
    plt.xlabel('X Position in Robot Frame (m)')
    plt.ylabel('Y Position in Robot Frame (m)')
    plt.title('Sensor Position in Robot Frame')
    plt.axis('equal')
    plt.grid(True)
    plt.show()

    # Posisi titik terdeteksi di frame dunia
    s3_to_world_data = np.array(s3_to_world_data)
    plt.figure()
    plt.plot(s3_to_world_data[:, 0], s3_to_world_data[:, 1], 'o')
    plt.xlabel('X Position in World Frame (m)')
    plt.ylabel('Y Position in World Frame (m)')
    plt.title('Sensor Position in World Frame')
    plt.axis('equal')
    plt.grid(True)
    plt.show()
else:
    print("Sensor tidak mendeteksi apa pun selama simulasi.")