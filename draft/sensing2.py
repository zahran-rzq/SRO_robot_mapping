    import time
    import math
    import numpy as np
    import matplotlib.pyplot as plt
    from datetime import datetime
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient

    # %%
    # 1. Setup Connection
    client = RemoteAPIClient()
    sim = client.require('sim')

    # %%
    # 2. Start Simulation
    sim.startSimulation()
    print("Simulation Started")

    # %%
    # 3. Simple Test: Post a message to CoppeliaSim status bar
    sim.addLog(1, "Hello from Python!")
    p3dx_RW = sim.getObject("/PioneerP3DX/rightMotor")
    p3dx_LW = sim.getObject("/PioneerP3DX/leftMotor")

    p3dx = sim.getObject("/PioneerP3DX")
    s3 = sim.getObject("/PioneerP3DX/visible/ultrasonicSensor[1]")


    #p3dx pose
    p3dx_pos_x = 1.5
    p3dx_pos_y = 2.0

    position_data = []
    t_data = []
    distance_data = []
    s3_to_p3dx_data = []
    s3_to_world_data = []


    #time step for integration


    #parameter
    rw = 0.195/2 #radius wheel
    rb = 0.381/2 #radius robot
    s3_orientation = np.array([-90, 80, 180])*np.pi/180 #orientation sensor 3 in radian
    s3_position = np.array([0.19061, 0.08313, 0.0681])  # sensor position in robot frame (m)
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
    s3_rotation_matrix = s3_rotation_matrix_z @ s3_rotation_matrix_y @ s3_rotation_matrix_x

    #print(s3_rotation_matrix)
    print(s3_rotation_matrix.tolist())
    #d = 0.05

    # %%
    try:
        # 4. Main Loop (Run for 10 seconds)
        start_time = time.time()
        elapsed_before = 0
        while (time.time() - start_time) < 10:
            
            # --- STUDENT CODE GOES HERE ---
            # Example: Print elapsed time
            elapsed = time.time() - start_time
            print(f"Running... {elapsed:.1f}s", end="\r")

            # Angular Velocity
            wr_vel = sim.getJointTargetVelocity(p3dx_RW)
            wl_vel = sim.getJointTargetVelocity(p3dx_LW)

            #Linear dan angular velocity robot
            v = (wr_vel + wl_vel) * rw / 2
            w = (wr_vel - wl_vel) * rw / (2 * rb)

            theta = sim.getObjectOrientation(p3dx, sim.handle_world)

            dt = elapsed - elapsed_before
            elapsed_before = elapsed

            vx = v * math.cos(theta[2]) #linear velocity in x direction
            vy = v * math.sin(theta[2]) #linear velocity in y direction

            p3dx_pos_x = p3dx_pos_x + vx * dt
            p3dx_pos_y = p3dx_pos_y + vy * dt
            p3dx_position_homogeneous = np.array([p3dx_pos_x, p3dx_pos_y, 0, 1])

            # sensor transformation exercise
            res, d, point, obj, n = sim.readProximitySensor(s3)
            sensor_point = np.array([*point, 1]) if res else np.array([0, 0, 0, 1])

            s3_transformation_matrix = np.eye(4)
            s3_transformation_matrix[:3, :3] = s3_rotation_matrix
            s3_transformation_matrix[:3, 3] = s3_position
            s3_position_in_p3dx_frame = s3_transformation_matrix @ sensor_point

            p3dx_rotation_matrix = np.array([
                [np.cos(theta[2]), -np.sin(theta[2]), 0],
                [np.sin(theta[2]), np.cos(theta[2]), 0],
                [0, 0, 1],
            ])
            p3dx_transformation_matrix = np.eye(4)
            p3dx_transformation_matrix[:3, :3] = p3dx_rotation_matrix
            p3dx_transformation_matrix[:3, 3] = p3dx_position_homogeneous[:3]
            s3_position_in_world_frame = (
                p3dx_transformation_matrix @ s3_position_in_p3dx_frame
            )

            position_data.append(s3_position_in_p3dx_frame.copy())
            t_data.append(elapsed)
            distance_data.append(d if res else np.nan)
            s3_to_world_data.append(s3_position_in_world_frame[:3].copy())
            s3_to_p3dx_data.append(s3_position_in_p3dx_frame[:3].copy())

            #sim.addLog(1, f", pos x: {p3dx_pos_x:.1f} m, pos y: {p3dx_pos_y:.1f} m, vx: {vx:.1f} m/s, vy: {vy:.1f} m/s, laju depan: {v:.1f} m/s, laju putar: {w:.1f} rad/s, RW: {wr_vel:.1f} rad/s, LW: {wl_vel:.1f} rad/s")
            sim.addLog(1, f"d: {d:.1f} m" if res else "d: no detection")
            time.sleep(0.1) 

    finally:
        # 5. Stop Simulation safely
        sim.stopSimulation()
        print("\nSimulation Stopped")

    # %%
    # 6. Plot Distance vs Time from Ultrasonic Sensor
    plt.figure(figsize=(8, 5))
    plt.plot(t_data, distance_data, marker='o', linestyle='-', color='b')
    plt.xlabel('Waktu t (s)')
    plt.ylabel('Jarak (m)')
    plt.title('Jarak Sensor Ultrasonik vs Waktu')
    plt.grid(True)
    plt.show()

    s3_to_p3dx_data = np.array(s3_to_p3dx_data)
    plt.figure()
    plt.plot(s3_to_p3dx_data[:, 0], s3_to_p3dx_data[:, 1], 'o-', color='r')
    plt.xlabel('Posisi X Sensor 3 (m)')
    plt.ylabel('Posisi Y Sensor 3 (m)')
    plt.title('Posisi Sensor 3 terhadap Robot Pioneer P3DX')
    plt.grid(True)
    plt.show()

    s3_to_world_data = np.array(s3_to_world_data)
    plt.figure()
    plt.plot(s3_to_world_data[:, 0], s3_to_world_data[:, 1], 'o-', color='g')
    plt.xlabel('Posisi X Sensor 3 (m)')
    plt.ylabel('Posisi Y Sensor 3 (m)')
    plt.title('Posisi Sensor 3 terhadap Frame Dunia')
    plt.grid(True)
    plt.show()