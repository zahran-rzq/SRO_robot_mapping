# %%
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

p3dx_RW = sim.getObject("/PioneerP3DX/rightMotor")
p3dx_LW = sim.getObject("/PioneerP3DX/leftMotor")
p3dx = sim.getObject("/PioneerP3DX")

rw = 0.195/2
rb = 0.381/2
d = 0.05

dt = 0.001
x_pose_int = 0
y_pose_int = 0
x_pose = []
y_pose = []
    

# %%
try:
    # 4. Main Loop (Run for 10 seconds)
    start_time = time.time()
    elapsed_prev = 0
    last_log_time = 0
    while (time.time() - start_time) < 20:
        
        # --- STUDENT CODE GOES HERE ---
        # Example: Print elapsed time
        elapsed = time.time() - start_time
      
        print(f"Running... {elapsed:.1f}s", end="\r")

        wr_vel = sim.getJointTargetVelocity(p3dx_RW)
        wl_vel = sim.getJointTargetVelocity(p3dx_LW)

        vx = (wr_vel + wl_vel) * rw / 2
        wx = (wr_vel - wl_vel) * rw / (2 * rb)

        theta = sim.getObjectOrientation(p3dx, sim.handle_world)
        x_dot = vx * math.cos(theta[2])
        y_dot = vx * math.sin(theta[2])

        dt = elapsed - elapsed_prev
        elapsed_prev = elapsed
                
        x_pose_int = x_pose_int + x_dot * dt
        y_pose_int = y_pose_int + y_dot * dt

        if elapsed - last_log_time >= 0.5:
            sim.addLog(1, f"RW: {wr_vel:.1f}, LW: {wl_vel:.1f}")
            sim.addLog(1, f"vx: {vx:.2f} m/s, wx: {wx:.2f} rad/s")
            sim.addLog(1, f"x_dot: {x_dot:.2f} m/s, y_dot: {y_dot:.2f} m/s")
            sim.addLog(1, f"Position: X = {x_pose_int:.2f} m, Y = {y_pose_int:.2f} m")
        
            last_log_time = elapsed
         
        x_pose.append(x_pose_int)
        y_pose.append(y_pose_int)    


finally:
    # 5. Stop Simulation safely
    sim.stopSimulation()
    print("\nSimulation Stopped")

plt.figure()
plt.plot(x_pose, y_pose)
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Robot Trajectory')
plt.grid(True)
plt.tight_layout()
plt.show()