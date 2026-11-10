import pybullet as p
import pybullet_data
import time

# Connect to the physics server (GUI mode for visualization)
client = p.connect(p.GUI)

# Set additional search path to find URDF files
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set gravity
p.setGravity(0, 0, -9.81)

# Load a plane
plane_id = p.loadURDF("plane.urdf")

# Load a cube (start position and orientation)
cube_start_pos = [0, 0, 1]  # x, y, z
cube_start_orientation = p.getQuaternionFromEuler([0, 0, 0])  # roll, pitch, yaw
cube_id = p.loadURDF("r2d2.urdf", cube_start_pos, cube_start_orientation)

# Set time step
p.setTimeStep(1. / 240.)

# Simulation loop
for i in range(2400):  # Run for 10 seconds (2400 steps at 240 Hz)
    p.stepSimulation()
    
    # Get the position and orientation of the cube
    pos, orn = p.getBasePositionAndOrientation(cube_id)
    print(f"Step {i}: Position = {pos}, Orientation = {orn}")
    
    time.sleep(1. / 240.)  # Sleep to match simulation speed

# Disconnect from the physics server
p.disconnect()
print("Simulation complete!")



