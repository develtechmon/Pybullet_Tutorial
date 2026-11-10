import pybullet as p
import pybullet_data
import time

# Connect to physic server
# p.GUI = visual windows
# p.DIRECT = headless (for training)
physicsClient = p.connect(p.GUI)

# Tell pybullet where to find built-in assets (plane.urdf, etc)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set gravity (Earth = -9.81 m/s^2 in Z)
p.setGravity(0, 0, -9.81)

# Load a flat ground plane
planeId = p.loadURDF("plane.urdf")

# Load a robot (Kuka arm - comes built-in)
startPos = [0, 0, 1]  # x, y, z
startOrientation = p.getQuaternionFromEuler([0, 0, 0])  # roll, pitch, yaw
robotId = p.loadURDF("r2d2.urdf", startPos, startOrientation)

# Step the simulation 500 times like ticking a clock
for i in range(1000):
    p.stepSimulation()  # Advance the simulation by one time step
    time.sleep(1. / 240.)  # Sleep to match real-time (240 Hz)
    
# Get robot position and orientation
pos, orn = p.getBasePositionAndOrientation(robotId)
print(f"Final Position: {pos}, Final Orientation: {orn}")

p.disconnect()  # Clean up and disconnect from the physics server
print("Simulation complete!")
