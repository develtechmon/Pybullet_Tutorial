import pybullet as p
import pybullet_data
import numpy as np

# Connect to the physics server
client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set gravity
p.setGravity(0, 0, -9.81)

# Load ground plane
plane_id = p.loadURDF("plane.urdf")

# Load a sphere with higher mass
sphere_start_pos = [0, 0, 2]
sphere_id = p.loadURDF("sphere_small.urdf", sphere_start_pos)

# Load a box
box_start_pos = [2, 0, 1]
box_start_orn = p.getQuaternionFromEuler([0, 0, 0])
box_id = p.loadURDF("cube_small.urdf", box_start_pos, box_start_orn)

# Set time step
p.setTimeStep(1. / 240.)

# Enable real-time simulation
p.setRealTimeSimulation(0)

print("Controls:")
print("- SPHERE: Arrow keys to apply forces")
print("- BOX: 'W/A/S/D' for rotation forces")
print("- 'R' to reset")
print("Running simulation for 30 seconds...\n")

# Simulation loop
for i in range(7200):  # 30 seconds at 240 Hz
    # Apply force to sphere based on keyboard input (simulated)
    if i % 60 == 0:  # Every 0.25 seconds, apply a force
        # Alternate pushing the sphere in different directions
        direction = (i // 60) % 4
        force_magnitude = 100
        
        if direction == 0:
            force = [force_magnitude, 0, 0]
        elif direction == 1:
            force = [-force_magnitude, 0, 0]
        elif direction == 2:
            force = [0, force_magnitude, 0]
        else:
            force = [0, -force_magnitude, 0]
        
        p.applyExternalForce(sphere_id, -1, force, [0, 0, 0], p.WORLD_FRAME)
    
    # Step the simulation
    p.stepSimulation()
    
    # Print positions every 240 steps (1 second)
    if i % 240 == 0:
        sphere_pos, _ = p.getBasePositionAndOrientation(sphere_id)
        box_pos, _ = p.getBasePositionAndOrientation(box_id)
        print(f"Time: {i/240:.1f}s | Sphere: {sphere_pos} | Box: {box_pos}")

# Keep the GUI window open so you can view the results
print("\nSimulation complete! Window will stay open. Close the GUI window to exit.")
try:
    while True:
        p.stepSimulation()
except KeyboardInterrupt:
    pass
finally:
    p.disconnect()
