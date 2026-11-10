import pybullet as p
import pybullet_data
import time

# Connect with GUI
client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

# Load R2D2 robot
r2d2 = p.loadURDF("r2d2.urdf", [0,0,1])

# Step the simulation 500 times like ticking a clock
# for i in range(1000):
#     p.stepSimulation()  # Advance the simulation by one time step
#     time.sleep(1. / 240.)  # Sleep to match real-time (240 Hz)
    
# Print all joints
num_joints = p.getNumJoints(r2d2) 
print(f"\nR2D2 has {num_joints} joints:\n")

for i in range(num_joints):
    joint_info = p.getJointInfo(r2d2, i)
    #print(joint_info)
    
    joint_index = joint_info[0]
    joint_name  = joint_info[1].decode('utf-8')
    joint_type  = joint_info[2]
    #print(f"Joint {joint_index}: {joint_name} (Type: {joint_type})")
    
    type_name = {
        0: "REVOLUTE",
        1: "PRISMATIC",
        2: "SPHERICAL",
        3: "PLANAR",
        4: "FIXED"
    }

    print(f"Joint {joint_index}: {joint_name} (Type: {type_name.get(joint_type, 'UNKNOWN')})")
