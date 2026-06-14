import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2
import os
from pynput import keyboard

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

p.loadURDF("plane.urdf")

# Load our robot — spawn it high enough so wheels clear the ground
# Chassis centre at z=0.065 means wheels (radius 0.025) bottom at z=0.0
urdf_path = os.path.join(os.path.dirname(__file__), "myrobot.urdf")
robot = p.loadURDF(urdf_path, basePosition=[0, 0, 0.065])

num_joints = p.getNumJoints(robot) 

print(num_joints)

# Step the simulation 500 times like ticking a clock
for i in range(num_joints):
    joint_info = p.getJointInfo(robot, i)
    #print(joint_info)
    joint_index = joint_info[0]
    joint_name  = joint_info[1].decode('utf-8')    
    joint_type  = joint_info[2]
    
    type_name = {
        0: "REVOLUTE",
        1: "PRISMATIC",
        2: "SPHERICAL",
        3: "PLANAR",
        4: "FIXED"
    }
    
    print(f"Joint {joint_index}: {joint_name} (Type: {type_name.get(joint_type, 'UNKNOWN')})")