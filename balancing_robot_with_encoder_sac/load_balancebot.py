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
urdf_path = os.path.join(os.path.dirname(__file__), "balancebot.urdf")
robot = p.loadURDF(urdf_path, basePosition=[0, 0, 0.065])

# Step the simulation 500 times like ticking a clock
for i in range(1000):
    p.stepSimulation()  # Advance the simulation by one time step
    time.sleep(1. / 240.)  # Sleep to match real-time (240 Hz)
    
# Get robot pos