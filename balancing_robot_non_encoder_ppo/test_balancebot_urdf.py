"""
STEP 2 — Run this first.
Pass: Robot loads upright in GUI, slowly falls over.
Fail: Robot explodes, sinks through ground, or doesn't load.
"""

import pybullet as p
import pybullet_data
import time
import os

client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

p.resetDebugVisualizerCamera(
    cameraDistance=0.8,
    cameraYaw=45,
    cameraPitch=-15,
    cameraTargetPosition=[0, 0, 0.1]
)

#init_orn = p.getQuaternionFromEuler([0, 0.175, 0])
#robot = p.loadURDF(urdf_path, basePosition=[0, 0, 0.0435],baseOrientation=init_orn)

urdf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "balancebot.urdf")
robot = p.loadURDF(urdf_path, basePosition=[0, 0, 0.0435])

print("\nJoint map:")
for i in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, i)
    print(f"  Joint {info[0]}: {info[1].decode()} (type {info[2]})")

print("\nExpected:")
print("  Joint 0: imu_joint         (type 4 = FIXED)")
print("  Joint 1: left_wheel_joint  (type 0 = CONTINUOUS)")
print("  Joint 2: right_wheel_joint (type 0 = CONTINUOUS)")
print("\nSimulating free fall — robot should tip over slowly...\n")

for step in range(600):
    p.stepSimulation()
    if step % 60 == 0:
        _, orn = p.getBasePositionAndOrientation(robot)
        euler  = p.getEulerFromQuaternion(orn)
        pos, _ = p.getBasePositionAndOrientation(robot)
        print(f"  Step {step:4d} | height={pos[2]:.4f}m | "
              f"pitch={euler[1]*57.2958:6.1f}°")
    time.sleep(1. / 240.)

p.disconnect()
print("\nPitch increased from 0° and robot fell = URDF correct.")