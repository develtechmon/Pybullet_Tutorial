import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2
import os
from pynput import keyboard

# ── Key state ─────────────────────────────────────────────────────────
keys_held = {'w': False, 's': False, 'a': False, 'd': False, 'q': False}

def on_press(key):
    try:
        if key.char in keys_held:
            keys_held[key.char] = True
    except AttributeError:
        pass

def on_release(key):
    try:
        if key.char in keys_held:
            keys_held[key.char] = False
    except AttributeError:
        pass

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

# ── PyBullet setup ────────────────────────────────────────────────────
client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

urdf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "myrobot.urdf")
robot = p.loadURDF(urdf_path, basePosition=[0, 0, 0.065])

# ── Wheel joints ──────────────────────────────────────────────────────
FRONT_LEFT  = 0
FRONT_RIGHT = 1
REAR_LEFT   = 2
REAR_RIGHT  = 3
LEFT_WHEELS  = [FRONT_LEFT, REAR_LEFT]
RIGHT_WHEELS = [FRONT_RIGHT, REAR_RIGHT]

MAX_SPEED = 20.0
MAX_FORCE = 20.0

# ── Camera settings ───────────────────────────────────────────────────
CAM_W, CAM_H = 320, 240
CAM_FOV      = 60

# Camera offset from chassis centre in LOCAL robot frame
# matches the camera_joint origin in URDF: xyz="0.05 0.0 0.06"
CAM_LOCAL_POS = np.array([0.05, 0.0, 0.06])

proj_matrix = p.computeProjectionMatrixFOV(
    fov=CAM_FOV,
    aspect=CAM_W / CAM_H,
    nearVal=0.02,
    farVal=10.0
)

p.resetDebugVisualizerCamera(
    cameraDistance=0.8,
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=[0, 0, 0]
)

print("\n=== Custom Robot Controller ===")
print("  W = Forward  |  S = Backward")
print("  A = Turn Left|  D = Turn Right")
print("  Q = Quit")
print("  Keys work from ANY window")
print("================================\n")


def set_wheels(left_spd, right_spd):
    for j in LEFT_WHEELS:
        p.setJointMotorControl2(
            bodyUniqueId=robot,
            jointIndex=j,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=left_spd,
            force=MAX_FORCE
        )
    for j in RIGHT_WHEELS:
        p.setJointMotorControl2(
            bodyUniqueId=robot,
            jointIndex=j,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=right_spd,
            force=MAX_FORCE
        )


def get_camera_image():
    """
    Compute camera world pose directly from base_link.
    This is identical to how the working R2D2 camera worked —
    get base position + orientation, rotate the local offset
    into world space, done. No getLinkState ambiguity.
    """
    # 1. Get base link world pose
    base_pos, base_orn = p.getBasePositionAndOrientation(robot)

    # 2. Build rotation matrix from base quaternion
    rot = np.array(p.getMatrixFromQuaternion(base_orn)).reshape(3, 3)

    # 3. Rotate local camera offset into world space and add to base pos
    # rot.dot(local_vec) = where that local vector points in world space
    cam_world_pos = np.array(base_pos) + rot.dot(CAM_LOCAL_POS)

    # 4. Forward direction = where robot's local X points in world
    # Local X = [1,0,0], rotated into world space
    forward_world = rot.dot(np.array([1.0, 0.0, 0.0]))

    # 5. Up direction = where robot's local Z points in world
    up_world = rot.dot(np.array([0.0, 0.0, 1.0]))

    # 6. Camera looks forward from its position
    cam_target = cam_world_pos + forward_world

    view_matrix = p.computeViewMatrix(
        cameraEyePosition=cam_world_pos.tolist(),
        cameraTargetPosition=cam_target.tolist(),
        cameraUpVector=up_world.tolist()
    )

    _, _, rgba, _, _ = p.getCameraImage(
        width=CAM_W,
        height=CAM_H,
        viewMatrix=view_matrix,
        projectionMatrix=proj_matrix,
        renderer=p.ER_TINY_RENDERER
    )

    img = np.array(rgba, dtype=np.uint8).reshape(CAM_H, CAM_W, 4)
    bgr = cv2.cvtColor(img[:, :, :3], cv2.COLOR_RGB2BGR)

    # Crosshair
    cx, cy = CAM_W // 2, CAM_H // 2
    cv2.line(bgr, (cx - 10, cy), (cx + 10, cy), (0, 255, 0), 1)
    cv2.line(bgr, (cx, cy - 10), (cx, cy + 10), (0, 255, 0), 1)

    return bgr


# ── Main loop ─────────────────────────────────────────────────────────
frame_count = 0

while True:
    if keys_held['q']:
        print("Quitting...")
        break

    left = right = 0.0

    if keys_held['w']:
        left, right = MAX_SPEED, MAX_SPEED
    elif keys_held['s']:
        left, right = -MAX_SPEED, -MAX_SPEED
    elif keys_held['a']:
        left, right = -MAX_SPEED ,  MAX_SPEED #* 0.5
    elif keys_held['d']:
        left, right =  MAX_SPEED , -MAX_SPEED #* 0.5

    set_wheels(left, right)
    p.stepSimulation()

    frame_count += 1
    if frame_count % 4 == 0:
        frame = get_camera_image()
        cv2.imshow("Robot Camera", frame)
        cv2.waitKey(1)

    time.sleep(1. / 240.)

listener.stop()
cv2.destroyAllWindows()
p.disconnect()
print("Done.")