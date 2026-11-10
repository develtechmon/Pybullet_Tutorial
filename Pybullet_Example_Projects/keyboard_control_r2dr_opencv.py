import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2
from pynput import keyboard

# ── Key state dictionary ──────────────────────────────────────────────
# pynput runs in a background thread and updates this dict
# Main loop reads from it every frame — like a shared register
keys_held = {
    'w': False,
    's': False,
    'a': False,
    'd': False,
    'q': False,
}

def on_press(key):
    try:
        if key.char in keys_held:
            keys_held[key.char] = True
    except AttributeError:
        pass  # special keys like Shift, Ctrl — ignore

def on_release(key):
    try:
        if key.char in keys_held:
            keys_held[key.char] = False
    except AttributeError:
        pass

# Start keyboard listener in background thread
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

# ── PyBullet setup ────────────────────────────────────────────────────
client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")
r2d2 = p.loadURDF("r2d2.urdf", [0, 0, 0.5])

# ── Wheel joints ──────────────────────────────────────────────────────
RIGHT_WHEELS = [2, 3]
LEFT_WHEELS  = [6, 7]
MAX_SPEED    = 30.0
MAX_FORCE    = 10.0

# ── Camera settings ───────────────────────────────────────────────────
CAM_WIDTH  = 320
CAM_HEIGHT = 240
CAM_FOV    = 60

CAM_OFFSET_UP      = 0.3
CAM_OFFSET_FORWARD = 0.1

projection_matrix = p.computeProjectionMatrixFOV(
    fov=CAM_FOV,
    aspect=CAM_WIDTH / CAM_HEIGHT,
    nearVal=0.05,
    farVal=10.0
)

p.resetDebugVisualizerCamera(
    cameraDistance=3.0,
    cameraYaw=50,
    cameraPitch=-30,
    cameraTargetPosition=[0, 0, 0]
)

print("\n=== R2D2 First-Person Camera ===")
print("  W/S/A/D = Drive  |  Q = Quit")
print("  You can type from ANY window")
print("================================\n")


def set_wheels(left_speed, right_speed):
    for j in LEFT_WHEELS:
        p.setJointMotorControl2(
            bodyUniqueId=r2d2,
            jointIndex=j,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=left_speed,
            force=MAX_FORCE
        )
    for j in RIGHT_WHEELS:
        p.setJointMotorControl2(
            bodyUniqueId=r2d2,
            jointIndex=j,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=right_speed,
            force=MAX_FORCE
        )


def get_camera_image():
    base_pos, base_orn = p.getBasePositionAndOrientation(r2d2)
    rot = p.getMatrixFromQuaternion(base_orn)

    forward = [rot[0], rot[3], rot[6]]
    up      = [rot[2], rot[5], rot[8]]

    cam_pos = [
        base_pos[0] + forward[0] * CAM_OFFSET_FORWARD + up[0] * CAM_OFFSET_UP,
        base_pos[1] + forward[1] * CAM_OFFSET_FORWARD + up[1] * CAM_OFFSET_UP,
        base_pos[2] + forward[2] * CAM_OFFSET_FORWARD + up[2] * CAM_OFFSET_UP,
    ]
    cam_target = [
        cam_pos[0] + forward[0],
        cam_pos[1] + forward[1],
        cam_pos[2] + forward[2],
    ]

    view_matrix = p.computeViewMatrix(cam_pos, cam_target, up)

    _, _, rgba, _, _ = p.getCameraImage(
        width=CAM_WIDTH,
        height=CAM_HEIGHT,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        renderer=p.ER_TINY_RENDERER
    )

    rgb = np.array(rgba, dtype=np.uint8).reshape(CAM_HEIGHT, CAM_WIDTH, 4)
    return cv2.cvtColor(rgb[:, :, :3], cv2.COLOR_RGB2BGR)


# ── Main loop ─────────────────────────────────────────────────────────
frame_counter = 0

while True:
    # --- Quit check ---
    if keys_held['q']:
        print("Quitting...")
        break

    # --- Drive logic ---
    left  = 0.0
    right = 0.0

    if keys_held['w']:
        left, right = MAX_SPEED, MAX_SPEED

    elif keys_held['s']:
        left, right = -MAX_SPEED, -MAX_SPEED

    elif keys_held['a']:
        left, right = -MAX_SPEED * 0.5, MAX_SPEED * 0.5

    elif keys_held['d']:
        left, right = MAX_SPEED * 0.5, -MAX_SPEED * 0.5

    set_wheels(left, right)

    # --- Step physics at 240Hz ---
    p.stepSimulation()

    # --- Render camera every 4 physics steps = ~60fps camera ---
    # No point rendering camera at 240Hz, it's expensive
    frame_counter += 1
    if frame_counter % 4 == 0:
        frame = get_camera_image()
        cv2.imshow("R2D2 First-Person View", frame)
        cv2.waitKey(1)  # must call this or OpenCV window freezes

    time.sleep(1. / 240.)

# ── Cleanup ───────────────────────────────────────────────────────────
listener.stop()
cv2.destroyAllWindows()
p.disconnect()
print("Done.")