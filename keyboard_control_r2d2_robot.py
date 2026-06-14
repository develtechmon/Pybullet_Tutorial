import pybullet as p
import pybullet_data
import time

# ── Connect ───────────────────────────────────────────────────────────
client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

r2d2 = p.loadURDF("r2d2.urdf", [0, 0, 0.5])

# ── Wheel joints (confirmed from your output) ─────────────────────────
RIGHT_WHEELS = [2, 3]   # right_front_wheel_joint, right_back_wheel_joint
LEFT_WHEELS  = [6, 7]   # left_front_wheel_joint,  left_back_wheel_joint

# ── Parameters ────────────────────────────────────────────────────────
MAX_SPEED = 10.0   # rad/s
MAX_FORCE = 10.0   # N·m

# ── Camera ────────────────────────────────────────────────────────────
p.resetDebugVisualizerCamera(
    cameraDistance=2.0,
    cameraYaw=50,
    cameraPitch=-30,
    cameraTargetPosition=[0, 0, 0]
)

# ── Key constants ─────────────────────────────────────────────────────
KEY_W     = ord('w')
KEY_S     = ord('s')
KEY_A     = ord('a')
KEY_D     = ord('d')
KEY_SPACE = 32
KEY_Q     = ord('q')

print("\n=== R2D2 Keyboard Controller ===")
print("  W = Forward")
print("  S = Backward")
print("  A = Turn Left")
print("  D = Turn Right")
print("  SPACE = Brake")
print("  Q = Quit")
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

# ── Main loop ─────────────────────────────────────────────────────────
while True:
    keys = p.getKeyboardEvents()

    if KEY_Q in keys and keys[KEY_Q] & p.KEY_WAS_TRIGGERED:
        print("Quitting...")
        break

    left  = 0.0
    right = 0.0

    if KEY_W in keys and keys[KEY_W] & p.KEY_IS_DOWN:
        left, right = MAX_SPEED, MAX_SPEED

    elif KEY_S in keys and keys[KEY_S] & p.KEY_IS_DOWN:
        left, right = -MAX_SPEED, -MAX_SPEED

    elif KEY_A in keys and keys[KEY_A] & p.KEY_IS_DOWN:
        left, right = -MAX_SPEED * 0.5, MAX_SPEED * 0.5

    elif KEY_D in keys and keys[KEY_D] & p.KEY_IS_DOWN:
        left, right = MAX_SPEED * 0.5, -MAX_SPEED * 0.5

    elif KEY_SPACE in keys and keys[KEY_SPACE] & p.KEY_IS_DOWN:
        left, right = 0.0, 0.0

    set_wheels(left, right)
    p.stepSimulation()
    time.sleep(1. / 240.)

p.disconnect()