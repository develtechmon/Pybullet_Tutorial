"""
keyboard_control.py — PPO version
=================================

Keyboard control using observation biasing.

The PPO policy stays in full control of balance.
Keys bias the observation to make the policy
naturally drive in the desired direction.

HOW IT WORKS:
  Normal mode:
      policy reads observations such as:
      [pitch, pitch_rate, yaw, yaw_rate, ...]

      and decides motor commands to stay balanced.

  Key pressed:
      we add a small bias to pitch or yaw observation.

      The policy thinks the robot is slightly tilted/rotated.
      The PPO policy naturally corrects this error.
      That correction causes the robot to move.

  Key released:
      bias is removed immediately.
      Robot returns to stationary balancing on its own.

CONTROLS:
  i     = forward
  k     = backward
  j     = turn left
  l     = turn right
  SPACE = stop all movement
  r     = reset robot to starting position
  q     = quit

TUNING:
  If robot falls when key pressed → reduce PITCH_BIAS / YAW_BIAS
  If movement is too slow         → increase PITCH_BIAS / YAW_BIAS

Run:
  python keyboard_control.py

Requires:
  pip install pynput
"""

import os
import sys
import time
import threading
import numpy as np

# Suppress PyBullet C++ output at OS level
# Works on Windows and Linux
devnull_fd = os.open(os.devnull, os.O_WRONLY)
os.dup2(devnull_fd, 2)
os.close(devnull_fd)

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from balance_env import BalanceBotEnv

try:
    from pynput import keyboard
except ImportError:
    print("pynput not installed.")
    print("Run: pip install pynput")
    exit(1)


# ── Bias parameters ────────────────────────────────────────────
# How much to nudge the observation to induce movement.
#
# Start conservative.
# Increase if movement is too slow.
# Decrease if robot falls when key is pressed.
PITCH_BIAS = 0.03   # radians — forward/backward tilt bias
YAW_BIAS   = 0.06   # radians — left/right turn bias


# ── Shared state between keyboard thread and main loop ─────────
bias = {
    "pitch": 0.0,     # negative = forward, positive = backward
    "yaw": 0.0,       # negative = turn left, positive = turn right
    "reset": False    # True = reset robot to spawn position
}

bias_lock = threading.Lock()
running = True


# ── Keyboard callbacks ─────────────────────────────────────────
def on_press(key):
    global running

    with bias_lock:
        try:
            if key.char == "i":        # forward
                bias["pitch"] = -PITCH_BIAS

            elif key.char == "k":      # backward
                bias["pitch"] = +PITCH_BIAS

            elif key.char == "j":      # turn left
                bias["yaw"] = -YAW_BIAS

            elif key.char == "l":      # turn right
                bias["yaw"] = +YAW_BIAS

            elif key.char == "r":      # reset
                bias["pitch"] = 0.0
                bias["yaw"] = 0.0
                bias["reset"] = True

            elif key.char == "q":      # quit
                running = False

        except AttributeError:
            # Special keys
            if key == keyboard.Key.space:
                bias["pitch"] = 0.0
                bias["yaw"] = 0.0


def on_release(key):
    """
    Releasing a key clears that axis bias immediately.
    The robot then returns to stationary balancing on its own.
    """
    with bias_lock:
        try:
            if key.char in ["i", "k"]:
                bias["pitch"] = 0.0

            elif key.char in ["j", "l"]:
                bias["yaw"] = 0.0

        except AttributeError:
            pass


# ── Load model and normalisation stats ─────────────────────────
norm_path = "./models/vecnormalize.pkl"
model_path = "./models/best/best_model"

if not os.path.exists(norm_path):
    print("ERROR: vecnormalize.pkl not found.")
    print("Solution: Let train_ppo.py run past 10,000 steps first.")
    exit(1)

if not os.path.exists(model_path + ".zip"):
    print("ERROR: best_model.zip not found.")
    print("Solution: Training must produce a 'New best mean reward!' message.")
    exit(1)


print("Loading PPO policy...")

env = DummyVecEnv([
    lambda: BalanceBotEnv(render_mode="human")
])

env = VecNormalize.load(norm_path, env)
env.training = False
env.norm_reward = False

model = PPO.load(model_path, env=env)

print("Loaded.\n")


print("=" * 50)
print("  Keyboard Control — PPO Observation Bias Mode")
print("=" * 50)
print("  i     = forward")
print("  k     = backward")
print("  j     = turn left")
print("  l     = turn right")
print("  SPACE = stop all movement")
print("  r     = reset robot to starting position")
print("  q     = quit")
print("=" * 50)
print(f"  Pitch bias : ±{PITCH_BIAS} rad")
print(f"  Yaw bias   : ±{YAW_BIAS} rad")
print("=" * 50)
print("\nRobot balancing autonomously.")
print("Press keys to move. Release to stop.\n")


# ── Start keyboard listener in background thread ───────────────
listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release
)

listener.start()


# ── Main control loop ──────────────────────────────────────────
obs = env.reset()
steps = 0
episode = 1


try:
    while running:

        # ── Read current bias ──────────────────────────────────
        with bias_lock:
            pitch_bias = bias["pitch"]
            yaw_bias = bias["yaw"]
            do_reset = bias["reset"]

            if do_reset:
                bias["reset"] = False

        # ── Manual reset ───────────────────────────────────────
        if do_reset:
            print(f"\n[RESET] Episode {episode} reset at step {steps}")

            obs = env.reset()
            steps = 0
            episode += 1

            print(f"[RESET] Episode {episode} started\n")
            continue

        # ── Apply observation bias ─────────────────────────────
        obs_biased = obs.copy()

        if pitch_bias != 0.0 or yaw_bias != 0.0:

            # Convert normalized observation back to raw observation
            raw = (
                obs_biased[0] * np.sqrt(env.obs_rms.var + 1e-8)
                + env.obs_rms.mean
            )

            # Bias pitch and yaw.
            #
            # Assumption:
            #   raw[0] = pitch
            #   raw[2] = yaw
            #
            # If your observation order is different,
            # change these indices.
            raw[0] += pitch_bias
            raw[2] += yaw_bias

            # Clip pitch and yaw to safe angular range
            raw[0] = np.clip(raw[0], -np.pi, np.pi)
            raw[2] = np.clip(raw[2], -np.pi, np.pi)

            # Convert raw observation back to normalized observation
            obs_biased[0] = (
                (raw - env.obs_rms.mean)
                / np.sqrt(env.obs_rms.var + 1e-8)
            )

        # ── PPO policy inference ───────────────────────────────
        action, _ = model.predict(obs_biased, deterministic=True)

        # ── Step environment ───────────────────────────────────
        obs, reward, done, info = env.step(action)

        steps += 1

        # ── Status display every 50 steps ──────────────────────
        if steps % 50 == 0:
            pitch = info[0].get("pitch_deg", 0)

            if pitch_bias < 0:
                direction = "FORWARD ▶"
            elif pitch_bias > 0:
                direction = "◀ BACKWARD"
            elif yaw_bias < 0:
                direction = "↺ TURN LEFT"
            elif yaw_bias > 0:
                direction = "TURN RIGHT ↻"
            else:
                direction = "HOLDING"

            print(
                f"Ep {episode:3d} | "
                f"Step {steps:5d} | "
                f"Pitch {pitch:+6.1f}° | "
                f"{direction}"
            )

        # ── Auto reset on fall or episode done ─────────────────
        if done[0]:
            pitch = info[0].get("pitch_deg", 0)
            fell = abs(pitch) > 20

            if fell:
                print(
                    f"\n[FELL] Episode {episode} — "
                    f"pitch={pitch:.1f}° at step {steps}"
                )
                print("Tip: reduce PITCH_BIAS or YAW_BIAS at top of script")
                print("     or press keys more gently\n")
            else:
                print(
                    f"\n[DONE] Episode {episode} completed "
                    f"{steps} steps\n"
                )

            obs = env.reset()
            steps = 0
            episode += 1

        # ── 100Hz control loop ─────────────────────────────────
        time.sleep(1.0 / 100.0)


except KeyboardInterrupt:
    print("\nCtrl+C detected.")


finally:
    listener.stop()
    env.close()
    print("Stopped. GUI closed.")