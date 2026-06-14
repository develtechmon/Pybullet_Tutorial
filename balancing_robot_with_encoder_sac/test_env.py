"""
STEP 3 — test_env.py
Pass: check_env no errors. Reward near +1.0 at start, drops as robot tilts.

Usage:
  python test_env.py          <- headless, fast, just terminal output
  python test_env.py --gui    <- shows PyBullet GUI so you can watch
"""

from stable_baselines3.common.env_checker import check_env
from balance_env import BalanceBotEnv
import numpy as np
import sys
import time

# Check if --gui flag passed
use_gui = "--gui" in sys.argv
render_mode = "human" if use_gui else None

if use_gui:
    print("GUI mode — you can watch the robot fall")
else:
    print("Headless mode — run with --gui to see PyBullet window")

# ── 1. Formal SB3 check (always headless — GUI interferes with checker) ──
print("\n" + "=" * 50)
print("Running SB3 environment checker...")
print("=" * 50)
env = BalanceBotEnv()
check_env(env)
env.close()
print("check_env PASSED\n")

# ── 2. Manual episode ─────────────────────────────────────────────────
print("=" * 50)
print("Running manual episode with random actions...")
print("=" * 50)

env    = BalanceBotEnv(render_mode=render_mode)
obs, _ = env.reset()

print(f"\nInitial obs:")
print(f"  pitch         = {np.degrees(obs[0]):6.2f} deg")
print(f"  pitch_rate    = {obs[1]:6.3f} rad/s")
print(f"  left_vel      = {obs[2]:6.3f} rad/s")
print(f"  right_vel     = {obs[3]:6.3f} rad/s")
print(f"  avg_wheel_pos = {obs[4]:6.3f} rad")

print(f"\n{'Step':>5} | {'Pitch(deg)':>10} | {'Reward':>8} | {'Total':>9}")
print("-" * 42)

total_reward = 0
for step in range(500):
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
    total_reward += reward

    # Slow down if GUI so you can watch
    if use_gui:
        time.sleep(1. / 60.)

    if step % 25 == 0:
        print(f"{step:5d} | {info['pitch_deg']:10.2f} | "
              f"{reward:8.4f} | {total_reward:9.2f}")

    if terminated or truncated:
        status = "FELL" if terminated else "TRUNCATED"
        print(f"{step:5d} | {info['pitch_deg']:10.2f} | "
              f"{reward:8.4f} | {total_reward:9.2f} | {status}")
        print(f"\nEpisode ended at step {step}")
        break

env.close()

print("\n" + "=" * 50)
print("EXPECTED RESULTS:")
print("  Reward near +1.0 at start")
print("  Reward drops as pitch grows")
print("  Episode ends before step 500 with FELL")
print("  Final reward includes -100 fall penalty")
print("=" * 50)