"""
STEP 5 — evaluate.py (SAC version)
Run: python evaluate.py
"""
"""
evaluate.py — DEFINITIVE FINAL VERSION
Requires vecnormalize.pkl — run after training finishes
or after at least 10k steps with SaveVecNormalizeCallback
"""
"""
evaluate.py — FINAL VERSION
Run after training finishes or after 10k+ steps
"""

import os
import sys
stderr_backup = sys.stderr
sys.stderr    = open(os.devnull, 'w')
import pybullet as p
sys.stderr    = stderr_backup

from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from balance_env import BalanceBotEnv
import time

norm_path = "./models/vecnormalize.pkl"

if not os.path.exists(norm_path):
    print("ERROR: vecnormalize.pkl not found.")
    print("Let training run past 10,000 steps first.")
    exit(1)

print("Loading trained SAC policy...")
env = DummyVecEnv([lambda: BalanceBotEnv(render_mode="human")])
env = VecNormalize.load(norm_path, env)
env.training    = False
env.norm_reward = False

model    = SAC.load("./models/best/best_model", env=env)
#model    = SAC.load("./models/checkpoints/balancebot_sac_370000_steps", env=env)

episodes = 10
print("Loaded.\n")

print(f"{'Episode':>8} | {'Result':>16} | {'Steps':>6} | "
      f"{'Reward':>8} | {'Pitch':>10}")
print("-" * 58)

survived = 0
obs      = env.reset()

for ep in range(episodes):
    total_reward = 0
    steps        = 0

    while True:
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        total_reward += float(reward[0])
        steps        += 1
        time.sleep(1.0 / 60.)

        if done[0]:
            result = "SURVIVED" if steps >= 2000 else "FELL"
            if steps >= 2000:
                survived += 1
            pitch = info[0].get('pitch_deg', 0)
            print(f"{ep+1:>8} | {result:>16} | {steps:>6} | "
                  f"{total_reward:>8.1f} | {pitch:>8.1f} deg")
            obs = env.reset()
            break

env.close()

print("\n" + "=" * 58)
print(f"SURVIVAL RATE: {survived}/{episodes}")
print("=" * 58)
print("8+/10 → proceed to export_to_onnx.py")
print("5-7/10 → train more")
print("< 5/10 → needs more training")
print("=" * 58)


