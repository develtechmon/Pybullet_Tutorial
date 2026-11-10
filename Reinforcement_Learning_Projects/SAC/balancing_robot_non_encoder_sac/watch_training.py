"""
watch_training.py — SAC version
Run: python watch_training.py

watch_training.py — DEFINITIVE FINAL VERSION
Loads vecnormalize.pkl with every checkpoint update.

watch_training.py — FINAL VERSION
Loads vecnormalize.pkl with every checkpoint update.
"""

# import os
# import sys
# stderr_backup = sys.stderr
# sys.stderr    = open(os.devnull, 'w')
# import pybullet as p
# sys.stderr    = stderr_backup

# from stable_baselines3 import SAC
# from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
# from balance_env import BalanceBotEnv
# import glob
# import time


# def get_latest_checkpoint(checkpoint_dir="./models/checkpoints/"):
#     files = glob.glob(os.path.join(checkpoint_dir, "*.zip"))
#     if not files:
#         return None
#     files.sort(key=os.path.getmtime)
#     return files[-1]


# def make_env():
#     norm_path = "./models/vecnormalize.pkl"
#     raw_env   = DummyVecEnv([lambda: BalanceBotEnv(render_mode="human")])
#     if os.path.exists(norm_path):
#         env = VecNormalize.load(norm_path, raw_env)
#         env.training    = False
#         env.norm_reward = False
#         return env, True
#     return raw_env, False


# def run_episode(model, env):
#     obs          = env.reset()
#     total_reward = 0
#     steps        = 0

#     while True:
#         action, _ = model.predict(obs, deterministic=True)
#         obs, reward, done, info = env.step(action)
#         total_reward += float(reward[0])
#         steps        += 1
#         time.sleep(1.0 / 60.)

#         if done[0]:
#             pitch = info[0].get('pitch_deg', 0)
#             fell  = steps < 2000
#             return steps, total_reward, pitch, fell


# print("=" * 55)
# print("  Balancebot SAC — Live Training Viewer")
# print("=" * 55)
# print("Waiting for checkpoints in ./models/checkpoints/")
# print("vecnormalize.pkl available after 10,000 steps")
# print("Press Ctrl+C to stop\n")

# last_ckpt     = None
# episode_count = 0
# model         = None
# gui_env       = None

# try:
#     while True:
#         latest = get_latest_checkpoint()

#         if latest is None:
#             print("No checkpoint yet — waiting...")
#             time.sleep(5)
#             continue

#         if latest != last_ckpt:
#             if gui_env is not None:
#                 gui_env.close()

#             gui_env, using_norm = make_env()
#             print(f"\nCheckpoint : {os.path.basename(latest)}")
#             print(f"Normalised : {'yes' if using_norm else 'no (pkl not ready)'}")

#             model     = SAC.load(latest, env=gui_env)
#             last_ckpt = latest

#         if model is None:
#             time.sleep(1)
#             continue

#         episode_count += 1
#         steps, reward, pitch, fell = run_episode(model, gui_env)
#         result = "FELL" if fell else "SURVIVED"

#         print(f"Ep {episode_count:4d} | {result:>8} | "
#               f"steps={steps:5d} | reward={reward:7.1f} | "
#               f"pitch={pitch:6.1f}°")

#         time.sleep(0.3)

# except KeyboardInterrupt:
#     print("\nStopped.")
# finally:
#     if gui_env is not None:
#         gui_env.close()
#     print("Viewer closed.")

"""
watch_training.py — FINAL VERSION (no C++ noise)
Keeps PyBullet GUI open permanently.
Only reloads VecNormalize stats and model per checkpoint.
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
import glob
import time


def get_latest_checkpoint(checkpoint_dir="./models/checkpoints/"):
    files = glob.glob(os.path.join(checkpoint_dir, "*.zip"))
    if not files:
        return None
    files.sort(key=os.path.getmtime)
    return files[-1]


def update_vecnormalize_stats(env, norm_path):
    """
    Update VecNormalize running stats from saved .pkl
    without closing/reopening the environment.
    Keeps PyBullet GUI alive — no C++ thread noise.
    """
    if not os.path.exists(norm_path):
        return False

    # Load stats from .pkl into a temporary env
    # then copy the running mean/var into the live env
    import pickle
    with open(norm_path, "rb") as f:
        saved = pickle.load(f)

    # Copy normalisation stats directly
    if hasattr(env, 'obs_rms') and hasattr(saved, 'obs_rms'):
        env.obs_rms.mean = saved.obs_rms.mean
        env.obs_rms.var  = saved.obs_rms.var
        env.obs_rms.count = saved.obs_rms.count
    if hasattr(env, 'ret_rms') and hasattr(saved, 'ret_rms'):
        env.ret_rms.mean = saved.ret_rms.mean
        env.ret_rms.var  = saved.ret_rms.var
        env.ret_rms.count = saved.ret_rms.count

    return True


def run_episode(model, env):
    obs          = env.reset()
    total_reward = 0
    steps        = 0

    while True:
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        total_reward += float(reward[0])
        steps        += 1
        time.sleep(1.0 / 60.)

        if done[0]:
            pitch = info[0].get('pitch_deg', 0)
            fell  = steps < 2000
            return steps, total_reward, pitch, fell


print("=" * 55)
print("  Balancebot SAC — Live Training Viewer")
print("=" * 55)
print("Waiting for checkpoints in ./models/checkpoints/")
print("Press Ctrl+C to stop\n")

# Create GUI environment ONCE — never close it during training
norm_path = "./models/vecnormalize.pkl"
raw_env   = DummyVecEnv([lambda: BalanceBotEnv(render_mode="human")])

# Wait for first .pkl before wrapping
print("Waiting for vecnormalize.pkl (available after 10k steps)...")
while not os.path.exists(norm_path):
    time.sleep(3)
print("vecnormalize.pkl found — wrapping environment\n")

gui_env = VecNormalize.load(norm_path, raw_env)
gui_env.training    = False
gui_env.norm_reward = False

last_ckpt     = None
episode_count = 0
model         = None

try:
    while True:
        latest = get_latest_checkpoint()

        if latest is None:
            print("No checkpoint yet — waiting...")
            time.sleep(5)
            continue

        if latest != last_ckpt:
            # Update normalisation stats without closing GUI
            updated = update_vecnormalize_stats(gui_env, norm_path)

            print(f"\nCheckpoint : {os.path.basename(latest)}")
            print(f"Stats updated: {'yes' if updated else 'no'}")

            # Load new model weights into same env
            model     = SAC.load(latest, env=gui_env)
            last_ckpt = latest

        if model is None:
            time.sleep(1)
            continue

        episode_count += 1
        steps, reward, pitch, fell = run_episode(model, gui_env)
        result = "FELL" if fell else "SURVIVED"

        print(f"Ep {episode_count:4d} | {result:>8} | "
              f"steps={steps:5d} | reward={reward:7.1f} | "
              f"pitch={pitch:6.1f}°")

        time.sleep(0.3)

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    gui_env.close()
    print("Viewer closed.")