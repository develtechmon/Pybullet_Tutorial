"""
keyboard_control.py — pitch bias movement
==========================================

Loads the trained balance model and uses pitch bias
to control movement direction.

CONTROLS:
  i     = forward
  k     = backward
  j     = turn left
  l     = turn right
  SPACE = stop
  r     = reset
  q     = quit

TUNING:
  TILT_BIAS  = how much to tilt (radians)
               too high → falls when moving
               too low  → barely moves
               start 0.05, adjust ±0.01

  DECAY      = coast when key released (0.85-0.95)

  TURN_BIAS  = sharpness of turns (0.05-0.20)
"""

import os
import time
import threading
import numpy as np

devnull_fd = os.open(os.devnull, os.O_WRONLY)
os.dup2(devnull_fd, 2)
os.close(devnull_fd)

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from balance_env import BalanceBotEnv

try:
    from pynput import keyboard
except ImportError:
    print("Install pynput: pip install pynput")
    exit(1)

# ── Tuning ────────────────────────────────────────────────────
TILT_BIAS = 0.20   # was 0.05 — increase for more visible movement
                   # 0.05 = 2.9 deg (barely moves)
                   # 0.15 = 8.6 deg (clearly moves)
                   # 0.25 = 14 deg  (fast, may fall)
DECAY     = 0.98   # was 0.90 — keeps bias alive longer when key held
                   # 0.90 = bias halves in ~7 steps  (too fast)
                   # 0.98 = bias halves in ~35 steps (feels natural)
TURN_BIAS = 0.40   # was 0.10 — sharper turns
# ─────────────────────────────────────────────────────────────


class Controller:
    def __init__(self):
        self._lock = threading.Lock()
        self.fwd = self.bwd = self.lft = self.rgt = False
        self.reset = self.quit = False

    def on_press(self, key):
        with self._lock:
            try:
                ch = key.char
                if   ch == 'i': self.fwd   = True
                elif ch == 'k': self.bwd   = True
                elif ch == 'j': self.lft   = True
                elif ch == 'l': self.rgt   = True
                elif ch == 'r': self.reset = True
                elif ch == 'q': self.quit  = True
            except AttributeError:
                if key == keyboard.Key.space:
                    self.fwd = self.bwd = self.lft = self.rgt = False

    def on_release(self, key):
        with self._lock:
            try:
                ch = key.char
                if   ch == 'i': self.fwd = False
                elif ch == 'k': self.bwd = False
                elif ch == 'j': self.lft = False
                elif ch == 'l': self.rgt = False
            except AttributeError:
                pass

    def read(self):
        with self._lock:
            return (self.fwd, self.bwd, self.lft, self.rgt,
                    self.reset, self.quit)

    def clear_reset(self):
        with self._lock:
            self.reset = False


# ── Load ──────────────────────────────────────────────────────
norm_path  = "./models/vecnormalize.pkl"
model_path = "./models/best/best_model"

for path, label in [(norm_path,          "models/vecnormalize.pkl"),
                    (model_path + ".zip", "models/best/best_model.zip")]:
    if not os.path.exists(path):
        print(f"ERROR: {label} not found. Run train_ppo.py first.")
        exit(1)

print("Loading policy...")
base_env        = BalanceBotEnv(render_mode="human")
vec_env         = DummyVecEnv([lambda: base_env])
env             = VecNormalize.load(norm_path, vec_env)
env.training    = False
env.norm_reward = False
model           = PPO.load(model_path, env=env)
print("Loaded.\n")

print("=" * 45)
print("  i=forward  k=backward  j=left  l=right")
print("  SPACE=stop  r=reset  q=quit")
print("=" * 45)
print(f"  TILT_BIAS={TILT_BIAS}  DECAY={DECAY}  TURN={TURN_BIAS}")
print("=" * 45)
print()

ctrl     = Controller()
listener = keyboard.Listener(
    on_press=ctrl.on_press, on_release=ctrl.on_release)
listener.start()

obs        = env.reset()
steps      = 0
episode    = 1
pitch_bias = 0.0

try:
    while True:
        t0 = time.time()
        fwd, bwd, lft, rgt, do_reset, do_quit = ctrl.read()

        if do_quit:
            break

        if do_reset:
            ctrl.clear_reset()
            obs        = env.reset()
            steps      = 0
            episode   += 1
            pitch_bias = 0.0
            print(f"[RESET] Episode {episode}\n")
            continue

        # Pitch bias — instant snap on keypress, smooth decay on release
        if   fwd and not bwd: pitch_bias = +TILT_BIAS
        elif bwd and not fwd: pitch_bias = -TILT_BIAS
        elif fwd and bwd:     pitch_bias =  0.0
        else:
            pitch_bias *= DECAY
            if abs(pitch_bias) < 0.001:
                pitch_bias = 0.0

        # Turn differential
        if   lft and not rgt: turn = -TURN_BIAS
        elif rgt and not lft: turn = +TURN_BIAS
        else:                  turn =  0.0

        # Apply pitch bias to obs
        biased_obs        = obs.copy()
        biased_obs[0][0] += pitch_bias

        action, _ = model.predict(biased_obs, deterministic=True)

        # Apply turn differential
        if turn != 0.0:
            action[0][0] = np.clip(action[0][0] - turn, -1.0, 1.0)
            action[0][1] = np.clip(action[0][1] + turn, -1.0, 1.0)

        obs, _, done, info = env.step(action)
        steps += 1

        if steps % 50 == 0:
            pitch = info[0].get("pitch_deg", 0)
            if   pitch_bias > 0.005: state = f"▶ FWD  {pitch_bias:+.3f}"
            elif pitch_bias < -0.005: state = f"◀ BWD  {pitch_bias:+.3f}"
            elif turn < 0:            state = "↺ LEFT"
            elif turn > 0:            state = "RIGHT ↻"
            else:                     state = "HOLD"
            print(f"Ep {episode:3d} | Step {steps:5d} | "
                  f"Pitch {pitch:+5.1f}° | {state}")

        if done[0]:
            pitch = info[0].get("pitch_deg", 0)
            print(f"\n[FELL] Ep {episode} pitch={pitch:.1f}° step={steps}")
            if abs(pitch_bias) > 0.01:
                print(f"  Try reducing TILT_BIAS (currently {TILT_BIAS})")
            obs        = env.reset()
            steps      = 0
            episode   += 1
            pitch_bias = 0.0

        wait = (1.0 / 100.) - (time.time() - t0)
        if wait > 0:
            time.sleep(wait)

except KeyboardInterrupt:
    print("\nCtrl+C.")
finally:
    listener.stop()
    env.close()
    print("Done.")