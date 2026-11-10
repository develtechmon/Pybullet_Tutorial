"""
export_model.py — run from project root
=========================================

Exports trained PPO model + VecNormalize stats into a
single .npz file deployable on RPi Zero 2W with numpy only.

No PyBullet or stable-baselines3 needed on RPi.

USAGE:
  Run from project root, same level as train_ppo.py:
    python export_model.py

OUTPUT:
  rpi/policy_export.npz   <- auto-created, copy this to RPi

Then copy to RPi:
  scp rpi/policy_export.npz pi@<rpi_ip>:~/balancebot/

REQUIRES (PC only):
  pip install stable-baselines3 numpy

PROJECT STRUCTURE:
  your_project/
  ├── models/
  │   ├── best/
  │   │   └── best_model.zip    <- trained PPO model
  │   └── vecnormalize.pkl      <- observation normalisation stats
  ├── rpi/
  │   └── policy_export.npz     <- auto-created by this script
  ├── balance_env.py
  ├── train_ppo.py
  ├── watch_training.py
  ├── keyboard_control.py
  └── export_model.py           <- run from here (project root)

WHY NUMPY INSTEAD OF ONNX:
  Two main export options exist for deploying PPO on embedded hardware:
    1. NumPy export (this approach)
    2. ONNX Runtime

  For this robot we chose NumPy for three reasons:

  Reason 1 - Network is too small for ONNX to matter:
    Our policy is [4 -> 256 -> 256 -> 2] with tanh activations.
    That is ~67,000 multiplications per inference step.
    On RPi Zero 2W this completes in ~0.1ms.
    We need 10ms per step at 100Hz — 100x headroom.
    ONNX Runtime gives ~1.6x speedup over PyTorch on ARM,
    but our network is already so fast that this difference
    is meaningless in practice.

  Reason 2 - Zero extra dependencies:
    NumPy is pre-installed on Raspberry Pi OS.
    ONNX Runtime requires: pip install onnxruntime (~50MB)
    which also consumes significant RAM on a 512MB device.

  Reason 3 - Simplicity and transparency:
    The forward pass is three matrix multiplications:
      x = tanh(obs @ w0.T + b0)
      x = tanh(x  @ w2.T + b2)
      a = tanh(x  @ wa.T + ba)
    Easy to verify, debug, and port to C/Arduino if needed.

  WHEN TO SWITCH TO ONNX:
    When policy becomes more complex — LSTM, CNN, camera input.
    ONNX is the industry standard for real robot deployment.
"""

import numpy as np
from stable_baselines3 import PPO
import pickle
import os

MODEL_PATH  = "./models/best/best_model.zip"
NORM_PATH   = "./models/vecnormalize.pkl"
OUTPUT_DIR  = "./rpi"
OUTPUT_PATH = os.path.join(OUTPUT_DIR, "policy_export.npz")

# ── Auto-create rpi/ directory if it does not exist ──────────
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Verify model files exist ──────────────────────────────────
for path in [MODEL_PATH, NORM_PATH]:
    if not os.path.exists(path):
        print(f"ERROR: {path} not found.")
        print("Run train_ppo.py first.")
        exit(1)

# ── Load model ────────────────────────────────────────────────
print(f"Loading: {MODEL_PATH}")
model      = PPO.load(MODEL_PATH)
state_dict = model.policy.state_dict()

print("\nPolicy layers:")
weights = {}
for key, tensor in state_dict.items():
    arr = tensor.cpu().numpy()
    weights[key] = arr
    print(f"  {key:55s} {list(arr.shape)}")

# ── Load VecNormalize stats ───────────────────────────────────
print(f"\nLoading: {NORM_PATH}")
with open(NORM_PATH, "rb") as f:
    vec_norm = pickle.load(f)

obs_mean = vec_norm.obs_rms.mean.astype(np.float32)
obs_var  = vec_norm.obs_rms.var.astype(np.float32)
obs_clip = np.array([float(vec_norm.clip_obs)], dtype=np.float32)

print(f"  obs_mean : {obs_mean}")
print(f"  obs_var  : {obs_var}")
print(f"  obs_clip : {obs_clip[0]}")
print(f"  obs shape: {obs_mean.shape}  (should be (4,))")

# ── Save to rpi/policy_export.npz ────────────────────────────
save_dict = {
    "obs_mean": obs_mean,
    "obs_var":  obs_var,
    "obs_clip": obs_clip
}
save_dict.update(weights)
np.savez(OUTPUT_PATH, **save_dict)
print(f"\nSaved: {OUTPUT_PATH}")

# ── Verify forward pass matches SB3 output ────────────────────
print("\nVerifying forward pass...")

test_obs = np.array([0.05, 0.0, 0.0, 0.0], dtype=np.float32)

normed = np.clip(
    (test_obs - obs_mean) / np.sqrt(obs_var + 1e-8),
    -obs_clip[0], obs_clip[0]
).astype(np.float64)

d  = np.load(OUTPUT_PATH)
w0 = d["mlp_extractor.policy_net.0.weight"].astype(np.float64)
b0 = d["mlp_extractor.policy_net.0.bias"].astype(np.float64)
w2 = d["mlp_extractor.policy_net.2.weight"].astype(np.float64)
b2 = d["mlp_extractor.policy_net.2.bias"].astype(np.float64)
wa = d["action_net.weight"].astype(np.float64)
ba = d["action_net.bias"].astype(np.float64)

x      = np.tanh(normed @ w0.T + b0)
x      = np.tanh(x @ w2.T + b2)
action = np.tanh(x @ wa.T + ba)

print(f"  Input obs  : {test_obs}")
print(f"  Action out : left={action[0]:+.4f}  right={action[1]:+.4f}")
print(f"  Shape check: {action.shape} (should be (2,))")

import torch
with torch.no_grad():
    obs_t      = torch.tensor(normed, dtype=torch.float32).unsqueeze(0)
    features   = model.policy.mlp_extractor.policy_net(obs_t)
    action_raw = model.policy.action_net(features)
    action_sb3 = torch.tanh(action_raw).numpy().flatten()

print(f"\n  SB3 action : left={action_sb3[0]:+.4f}  right={action_sb3[1]:+.4f}")
diff = np.abs(action - action_sb3).max()
print(f"  Max diff   : {diff:.2e}  (should be < 1e-5)")

if diff < 1e-4:
    print("\n✅ Export verified — ready to deploy on RPi")
else:
    print("\n❌ Mismatch detected — check layer names above")

print(f"\nNext step:")
print(f"  scp {OUTPUT_PATH} pi@<rpi_ip>:~/balancebot/")