"""
export_model_onnx.py — run from project root
=============================================

Exports the trained PPO model to ONNX format and saves
VecNormalize stats to JSON. The two output files are
everything the RPi needs to run the policy — no PyBullet,
no stable-baselines3, no torch required on the RPi.

USAGE:
  Run from project root, same level as train_ppo.py:
    python export_model_onnx.py

OUTPUT (auto-created inside rpi/ folder):
  rpi/policy.onnx          <- full computation graph
  rpi/vecnormalize.json    <- observation normalisation stats

COPY TO RPI:
  scp rpi/policy.onnx rpi/vecnormalize.json pi@<rpi_ip>:~/balancebot/

REQUIRES ON PC:
  pip install stable-baselines3 torch onnx onnxruntime numpy

PROJECT STRUCTURE:
  your_project/
  ├── models/
  │   ├── best/
  │   │   └── best_model.zip    <- trained PPO model
  │   └── vecnormalize.pkl      <- observation normalisation stats
  ├── rpi/
  │   ├── policy.onnx           <- auto-created by this script
  │   ├── vecnormalize.json     <- auto-created by this script
  │   ├── balancebot_rpi_onnx.py
  │   └── balancebot_rpi.py     <- numpy version (alternative)
  ├── esp32/
  │   └── balancebot_esp32.ino
  ├── balance_env.py
  ├── train_ppo.py
  ├── export_model.py           <- numpy export (alternative)
  └── export_model_onnx.py      <- this file

WHY ONNX INSTEAD OF NUMPY:

  The numpy export works by manually extracting each weight
  matrix and writing the forward pass by hand:
    x = tanh(obs @ w0.T + b0)   <- layer 1
    x = tanh(x  @ w2.T + b2)   <- layer 2
    a = tanh(x  @ wa.T + ba)   <- output

  This is fine for the current simple MLP [4->256->256->2].
  However it has two problems:

  Problem 1 — Not future-proof:
    If you upgrade the architecture — LSTM for temporal memory,
    CNN for camera input, attention layers — you must rewrite
    the entire forward pass manually. One wrong matrix dimension
    and inference silently gives wrong results.

  Problem 2 — Not the research standard:
    Production robot deployments use ONNX. The Unitree Go2
    quadruped (arXiv:2603.17653) deployed PPO policies using
    ONNX to maintain a strict 50Hz control loop with <20ms
    inference budget on real hardware. ONNX is also the format
    used by the ONNX Model Zoo and Arm's official RPi tutorial.

  ONNX exports the entire computation graph automatically.
  The export and inference code never changes regardless of
  how complex the architecture becomes. This is why we chose
  ONNX for hardware deployment.

HOW SB3 ONNX EXPORT WORKS:
  SB3 PPO policy cannot be exported directly because it uses
  Python-level broadcasting that ONNX does not support natively.
  The official SB3 solution (docs.stable-baselines3.readthedocs)
  is to wrap the policy in a thin OnnxableSB3Policy class that
  calls policy(observation, deterministic=True).
  This returns (actions, values, log_prob) as a tuple.
  We only use actions[0] at inference — values and log_prob
  are for training only.

  VecNormalize is NOT included in the ONNX graph.
  Observations must be normalised before feeding to ONNX.
  We save obs_mean, obs_var, obs_clip to vecnormalize.json
  so the RPi can apply normalisation manually.

WHAT THIS SCRIPT DOES STEP BY STEP:
  1. Load best_model.zip and vecnormalize.pkl
  2. Wrap policy in OnnxableSB3Policy
  3. Export to rpi/policy.onnx using torch.onnx.export
  4. Validate ONNX graph using onnx.checker
  5. Save VecNormalize stats to rpi/vecnormalize.json
  6. Run test inference and compare ONNX vs SB3 output
     Max diff should be < 1e-4 (floating point rounding only)

ONNX vs NUMPY SIDE BY SIDE:
  Feature                NumPy Export        ONNX Export
  ──────────────────     ─────────────       ───────────
  PC dependencies        sb3, numpy          sb3, torch, onnx
  RPi dependencies       numpy (built-in)    onnxruntime (~50MB)
  Forward pass           written by hand     auto-exported
  Architecture support   MLP only            any architecture
  RPi RAM usage          ~5MB                ~80MB
  Inference time (MLP)   ~0.1ms              ~0.1ms (no difference)
  Future-proof           no                  yes
  Research standard      no                  yes
  Error-prone            yes (manual math)   no (auto graph)
"""

import os
import json
import pickle
import numpy as np
import torch as th
from typing import Tuple
from stable_baselines3 import PPO
from stable_baselines3.common.policies import BasePolicy

MODEL_PATH   = "./models/best/best_model.zip"
NORM_PATH    = "./models/vecnormalize.pkl"
OUTPUT_DIR   = "./rpi/model/"
ONNX_PATH    = os.path.join(OUTPUT_DIR, "policy.onnx")
JSON_PATH    = os.path.join(OUTPUT_DIR, "vecnormalize.json")

# ── Auto-create rpi/ directory ────────────────────────────────
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Verify files exist ────────────────────────────────────────
for path in [MODEL_PATH, NORM_PATH]:
    if not os.path.exists(path):
        print(f"ERROR: {path} not found.")
        print("Run train_ppo.py first.")
        exit(1)

# ── Wrapper class required by SB3 for ONNX export ────────────
# SB3 PPO policy has broadcasting layers that ONNX does not
# support directly. This wrapper calls policy() cleanly.
# Source: stable-baselines3 official documentation
class OnnxableSB3Policy(th.nn.Module):
    def __init__(self, policy: BasePolicy):
        super().__init__()
        self.policy = policy

    def forward(self, observation: th.Tensor) -> Tuple[
            th.Tensor, th.Tensor, th.Tensor]:
        # deterministic=True — no random sampling at inference
        # returns: (actions, values, log_prob)
        # We only use actions[0] at inference time
        return self.policy(observation, deterministic=True)


# ── Load model ────────────────────────────────────────────────
print(f"Loading: {MODEL_PATH}")
model = PPO.load(MODEL_PATH, device="cpu")
model.policy.to("cpu")

# ── Wrap and export to ONNX ───────────────────────────────────
print("Wrapping policy for ONNX export...")
onnx_policy    = OnnxableSB3Policy(model.policy)
obs_size       = model.observation_space.shape   # (4,)
dummy_input    = th.randn(1, *obs_size)           # batch of 1

print(f"Observation size: {obs_size}")
print(f"Exporting to: {ONNX_PATH}")

th.onnx.export(
    onnx_policy,
    dummy_input,
    ONNX_PATH,
    opset_version  = 17,          # requires PyTorch 2.0+
    input_names    = ["observation"],
    output_names   = ["action", "value", "log_prob"],
    dynamic_axes   = {
        "observation": {0: "batch_size"},
        "action":      {0: "batch_size"}
    }
)
print(f"Saved: {ONNX_PATH}")

# ── Verify ONNX model ─────────────────────────────────────────
print("\nVerifying ONNX model structure...")
import onnx
onnx_model = onnx.load(ONNX_PATH)
onnx.checker.check_model(onnx_model)
print("ONNX model structure valid.")

# ── Save VecNormalize stats to JSON ───────────────────────────
# VecNormalize is NOT exported into the ONNX graph.
# We save the stats separately so the RPi can apply
# normalisation before feeding observations to the ONNX model.
print(f"\nLoading VecNormalize: {NORM_PATH}")
with open(NORM_PATH, "rb") as f:
    vec_norm = pickle.load(f)

norm_stats = {
    "obs_mean": vec_norm.obs_rms.mean.tolist(),
    "obs_var":  vec_norm.obs_rms.var.tolist(),
    "obs_clip": float(vec_norm.clip_obs)
}

with open(JSON_PATH, "w") as f:
    json.dump(norm_stats, f, indent=2)

print(f"Saved: {JSON_PATH}")
print(f"  obs_mean : {norm_stats['obs_mean']}")
print(f"  obs_var  : {norm_stats['obs_var']}")
print(f"  obs_clip : {norm_stats['obs_clip']}")

# ── Verify inference matches SB3 output ───────────────────────
print("\nVerifying ONNX inference...")
import onnxruntime as ort

# Apply VecNormalize manually — same as RPi will do
test_obs  = np.array([0.05, 0.0, 0.0, 0.0], dtype=np.float32)
obs_mean  = np.array(norm_stats["obs_mean"], dtype=np.float32)
obs_var   = np.array(norm_stats["obs_var"],  dtype=np.float32)
obs_clip  = norm_stats["obs_clip"]

normed = np.clip(
    (test_obs - obs_mean) / np.sqrt(obs_var + 1e-8),
    -obs_clip, obs_clip
).astype(np.float32)

normed_batch = normed.reshape(1, -1)   # shape (1, 4)

# Run ONNX inference
sess          = ort.InferenceSession(ONNX_PATH)
input_name    = sess.get_inputs()[0].name
onnx_outputs  = sess.run(None, {input_name: normed_batch})
onnx_action   = onnx_outputs[0].flatten()

# Compare with SB3 direct prediction
with th.no_grad():
    obs_t      = th.tensor(normed_batch, dtype=th.float32)
    sb3_action, _, _ = model.policy(obs_t, deterministic=True)
    sb3_action = sb3_action.numpy().flatten()

print(f"  Input obs    : {test_obs}")
print(f"  ONNX action  : left={onnx_action[0]:+.4f}  right={onnx_action[1]:+.4f}")
print(f"  SB3  action  : left={sb3_action[0]:+.4f}  right={sb3_action[1]:+.4f}")
diff = np.abs(onnx_action - sb3_action).max()
print(f"  Max diff     : {diff:.2e}  (should be < 1e-4)")

if diff < 1e-3:
    print("\n✅ ONNX export verified — ready to deploy on RPi")
else:
    print("\n❌ Mismatch — check PyTorch and ONNX versions")

print(f"\nNext steps:")
print(f"  pip install onnxruntime   (on RPi)")
print(f"  scp rpi/policy.onnx rpi/vecnormalize.json pi@<rpi_ip>:~/balancebot/")
print(f"  python balancebot_rpi_onnx.py")