"""
export_to_onnx.py
Converts trained SAC policy to ONNX format for RPi Zero 2W deployment.
Run: python export_to_onnx.py
"""

import os
import torch
import numpy as np
from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from balance_env import BalanceBotEnv

norm_path  = "./models/vecnormalize.pkl"
model_path = "./models/best/best_model"

print("Loading model...")
env   = DummyVecEnv([lambda: BalanceBotEnv()])
env   = VecNormalize.load(norm_path, env)
env.training    = False
env.norm_reward = False

model = SAC.load(model_path, env=env)
print("Loaded.\n")

# Extract the policy network
policy = model.policy
policy.eval()

# Get normalisation stats for embedding into deployment script
obs_mean = env.obs_rms.mean
obs_var  = env.obs_rms.var
obs_std  = np.sqrt(obs_var + 1e-8)
clip_obs = env.clip_obs

print("VecNormalize stats:")
print(f"  mean: {obs_mean}")
print(f"  std:  {obs_std}\n")

# Create dummy input matching observation space
dummy_obs = torch.zeros(1, 7, dtype=torch.float32)

# Export actor network to ONNX
onnx_path = "./models/balancebot_policy.onnx"

torch.onnx.export(
    policy.actor,
    dummy_obs,
    onnx_path,
    input_names=["obs"],
    output_names=["action"],
    opset_version=11,
    verbose=False
)

print(f"ONNX model saved: {onnx_path}")

# Save normalisation stats separately for RPi deployment
stats_path = "./models/norm_stats.npz"
np.savez(
    stats_path,
    mean=obs_mean,
    std=obs_std,
    clip_obs=np.array([clip_obs])
)
print(f"Norm stats saved: {stats_path}")

# Verify ONNX model
import onnxruntime as ort
sess = ort.InferenceSession(onnx_path)
test_obs = np.zeros((1, 7), dtype=np.float32)
action   = sess.run(None, {"obs": test_obs})[0]
print(f"\nONNX verification:")
print(f"  Input shape:  {test_obs.shape}")
print(f"  Output shape: {action.shape}")
print(f"  Test action:  {action}")
print("\n✅ Export complete — ready for RPi deployment")

env.close()