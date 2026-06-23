"""
train_bc.py
============
Trains a Behavioural Cloning policy from expert PID data.
Exports to ONNX + normalizer JSON ready for RPi deployment.

Run:
    python train_bc.py

Input:
    expert_data.csv  (from collect_data.py)

Output:
    bc_policy.onnx
    bc_normalizer.json
"""

import json
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset, random_split
from sklearn.preprocessing import StandardScaler
import onnx
import onnxruntime as ort
import os

# ── Config ───────────────────────────────────────────────────────
DATA_FILE  = "expert_data.csv"
ONNX_OUT   = "bc_policy.onnx"
NORM_OUT   = "bc_normalizer.json"
EPOCHS     = 150
BATCH_SIZE = 256
LR         = 1e-3
HIDDEN     = [64, 64]

# ── Load data ────────────────────────────────────────────────────
print(f"Loading {DATA_FILE}...")
if not os.path.exists(DATA_FILE):
    print(f"ERROR: {DATA_FILE} not found. Run collect_data.py first.")
    exit(1)

df = pd.read_csv(DATA_FILE)
print(f"  Total samples : {len(df)}")

# Filter fallen samples (should already be filtered by collect_data.py)
FALL_RAD = 0.262   # 15 degrees
df = df[df['pitch_rad'].abs() < FALL_RAD]
print(f"  After filter  : {len(df)} samples")

if len(df) < 1000:
    print("ERROR: Not enough clean data. Collect more.")
    exit(1)

X = df[['pitch_rad', 'pitch_rate_rad', 'yaw_rad', 'yaw_rate_rad']].values

# Negate PWM — PID outputs negative for forward lean (motor wiring)
# RPi expects positive for forward lean
y = (-df['pwm'].values / 255.0).astype(np.float32)

print(f"  Pitch range   : {X[:,0].min()*57.3:.2f} to {X[:,0].max()*57.3:.2f} deg")
print(f"  PWM range     : {(y*255).min():.0f} to {(y*255).max():.0f}")

# ── Normalise ────────────────────────────────────────────────────
scaler = StandardScaler()
X_norm = scaler.fit_transform(X).astype(np.float32)
y_norm = y.reshape(-1, 1)

norm_stats = {
    "obs_mean": scaler.mean_.tolist(),
    "obs_var":  scaler.var_.tolist(),
    "obs_clip": 10.0
}
with open(NORM_OUT, 'w') as f:
    json.dump(norm_stats, f, indent=2)
print(f"\nNormaliser saved: {NORM_OUT}")
print(f"  obs_mean: {[round(v,4) for v in norm_stats['obs_mean']]}")

# ── Dataset ──────────────────────────────────────────────────────
dataset  = TensorDataset(torch.tensor(X_norm), torch.tensor(y_norm))
n_train  = int(0.9 * len(dataset))
n_val    = len(dataset) - n_train
train_ds, val_ds = random_split(
    dataset, [n_train, n_val],
    generator=torch.Generator().manual_seed(42)
)
train_loader = DataLoader(train_ds, batch_size=BATCH_SIZE, shuffle=True)
val_loader   = DataLoader(val_ds,   batch_size=BATCH_SIZE)
print(f"  Train: {n_train}  Val: {n_val}")

# ── Model ────────────────────────────────────────────────────────
class BCPolicy(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(4,          HIDDEN[0]), nn.Tanh(),
            nn.Linear(HIDDEN[0],  HIDDEN[1]), nn.Tanh(),
            nn.Linear(HIDDEN[1],  1),         nn.Tanh()
        )
    def forward(self, x):
        return self.net(x)

model     = BCPolicy()
optimizer = torch.optim.Adam(model.parameters(), lr=LR)
criterion = nn.MSELoss()

# ── Train ────────────────────────────────────────────────────────
print(f"\nTraining {EPOCHS} epochs...")
best_val, best_state = float('inf'), None

for epoch in range(1, EPOCHS + 1):
    model.train()
    for xb, yb in train_loader:
        loss = criterion(model(xb), yb)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

    model.eval()
    vl = 0
    with torch.no_grad():
        for xb, yb in val_loader:
            vl += criterion(model(xb), yb).item() * len(xb)
    vl /= n_val

    if vl < best_val:
        best_val   = vl
        best_state = {k: v.clone() for k, v in model.state_dict().items()}

    if epoch % 25 == 0 or epoch == 1:
        print(f"  Epoch {epoch:3d} | val={vl**0.5*255:.1f} PWM")

model.load_state_dict(best_state)
model.eval()
print(f"\nBest val error: {best_val**0.5*255:.1f} PWM units")

# ── Export ONNX ──────────────────────────────────────────────────
print(f"\nExporting {ONNX_OUT}...")
dummy = torch.zeros(1, 4, dtype=torch.float32)
with torch.no_grad():
    torch.onnx.export(
        model, dummy, ONNX_OUT,
        input_names=["obs"],
        output_names=["action"],
        opset_version=11,
        dynamic_axes={"obs": {0: "batch"}}
    )
onnx.checker.check_model(onnx.load(ONNX_OUT))
print(f"ONNX verified.")

# ── Sanity check ─────────────────────────────────────────────────
sess       = ort.InferenceSession(ONNX_OUT, providers=["CPUExecutionProvider"])
input_name = sess.get_inputs()[0].name
obs_mean   = np.array(norm_stats["obs_mean"], dtype=np.float32)
obs_var    = np.array(norm_stats["obs_var"],  dtype=np.float32)

def predict(pitch_deg, pitch_rate=0.0):
    obs_raw  = np.array([pitch_deg * (3.14159/180), pitch_rate, 0.0, 0.0],
                        dtype=np.float32)
    obs_norm = np.clip((obs_raw - obs_mean) / np.sqrt(obs_var + 1e-8), -10, 10)
    out      = sess.run(None, {input_name: obs_norm.reshape(1, -1)})
    return int(out[0].flatten()[0] * 255)

print(f"\nSanity check (direction must be correct):")
print(f"  upright   ( 0°) → PWM {predict(0):+4d}   (should be ~0)")
print(f"  fwd lean  (+3°) → PWM {predict(3):+4d}   (should be positive)")
print(f"  bwd lean  (-3°) → PWM {predict(-3):+4d}   (should be negative)")
print(f"  fwd lean  (+5°) → PWM {predict(5):+4d}   (larger positive)")
print(f"  bwd lean  (-5°) → PWM {predict(-5):+4d}   (larger negative)")

print(f"\nDone.")
print(f"  scp {ONNX_OUT} {NORM_OUT} jlukas@balancebot.local:~/balancebot/")
