Step 1 — pip install pybullet gymnasium stable-baselines3 numpy onnx onnxruntime tensorboard tqdm rich

Step 2 — python test_balancebot_urdf.py        ← robot loads and falls
Step 3 — python test_env.py         ← reward behaves correctly
Step 4 — python train_ppo.py        ← train 2M steps
Step 5 — python evaluate.py         ← watch it balance
Step 6 — python export_to_onnx.py   ← get .onnx for hardware

PPO (what you used):
  - Discards all experience after each update
  - Needs millions of steps to converge
  - Collapsed at entropy_loss = -15.7
  - std exploded to 640

SAC (switching to):
  - Reuses all past experience via replay buffer
  - Auto-tunes entropy — won't collapse
  - Should show improvement within 100k-200k steps
  - std will stay bounded because VecNormalize keeps obs in [-10,10]

# 1. Stop current training (Ctrl+C)
# 2. Delete models/ and logs/ folders
# 3. Terminal 1
python train_sac.py

# 4. Terminal 2 — wait until you see "vecnormalize.pkl saved" message
#    (happens after 10,000 steps, ~8 seconds at 1200 fps)
python watch_training.py

# 5. Terminal 3
tensorboard --logdir ./logs/tensorboard/

# 6. After training finishes or Ctrl+C after 45k+ steps
python evaluate.py

## The Complete Health Dashboard

Here's a one-page reference for reading any future training output:

METRIC          EARLY TRAINING    HEALTHY LATE    BAD SIGN
──────────────────────────────────────────────────────────
ep_length       low (40-200)      high (2000)     stuck <200
ep_reward       negative/low      high, climbing  stuck or falling
variance        high (±500)       low (±50)       high after 50k steps
ent_coef        high (0.5-1.0)    stable (0.001)  crashed to 0 fast
ent_coef_loss   large negative    near zero       flipping signs
actor_loss      small negative    large negative  getting less negative
critic_loss     high (0.3+)       low (0.01-)     spiking up and down
fps             high (1000+)      lower (50-100)  normal, not a problem