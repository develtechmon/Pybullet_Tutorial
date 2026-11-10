"""
test_policy.py — RPi policy test without ESP32
================================================

PURPOSE:
  Test the trained PPO policy on RPi Zero 2W without
  any hardware connected. Injects fake MPU6050 readings
  and prints what the policy decides to do.

  This verifies:
    1. policy.onnx and vecnormalize.json load correctly on RPi
    2. VecNormalize normalisation works correctly
    3. Policy outputs sensible actions for given sensor states
    4. RPi is fast enough to run inference at 100Hz

HOW IT WORKS:
  Feeds predefined sensor scenarios into the ONNX policy.
  Each scenario simulates a specific physical state.
  Policy output (left/right wheel actions) is interpreted
  and printed as a direction label.

ACTION INTERPRETATION:
  Policy outputs [left_wheel, right_wheel] in range [-1, +1]
  Positive = forward rotation, Negative = backward rotation

  Both wheels strongly forward          -> FORWARD
  Both wheels strongly backward         -> BACKWARD
  Large differential, left > right      -> TURN RIGHT
  Large differential, right > left      -> TURN LEFT
  Both near zero                        -> BALANCED

REQUIRES:
  pip install onnxruntime numpy
  policy.onnx and vecnormalize.json in same folder

  Copy from PC after running export_model_onnx.py:
    scp rpi/policy.onnx rpi/vecnormalize.json pi@<rpi_ip>:~/balancebot/

USAGE:
  python test_policy.py
  python test_policy.py --loop    <- continuous loop + measures Hz
"""

import os
import sys

# Must be set BEFORE importing onnxruntime.
# The GPU warning comes from the C++ shared library during initialisation.
# Setting env var after import is too late — the library is already loaded.
os.environ["ORT_LOGGING_LEVEL"]     = "3"
os.environ["ONNXRUNTIME_LOG_LEVEL"] = "3"

try:
    import onnxruntime as ort
except ImportError:
    print("ERROR: onnxruntime not installed.")
    print("Run: pip install onnxruntime")
    sys.exit(1)

import numpy as np
import json
import time

# ── Load ONNX policy ──────────────────────────────────────────
ONNX_FILE = "policy.onnx"
NORM_FILE = "vecnormalize.json"

for f in [ONNX_FILE, NORM_FILE]:
    if not os.path.exists(f):
        print(f"ERROR: {f} not found.")
        print("Run export_model_onnx.py on PC and copy here:")
        print("  scp rpi/policy.onnx rpi/vecnormalize.json pi@<ip>:~/balancebot/")
        sys.exit(1)

print(f"Loading {ONNX_FILE}...")
sess_options = ort.SessionOptions()
sess_options.log_severity_level = 3   # suppress warnings

sess       = ort.InferenceSession(
    ONNX_FILE,
    sess_options=sess_options,
    providers=["CPUExecutionProvider"]
)
input_name = sess.get_inputs()[0].name
print(f"  Input  : {input_name} {sess.get_inputs()[0].shape}")
print(f"  Outputs: {[o.name for o in sess.get_outputs()]}")

print(f"\nLoading {NORM_FILE}...")
with open(NORM_FILE, "r") as f:
    stats = json.load(f)

obs_mean = np.array(stats["obs_mean"], dtype=np.float32)
obs_var  = np.array(stats["obs_var"],  dtype=np.float32)
obs_clip = float(stats["obs_clip"])

print(f"  obs_mean : {obs_mean}")
print(f"  obs_var  : {obs_var}")
print(f"  obs_clip : {obs_clip}")
print("ONNX policy loaded.\n")


def normalise(obs):
    n = (obs - obs_mean) / np.sqrt(obs_var + 1e-8)
    return np.clip(n, -obs_clip, obs_clip).astype(np.float32)


def policy_forward(obs):
    """
    Run ONNX inference.
    Returns action [left, right] clamped to [-1, 1].
    Clamp applied defensively — ONNX export may skip
    final tanh squashing in some torch/onnx version combos.
    """
    obs_norm  = normalise(obs.astype(np.float32))
    obs_batch = obs_norm.reshape(1, -1)
    outputs   = sess.run(None, {input_name: obs_batch})
    action    = outputs[0].flatten()
    return np.clip(action, -1.0, 1.0)   # safety clamp


def interpret_action(action, threshold=0.05):
    """
    Convert [left, right] wheel actions to human-readable label.

    Priority order:
      1. Check differential first (turning)
      2. Then check average (forward/backward)
      3. Finally check magnitude (balanced)

    Note: policy corrects tilt, so:
      Robot tilting FORWARD  -> wheels spin FORWARD  (corrects)
      Robot tilting BACKWARD -> wheels spin BACKWARD (corrects)
    """
    left, right = float(action[0]), float(action[1])
    avg  = (left + right) / 2.0
    diff = left - right   # positive = left faster = turning right

    # Both near zero — robot thinks it is balanced
    if abs(left) < threshold and abs(right) < threshold:
        return "BALANCED"

    # Large differential — turning (check before forward/backward)
    if abs(diff) > 0.4:
        return "TURN RIGHT" if diff > 0 else "TURN LEFT"

    # Both forward or backward
    if avg > threshold:
        return "FORWARD"
    elif avg < -threshold:
        return "BACKWARD"

    return "BALANCED"


# ── Fake sensor scenarios ─────────────────────────────────────
# [pitch (rad), pitch_rate (rad/s), yaw (rad), yaw_rate (rad/s)]
#
# pitch positive = tilting forward (nose down)
# pitch negative = tilting backward (nose up)

scenarios = [
    ("Perfectly upright",            [ 0.000,  0.00,  0.000,  0.00]),
    ("Slight noise upright",         [ 0.005, -0.01,  0.001, -0.01]),
    ("Small forward tilt  (2°)",     [ 0.035,  0.00,  0.000,  0.00]),
    ("Medium forward tilt (5°)",     [ 0.087,  0.00,  0.000,  0.00]),
    ("Large forward tilt  (10°)",    [ 0.175,  0.00,  0.000,  0.00]),
    ("Forward tilt + falling fast",  [ 0.087,  1.50,  0.000,  0.00]),
    ("Small backward tilt  (-2°)",   [-0.035,  0.00,  0.000,  0.00]),
    ("Medium backward tilt (-5°)",   [-0.087,  0.00,  0.000,  0.00]),
    ("Backward tilt + falling fast", [-0.087, -1.50,  0.000,  0.00]),
    ("Spinning left  (+yaw_rate)",   [ 0.000,  0.00,  0.000, +1.50]),
    ("Spinning right (-yaw_rate)",   [ 0.000,  0.00,  0.000, -1.50]),
    ("Forward tilt + spinning",      [ 0.087,  0.00,  0.000, +1.00]),
    ("Near fall threshold (28°)",    [ 0.489,  0.00,  0.000,  0.00]),
]

# ── Run ───────────────────────────────────────────────────────
LOOP_MODE = "--loop" in sys.argv

print("=" * 70)
print(f"  {'Scenario':<38} {'Decision':>12}   L       R")
print("=" * 70)

if not LOOP_MODE:
    for label, obs_vals in scenarios:
        obs    = np.array(obs_vals, dtype=np.float32)
        action = policy_forward(obs)
        result = interpret_action(action)
        pitch_deg = np.degrees(obs_vals[0])
        print(f"  {label:<38} {result:>12} "
              f"  {action[0]:+.3f}  {action[1]:+.3f}"
              f"   (pitch={pitch_deg:+.1f}°)")

    print("=" * 70)
    print("\nAll scenarios complete.")
    print("\nRun with --loop to measure inference speed:")
    print("  python test_policy.py --loop")

else:
    print("LOOP MODE — cycling through scenarios. Ctrl+C to stop.\n")
    idx     = 0
    count   = 0
    t_start = time.time()

    try:
        while True:
            t0 = time.time()

            label, obs_vals = scenarios[idx]
            obs    = np.array(obs_vals, dtype=np.float32)
            action = policy_forward(obs)
            result = interpret_action(action)
            pitch_deg = np.degrees(obs_vals[0])

            print(f"\r  {label:<38} {result:>12} "
                  f"  L={action[0]:+.3f}  R={action[1]:+.3f}"
                  f"  pitch={pitch_deg:+.1f}°",
                  end="", flush=True)

            idx   = (idx + 1) % len(scenarios)
            count += 1

            if count % 100 == 0:
                elapsed = time.time() - t_start
                hz      = count / elapsed
                print(f"\n  [{count} steps]  "
                      f"Inference: {hz:.1f} Hz  "
                      f"({1000/hz:.2f} ms/step)\n")

            wait = 0.01 - (time.time() - t0)
            if wait > 0:
                time.sleep(wait)

    except KeyboardInterrupt:
        elapsed = time.time() - t_start
        hz      = count / elapsed
        print(f"\n\nStopped after {count} steps.")
        print(f"Average inference rate : {hz:.1f} Hz")
        print(f"Average step time      : {1000/hz:.3f} ms")
        print(f"100Hz requirement      : 10.000 ms")
        if hz >= 95:
            print("✅ RPi fast enough for 100Hz control loop")
            print(f"   ({100-hz:.1f} Hz under target — normal Python timing jitter)")
        else:
            print("❌ Too slow — consider reducing network size")