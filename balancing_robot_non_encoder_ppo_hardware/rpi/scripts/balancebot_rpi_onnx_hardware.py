"""
balancebot_rpi_hardware.py
RPi Zero 2W — BC or PPO policy inference
==========================================

Supports both:
  - BC policy  (bc_policy.onnx  → single output, both wheels same)
  - PPO policy (policy.onnx     → two outputs, left/right independent)

Auto-detects by checking ONNX output shape.

Serial protocol (115200 baud):
  ESP32 → RPi : "pitch,pitch_rate,yaw,yaw_rate\\n"  (radians, 100Hz)
  RPi → ESP32 : "left_pwm,right_pwm\\n"              (int [-255, 255])

Run:
  cd ~/balancebot
  python balancebot_rpi_hardware.py

Files needed in same folder:
  policy.onnx
  vecnormalize.json
"""

import os
import sys
import time
import json
import numpy as np
import serial

os.environ["ORT_LOGGING_LEVEL"]     = "3"
os.environ["ONNXRUNTIME_LOG_LEVEL"] = "3"

try:
    import onnxruntime as ort
except ImportError:
    print("ERROR: pip install onnxruntime --break-system-packages")
    sys.exit(1)

# ── Config ───────────────────────────────────────────────────────
SERIAL_PORT    = "/dev/ttyAMA0"
BAUD_RATE      = 115200
ONNX_FILE      = "policy.onnx"
NORM_FILE      = "vecnormalize.json"
CONTROL_HZ     = 100
FALL_ANGLE_DEG = 25.0   # matches balance_env_v2.py FALL_ANGLE=0.523rad=30°

# PWM_GAIN — different starting points for BC vs PPO:
#   BC  policy: start at 0.5 (BC overcorrects due to narrow training data)
#   PPO policy: start at 1.0 (PPO learned proportional corrections already)
# If oscillating: reduce by 0.1. If sluggish: increase by 0.1.
PWM_GAIN = 0.28

# ── Load policy ──────────────────────────────────────────────────
for f in [ONNX_FILE, NORM_FILE]:
    if not os.path.exists(f):
        print(f"ERROR: {f} not found.")
        sys.exit(1)

print(f"Loading {ONNX_FILE}...")
sess_options                    = ort.SessionOptions()
sess_options.log_severity_level = 3
sess       = ort.InferenceSession(
    ONNX_FILE,
    sess_options=sess_options,
    providers=["CPUExecutionProvider"]
)
input_name   = sess.get_inputs()[0].name
output_shape = sess.get_outputs()[0].shape
print(f"  Input  : {sess.get_inputs()[0].shape}")
print(f"  Output : {output_shape}")

# Auto-detect policy type from output shape
IS_BC = (len(output_shape) == 2 and output_shape[1] == 1)
print(f"  Type   : {'BC (single output)' if IS_BC else 'PPO (two outputs)'}")
print(f"  Gain   : {PWM_GAIN}")

print(f"Loading {NORM_FILE}...")
with open(NORM_FILE) as f:
    stats = json.load(f)

# Keep everything float32 — consistent with ONNX model
obs_mean = np.array(stats["obs_mean"], dtype=np.float32)
obs_var  = np.array(stats["obs_var"],  dtype=np.float32)
obs_clip = float(stats["obs_clip"])
print(f"  obs_mean : {obs_mean}")
print(f"  obs_var  : {obs_var}")
print("Policy ready.\n")


def normalise(obs):
    n = (obs - obs_mean) / np.sqrt(obs_var + 1e-8)
    return np.clip(n, -obs_clip, obs_clip).astype(np.float32)


def policy_forward(obs):
    """
    Run ONNX inference.
    Returns (left_pwm, right_pwm) as integers.
    """
    obs_norm = normalise(obs).reshape(1, -1)
    outputs  = sess.run(None, {input_name: obs_norm})
    action   = outputs[0].flatten()

    if IS_BC:
        pwm = int(np.clip(action[0] * 255 * PWM_GAIN, -255, 255))
        return pwm, pwm
    else:
        left  = int(np.clip(action[0] * 255 * PWM_GAIN, -255, 255))
        right = int(np.clip(action[1] * 255 * PWM_GAIN, -255, 255))
        return left, right


# ── Serial ───────────────────────────────────────────────────────
print(f"Opening {SERIAL_PORT}...")
try:
    ser = serial.Serial(
        SERIAL_PORT,
        BAUD_RATE,
        timeout=0.015,        # 15ms — just over one ESP32 tick (10ms)
                              # long enough to receive a full line
                              # short enough not to block the control loop
        write_timeout=0.01
    )
    time.sleep(2.0)
    ser.reset_input_buffer()
except serial.SerialException as e:
    print(f"ERROR: {e}")
    sys.exit(1)

# ── Wait for READY ───────────────────────────────────────────────
print("Waiting for ESP32 READY...")
deadline = time.time() + 15.0
while time.time() < deadline:
    line = ser.readline().decode("ascii", errors="ignore").strip()
    if line == "READY":
        print("ESP32 ready.\n")
        break
    if line:
        print(f"  ESP32: {line}")
else:
    print("WARNING: No READY — continuing anyway.")

print("Balancing. Ctrl+C to stop.\n")

# ── Main loop ────────────────────────────────────────────────────
# No sleep() here — readline() with timeout=0.015 IS the pacing.
# The ESP32 sends at 100Hz (every 10ms). readline() blocks until
# it gets a line or times out at 15ms. This naturally runs at ~100Hz
# without needing an extra sleep that would cause double-waiting.
step      = 0
left_pwm  = 0
right_pwm = 0

try:
    while True:
        # Blocking read — waits up to 15ms for a complete line
        raw = ser.readline().decode("ascii", errors="ignore").strip()

        if not raw or raw.count(',') != 3 or raw.startswith("ERR"):
            # Timeout or bad line — send last known PWM to keep motors alive
            # This prevents watchdog timeout on ESP32 side
            ser.write(f"{left_pwm},{right_pwm}\n".encode("ascii"))
            continue

        try:
            parts      = raw.split(',')
            pitch      = float(parts[0])   # radians
            pitch_rate = float(parts[1])   # radians/s
            yaw        = float(parts[2])   # radians
            yaw_rate   = float(parts[3])   # radians/s
        except (ValueError, IndexError):
            ser.write(f"{left_pwm},{right_pwm}\n".encode("ascii"))
            continue

        fallen = abs(np.degrees(pitch)) > FALL_ANGLE_DEG

        if not fallen:
            obs = np.array([pitch, pitch_rate, yaw, yaw_rate],
                           dtype=np.float32)
            left_pwm, right_pwm = policy_forward(obs)
        else:
            left_pwm = right_pwm = 0

        ser.write(f"{left_pwm},{right_pwm}\n".encode("ascii"))

        # Debug 1Hz
        if step % 100 == 0:
            status = "FALLEN" if fallen else "OK"
            print(f"step={step:6d} | "
                  f"pitch={np.degrees(pitch):+6.2f}deg | "
                  f"L={left_pwm:+4d} R={right_pwm:+4d} | "
                  f"{status}")

        step += 1

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    try:
        ser.write(b"0,0\n")
        time.sleep(0.1)
    except Exception:
        pass
    ser.close()
    print("Motors stopped. Done.")
