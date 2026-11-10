"""
collect_data.py — timed data collection
========================================
Collects expert data for exactly COLLECT_MINUTES minutes
then saves and exits automatically. No Ctrl+C needed.

Run:
    python collect_data.py
"""

import serial
import serial.tools.list_ports
import csv
import time
import sys
import os

OUTPUT_FILE      = "expert_data.csv"
BAUD_RATE        = 115200
COLLECT_MINUTES  = 5        # change as needed — recommend 5 min minimum
WARMUP_SECS      = 5        # skip first 5 seconds while you place robot upright
FALL_THRESHOLD   = 0.262    # 15 degrees in radians — skip fallen samples

# ── Find ESP32 port ──────────────────────────────────────────────
def find_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if any(x in p.description.upper() for x in
               ["CP210", "CH340", "UART", "USB SERIAL", "ESP"]):
            return p.device
    print("Available ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device} — {p.description}")
    idx = int(input("Select port number: "))
    return ports[idx].device

port = find_port()
print(f"Connecting to {port}...")

try:
    ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
    time.sleep(2.0)
    ser.reset_input_buffer()
except serial.SerialException as e:
    print(f"ERROR: {e}")
    sys.exit(1)

# ── Timed collection ─────────────────────────────────────────────
COLLECT_SECS = COLLECT_MINUTES * 60
t_start      = time.time()
deadline     = t_start + WARMUP_SECS + COLLECT_SECS
sample_count = 0
skipped_fall = 0

print(f"Warmup {WARMUP_SECS}s — stand robot upright now...")
time.sleep(WARMUP_SECS)
ser.reset_input_buffer()   # discard noisy warmup data

print(f"Collecting for {COLLECT_MINUTES} minute(s)...\n")
collect_start = time.time()
collect_deadline = collect_start + COLLECT_SECS

csvfile = open(OUTPUT_FILE, 'w', newline='')
writer  = csv.writer(csvfile)
writer.writerow(['pitch_rad', 'pitch_rate_rad', 'yaw_rad', 'yaw_rate_rad', 'pwm'])
csvfile.flush()

while time.time() < collect_deadline:
    remaining = int(collect_deadline - time.time())

    try:
        line = ser.readline().decode("ascii", errors="ignore").strip()
    except Exception:
        continue

    if not line or line[0] not in '0123456789.-':
        continue

    parts = line.split(',')
    if len(parts) != 5:
        continue

    try:
        row = [float(x) for x in parts]
    except ValueError:
        continue

    # Skip fallen samples — bad training data
    pitch = row[0]
    if abs(pitch) > FALL_THRESHOLD:
        skipped_fall += 1
        continue

    writer.writerow(row)
    csvfile.flush()
    sample_count += 1

    if sample_count % 400 == 0:
        print(f"  {sample_count:6d} saved | {remaining}s left | "
              f"pitch={pitch*57.3:+.2f}deg | "
              f"pwm={int(row[4]):+4d} | "
              f"skipped_fallen={skipped_fall}")

csvfile.close()
ser.close()

print(f"\nDone.")
print(f"Samples saved   : {sample_count}")
print(f"Samples skipped : {skipped_fall} (fallen — abs pitch > 15deg)")
print(f"Saved to        : {os.path.abspath(OUTPUT_FILE)}")

if sample_count < 1000:
    print("\nWARNING: Too few samples. Is the robot balancing stably?")
elif skipped_fall > sample_count:
    print("\nWARNING: More fallen samples than good ones.")
    print("Robot is falling too often — improve PID stability first.")
else:
    print("\nRun: python train_bc.py")
