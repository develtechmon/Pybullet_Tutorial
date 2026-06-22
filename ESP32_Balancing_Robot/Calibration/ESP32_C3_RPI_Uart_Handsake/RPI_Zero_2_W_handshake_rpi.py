"""
handshake_rpi.py — RPi Zero 2W
================================

PURPOSE:
  Minimal script to verify UART wiring between RPi Zero 2W
  and XIAO ESP32-C3 works correctly, BEFORE adding ONNX
  inference or motor control into the mix.

WHAT IT DOES:
  Listens for "PING\n" sent by the ESP32-C3 over UART.
  Replies with "PONG\n" immediately on every PING received.
  Prints a counter so you can see the link is alive and stable.

REQUIRES:
  pip install pyserial

WIRING (same as full robot script):
  RPi GPIO14 (TX, pin 8)  -> XIAO ESP32-C3 GPIO20 (D7, RX)
  RPi GPIO15 (RX, pin 10) -> XIAO ESP32-C3 GPIO21 (D6, TX)
  RPi GND    (pin 6)      -> XIAO ESP32-C3 GND
  Both devices 3.3V — direct connection, no divider needed.

  Note: ESP32-C3 sketch uses direct GPIO numbers (21, 20) instead
  of D6/D7 macros, which fail to compile on this board variant.
  Physical wiring is unchanged — only the Arduino-side code differs.

UART SETUP ON RPI (one time, requires reboot):
  sudo raspi-config
    Interface Options -> Serial Port
    "Login shell over serial" -> No
    "Serial hardware enabled" -> Yes

  Add to /boot/firmware/config.txt under [all]:
    enable_uart=1
    dtoverlay=disable-bt
  Then:
    sudo systemctl disable hciuart
    sudo reboot

  Verify: ls /dev/ttyAMA0   <- should appear after reboot

USAGE:
  1. Flash handshake_esp32c3.ino to the XIAO ESP32-C3 first
  2. Run this script on the RPi:
       python handshake_rpi.py
  3. You should see PING received and PONG count incrementing

EXPECTED OUTPUT:
  Opening /dev/ttyAMA0 at 115200...
  Listening for PING from ESP32-C3...
  [1] Received: PING -> Sent: PONG
  [2] Received: PING -> Sent: PONG
  [3] Received: PING -> Sent: PONG
  ...

IF NOTHING IS RECEIVED:
  Check wiring — TX/RX are commonly swapped by accident.
  Check ESP32-C3 sketch is actually running (check its USB
  Serial Monitor for "Sent: PING" lines).
  Check SERIAL_PORT below matches your setup.
  Check common GND is connected between both boards.
"""

import sys
import time
import serial

SERIAL_PORT = "/dev/ttyAMA0"   # change to /dev/ttyUSB0 if using USB adapter
BAUD_RATE   = 115200

print(f"Opening {SERIAL_PORT} at {BAUD_RATE}...")
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1.0)
    time.sleep(2.0)   # allow ESP32-C3 to finish booting
    ser.reset_input_buffer()
except serial.SerialException as e:
    print(f"ERROR: {e}")
    print("Check wiring and SERIAL_PORT setting.")
    sys.exit(1)

print("Listening for PING from ESP32-C3...")
print("Press Ctrl+C to stop.\n")

count = 0

try:
    while True:
        line = ser.readline().decode("ascii", errors="ignore").strip()

        if line == "PING":
            count += 1
            ser.write(b"PONG\n")
            print(f"[{count:4d}] Received: PING -> Sent: PONG")
        elif line:
            print(f"        Received (unexpected): {line!r}")

except KeyboardInterrupt:
    print(f"\n\nStopped. Total PING/PONG exchanges: {count}")
    if count > 0:
        print("✅ UART link confirmed working both directions.")
    else:
        print("❌ No PING received — check wiring before proceeding.")

finally:
    ser.close()
