/*
  find_vertical_offset.ino
  ==========================================================
  Measurement-only sketch. NO PID, NO motors. Reads the IMU
  through the exact same pipeline as your real balance sketch
  (same calibration values, same Kalman filter) and prints the
  angle so you can find your robot's TRUE mechanical "upright"
  reading -- which is what DesiredAnglePitch should be set to.

  WHY THIS EXISTS:
    DesiredAnglePitch is supposed to be a MEASURED constant --
    the angle the sensor reports when the robot is physically
    vertical -- not a number you nudge by hand while watching
    it wobble. Hand-tuning that value during live instability
    makes it impossible to tell whether a fall is caused by a
    wrong setpoint or a bad gain, because you're changing two
    things that look like one symptom. This sketch isolates
    the measurement so you only ever guess at gains, never at
    the setpoint itself.

  HOW TO USE:
    1. Flash this (uses the same wiring as your balance sketch,
       SDA=GPIO6, SCL=GPIO7, no motor pins needed at all).
    2. Open Serial Monitor at 115200 baud.
    3. Hold the robot upright by hand. Use a wall, door frame,
       or anything with a true vertical edge as a visual guide
       -- eyeballing "looks straight" is surprisingly unreliable
       on its own.
    4. Hold still for 3-5 seconds and watch the AVG number
       settle. The "Instant" reading jitters; the "AVG" reading
       (a rolling ~0.5s average) is the one to trust.
    5. Repeat the hold 2-3 times. If you get consistent numbers
       each time (e.g. -3.x, -3.x, -3.x), that's your real
       offset. If the numbers are wildly inconsistent between
       holds, that points to a different problem entirely (loose
       IMU mount, vibration during the hold) -- not a setpoint
       you can fix by picking a number.
    6. Plug whatever you measured into your balance sketch:
         float DesiredAnglePitch = -3.x;   // your real number

  IMPORTANT -- re-run this anytime the mechanical build changes.
    Your offset depends on exactly how the IMU is mounted and
    how mass is distributed (e.g. the battery move to the top of
    the frame). A number measured before that change is not
    guaranteed to still be correct after it.

  Uses your CURRENT calibration values, matching your latest
  balance sketch exactly, so this measurement is taken through
  the identical sensing pipeline you're actually flying on.
*/

#include <Wire.h>
#include <math.h>

// ==== Pin Definitions ====
#define CUSTOM_SDA_PIN 6
#define CUSTOM_SCL_PIN 7

#define MPU6050_ADDRESS 0x68

// ==== Timing ====
const float dt = 0.01;          // 100Hz loop
uint32_t LoopTimer;

// ==== IMU Raw + Converted Variables ====
int16_t AccXLSB, AccYLSB, AccZLSB;
int16_t GyroXLSB, GyroYLSB, GyroZLSB;
float AccX, AccY, AccZ;
float RateRoll, RatePitch, RateYaw;
float AnglePitch;

// ==== Your CURRENT Calibration Values (matches latest balance sketch) ====
float RateCalibrationRoll  = -3.74;
float RateCalibrationPitch = -0.93;
float RateCalibrationYaw   =  0.91;
float AccXCalibration      = -0.04;
float AccYCalibration      = -0.02;
float AccZCalibration      = -0.03;

// ==== Kalman Filter State - Pitch Only (identical to balance sketch) ====
float KalmanAnglePitch = 0;
float KalmanUncertaintyAnglePitch = 4;
volatile float Kalman1DOutput[] = {0, 0};

// ==== Rolling average, so the printed number doesn't jitter
//      while you're trying to hold the robot still and read it ====
const int AVG_WINDOW = 50;   // 50 samples * 10ms = ~0.5s
float angleBuffer[AVG_WINDOW];
int angleBufferIndex = 0;
bool bufferFilled = false;

// ==== Function Prototypes ====
void read_imu();
void kalman_1d(float& KalmanState, float& KalmanUncertainty,
               float KalmanInput, float KalmanMeasurement);

// -----------------------------------------------------------
void kalman_1d(float& KalmanState, float& KalmanUncertainty,
               float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + dt * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + dt * dt * 4 * 4;

  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;

  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

// -----------------------------------------------------------
void read_imu() {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU6050_ADDRESS, 6);
  AccXLSB = Wire.read() << 8 | Wire.read();
  AccYLSB = Wire.read() << 8 | Wire.read();
  AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(MPU6050_ADDRESS, 6);
  GyroXLSB = Wire.read() << 8 | Wire.read();
  GyroYLSB = Wire.read() << 8 | Wire.read();
  GyroZLSB = Wire.read() << 8 | Wire.read();

  RateRoll  = (float)GyroXLSB / 65.5 - RateCalibrationRoll;
  RatePitch = (float)GyroYLSB / 65.5 - RateCalibrationPitch;
  RateYaw   = (float)GyroZLSB / 65.5 - RateCalibrationYaw;

  AccX = (float)AccXLSB / 4096.0 - AccXCalibration;
  AccY = (float)AccYLSB / 4096.0 - AccYCalibration;
  AccZ = (float)AccZLSB / 4096.0 - AccZCalibration;

  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29578;
}

// -----------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== Find Vertical Offset ===");
  Serial.println("No PID, no motors -- measurement only.");

  Wire.setClock(400000);
  Wire.begin(CUSTOM_SDA_PIN, CUSTOM_SCL_PIN);
  delay(250);

  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);

  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  delay(100);

  Serial.println("Hold the robot upright (use a wall/door frame as a guide).");
  Serial.println("Hold still 3-5s and watch the AVG value settle.");
  Serial.println("Repeat 2-3 times -- consistent AVG = your real offset.");
  Serial.println();

  LoopTimer = micros();
}

// -----------------------------------------------------------
void loop() {
  read_imu();

  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch,
            RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  // Update rolling average buffer
  angleBuffer[angleBufferIndex] = KalmanAnglePitch;
  angleBufferIndex++;
  if (angleBufferIndex >= AVG_WINDOW) {
    angleBufferIndex = 0;
    bufferFilled = true;
  }

  int count = bufferFilled ? AVG_WINDOW : angleBufferIndex;
  if (count == 0) count = 1;   // avoid divide-by-zero on first sample

  float sum = 0;
  for (int i = 0; i < count; i++) sum += angleBuffer[i];
  float avgAngle = sum / count;

  // Print at ~10Hz, not full 100Hz, so it's readable while holding still
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 100) {
    Serial.print("Instant: ");
    Serial.print(KalmanAnglePitch, 2);
    Serial.print(" deg  |  AVG (last ~0.5s): ");
    Serial.print(avgAngle, 2);
    Serial.println(" deg   <-- use this AVG value when holding still");
    lastPrint = millis();
  }

  while (micros() - LoopTimer < (dt * 1000000));
  LoopTimer = micros();
}
