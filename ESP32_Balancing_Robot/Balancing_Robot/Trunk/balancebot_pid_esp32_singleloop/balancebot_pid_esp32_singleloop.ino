/*
  balancebot_pid_esp32_singleloop.ino
  ==========================================================
  Standalone PID self-balancing robot - SINGLE LOOP VERSION.
  NO RPi, NO PPO, NO UART. Pure ESP32 + MPU6050 + Cytron MDD3A.

  WHY THIS VERSION EXISTS:
    The original cascade version (balancebot_pid_esp32.ino,
    outer Angle loop -> inner Rate loop, mirroring the bicopter
    flight controller structure) kept vibrating even after
    fixing two real structural bugs (saturation overshoot,
    deadband snap-to-zero oscillation). That persistence is a
    signal worth listening to: cascade PID is the right choice
    for a flight controller (where fast inner-loop rate response
    matters most and the outer loop just nudges a setpoint), but
    it is NOT the universal standard for two-wheel balance bots.
    Most working balance-bot implementations use a SINGLE PID
    loop directly on angle - simpler, one set of gains, one
    less place for inter-loop resonance/phase-lag to hide.

    This version exists to isolate whether the cascade
    STRUCTURE itself was contributing to the vibration,
    independent of gain tuning, offset, or sensor issues.

  WHAT CHANGED FROM THE CASCADE VERSION:
    REMOVED: outer Angle->Rate cascade entirely
    REMOVED: PAngle/IAngle/DAngle, PRate/IRate/DRate (8 gains)
    REMOVED: DesiredRatePitch intermediate variable
    ADDED:   single PID directly: KalmanAnglePitch -> MotorOutput
    ADDED:   just 3 gains: P, I, D
    KEPT:    identical MPU6050 reading, calibration values,
             Kalman filter (same function, same gains) -
             only the CONTROL LAW changed, not the sensing
    KEPT:    identical motor mixing, sign (confirmed correct
             via your physical push-test), fixed deadband,
             fall-detection safety cutoff

  SINGLE-LOOP STRUCTURE:
    Error = DesiredAnglePitch - KalmanAnglePitch
    P-term = P * Error
    I-term = accumulated, anti-windup clamped
    D-term = D * (Error - PrevError) / dt, then low-pass
             filtered (same noise-suppression reasoning as
             before - raw differentiation of a noisy signal
             amplifies that noise by 1/dt)
    MotorOutput = P-term + I-term + filtered-D-term
    -> directly drives both wheels (mixing/deadband unchanged)

  CYTRON MDD3A WIRING (unchanged, already confirmed working):
    M1A = GPIO5    left motor forward PWM
    M1B = GPIO10   left motor backward PWM
    M2A = GPIO3    right motor forward PWM
    M2B = GPIO4    right motor backward PWM
    Direction = which pin gets the duty cycle (PWM/PWM
    interface, NOT IN1/IN2 like L298N).

  MPU6050 WIRING (unchanged):
    SDA = GPIO6
    SCL = GPIO7

  YOUR CALIBRATION VALUES (unchanged, already plugged in):
    RateCalibrationRoll  = -3.73
    RateCalibrationPitch = -0.85
    RateCalibrationYaw   =  0.87
    AccXCalibration      = -0.03
    AccYCalibration      = -0.03
    AccZCalibration      = -0.03

  MOTOR DIRECTION SIGN (unchanged, confirmed via physical test):
    pwmCmd = -MotorOutput. Already verified correct - wheels
    drive toward the fall direction to catch it, not away
    from it. Do not re-flip without re-testing physically.

  BEFORE FIRST RUN - SET YOUR MEASURED OFFSET:
    If you ran find_vertical_offset.ino and got a real offset
    value (the robot's TRUE mechanical balance angle, which
    may not be exactly 0), set it here:
      float DesiredAnglePitch = <your measured value>;
    If you have not run that test yet, leave at 0.0 for now,
    but be aware a real offset will show up as the robot
    settling near a constant nonzero lean rather than vibrating
    - that is a DIFFERENT symptom from vibration and points
    back to the offset, not these gains.

  PID TUNING - SINGLE LOOP, START HERE:
    Much simpler tuning than cascade - only one set of gains.

    1. Start with I=0, D=0. Raise P slowly from a low value
       (try starting around P=8-12 for a typical small balance
       bot - this is a DIFFERENT scale than the old PAngle=2.5,
       because there's no inner loop multiplying it anymore).
       Increase P until the robot visibly tries to correct but
       starts a small, fast wobble around upright.
    2. Add D to damp that wobble (try starting D=0.3-0.6).
       Increase until the wobble smooths into a controlled
       settle, not a slow rock and not a sustained buzz.
    3. Only add small I if the robot settles at a consistent
       lean (steady-state error) rather than true vertical -
       I corrects that residual offset. Keep I small; too much
       causes slow oscillation as the integral overshoots.
    4. If you still see fast buzzing after this, lower
       DTERM_FILTER_ALPHA (more filtering) before raising D
       further - high D on a noisy signal often LOOKS like it
       needs more D when it actually needs more filtering.

  SAFETY:
    Same fall-detection cutoff as before (FALL_ANGLE_DEG).
    Always test holding the robot or with wheels off the
    ground first before setting it down to balance freely.
*/

#include <Wire.h>
#include <math.h>

// ==== Pin Definitions ====
#define CUSTOM_SDA_PIN 6
#define CUSTOM_SCL_PIN 7

#define PIN_M1A   5    // Left  motor forward PWM
#define PIN_M1B   10   // Left  motor backward PWM
#define PIN_M2A   3    // Right motor forward PWM
#define PIN_M2B   4    // Right motor backward PWM

#define PWM_FREQ       20000
#define PWM_RESOLUTION 8

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

// ==== Your Measured Calibration Values ====
float RateCalibrationRoll  = -3.73;
float RateCalibrationPitch = -0.85;
float RateCalibrationYaw   =  0.87;
float AccXCalibration      = -0.03;
float AccYCalibration      = -0.03;
float AccZCalibration      = -0.03;

// ==== Kalman Filter State - Pitch Only (unchanged) ====
float KalmanAnglePitch = 0;
float KalmanUncertaintyAnglePitch = 4;
volatile float Kalman1DOutput[] = {0, 0};

// ==== SINGLE-LOOP PID - Angle directly to Motor Output ====
// START HERE - see header tuning notes. These are placeholder
// starting values, NOT pre-tuned for your specific robot.
// float P = 15.0;
// float I = 0.;
// float D = 1.1;

float P = 15.0;
float I = 0.0;
float D = 1.1;

float Error = 0;
float PrevError = 0;
float PrevIterm = 0;
float MotorOutput = 0;

// Target angle - 0 = perfectly upright, OR your measured
// offset from find_vertical_offset.ino if you've run that test.
// float DesiredAnglePitch = 0.5;
float DesiredAnglePitch = 0.5;

// D-term low-pass filter - same reasoning as cascade version:
// raw differentiation of a noisy signal amplifies that noise
// by 1/dt (100x at 100Hz), which is a common cause of buzz.
float filteredDterm = 0;
const float DTERM_FILTER_ALPHA = 0.2;

// PWM deadband - SUBTRACTIVE, not snap-to-zero (the cascade
// version's snap-to-zero deadband was identified as a real
// structural bug causing its own bang-bang oscillation).
// const int PWM_DEADBAND = 2;
const int PWM_DEADBAND = 2;

// ==== Safety ====
const float FALL_ANGLE_DEG = 35.0;
bool fallen = false;

// ==== Function Prototypes ====
void read_imu();
void kalman_1d(float& KalmanState, float& KalmanUncertainty,
               float KalmanInput, float KalmanMeasurement);
float constrain_float(float value, float min_val, float max_val);
void set_motor_left(int pwm_signed);
void set_motor_right(int pwm_signed);
void motors_stop();

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
float constrain_float(float value, float min_val, float max_val) {
  if (value < min_val) return min_val;
  if (value > max_val) return max_val;
  return value;
}

// -----------------------------------------------------------
void set_motor_left(int pwm_signed) {
  pwm_signed = (int)constrain_float(pwm_signed, -255, 255);
  uint32_t duty = (uint32_t)abs(pwm_signed);
  if (pwm_signed > 0) {
    ledcWrite(PIN_M1A, duty);
    ledcWrite(PIN_M1B, 0);
  } else if (pwm_signed < 0) {
    ledcWrite(PIN_M1A, 0);
    ledcWrite(PIN_M1B, duty);
  } else {
    ledcWrite(PIN_M1A, 0);
    ledcWrite(PIN_M1B, 0);
  }
}

void set_motor_right(int pwm_signed) {
  pwm_signed = (int)constrain_float(pwm_signed, -255, 255);
  uint32_t duty = (uint32_t)abs(pwm_signed);
  if (pwm_signed > 0) {
    ledcWrite(PIN_M2A, duty);
    ledcWrite(PIN_M2B, 0);
  } else if (pwm_signed < 0) {
    ledcWrite(PIN_M2A, 0);
    ledcWrite(PIN_M2B, duty);
  } else {
    ledcWrite(PIN_M2A, 0);
    ledcWrite(PIN_M2B, 0);
  }
}

void motors_stop() {
  ledcWrite(PIN_M1A, 0); ledcWrite(PIN_M1B, 0);
  ledcWrite(PIN_M2A, 0); ledcWrite(PIN_M2B, 0);
}

// -----------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Balance Bot - SINGLE LOOP PID Starting...");

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

  ledcAttach(PIN_M1A, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PIN_M1B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PIN_M2A, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PIN_M2B, PWM_FREQ, PWM_RESOLUTION);
  motors_stop();

  Serial.println("Lift robot, watch angle output for 3 seconds...");
  delay(3000);

  LoopTimer = micros();
  Serial.println("Setup complete. Balancing active.");
}

// -----------------------------------------------------------
void loop() {
  // -- 1. Read sensors -----------------------------------------
  read_imu();

  // -- 2. Kalman filter (unchanged) ------------------------------
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch,
            RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
  KalmanAnglePitch = constrain_float(KalmanAnglePitch, -45, 45);

  // -- 3. Fall detection (unchanged) ------------------------------
  fallen = (fabs(KalmanAnglePitch) > FALL_ANGLE_DEG);
  if (fallen) {
    motors_stop();
    PrevError = 0;
    PrevIterm = 0;
    filteredDterm = 0;

    static uint32_t lastPrintFallen = 0;
    if (millis() - lastPrintFallen > 500) {
      Serial.print("FALLEN | angle=");
      Serial.println(KalmanAnglePitch);
      lastPrintFallen = millis();
    }

    while (micros() - LoopTimer < (dt * 1000000));
    LoopTimer = micros();
    return;
  }

  // -- 4. SINGLE-LOOP PID - directly Angle -> MotorOutput ----------
  Error = DesiredAnglePitch - KalmanAnglePitch;

  float Pterm = P * Error;

  float Iterm = PrevIterm + I * (Error + PrevError) * dt / 2;
  Iterm = constrain_float(Iterm, -100, 100);   // anti-windup clamp

  float DtermRaw = D * (Error - PrevError) / dt;
  filteredDterm = DTERM_FILTER_ALPHA * DtermRaw +
                  (1 - DTERM_FILTER_ALPHA) * filteredDterm;

  MotorOutput = constrain_float(Pterm + Iterm + filteredDterm, -255, 255);

  PrevError = Error;
  PrevIterm = Iterm;

  // -- 5. Motor mixing (unchanged, sign confirmed correct) ---------
  int pwmCmd = (int)(-MotorOutput);

  // Subtractive deadband (fixed version - no snap-to-zero bug)
  if (pwmCmd > 0 && pwmCmd < PWM_DEADBAND) {
    pwmCmd = 0;
  } else if (pwmCmd < 0 && pwmCmd > -PWM_DEADBAND) {
    pwmCmd = 0;
  } else if (pwmCmd >= PWM_DEADBAND) {
    pwmCmd = pwmCmd - PWM_DEADBAND;
  } else if (pwmCmd <= -PWM_DEADBAND) {
    pwmCmd = pwmCmd + PWM_DEADBAND;
  }

  set_motor_left(pwmCmd);
  set_motor_right(pwmCmd);

  // -- 6. Debug print (throttled to ~10Hz) --------------------------
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 100) {
    Serial.print("Angle: ");
    Serial.print(KalmanAnglePitch);
    Serial.print(" | Error: ");
    Serial.print(Error);
    Serial.print(" | Pterm: ");
    Serial.print(Pterm);
    Serial.print(" | Dterm: ");
    Serial.print(filteredDterm);
    Serial.print(" | PWM: ");
    Serial.println(pwmCmd);
    lastPrint = millis();
  }

  // -- 7. Maintain fixed 100Hz loop timing ---------------------------
  while (micros() - LoopTimer < (dt * 1000000));
  LoopTimer = micros();
}
