/*  ============================================================
    balancebot_arduino.ino
    XIAO ESP32-C3 — Sensor bridge for RPi PPO policy

    Role:
      1. Read MPU6050 via I2C at 100Hz
      2. Compute clean pitch via Kalman filter
      3. Send pitch, pitch_rate, yaw, yaw_rate to RPi in RADIANS
      4. Receive left_pwm, right_pwm from RPi
      5. Drive Cytron MDD3A motors

    This sketch has NO PID. The PPO policy running on the RPi
    is the controller. ESP32 is purely a sensor+motor bridge.

    Why Kalman is kept:
      Raw MPU6050 pitch from accelerometer is noisy (vibration
      from motors, power spikes from 3S LiPo). Raw gyro drifts
      over time. Kalman fuses both — clean drift-corrected pitch
      with no memory of previous errors. The PPO model was
      trained on clean simulated observations. Feeding raw noisy
      sensor data widens the sim-to-real gap and degrades policy
      performance.

    Why radians:
      RPi balancebot_rpi_hardware.py calls np.degrees(pitch) on
      the received value — confirming it expects radians. The PPO
      model was trained with observations in radians. No unit
      conversion is done on the RPi side before policy_forward().

    Serial protocol (115200 baud):
      ESP32 → RPi : "pitch,pitch_rate,yaw,yaw_rate\n"
                     all values in RADIANS or RADIANS/S
                     sent every 10ms (100Hz)
      RPi → ESP32 : "left_pwm,right_pwm\n"
                     integers in [-255, 255]

    Pin map:
      MPU6050 SDA : GPIO6
      MPU6050 SCL : GPIO7
      Left  motor : M1A=GPIO5   M1B=GPIO10
      Right motor : M2A=GPIO3   M2B=GPIO4
      UART TX     : GPIO21 → RPi GPIO15 (RX)
      UART RX     : GPIO20 → RPi GPIO14 (TX)
      Both devices 3.3V — no voltage divider needed

    Kalman filter:
      Taken directly from working balancebot_PD_v2.ino.
      IMU config unchanged: ±500°/s gyro, ±8g accel.
      Your calibration offsets kept exactly.
    ============================================================ */

#include <Wire.h>
#include <math.h>

// ─── Pins ────────────────────────────────────────────────────────
#define CUSTOM_SDA_PIN  6
#define CUSTOM_SCL_PIN  7
#define PIN_M1A         5     // Left  motor forward
#define PIN_M1B         10    // Left  motor backward
#define PIN_M2A         3     // Right motor forward
#define PIN_M2B         4     // Right motor backward

#define PWM_FREQ        20000
#define PWM_RESOLUTION  8
#define MPU6050_ADDRESS 0x68

// ─── UART to RPi ─────────────────────────────────────────────────
// Serial  = USB  → PC debug monitor (Arduino IDE Serial Monitor)
// Serial1 = GPIO → RPi (sensor CSV + PWM commands)
// GPIO21 = TX → RPi GPIO15 (RX, pin 10)
// GPIO20 = RX → RPi GPIO14 (TX, pin 8)
// Same wiring confirmed working in handshake_esp32c3.ino
#define RPI_SERIAL  Serial1
#define RPI_TX_PIN  21
#define RPI_RX_PIN  20
#define RPI_BAUD    115200

// ─── Timing — 100Hz to match RPi control loop ────────────────────
const float dt = 0.01;
uint32_t LoopTimer;

// ─── IMU raw — your setup, unchanged ────────────────────────────
int16_t AccXLSB, AccYLSB, AccZLSB;
int16_t GyroXLSB, GyroYLSB, GyroZLSB;
float AccX, AccY, AccZ;
float RateRoll, RatePitch, RateYaw;
float AnglePitch;

// ─── Calibration — your measured values, unchanged ───────────────
float RateCalibrationRoll  = -3.57;
float RateCalibrationPitch = -1.08;
float RateCalibrationYaw   =  0.74;
float AccXCalibration      =  0.00;
float AccYCalibration      =  0.01;
float AccZCalibration      = -0.03;

// ─── Kalman — taken directly from your working PD_v2 code ────────
float KalmanAnglePitch            = 0;
float KalmanUncertaintyAnglePitch = 4;
float Kalman1DOutput[2]           = {0, 0};

void kalman_1d(float& KalmanState, float& KalmanUncertainty,
               float KalmanInput, float KalmanMeasurement)
{
    KalmanState       = KalmanState + dt * KalmanInput;
    KalmanUncertainty = KalmanUncertainty + dt * dt * 4 * 4;

    float KalmanGain  = KalmanUncertainty / (KalmanUncertainty + 3 * 3);
    KalmanState       = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;

    Kalman1DOutput[0] = KalmanState;
    Kalman1DOutput[1] = KalmanUncertainty;
}

// ─── Outputs in radians (what RPi expects) ───────────────────────
float pitch_rad      = 0;   // Kalman output converted to radians
float pitch_rate_rad = 0;   // gyro Y in radians/s
float yaw_rad        = 0;   // integrated yaw in radians
float yaw_rate_rad   = 0;   // gyro Z in radians/s

// ─── Motor PWM from RPi ──────────────────────────────────────────
int left_pwm  = 0;
int right_pwm = 0;

// ─── Watchdog — stop motors if RPi goes silent ───────────────────
uint32_t last_cmd_ms          = 0;
const uint32_t CMD_TIMEOUT_MS = 500;

// ─── Serial receive buffer ───────────────────────────────────────
String rx_buf = "";

// ─── Safety ──────────────────────────────────────────────────────
// In radians: 45° = 0.785 rad
const float FALL_ANGLE_RAD = 0.785;


// ================================================================
//  IMU READ — your reference, unchanged
//  ±500°/s gyro → divide by 65.5 → deg/s → convert to rad/s
//  ±8g accel    → divide by 4096 → g units
// ================================================================
void read_imu()
{
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

    // Gyro in deg/s (your working values)
    RateRoll  = (float)GyroXLSB / 65.5 - RateCalibrationRoll;
    RatePitch = (float)GyroYLSB / 65.5 - RateCalibrationPitch;
    RateYaw   = (float)GyroZLSB / 65.5 - RateCalibrationYaw;

    // Accel in g
    AccX = (float)AccXLSB / 4096.0 - AccXCalibration;
    AccY = (float)AccYLSB / 4096.0 - AccYCalibration;
    AccZ = (float)AccZLSB / 4096.0 - AccZCalibration;

    // Accel pitch in degrees (Kalman input — your formula, unchanged)
    AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29578;
}


// ================================================================
//  COMPUTE OUTPUTS IN RADIANS
//
//  Kalman runs in degrees (matches your working PD_v2 setup).
//  Convert to radians only at the serial send step.
//  This keeps your Kalman numerically identical to the working
//  version — no risk of breaking what already works.
//
//  Yaw: integrate gyro Z over time.
//       Wrap to [-π, π] to prevent overflow.
//       Gyro drift accumulates here — acceptable for short runs.
//       If long-run yaw drift is a problem, add a magnetometer.
// ================================================================
void compute_outputs()
{
    const float DEG2RAD = PI / 180.0;

    // Pitch — Kalman output (degrees) → radians
    pitch_rad = KalmanAnglePitch * DEG2RAD;

    // Pitch rate — gyro Y (deg/s) → rad/s
    // Using raw calibrated gyro directly, not Kalman angle_speed.
    // Kalman already used this for the prediction step — using it
    // again here is redundant double-processing. Direct gyro is
    // cleaner for the derivative signal the PPO model expects.
    pitch_rate_rad = RatePitch * DEG2RAD;

    // Yaw rate — gyro Z (deg/s) → rad/s
    yaw_rate_rad = RateYaw * DEG2RAD;

    // Yaw — integrate yaw rate over time
    yaw_rad += yaw_rate_rad * dt;
    if (yaw_rad >  PI) yaw_rad -= 2.0 * PI;
    if (yaw_rad < -PI) yaw_rad += 2.0 * PI;
}


// ================================================================
//  SERIAL RECEIVE — PWM commands from RPi
//  Format: "left_pwm,right_pwm\n"
//  Non-blocking — reads whatever is available this tick
// ================================================================
void read_serial()
{
    while (RPI_SERIAL.available()) {
        char c = (char)RPI_SERIAL.read();
        if (c == '\n') {
            int comma = rx_buf.indexOf(',');
            if (comma > 0) {
                left_pwm  = rx_buf.substring(0, comma).toInt();
                right_pwm = rx_buf.substring(comma + 1).toInt();
                left_pwm  = constrain(left_pwm,  -255, 255);
                right_pwm = constrain(right_pwm, -255, 255);
                last_cmd_ms = millis();
            }
            rx_buf = "";
        } else {
            rx_buf += c;
            if (rx_buf.length() > 24) rx_buf = "";  // overflow guard
        }
    }
}


// ================================================================
//  MOTOR HELPERS — Cytron MDD3A PWM/PWM dual mode
//
//  MDD3A has no direction pins. Direction is set by which
//  pin gets the duty cycle:
//    Forward  : pin_A = duty, pin_B = 0
//    Backward : pin_A = 0,    pin_B = duty
//
//  Deadband: abs(pwm) < 26 → stop
//  Matches balance_env.py MOTOR_DEADBAND = 0.10
//  (0.10 × 255 = 25.5 ≈ 26)
// ================================================================
float constrain_float(float v, float lo, float hi) {
    return v < lo ? lo : v > hi ? hi : v;
}

void set_motor_left(int pwm)
{
    if (abs(pwm) < 26) {
        ledcWrite(PIN_M1A, 0); ledcWrite(PIN_M1B, 0);
        return;
    }
    uint32_t duty = (uint32_t)constrain(abs(pwm), 0, 255);
    if (pwm > 0) { ledcWrite(PIN_M1A, duty); ledcWrite(PIN_M1B, 0);    }
    else         { ledcWrite(PIN_M1A, 0);    ledcWrite(PIN_M1B, duty); }
}

void set_motor_right(int pwm)
{
    if (abs(pwm) < 26) {
        ledcWrite(PIN_M2A, 0); ledcWrite(PIN_M2B, 0);
        return;
    }
    uint32_t duty = (uint32_t)constrain(abs(pwm), 0, 255);
    if (pwm > 0) { ledcWrite(PIN_M2A, duty); ledcWrite(PIN_M2B, 0);    }
    else         { ledcWrite(PIN_M2A, 0);    ledcWrite(PIN_M2B, duty); }
}

void motors_stop()
{
    ledcWrite(PIN_M1A, 0); ledcWrite(PIN_M1B, 0);
    ledcWrite(PIN_M2A, 0); ledcWrite(PIN_M2B, 0);
}


// ================================================================
//  SETUP
// ================================================================
void setup()
{
    Serial.begin(115200);          // USB — PC debug monitor
    RPI_SERIAL.begin(RPI_BAUD, SERIAL_8N1, RPI_RX_PIN, RPI_TX_PIN);  // GPIO UART — RPi

    // Motor pins
    ledcAttach(PIN_M1A, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_M1B, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_M2A, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_M2B, PWM_FREQ, PWM_RESOLUTION);
    motors_stop();

    // IMU
    Wire.setClock(400000);
    Wire.begin(CUSTOM_SDA_PIN, CUSTOM_SCL_PIN);
    delay(250);

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x6B); Wire.write(0x00);   // wake up
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x1C); Wire.write(0x10);   // ±8g
    Wire.endTransmission();

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x1B); Wire.write(0x08);   // ±500°/s
    Wire.endTransmission();
    delay(100);

    // Let IMU settle, then signal RPi we are ready
    delay(2000);
    last_cmd_ms = millis();

    // RPi waits for this exact string before starting control loop
    RPI_SERIAL.println("READY");
    Serial.println("DEBUG: Sent READY to RPi");

    LoopTimer = micros();
}


// ================================================================
//  MAIN LOOP — 100Hz
// ================================================================
void loop()
{
    // Enforce 10ms tick
    if (micros() - LoopTimer < (uint32_t)(dt * 1000000)) return;
    LoopTimer = micros();

    // ── 1. Read IMU ────────────────────────────────────────────
    read_imu();

    // ── 2. Kalman filter (degrees internally) ──────────────────
    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch,
              RatePitch, AnglePitch);
    KalmanAnglePitch            = Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
    KalmanAnglePitch = constrain_float(KalmanAnglePitch, -45, 45);

    // ── 3. Convert to radians for serial ───────────────────────
    compute_outputs();

    // ── 4. Read PWM commands from RPi ──────────────────────────
    read_serial();

    // ── 5. Watchdog — stop if RPi silent for 500ms ─────────────
    if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
        left_pwm  = 0;
        right_pwm = 0;
    }

    // ── 6. Safety — stop if fallen ─────────────────────────────
    if (fabs(pitch_rad) > FALL_ANGLE_RAD) {
        motors_stop();
        // Still send sensor data so RPi knows robot has fallen
        RPI_SERIAL.print(pitch_rad,      6); RPI_SERIAL.print(',');
        RPI_SERIAL.print(pitch_rate_rad, 6); RPI_SERIAL.print(',');
        RPI_SERIAL.print(yaw_rad,        6); RPI_SERIAL.print(',');
        RPI_SERIAL.print(yaw_rate_rad,   6); RPI_SERIAL.print('\n');
        return;
    }

    // ── 7. Drive motors ────────────────────────────────────────
    set_motor_left(-left_pwm);
    set_motor_right(-right_pwm);

    // ── 8. Send sensor data to RPi via GPIO UART ──────────────────
    // Format: pitch,pitch_rate,yaw,yaw_rate  (all radians)
    // RPi parses with raw.split(',') — keep exactly 4 values
    RPI_SERIAL.print(pitch_rad,      6); RPI_SERIAL.print(',');
    RPI_SERIAL.print(pitch_rate_rad, 6); RPI_SERIAL.print(',');
    RPI_SERIAL.print(yaw_rad,        6); RPI_SERIAL.print(',');
    RPI_SERIAL.print(yaw_rate_rad,   6); RPI_SERIAL.print('\n');

    // ── 9. USB debug → PC monitor only, throttled 10Hz ─────────
    static uint32_t lastDebug = 0;
    if (millis() - lastDebug >= 100) {
        lastDebug = millis();
        Serial.print("pitch=");    Serial.print(pitch_rad * 57.296, 2);
        Serial.print("deg  rate="); Serial.print(pitch_rate_rad * 57.296, 2);
        Serial.print("deg/s  L="); Serial.print(left_pwm);
        Serial.print("  R=");      Serial.println(right_pwm);
    }
}
