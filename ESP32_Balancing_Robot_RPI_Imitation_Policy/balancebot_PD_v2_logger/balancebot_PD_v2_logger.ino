/*  ============================================================
    balancebot_PD_v2_logger.ino
    PD balance robot with CSV data logging for BC training

    Based on balancebot_PD_v2.ino — three additions only:
      1. LOG_MODE flag — set true to enable CSV streaming
      2. CSV format: pitch_rad,pitch_rate_rad,yaw_rad,yaw_rate_rad,pwm
      3. Logging throttled to every 5th tick (40Hz) so 200Hz
         control loop is never slowed by Serial writes

    HOW TO USE:
      1. Set LOG_MODE = true
      2. Flash to ESP32
      3. Stand robot upright, let it balance
      4. Open Arduino Serial Monitor at 115200 OR
         run collect_data.py on PC to save to CSV file
      5. Collect at least 5 minutes of data (12,000 samples at 40Hz)
      6. Set LOG_MODE = false, reflash for normal use

    CSV columns (all in radians / radians per second):
      pitch      — Kalman angle output, converted from degrees
      pitch_rate — gyro Y, converted from deg/s
      yaw        — integrated gyro Z
      yaw_rate   — gyro Z, converted from deg/s
      pwm        — PID motor output [-255, 255]

    Units are radians because that is what the RPi PPO/BC
    policy expects. Conversion happens here, not on RPi.
    ============================================================ */

#include <Wire.h>
#include <math.h>

// ─── Set true to stream CSV, false for normal debug output ───────
#define LOG_MODE  true
#define LOG_EVERY 5        // log every Nth tick (5 = 40Hz at 200Hz loop)

// ─── Pins ────────────────────────────────────────────────────────
#define CUSTOM_SDA_PIN  6
#define CUSTOM_SCL_PIN  7
#define PIN_M1A         5
#define PIN_M1B         10
#define PIN_M2A         3
#define PIN_M2B         4

#define PWM_FREQ        20000
#define PWM_RESOLUTION  8
#define MPU6050_ADDRESS 0x68

// ─── Timing ──────────────────────────────────────────────────────
const float dt = 0.005;    // 200Hz
uint32_t LoopTimer;
int tickCount = 0;

// ─── IMU ─────────────────────────────────────────────────────────
int16_t AccXLSB, AccYLSB, AccZLSB;
int16_t GyroXLSB, GyroYLSB, GyroZLSB;
float AccX, AccY, AccZ;
float RateRoll, RatePitch, RateYaw;
float AnglePitch;

// ─── Calibration ─────────────────────────────────────────────────
float RateCalibrationRoll  = -3.57;
float RateCalibrationPitch = -1.08;
float RateCalibrationYaw   =  0.74;
float AccXCalibration      =  0.00;
float AccYCalibration      =  0.01;
float AccZCalibration      = -0.03;

// ─── Kalman ──────────────────────────────────────────────────────
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

// ─── PID — your tuned values, unchanged ──────────────────────────
float P_gain         = 40.0;
float I_gain         = 20.0;
float D_gain         = 0.07;
float DesiredAnglePitch = 0.0;

float Error = 0, PrevError = 0, PrevIterm = 0;
float MotorOutput = 0;
float filteredDterm = 0;
const float DTERM_FILTER_ALPHA = 0.80;

// ─── Yaw integration (for logging) ───────────────────────────────
float yaw_rad = 0;

// ─── Safety ──────────────────────────────────────────────────────
const float FALL_ANGLE_DEG = 35.0;


// ================================================================
//  IMU READ
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

    RateRoll  = (float)GyroXLSB / 65.5 - RateCalibrationRoll;
    RatePitch = (float)GyroYLSB / 65.5 - RateCalibrationPitch;
    RateYaw   = (float)GyroZLSB / 65.5 - RateCalibrationYaw;

    AccX = (float)AccXLSB / 4096.0 - AccXCalibration;
    AccY = (float)AccYLSB / 4096.0 - AccYCalibration;
    AccZ = (float)AccZLSB / 4096.0 - AccZCalibration;

    AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29578;

    // Yaw integration (degrees → radians)
    yaw_rad += (RateYaw * (PI / 180.0)) * dt;
    if (yaw_rad >  PI) yaw_rad -= 2.0 * PI;
    if (yaw_rad < -PI) yaw_rad += 2.0 * PI;
}


// ================================================================
//  MOTOR HELPERS
// ================================================================
float constrain_float(float v, float lo, float hi) {
    return v < lo ? lo : v > hi ? hi : v;
}

void set_motor_left(int pwm) {
    pwm = (int)constrain_float(pwm, -255, 255);
    uint32_t duty = abs(pwm);
    if      (pwm > 0) { ledcWrite(PIN_M1A, duty); ledcWrite(PIN_M1B, 0);    }
    else if (pwm < 0) { ledcWrite(PIN_M1A, 0);    ledcWrite(PIN_M1B, duty); }
    else              { ledcWrite(PIN_M1A, 0);    ledcWrite(PIN_M1B, 0);    }
}

void set_motor_right(int pwm) {
    pwm = (int)constrain_float(pwm, -255, 255);
    uint32_t duty = abs(pwm);
    if      (pwm > 0) { ledcWrite(PIN_M2A, duty); ledcWrite(PIN_M2B, 0);    }
    else if (pwm < 0) { ledcWrite(PIN_M2A, 0);    ledcWrite(PIN_M2B, duty); }
    else              { ledcWrite(PIN_M2A, 0);    ledcWrite(PIN_M2B, 0);    }
}

void motors_stop() {
    ledcWrite(PIN_M1A, 0); ledcWrite(PIN_M1B, 0);
    ledcWrite(PIN_M2A, 0); ledcWrite(PIN_M2B, 0);
}


// ================================================================
//  SETUP
// ================================================================
void setup()
{
    Serial.begin(115200);
    delay(500);

    Wire.setClock(400000);
    Wire.begin(CUSTOM_SDA_PIN, CUSTOM_SCL_PIN);
    delay(250);

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x6B); Wire.write(0x00);
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x1C); Wire.write(0x10);   // ±8g
    Wire.endTransmission();

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x1B); Wire.write(0x08);   // ±500°/s
    Wire.endTransmission();
    delay(100);

    ledcAttach(PIN_M1A, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_M1B, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_M2A, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_M2B, PWM_FREQ, PWM_RESOLUTION);
    motors_stop();

    if (LOG_MODE) {
        // Print CSV header so collect_data.py knows column names
        Serial.println("pitch_rad,pitch_rate_rad,yaw_rad,yaw_rate_rad,pwm");
    } else {
        Serial.println("balancebot_PD_v2 — normal mode");
    }

    delay(2000);    // time to stand robot up
    LoopTimer = micros();
}


// ================================================================
//  MAIN LOOP — 200Hz
// ================================================================
void loop()
{
    if (micros() - LoopTimer < (uint32_t)(dt * 1000000)) return;
    LoopTimer = micros();
    tickCount++;

    // ── 1. IMU ─────────────────────────────────────────────────
    read_imu();

    // ── 2. Kalman ──────────────────────────────────────────────
    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch,
              RatePitch, AnglePitch);
    KalmanAnglePitch            = Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
    KalmanAnglePitch = constrain_float(KalmanAnglePitch, -45, 45);

    // ── 3. Safety ──────────────────────────────────────────────
    if (fabs(KalmanAnglePitch) > FALL_ANGLE_DEG) {
        motors_stop();
        PrevError = PrevIterm = filteredDterm = 0;
        while (micros() - LoopTimer < (uint32_t)(dt * 1000000));
        LoopTimer = micros();
        return;
    }

    // ── 4. PID ─────────────────────────────────────────────────
    Error = DesiredAnglePitch - KalmanAnglePitch;

    float Pterm = P_gain * Error;

    float Iterm = PrevIterm + I_gain * (Error + PrevError) * dt / 2.0;
    Iterm = constrain_float(Iterm, -100, 100);

    float DtermRaw = D_gain * RatePitch;
    filteredDterm  = DTERM_FILTER_ALPHA * DtermRaw
                   + (1.0 - DTERM_FILTER_ALPHA) * filteredDterm;

    MotorOutput = constrain_float(Pterm + Iterm - filteredDterm, -255, 255);

    PrevError = Error;
    PrevIterm = Iterm;

    int pwmCmd = (int)MotorOutput;

    // ── 5. Motors ──────────────────────────────────────────────
    set_motor_left(pwmCmd);
    set_motor_right(pwmCmd);

    // ── 6. Log or debug ────────────────────────────────────────
    if (tickCount % LOG_EVERY == 0) {
        if (LOG_MODE) {
            // CSV — only log when actively balancing (not fallen)
            // Columns: pitch_rad, pitch_rate_rad, yaw_rad, yaw_rate_rad, pwm
            const float D2R = PI / 180.0;
            Serial.print(KalmanAnglePitch * D2R,  6); Serial.print(',');
            Serial.print(RatePitch        * D2R,  6); Serial.print(',');
            Serial.print(yaw_rad,                 6); Serial.print(',');
            Serial.print(RateYaw          * D2R,  6); Serial.print(',');
            Serial.println(pwmCmd);
        } else {
            Serial.print("Angle: ");    Serial.print(KalmanAnglePitch);
            Serial.print(" | Error: "); Serial.print(Error);
            Serial.print(" | Pterm: "); Serial.print(Pterm);
            Serial.print(" | Iterm: "); Serial.print(Iterm);
            Serial.print(" | Dterm: "); Serial.print(filteredDterm);
            Serial.print(" | PWM: ");   Serial.println(pwmCmd);
        }
    }
}
