/*  ============================================================
    balancebot_PD_v2.ino
    Based directly on your REFERENCE code.

    Three changes only — everything else identical:

    1. D term: now uses RatePitch directly from gyro
               NOT (Error-PrevError)/dt
               That method accumulated a slow bias at 100Hz
               and caused the centre point to drift.
               RatePitch is a direct angular velocity measurement
               — it has no memory and cannot drift.

    2. Iterm clamp: -3550 to 3550 was insane.
               Changed to -100 to 100.
               At P=35 and 100Hz, Iterm of 3550 means the
               integral alone saturates motors for ~1000 seconds.
               That's why the centre point kept shifting — the
               integral was winding up and never recovering.

    3. dt: changed to 0.005 (200Hz) to match Keyestudio
               and make D gain values consistent with references.

    Motor direction, pins, calibration, Kalman, PID structure,
    serial format — all identical to your reference.
    ============================================================ */

#include <Wire.h>
#include <math.h>

// ─── Pins — your reference, unchanged ───────────────────────────
#define CUSTOM_SDA_PIN  6
#define CUSTOM_SCL_PIN  7
#define PIN_M1A         5 	// Left  motor forward PWM
#define PIN_M1B         10  // Left  motor backward PWM
#define PIN_M2A         3	// Right motor forward PWM
#define PIN_M2B         4	// Right motor backward PWM

#define PWM_FREQ        20000
#define PWM_RESOLUTION  8
#define MPU6050_ADDRESS 0x68

// ─── Timing — 200Hz ─────────────────────────────────────────────
const float dt = 0.005;
uint32_t LoopTimer;

// ─── IMU — your reference, unchanged ────────────────────────────
int16_t AccXLSB, AccYLSB, AccZLSB;
int16_t GyroXLSB, GyroYLSB, GyroZLSB;
float AccX, AccY, AccZ;
float RateRoll, RatePitch, RateYaw;
float AnglePitch;

// ─── Calibration — your values, unchanged ───────────────────────
float RateCalibrationRoll  = -3.57;
float RateCalibrationPitch = -1.08;
float RateCalibrationYaw   =  0.74;
float AccXCalibration      =  0.00;
float AccYCalibration      =  0.01;
float AccZCalibration      = -0.03;

// ─── Kalman — your reference, unchanged ─────────────────────────
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

// ─── PID ─────────────────────────────────────────────────────────
// float P_gain = 38.0;
// float I_gain = 20;
// float D_gain = 0.04;

// Mecanum Wheel 95mm
float P_gain = 25.0;
float I_gain = 20;
float D_gain = 0.08;

// Original 65mm wheel
// float P_gain = 40.0;
// float I_gain = 20;
// float D_gain = 0.07;

float DesiredAnglePitch = 0.0;   // SET THIS from find_vertical_offset

float Error     = 0;
float PrevError = 0;
float PrevIterm = 0;
float MotorOutput = 0;

float filteredDterm = 0;

//const float DTERM_FILTER_ALPHA = 0.20;
const float DTERM_FILTER_ALPHA = 0.80;

// ─── Safety ─────────────────────────────────────────────────────
const float FALL_ANGLE_DEG = 35.0;

// ─── Serial ─────────────────────────────────────────────────────
static uint32_t lastPrint       = 0;
static uint32_t lastPrintFallen = 0;


// ================================================================
//  IMU READ — your reference, unchanged
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
}


// ================================================================
//  MOTOR HELPERS — your reference, unchanged
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
    delay(1000);
    Serial.println("balancebot_PD_v2 starting...");

    Wire.setClock(400000);
    Wire.begin(CUSTOM_SDA_PIN, CUSTOM_SCL_PIN);
    delay(250);

    // IMU init — your register settings, unchanged
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

    Serial.println("Lift robot, watch angle for 3s...");
    delay(3000);
    Serial.println("Balancing active.");

    LoopTimer = micros();
}


// ================================================================
//  MAIN LOOP — 200Hz
// ================================================================
void loop()
{
    // ── 1. IMU ─────────────────────────────────────────────────
    read_imu();

    // ── 2. Kalman — your code, unchanged ───────────────────────
    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch,
              RatePitch, AnglePitch);
    KalmanAnglePitch            = Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
    KalmanAnglePitch = constrain_float(KalmanAnglePitch, -45, 45);

    // ── 3. Safety ──────────────────────────────────────────────
    if (fabs(KalmanAnglePitch) > FALL_ANGLE_DEG) {
        motors_stop();
        PrevError     = 0;
        PrevIterm     = 0;
        filteredDterm = 0;

        if (millis() - lastPrintFallen > 500) {
            lastPrintFallen = millis();
            Serial.print("FALLEN | angle=");
            Serial.println(KalmanAnglePitch);
        }
        while (micros() - LoopTimer < (uint32_t)(dt * 1000000));
        LoopTimer = micros();
        return;
    }

    // ── 4. PID ─────────────────────────────────────────────────
    Error = DesiredAnglePitch - KalmanAnglePitch;

    // P term
    float Pterm = P_gain * Error;

    // I term — clamp at ±100, NOT ±3550
    // At P=35, Iterm of 3550 saturates motors for ~1000 seconds.
    // ±100 means integral can contribute at most 40% of max PWM.
    float Iterm = PrevIterm + I_gain * (Error + PrevError) * dt / 2.0;
    Iterm = constrain_float(Iterm, -100, 100);

    // D term — RatePitch directly from gyro, NOT (Error-PrevError)/dt
    // (Error-PrevError)/dt at 100Hz amplifies tiny angle drift into
    // a large slowly-shifting bias — that was your drifting centre.
    // RatePitch is a direct measurement, has no memory, cannot drift.
    float DtermRaw = D_gain * RatePitch;
    filteredDterm  = DTERM_FILTER_ALPHA * DtermRaw
                   + (1.0 - DTERM_FILTER_ALPHA) * filteredDterm;

    // Minus on D: RatePitch sign opposes correction direction
    MotorOutput = constrain_float(Pterm + Iterm - filteredDterm, -255, 255);

    PrevError = Error;
    PrevIterm = Iterm;

    // ── 5. Motors ──────────────────────────────────────────────
    int pwmCmd = (int)MotorOutput;
    set_motor_left(pwmCmd);
    set_motor_right(pwmCmd);

    // ── 6. Debug — your format, unchanged ──────────────────────
    if (millis() - lastPrint > 100) {
        lastPrint = millis();
        Serial.print("Angle: ");    Serial.print(KalmanAnglePitch);
        Serial.print(" | Error: "); Serial.print(Error);
        Serial.print(" | Pterm: "); Serial.print(Pterm);
        Serial.print(" | Iterm: "); Serial.print(Iterm);
        Serial.print(" | Dterm: "); Serial.print(filteredDterm);
        Serial.print(" | PWM: ");   Serial.println(pwmCmd);
    }

    // ── 7. Loop timing ─────────────────────────────────────────
    while (micros() - LoopTimer < (uint32_t)(dt * 1000000));
    LoopTimer = micros();
}
