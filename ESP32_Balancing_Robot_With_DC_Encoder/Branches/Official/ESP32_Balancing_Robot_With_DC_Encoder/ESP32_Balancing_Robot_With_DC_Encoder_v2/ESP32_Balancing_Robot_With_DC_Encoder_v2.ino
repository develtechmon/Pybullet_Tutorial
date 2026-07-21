/*  ============================================================
    balancebot_PD_encoder_v1.ino

    STEP 1: Balance only + encoder drift correction.
    Position hold removed for now — testing drift correction
    in isolation first. No RC. No turning yet.

    PIN ASSIGNMENT (new PCB pinout, Super Mini ESP32-S3):
    ------------------------------------------------------
    Left motor  : M1A=8, M1B=9
    Right motor : M2A=4, M2B=5
    IMU         : SDA=7, SCL=6

    Left encoder  : Yellow(A)=10, Green(B)=11
    Right encoder : Yellow(A)=2,  Green(B)=3

    PPM pin (for future RC): GPIO 4 //13
    (not used in this balance-only version yet)

    Note: GPIO3 is a strapping pin (JTAG select at boot). Using it
    as ENC_R_B is common practice and should be fine, but flagging
    it in case of unexpected boot behavior down the line.

    MOTOR DIRECTION FIX (found by hardware testing):
    --------------------------------------------------
    Left motor is wired with inverted polarity relative to right.
    Global negation on MotorOutput was wrong — the negative sign
    now applies ONLY to the left motor call, not to both.

    WHAT ENCODERS ADD HERE:
    -----------------------
    Your robot balances via IMU. But it drifts because:
    - Battery voltage changes → PWM output shifts slightly
    - Weight distribution changes → true balance point shifts
    - Floor slope you can't detect → constant drift force

    Without encoders: you tune AngleOffset manually to fight drift.
    With encoders: the code measures actual wheel motion and slowly
    adjusts the balance point to kill drift automatically.

    ANALOGY:
    You know when you stand on one leg with your eyes closed and
    slowly wobble? Now imagine someone whispers "you're leaning
    slightly forward" — you shift your weight back to compensate.
    The whisper is the encoder. Your weight shift is AutoAngleOffset.

    IMPROVEMENT IN THIS VERSION — drift correction timing fixed:
    --------------------------------------------------------------
    Previously, the correction ran every main loop (200Hz) but the
    velocity value only updated every 20ms (50Hz) — meaning each
    velocity reading got applied 4 times before a fresh one arrived.
    That's not broken, but it makes the rate constant ambiguous.

    Now: the correction runs ONCE per fresh velocity sample, using
    the actual elapsed time. This makes DRIFT_CORRECTION_RATE mean
    exactly what its comment says, independent of main loop speed.

    Also added: a velocity deadband, so tiny encoder noise (a few
    pulses/sec of jitter when the robot is actually still) doesn't
    slowly bias AutoAngleOffset in the wrong direction over time.
    ============================================================ */

#include <Wire.h>
#include <math.h>

#define CUSTOM_SDA_PIN  7
#define CUSTOM_SCL_PIN  6
#define PIN_M1A         8
#define PIN_M1B         9
#define PIN_M2A         13 
#define PIN_M2B         12 

// ─── Encoder pins ───────────────────────────────────────────────
// Note: GPIO3 is a strapping pin on ESP32-S3 (JTAG signal select
// at boot). Using it as ENC_R_B is common and generally safe.
#define ENC_L_A         10
#define ENC_L_B         11
#define ENC_R_A         2
#define ENC_R_B         3

#define PWM_FREQ        20000
#define PWM_RESOLUTION  8
#define MPU6050_ADDRESS 0x68

const float dt = 0.005;
uint32_t LoopTimer;

int16_t AccXLSB, AccYLSB, AccZLSB;
int16_t GyroXLSB, GyroYLSB, GyroZLSB;
float AccX, AccY, AccZ;
float RateRoll, RatePitch, RateYaw;
float AnglePitch;

float RateCalibrationRoll  = -4.90;
float RateCalibrationPitch =  0.89;
float RateCalibrationYaw   = -0.40;
float AccXCalibration      =  0.08;
float AccYCalibration      =  0.00;
float AccZCalibration      =  0.05;

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

// PID gains — unchanged from your working code
float P_gain = 40.0; //18
float I_gain = 100; //20
float D_gain = 0.4; //0.1

float DesiredAnglePitch = 2.52;   // stays 0 for balance-only mode

float Error       = 0;
float PrevError   = 0;
float PrevIterm   = 0;
float MotorOutput = 0;
float filteredDterm      = 0;
const float DTERM_FILTER_ALPHA = 0.80;

const float FALL_ANGLE_DEG = 35.0;

// ================================================================
//  ENCODER READING
//
//  Quadrature explained:
//  Each motor has two hall sensors (A and B) placed 90 degrees apart.
//  As the wheel turns, A and B produce square waves offset in phase.
//
//  We use equality check of A and B state to determine direction:
//    Same state (both 0 or both 1) after A changes = one direction
//    Different state = other direction
//
//  Interrupt fires only on channel A CHANGE (both edges).
// ================================================================
volatile long encoderCountL = 0;
volatile long encoderCountR = 0;

void IRAM_ATTR encoderISR_L() {
    bool a = digitalRead(ENC_L_A);
    bool b = digitalRead(ENC_L_B);
    if (a == b) {
        encoderCountL++;
    } else {
        encoderCountL--;
    }
}

void IRAM_ATTR encoderISR_R() {
    bool a = digitalRead(ENC_R_A);
    bool b = digitalRead(ENC_R_B);
    if (a == b) {
        encoderCountR++;
    } else {
        encoderCountR--;
    }
}

// ================================================================
//  VELOCITY TRACKING (measured every 20ms = 50Hz)
//
//  Why 50Hz instead of 200Hz (main loop rate)?
//  At 200Hz, we'd sample every 5ms — too few pulses for a clean
//  reading on a slow-moving robot. At 50Hz (every 20ms), we get
//  4x more pulses per sample, so velocity is much less noisy.
// ================================================================
long lastEncoderL = 0;
long lastEncoderR = 0;
float velocityL = 0.0;
float velocityR = 0.0;
float avgVelocity = 0.0;     // positive = moving forward
unsigned long lastVelocityUpdate = 0;
const unsigned long VELOCITY_UPDATE_MS = 20;

// ================================================================
//  DRIFT CORRECTION
//
//  DRIFT_CORRECTION_RATE units: degrees of AutoAngleOffset shift,
//  per second, per (pulse/sec) of sustained velocity.
//
//  Example with DRIFT_CORRECTION_RATE = 0.02:
//  Robot drifts forward at a steady 50 pulses/sec.
//  Shift per second = 50 × 0.02 = 1.0 degree/second.
//  After 2 seconds of sustained drift, AutoOffset has moved ~2°.
//  As AutoOffset approaches the true balance point, the drift
//  itself shrinks, so the correction naturally slows down and
//  settles — this is standard integral-control behavior.
//
//  VELOCITY_DEADBAND: below this, treat velocity as sensor noise,
//  not real drift. Without this, tiny encoder jitter at rest could
//  slowly bias AutoOffset in a random direction over many minutes.
// ================================================================
const float DRIFT_CORRECTION_RATE = 0.02; // Tune this is needed;
const float DRIFT_CORRECTION_LIMIT = 3.0;   // never shift more than ±3°
const float VELOCITY_DEADBAND = 5.0;        // pulses/sec — ignore below this
float AutoAngleOffset = 0.0;

// ================================================================
//  IMU READ — unchanged
// ================================================================
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

// ================================================================
//  MOTOR HELPERS — unchanged
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
//  VELOCITY UPDATE + DRIFT CORRECTION
//  (combined into one function so the correction always uses the
//   correct elapsed time — this is the timing fix)
//
//  Runs at 50Hz. Each call:
//   1. Computes fresh velocity from encoder counts
//   2. Applies drift correction using THAT SAME elapsed time
//  This keeps the two mathematically tied together properly,
//  instead of drift correction running on a stale value 4 extra
//  times per velocity update like the previous version did.
// ================================================================
void update_velocity_and_drift() {
    unsigned long now = millis();
    if (now - lastVelocityUpdate < VELOCITY_UPDATE_MS) return;

    float elapsed = (now - lastVelocityUpdate) / 1000.0;  // seconds
    lastVelocityUpdate = now;

    noInterrupts();
    long currentL = encoderCountL;
    long currentR = encoderCountR;
    interrupts();

    velocityL = (currentL - lastEncoderL) / elapsed;
    velocityR = -(currentR - lastEncoderR) / elapsed;   // negate right side
    lastEncoderL = currentL;
    lastEncoderR = currentR;

    avgVelocity = (velocityL + velocityR) / 2.0;

    // ── Drift correction, using THIS call's actual elapsed time ──
    // Deadband: ignore small velocity readings (sensor noise at rest)
    if (fabs(avgVelocity) > VELOCITY_DEADBAND) {
        AutoAngleOffset -= avgVelocity * DRIFT_CORRECTION_RATE * elapsed;
        AutoAngleOffset = constrain_float(AutoAngleOffset,
                                           -DRIFT_CORRECTION_LIMIT,
                                            DRIFT_CORRECTION_LIMIT);
    }
}

static uint32_t lastPrint       = 0;
static uint32_t lastPrintFallen = 0;

// ================================================================
//  SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("balancebot_PD_encoder_v1 starting...");

    Wire.setClock(400000);
    Wire.begin(CUSTOM_SDA_PIN, CUSTOM_SCL_PIN);
    delay(250);

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x6B); Wire.write(0x00);
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x1C); Wire.write(0x10);
    Wire.endTransmission();

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x1B); Wire.write(0x08);
    Wire.endTransmission();
    delay(100);

    ledcAttach(PIN_M1A, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_M1B, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_M2A, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_M2B, PWM_FREQ, PWM_RESOLUTION);
    motors_stop();

    pinMode(ENC_L_A, INPUT_PULLUP);
    pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP);
    pinMode(ENC_R_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC_L_A), encoderISR_L, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), encoderISR_R, CHANGE);

    Serial.println("Lift robot, hold steady for 3s...");
    delay(3000);
    Serial.println("Balancing active. Drift correction enabled.");

    LoopTimer = micros();
    lastVelocityUpdate = millis();
}

// ================================================================
//  MAIN LOOP
// ================================================================
void loop() {
    read_imu();

    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch,
              RatePitch, AnglePitch);
    KalmanAnglePitch            = Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
    KalmanAnglePitch = constrain_float(KalmanAnglePitch, -45, 45);

    if (fabs(KalmanAnglePitch) > FALL_ANGLE_DEG) {
        motors_stop();
        PrevError = 0;
        PrevIterm = 0;
        filteredDterm = 0;
        AutoAngleOffset = 0.0;   // reset on fall — don't carry bad state over

        if (millis() - lastPrintFallen > 500) {
            lastPrintFallen = millis();
            Serial.print("FALLEN | angle=");
            Serial.println(KalmanAnglePitch);
        }
        while (micros() - LoopTimer < (uint32_t)(dt * 1000000));
        LoopTimer = micros();
        return;
    }

    // Velocity + drift correction, tied together with correct timing
    // (only actually recalculates every 20ms internally)
    update_velocity_and_drift();

    // ── Balance PID ─────────────────────────────────────────
    float effectiveTarget = DesiredAnglePitch + AutoAngleOffset;
    Error = effectiveTarget - KalmanAnglePitch;

    float Pterm = P_gain * Error;

    float Iterm = PrevIterm + I_gain * (Error + PrevError) * dt / 2.0;
    if (Iterm >  50.0) { Iterm =  50.0; }
    if (Iterm < -50.0) { Iterm = -50.0; }

    float DtermRaw = D_gain * RatePitch;
    filteredDterm  = DTERM_FILTER_ALPHA * DtermRaw
                   + (1.0 - DTERM_FILTER_ALPHA) * filteredDterm;

    MotorOutput = constrain_float(Pterm + Iterm - filteredDterm, -255, 255);

    PrevError = Error;
    PrevIterm = Iterm;

    // ── Motor output — direction fix from hardware testing ──────
    // Left motor has inverted polarity relative to right, so the
    // negative sign applies ONLY to the left motor call now.
    int motorCmd = (int)(MotorOutput);
    set_motor_left(-motorCmd);
    set_motor_right(motorCmd);

    // ── Debug output ──────────────────────────────────────────
    // Angle   : should stay near 0 (or near AutoOff)
    // Vel     : should trend toward 0 over 10-30 seconds
    // AutoOff : will slowly settle at the robot's true balance point
    if (millis() - lastPrint > 100) {
        lastPrint = millis();
        Serial.print("Angle: ");     Serial.print(KalmanAnglePitch, 2);
        Serial.print(" | Target: "); Serial.print(effectiveTarget, 2);
        Serial.print(" | Vel: ");    Serial.print(avgVelocity, 0);
        Serial.print(" | AutoOff: "); Serial.print(AutoAngleOffset, 3);
        Serial.print(" | PWM: ");    Serial.println(motorCmd);
    }

    while (micros() - LoopTimer < (uint32_t)(dt * 1000000));
    LoopTimer = micros();
}