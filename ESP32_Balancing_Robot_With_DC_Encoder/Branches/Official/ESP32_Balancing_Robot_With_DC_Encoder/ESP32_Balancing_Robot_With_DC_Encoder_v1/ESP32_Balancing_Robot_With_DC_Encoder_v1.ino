/*  ============================================================
    balancebot_PD_encoder_v1.ino

    STEP 1: Balance only + encoder drift correction.
    No RC. No turning. No position hold. Just balancing.

    Once this works, we add position hold. Then RC. One thing at a time.

    PIN ASSIGNMENT (new PCB pinout, Super Mini ESP32-S3):
    ------------------------------------------------------
    Left motor  : M1A=8, M1B=9
    Right motor : M2A=4, M2B=5
    IMU         : SDA=6, SCL=7

    Left encoder  : Yellow(A)=10, Green(B)=11
    Right encoder : Yellow(A)=2,  Green(B)=3

    PPM pin (for future RC): GPIO 13
    (not used in this balance-only version yet)

    Note: GPIO3 is a strapping pin (JTAG select at boot). Using it
    as ENC_R_B is common practice and should be fine, but flagging
    it in case of unexpected boot behavior down the line.

    WHAT ENCODERS ADD HERE:
    -----------------------
    Your robot balances via IMU. But it drifts because:
    - Battery voltage changes → PWM output shifts slightly
    - Weight distribution changes → true balance point shifts
    - Floor slope you can't detect → constant drift force

    Without encoders: you tune AngleOffset manually to fight drift.
    Every session, every battery, every floor — you retune.

    With encoders: the code measures actual wheel motion and slowly
    adjusts the balance point to kill drift automatically.

    THE ONE FEATURE:
    Drift correction — auto-adjusts AutoAngleOffset in real time.

    ANALOGY:
    You know when you stand on one leg with your eyes closed and
    slowly wobble? Now imagine someone whispers "you're leaning
    slightly forward" — you shift your weight back to compensate.
    The whisper is the encoder. Your weight shift is AutoAngleOffset.
    ============================================================ */

#include <Wire.h>
#include <math.h>

#define CUSTOM_SDA_PIN  7
#define CUSTOM_SCL_PIN  6
#define PIN_M1A         8
#define PIN_M1B         9
#define PIN_M2A         13
#define PIN_M2B         12

// ─── Encoder pins (updated for new PCB pinout) ─────────────────
// Note: GPIO3 is a strapping pin on ESP32-S3 (JTAG signal select
// at boot). Using it as ENC_R_B is common and generally safe, but
// if you ever see odd boot behavior, this pin is the first suspect.
// Left motor encoder: Yellow (A channel) and Green (B channel)
#define ENC_L_A         10
#define ENC_L_B         11
// Right motor encoder: Yellow (A channel) and Green (B channel)
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
float I_gain = 70; //20
float D_gain = 0.4; //0.1

float DesiredAnglePitch = 2.0;   // stays 0 for balance-only mode

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
//  When A rises:
//    if B is currently LOW  → wheel spinning one direction
//    if B is currently HIGH → wheel spinning other direction
//
//  We use equality check of A and B state to determine direction:
//    Same state (both 0 or both 1) after A changes = forward
//    Different state = backward
//
//  Interrupt fires only on channel A CHANGE (both edges).
//  We could interrupt on B too for 4x resolution, but for a slow
//  balancing loop it's unnecessary and wastes CPU on ISR calls.
// ================================================================
volatile long encoderCountL = 0;
volatile long encoderCountR = 0;

void IRAM_ATTR encoderISR_L() {
    // Read both pins fast — we're inside an ISR
    bool a = digitalRead(ENC_L_A);
    bool b = digitalRead(ENC_L_B);
    // Same state after A changes → one direction
    // Different state → other direction
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
//
//  At 200Hz, we sample every 5ms. In 5ms a slowly-moving robot
//  might only accumulate 1-2 pulses. That's terrible resolution.
//  Velocity would jitter between 200 and 400 pulses/sec.
//
//  At 200Hz, we'd sample every 5ms — too few pulses for a clean
//  reading on a slow-moving robot. At 50Hz (every 20ms), we get
//  4x more pulses per sample, so velocity is much less noisy.
//
//  At 50Hz, we sample every 20ms. Same slow motion accumulates 8
//  pulses. Velocity reading is 4x cleaner.
//
//  Trade-off: 50Hz means our drift signal has ~20ms of lag. But
//  drift is a slow phenomenon anyway — 20ms delay is invisible.
// ================================================================
long lastEncoderL = 0;
long lastEncoderR = 0;
float velocityL = 0.0;       // pulses per second, left wheel
float velocityR = 0.0;       // pulses per second, right wheel
float avgVelocity = 0.0;     // average (positive = moving forward)
unsigned long lastVelocityUpdate = 0;
const unsigned long VELOCITY_UPDATE_MS = 20;

// ================================================================
//  DRIFT CORRECTION
//
//  DRIFT_CORRECTION_RATE:
//  How aggressively to shift the balance point per loop cycle.
//
//  Think of it as an integral gain. Small values = slow but stable
//  correction. Large values = fast but can overshoot and oscillate.
//
//  Starting value: 0.0001
//  This means: for every 1 pulse/sec of drift, we shift
//  AutoAngleOffset by 0.0001 degrees every main loop cycle.
//
//  Over 200 loops (1 second), that's 0.02 degrees per pulse/sec.
//  So 100 pulses/sec drift → 2 degrees shift per second.
//  Slow enough to feel natural, fast enough to actually work.
//
//  TUNING:
//  - If robot drifts and never corrects → increase to 0.0003
//  - If robot oscillates (rocks forward-back rhythmically) → decrease
//  - Reset to 0 on fall to prevent bad state accumulating
// ================================================================
const float DRIFT_CORRECTION_RATE = 0.0001;
const float DRIFT_CORRECTION_LIMIT = 3.0;  // never shift more than 3°
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
//  VELOCITY UPDATE (runs at 50Hz, called every main loop)
//
//  Reads encoder count difference over 20ms window.
//  Converts to pulses per second (velocity).
//
//  Why average left and right?
//  - If both wheels turn same speed → robot translates (drift)
//  - If they differ → robot is rotating (yaw)
//  - For balance drift correction, we only care about translation
//  - So we average both wheels
//
//  Sign of right wheel is negated because the motors face
//  opposite directions physically. Positive avgVelocity means
//  robot is moving forward.
// ================================================================
void update_velocity() {
    unsigned long now = millis();
    if (now - lastVelocityUpdate < VELOCITY_UPDATE_MS) return;

    float elapsed = (now - lastVelocityUpdate) / 1000.0;  // seconds
    lastVelocityUpdate = now;

    // Read encoder counts atomically (disable interrupts briefly)
    // Otherwise the ISR might update mid-read and give garbage
    noInterrupts();
    long currentL = encoderCountL;
    long currentR = encoderCountR;
    interrupts();

    // Velocity = change in pulses / elapsed time
    velocityL = (currentL - lastEncoderL) / elapsed;
    velocityR = -(currentR - lastEncoderR) / elapsed;  // negate right side
    lastEncoderL = currentL;
    lastEncoderR = currentR;

    // Average velocity — positive means forward drift
    avgVelocity = (velocityL + velocityR) / 2.0;
}

// ================================================================
//  DRIFT CORRECTION
//
//  Every main loop, slowly nudge AutoAngleOffset opposite to drift.
//
//  Logic:
//  - Robot moving forward (velocity > 0) → shift balance point back
//    (make it think it's already leaning back, so it corrects forward
//     — actually this stops the forward drift)
//  - Robot moving backward → shift balance point forward
//
//  Over time, this converges to the "true balance point" where drift
//  goes to zero.
//
//  IMPORTANT: this is a slow integral action. Any run-off is bounded
//  by DRIFT_CORRECTION_LIMIT so it can't shift the robot into a bad
//  state if something goes wrong.
// ================================================================
void update_drift_correction() {
    // Negative sign: forward drift → decrease target angle (lean back)
    AutoAngleOffset -= avgVelocity * DRIFT_CORRECTION_RATE;

    // Safety limit
    AutoAngleOffset = constrain_float(AutoAngleOffset,
                                       -DRIFT_CORRECTION_LIMIT,
                                        DRIFT_CORRECTION_LIMIT);
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

    // ── Encoder setup ───────────────────────────────────────
    // INPUT_PULLUP because hall sensors are typically open-collector.
    // They pull LOW when active, but need external pullup to reach HIGH.
    pinMode(ENC_L_A, INPUT_PULLUP);
    pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP);
    pinMode(ENC_R_B, INPUT_PULLUP);

    // Attach interrupts on CHANGE (rising + falling edges) of channel A.
    // Channel B is only read inside the ISR to determine direction.
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
        // Reset drift correction on fall — don't carry bad state over
        AutoAngleOffset = 0.0;

        if (millis() - lastPrintFallen > 500) {
            lastPrintFallen = millis();
            Serial.print("FALLEN | angle=");
            Serial.println(KalmanAnglePitch);
        }
        while (micros() - LoopTimer < (uint32_t)(dt * 1000000));
        LoopTimer = micros();
        return;
    }

    // Encoder updates — only actually runs every 20ms
    update_velocity();

    // Drift correction — runs every loop (uses last velocity reading)
    update_drift_correction();

    // ── Balance PID ─────────────────────────────────────────
    // The target angle now includes AutoAngleOffset from drift correction.
    // DesiredAnglePitch stays 0 in this mode (no RC input yet).
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

    int motorCmd = (int)(MotorOutput);

    // Both motors same speed (no turning in this version)
    set_motor_left(-motorCmd);
    set_motor_right(motorCmd);

    // ── Debug output — shows what's happening ────────────────
    // Watch these values while it balances:
    // - Angle: should stay near 0 (or near AutoOff)
    // - Vel: should trend toward 0 over 10-30 seconds

    // Let the robot balance, then gently push it forward
    // Watch Vel: in Serial Monitor while it's moving forward
    // If Vel reads positive while moving forward → the current sign convention is correct, keep it
    // If Vel reads negative while moving forward → flip which side gets the negative sign (move it from velocityR to velocityL), or just negate the whole avgVelocity line
    // - AutoOff: will slowly settle at your robot's true balance point
    // - EncL/EncR: should be growing similarly (both wheels balanced)
    if (millis() - lastPrint > 100) {
        lastPrint = millis();
        Serial.print("Angle: ");     Serial.print(KalmanAnglePitch, 2);
        Serial.print(" | Target: "); Serial.print(effectiveTarget, 2);
        Serial.print(" | Vel: ");    Serial.print(avgVelocity, 0);
        Serial.print(" | AutoOff: "); Serial.print(AutoAngleOffset, 3);
        Serial.print(" | EncL: ");   Serial.print(encoderCountL);
        Serial.print(" | EncR: ");   Serial.print(encoderCountR);
        Serial.print(" | PWM: ");    Serial.println(motorCmd);
    }

    while (micros() - LoopTimer < (uint32_t)(dt * 1000000));
    LoopTimer = micros();
}
