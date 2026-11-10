/*  ============================================================
    balancebot_direction_test_v1.ino

    HuskyLens completely removed. Pure open-loop direction test:
    BALANCE -> FORWARD -> TURN LEFT -> TURN RIGHT -> BACKWARD -> DONE

    Purpose: verify forward/backward/left/right all work correctly
    in isolation, on THIS board's actual pins and gains, before
    layering any vision tracking back on top.

    ================================================================
    CRITICAL FIX INCLUDED IN THIS VERSION
    ================================================================
    The motor mixing formula used in every previous version had a
    real bug:
        pwmLeft  = -motorCmd - turnBias
        pwmRight =  motorCmd + turnBias
    Algebraically, pwmLeft = -(motorCmd + turnBias) = -pwmRight ALWAYS,
    regardless of turnBias. This means turnBias never actually
    created a differential between wheels -- it just changed the
    shared magnitude, structurally identical to changing motorCmd.
    That's confirmed by your own logged data: motorCmd=7, turnBias=60
    produced PWM_L=-67, PWM_R=67, which is exactly -(7+60) -- the
    degenerate pattern. This is why turning looked like "just going
    forward": the formula was never capable of producing a turn.

    FIXED to:
        pwmLeft  = -motorCmd + turnBias
        pwmRight =  motorCmd + turnBias
    Now pwmLeft = -pwmRight only when turnBias = 0 -- genuinely
    differential for any nonzero turn command.

    PIN ASSIGNMENT (same as your working encoder file):
    Left motor  : M1A=8, M1B=9
    Right motor : M2A=13, M2B=12
    IMU         : SDA=7, SCL=6
    Left encoder  : Yellow(A)=10, Green(B)=11
    Right encoder : Yellow(A)=2,  Green(B)=3
    ============================================================ */

#include <Wire.h>
#include <math.h>

#define CUSTOM_SDA_PIN  7
#define CUSTOM_SCL_PIN  6
#define PIN_M1A         8
#define PIN_M1B         9
#define PIN_M2A         13
#define PIN_M2B         12

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

// PID gains -- unchanged from your working balance code
float P_gain = 40.0;
float I_gain = 100;
float D_gain = 0.4;

float DesiredAnglePitch = 2.52;   // your tuned baseline balance point

float Error       = 0;
float PrevError   = 0;
float PrevIterm   = 0;
float MotorOutput = 0;
float filteredDterm      = 0;
const float DTERM_FILTER_ALPHA = 0.80;

const float FALL_ANGLE_DEG = 35.0;

// ================================================================
//  ENCODER + DRIFT CORRECTION -- unchanged from your working code
// ================================================================
volatile long encoderCountL = 0;
volatile long encoderCountR = 0;

void IRAM_ATTR encoderISR_L() {
    bool a = digitalRead(ENC_L_A);
    bool b = digitalRead(ENC_L_B);
    if (a == b) { encoderCountL++; } else { encoderCountL--; }
}

void IRAM_ATTR encoderISR_R() {
    bool a = digitalRead(ENC_R_A);
    bool b = digitalRead(ENC_R_B);
    if (a == b) { encoderCountR++; } else { encoderCountR--; }
}

long lastEncoderL = 0;
long lastEncoderR = 0;
float velocityL = 0.0;
float velocityR = 0.0;
float avgVelocity = 0.0;
unsigned long lastVelocityUpdate = 0;
const unsigned long VELOCITY_UPDATE_MS = 20;

const float DRIFT_CORRECTION_RATE = 0.02;
const float DRIFT_CORRECTION_LIMIT = 3.0;
const float VELOCITY_DEADBAND = 5.0;
float AutoAngleOffset = 0.0;

bool isMoving = false;   // pauses drift correction during the sequence

void update_velocity_and_drift() {
    unsigned long now = millis();
    if (now - lastVelocityUpdate < VELOCITY_UPDATE_MS) return;

    float elapsed = (now - lastVelocityUpdate) / 1000.0;
    lastVelocityUpdate = now;

    noInterrupts();
    long currentL = encoderCountL;
    long currentR = encoderCountR;
    interrupts();

    velocityL = (currentL - lastEncoderL) / elapsed;
    velocityR = -(currentR - lastEncoderR) / elapsed;
    lastEncoderL = currentL;
    lastEncoderR = currentR;

    avgVelocity = (velocityL + velocityR) / 2.0;

    if (!isMoving && fabs(avgVelocity) > VELOCITY_DEADBAND) {
        AutoAngleOffset -= avgVelocity * DRIFT_CORRECTION_RATE * elapsed;
        AutoAngleOffset = constrain_float(AutoAngleOffset,
                                           -DRIFT_CORRECTION_LIMIT,
                                            DRIFT_CORRECTION_LIMIT);
    }
}

// ================================================================
//  SIMPLE TIMED STATE MACHINE -- no HuskyLens, pure open loop
// ================================================================
enum MoveState { STATE_BALANCE, STATE_FORWARD, STATE_TURN_LEFT,
                  STATE_TURN_RIGHT, STATE_BACKWARD, STATE_DONE };
MoveState currentState = STATE_BALANCE;
uint32_t stateStartMillis = 0;

// Conservative starting magnitudes -- the turn mechanism is newly
// fixed and hasn't been physically validated yet, so starting lower
// than the old (broken) TURN_STEP=60 until confirmed safe.
const float FORWARD_STEP  = 5.0;    // scaled for this file's P_gain=40
const float BACKWARD_STEP = -5.0;
const float TURN_STEP     = 40.0;   // start conservative, increase once confirmed

const uint32_t DUR_BALANCE    = 3000;
const uint32_t DUR_FORWARD    = 2000;
const uint32_t DUR_TURN_LEFT  = 2000;
const uint32_t DUR_TURN_RIGHT = 2000;
const uint32_t DUR_BACKWARD   = 2000;

float MoveTilt = 0.0;
float turnBias  = 0.0;

void update_move_state() {
    uint32_t elapsed = millis() - stateStartMillis;

    switch (currentState) {
        case STATE_BALANCE:
            MoveTilt = 0.0;
            turnBias = 0.0;
            isMoving = false;
            if (elapsed > DUR_BALANCE) {
                currentState = STATE_FORWARD;
                stateStartMillis = millis();
                Serial.println(">>> FORWARD");
            }
            break;

        case STATE_FORWARD:
            MoveTilt = FORWARD_STEP;
            turnBias = 0.0;
            isMoving = true;
            if (elapsed > DUR_FORWARD) {
                MoveTilt = 0.0;
                currentState = STATE_TURN_LEFT;
                stateStartMillis = millis();
                Serial.println(">>> TURN LEFT");
            }
            break;

        case STATE_TURN_LEFT:
            MoveTilt = 0.0;
            turnBias = -TURN_STEP;   // was +TURN_STEP -- swapped per your
                                      // confirmed observation (was inverted)
            isMoving = false;
            if (elapsed > DUR_TURN_LEFT) {
                turnBias = 0.0;
                currentState = STATE_TURN_RIGHT;
                stateStartMillis = millis();
                Serial.println(">>> TURN RIGHT");
            }
            break;

        case STATE_TURN_RIGHT:
            MoveTilt = 0.0;
            turnBias = TURN_STEP;    // was -TURN_STEP -- swapped per your
                                      // confirmed observation (was inverted)
            isMoving = false;
            if (elapsed > DUR_TURN_RIGHT) {
                turnBias = 0.0;
                currentState = STATE_BACKWARD;
                stateStartMillis = millis();
                Serial.println(">>> BACKWARD");
            }
            break;

        case STATE_BACKWARD:
            MoveTilt = BACKWARD_STEP;
            turnBias = 0.0;
            isMoving = true;
            if (elapsed > DUR_BACKWARD) {
                MoveTilt = 0.0;
                currentState = STATE_DONE;
                stateStartMillis = millis();
                Serial.println(">>> DONE -- holding balance");
            }
            break;

        case STATE_DONE:
            MoveTilt = 0.0;
            turnBias = 0.0;
            isMoving = false;
            break;
    }
}

// ================================================================
//  IMU READ -- unchanged
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
//  MOTOR HELPERS -- unchanged
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

static uint32_t lastPrint       = 0;
static uint32_t lastPrintFallen = 0;

// ================================================================
//  SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("balancebot_direction_test_v1 starting...");
    Serial.println("No HuskyLens -- pure open-loop direction test.");

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
    Serial.println("Sequence starting: BALANCE -> FORWARD -> LEFT -> RIGHT -> BACKWARD");

    stateStartMillis = millis();
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
        AutoAngleOffset = 0.0;
        MoveTilt = 0.0;
        turnBias = 0.0;

        if (millis() - lastPrintFallen > 500) {
            lastPrintFallen = millis();
            Serial.print("FALLEN | angle=");
            Serial.println(KalmanAnglePitch);
        }
        while (micros() - LoopTimer < (uint32_t)(dt * 1000000));
        LoopTimer = micros();
        return;
    }

    update_move_state();
    update_velocity_and_drift();

    // --- Balance PID ------------------------------------------
    float effectiveTarget = DesiredAnglePitch + AutoAngleOffset + MoveTilt;
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

    // --- Motor output -- CORRECTED formula ------------------------
    // pwmLeft now uses +turnBias (was -turnBias), fixing the
    // degeneracy that made turning impossible. See header comment
    // for the full derivation.
    int motorCmd = (int)(MotorOutput);
    int pwmLeft  = -motorCmd + (int)turnBias;
    int pwmRight =  motorCmd + (int)turnBias;
    set_motor_left(pwmLeft);
    set_motor_right(pwmRight);

    // --- Debug ---------------------------------------------------
    if (millis() - lastPrint > 100) {
        lastPrint = millis();
        Serial.print("State: ");    Serial.print(currentState);
        Serial.print(" | Angle: "); Serial.print(KalmanAnglePitch, 2);
        Serial.print(" | Target: ");Serial.print(effectiveTarget, 2);
        Serial.print(" | Tilt: ");  Serial.print(MoveTilt, 1);
        Serial.print(" | Turn: ");  Serial.print(turnBias, 1);
        Serial.print(" | PWM_L: "); Serial.print(pwmLeft);
        Serial.print(" | PWM_R: "); Serial.println(pwmRight);
    }

    while (micros() - LoopTimer < (uint32_t)(dt * 1000000));
    LoopTimer = micros();
}
