/*  ============================================================
    balancebot_PD_v2.ino  — with directional movement sequence
    
    Added on top of your existing code:
    - moveState enum for sequence control
    - DesiredAnglePitch offset for forward/backward
    - turnBias offset for left/right differential
    - Timed state machine: FWD → LEFT → RIGHT → BWD → BALANCE
    
    Core PID, IMU, Kalman, motor helpers — all unchanged.
    ============================================================ */

#include <Wire.h>
#include <math.h>

// ─── Pins — unchanged ───────────────────────────────────────────
#define CUSTOM_SDA_PIN  6
#define CUSTOM_SCL_PIN  7
#define PIN_M1A         5
#define PIN_M1B         10
#define PIN_M2A         3
#define PIN_M2B         4

#define PWM_FREQ        20000
#define PWM_RESOLUTION  8
#define MPU6050_ADDRESS 0x68

// ─── Timing — unchanged ─────────────────────────────────────────
const float dt = 0.005;
uint32_t LoopTimer;

// ─── IMU — unchanged ────────────────────────────────────────────
int16_t AccXLSB, AccYLSB, AccZLSB;
int16_t GyroXLSB, GyroYLSB, GyroZLSB;
float AccX, AccY, AccZ;
float RateRoll, RatePitch, RateYaw;
float AnglePitch;

// ─── Calibration — unchanged ────────────────────────────────────
float RateCalibrationRoll  = -3.57;
float RateCalibrationPitch = -1.08;
float RateCalibrationYaw   =  0.74;
float AccXCalibration      =  0.00;
float AccYCalibration      =  0.01;
float AccZCalibration      = -0.03;

// ─── Kalman — unchanged ─────────────────────────────────────────
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

// ─── PID — unchanged ────────────────────────────────────────────
float P_gain = 25.0;
float I_gain = 20;
float D_gain = 0.05;

float DesiredAnglePitch = 0.0;  // modified by movement state

float Error       = 0;
float PrevError   = 0;
float PrevIterm   = 0;
float MotorOutput = 0;
float filteredDterm      = 0;
const float DTERM_FILTER_ALPHA = 0.80;

// ─── Safety ─────────────────────────────────────────────────────
const float FALL_ANGLE_DEG = 35.0;

// ─── Serial ─────────────────────────────────────────────────────
static uint32_t lastPrint       = 0;
static uint32_t lastPrintFallen = 0;

// ================================================================
//  MOVEMENT STATE MACHINE  ← NEW
// ================================================================
// Analogy: think of this like a traffic light sequence.
// Each state has a fixed duration. When time expires, next state.
// The balance PID runs every loop regardless — the state machine
// only changes the INPUTS to the PID (angle setpoint + turn bias).
// ================================================================

enum MoveState {
    STATE_BALANCE,    // just balance, no movement
    STATE_FORWARD,    // lean forward
    STATE_TURN_LEFT,  // differential: right faster than left
    STATE_TURN_RIGHT, // differential: left faster than right
    STATE_BACKWARD,   // lean backward
    STATE_DONE        // finished, hold balance
};

MoveState currentState     = STATE_BALANCE;
uint32_t  stateStartMillis = 0;

// Tune these to taste:
const float FORWARD_ANGLE_OFFSET  =  6;  // degrees — positive = lean forward
const float BACKWARD_ANGLE_OFFSET = -6;  // degrees — negative = lean backward
const float TURN_BIAS             =  60;   // PWM units added/subtracted per side

// Duration of each movement phase (ms)
const uint32_t DUR_BALANCE   = 3000;  // 3s settle before starting
const uint32_t DUR_FORWARD   = 2000;  // 2s forward
const uint32_t DUR_TURN_LEFT = 2000;  // 2s turn left
const uint32_t DUR_TURN_RIGHT= 2000;  // 2s turn right
const uint32_t DUR_BACKWARD  = 2000;  // 2s backward

// Active turn bias this loop (set by state machine, applied at motor output)
float turnBias = 0.0;

void update_move_state() {
    uint32_t elapsed = millis() - stateStartMillis;

    switch (currentState) {

        case STATE_BALANCE:
            DesiredAnglePitch = 0.0;
            turnBias          = 0.0;
            if (elapsed > DUR_BALANCE) {
                currentState     = STATE_FORWARD;
                stateStartMillis = millis();
                Serial.println(">>> FORWARD");
            }
            break;

        case STATE_FORWARD:
            DesiredAnglePitch = FORWARD_ANGLE_OFFSET;
            turnBias          = 0.0;
            if (elapsed > DUR_FORWARD) {
                DesiredAnglePitch = 0.0;  // snap back to balance first
                currentState      = STATE_TURN_LEFT;
                stateStartMillis  = millis();
                Serial.println(">>> TURN LEFT");
            }
            break;

        case STATE_TURN_LEFT:
            // Left turn: right motor gets +bias, left motor gets -bias
            // Analogy: like a tank — one track faster, one slower
            DesiredAnglePitch = 0.0;
            turnBias          = TURN_BIAS;   // applied as +right, -left below
            if (elapsed > DUR_TURN_LEFT) {
                turnBias         = 0.0;
                currentState     = STATE_TURN_RIGHT;
                stateStartMillis = millis();
                Serial.println(">>> TURN RIGHT");
            }
            break;

        case STATE_TURN_RIGHT:
            // Right turn: left motor gets +bias, right motor gets -bias
            DesiredAnglePitch = 0.0;
            turnBias          = -TURN_BIAS;  // negative = opposite direction
            if (elapsed > DUR_TURN_RIGHT) {
                turnBias         = 0.0;
                currentState     = STATE_BACKWARD;
                stateStartMillis = millis();
                Serial.println(">>> BACKWARD");
            }
            break;

        case STATE_BACKWARD:
            DesiredAnglePitch = BACKWARD_ANGLE_OFFSET;
            turnBias          = 0.0;
            if (elapsed > DUR_BACKWARD) {
                DesiredAnglePitch = 0.0;
                currentState      = STATE_DONE;
                stateStartMillis  = millis();
                Serial.println(">>> DONE — holding balance");
            }
            break;

        case STATE_DONE:
            DesiredAnglePitch = 0.0;
            turnBias          = 0.0;
            break;
    }
}

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
//  SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("balancebot_PD_v2 + movement starting...");

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

    Serial.println("Lift robot, watch angle for 3s...");
    delay(3000);
    Serial.println("Balancing active. Sequence starts in 3s.");

    stateStartMillis = millis();
    LoopTimer = micros();
}

// ================================================================
//  MAIN LOOP — 200Hz
// ================================================================
void loop() {
    // ── 1. IMU ─────────────────────────────────────────────────
    read_imu();

    // ── 2. Kalman — unchanged ───────────────────────────────────
    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch,
              RatePitch, AnglePitch);
    KalmanAnglePitch            = Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
    KalmanAnglePitch = constrain_float(KalmanAnglePitch, -45, 45);

    // ── 3. Safety — unchanged ───────────────────────────────────
    if (fabs(KalmanAnglePitch) > FALL_ANGLE_DEG) {
        motors_stop();
        PrevError = 0; PrevIterm = 0; filteredDterm = 0;
        if (millis() - lastPrintFallen > 500) {
            lastPrintFallen = millis();
            Serial.print("FALLEN | angle=");
            Serial.println(KalmanAnglePitch);
        }
        while (micros() - LoopTimer < (uint32_t)(dt * 1000000));
        LoopTimer = micros();
        return;
    }

    // ── 4. Update movement state machine  ← NEW ────────────────
    update_move_state();

    // ── 5. PID — unchanged ─────────────────────────────────────
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

    // ── 6. Apply motor output + turn bias  ← NEW ───────────────
    // turnBias > 0 = turn left  (right faster, left slower)
    // turnBias < 0 = turn right (left faster, right slower)
    int pwmLeft  = (int)constrain_float(MotorOutput - turnBias, -255, 255);
    int pwmRight = (int)constrain_float(MotorOutput + turnBias, -255, 255);

    set_motor_left(pwmLeft);
    set_motor_right(pwmRight);

    // ── 7. Debug ────────────────────────────────────────────────
    if (millis() - lastPrint > 100) {
        lastPrint = millis();
        Serial.print("State: "); Serial.print(currentState);
        Serial.print(" | Angle: ");  Serial.print(KalmanAnglePitch);
        Serial.print(" | Setpt: ");  Serial.print(DesiredAnglePitch);
        Serial.print(" | PWM L: ");  Serial.print(pwmLeft);
        Serial.print(" | PWM R: ");  Serial.println(pwmRight);
    }

    // ── 8. Loop timing — unchanged ──────────────────────────────
    while (micros() - LoopTimer < (uint32_t)(dt * 1000000));
    LoopTimer = micros();
}