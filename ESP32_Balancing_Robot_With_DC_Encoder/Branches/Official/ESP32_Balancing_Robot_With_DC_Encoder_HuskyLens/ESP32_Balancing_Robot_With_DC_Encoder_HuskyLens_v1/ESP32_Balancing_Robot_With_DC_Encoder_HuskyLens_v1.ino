/*  ============================================================
    balancebot_PD_encoder_husky_track_v1.ino

    Your working balance + drift correction code, with HuskyLens 2
    object tracking added on top. Robot follows a learned object:
    steers to keep it centered, moves forward if too far.

    PIN ASSIGNMENT:
    --------------------------------------------------------------
    Left motor  : M1A=8, M1B=9
    Right motor : M2A=13, M2B=12
    IMU         : SDA=7, SCL=6
    Left encoder  : Yellow(A)=10, Green(B)=11
    Right encoder : Yellow(A)=2,  Green(B)=3
    HuskyLens Serial1 : RX=5, TX=4

    ================================================================
    JITTER FIX IN THIS VERSION
    ================================================================
    Forward lean previously snapped instantly to a freshly computed
    value every 100ms, with zero smoothing -- HuskyLens's detected
    width naturally wobbles a few pixels frame to frame from normal
    detection noise, and that noise was being reflected directly and
    immediately into commanded lean. Release had smoothing (x0.90
    decay), engage didn't -- that asymmetry caused the jitter.

    Fixed: ONE consistent blend filter (TILT_SMOOTHING_ALPHA) now
    applies to both engaging and releasing, at the same rate the
    release side already proved comfortable.
    ============================================================ */

#include <Wire.h>
#include <math.h>
#include "DFRobot_HuskylensV2.h"

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

float P_gain = 40.0;
float I_gain = 100;
float D_gain = 0.4;

float DesiredAnglePitch = 2.52;

float Error       = 0;
float PrevError   = 0;
float PrevIterm   = 0;
float MotorOutput = 0;
float filteredDterm      = 0;
const float DTERM_FILTER_ALPHA = 0.80;

const float FALL_ANGLE_DEG = 35.0;

// ================================================================
//  ENCODER READING
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

// ================================================================
//  VELOCITY TRACKING
// ================================================================
long lastEncoderL = 0;
long lastEncoderR = 0;
float velocityL = 0.0;
float velocityR = 0.0;
float avgVelocity = 0.0;
unsigned long lastVelocityUpdate = 0;
const unsigned long VELOCITY_UPDATE_MS = 20;

// ================================================================
//  DRIFT CORRECTION
// ================================================================
const float DRIFT_CORRECTION_RATE = 0.02;
const float DRIFT_CORRECTION_LIMIT = 3.0;
const float VELOCITY_DEADBAND = 5.0;
float AutoAngleOffset = 0.0;

// ================================================================
//  HUSKYLENS OBJECT TRACKING
// ================================================================
HuskylensV2 huskylens;
HardwareSerial HuskylensSerial(1);
#define HUSKY_RX_PIN 5
#define HUSKY_TX_PIN 4

const int CAMERA_WIDTH  = 640;
const int CAMERA_HEIGHT = 480;
const int CENTER_X      = CAMERA_WIDTH / 2;   // = 320

const int TARGET_WIDTH        = 320;   // interim calibration, refine with real testing

const int DISTANCE_DEADBAND   = 20;
const int HORIZONTAL_DEADBAND = 30;

// --- Forward: PROPORTIONAL control + SPEED GOVERNOR + SMOOTHING ---
// FORWARD_KP: degrees of lean per pixel of (deadband-adjusted) size
// error. MAX_FORWARD_SPEED: encoder pulses/sec ceiling -- as the
// robot approaches this speed, allowed lean tapers to zero (speed
// governor), same idea as cruise control easing off the accelerator.
// TILT_SMOOTHING_ALPHA: blend rate applied to BOTH engaging and
// releasing -- this is the actual jitter fix (see header comment).
const float FORWARD_KP        = 0.2;
const float MAX_FORWARD_SPEED = 300.0;
const float TILT_SMOOTHING_ALPHA = 0.3;

// --- Turn: PROPORTIONAL control ------------------------------------
// Subtractive deadband: turnBias starts at 0 right at the deadband
// edge, grows smoothly from there. Sign confirmed via isolated
// direction test: negative turnBias = turn left.
const float TURN_KP  = 0.25;
const float TURN_MAX = 40.0;
const float TRACK_MAX_TILT = 8.0;

const unsigned long OBJECT_LOST_TIMEOUT_MS = 300;

float TrackingTilt = 0.0;
float turnBias      = 0.0;
bool  isActivelyTracking = false;
int   lastXError    = 0;
int   lastSizeError = 0;
unsigned long lastObjectSeenTime = 0;
bool  haveEverSeenObject = false;

unsigned long lastTrackingUpdate = 0;
const unsigned long TRACKING_UPDATE_MS = 50;

// ================================================================
//  IMU READ
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
//  VELOCITY UPDATE + DRIFT CORRECTION
// ================================================================
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

    if (!isActivelyTracking && fabs(avgVelocity) > VELOCITY_DEADBAND) {
        AutoAngleOffset -= avgVelocity * DRIFT_CORRECTION_RATE * elapsed;
        AutoAngleOffset = constrain_float(AutoAngleOffset,
                                           -DRIFT_CORRECTION_LIMIT,
                                            DRIFT_CORRECTION_LIMIT);
    }
}

// ================================================================
//  OBJECT TRACKING UPDATE
// ================================================================
void update_object_tracking() {
    unsigned long now = millis();
    if (now - lastTrackingUpdate < TRACKING_UPDATE_MS) return;
    lastTrackingUpdate = now;

    huskylens.getResult(ALGORITHM_OBJECT_TRACKING);

    bool objectVisible = false;

    if (huskylens.available(ALGORITHM_OBJECT_TRACKING)) {
        auto obj = huskylens.getCachedResultByID(ALGORITHM_OBJECT_TRACKING, 1);

        if (obj != NULL) {
            int x = RET_ITEM_NUM(obj, Result, xCenter);
            int w = RET_ITEM_NUM(obj, Result, width);

            lastObjectSeenTime = now;
            haveEverSeenObject = true;
            objectVisible = true;

            // --- Horizontal: proportional turn ---------------------
            int xError = x - CENTER_X;
            float rawTurnBias = 0.0;

            if (xError > HORIZONTAL_DEADBAND) {
                rawTurnBias = (float)(xError - HORIZONTAL_DEADBAND) * TURN_KP;
            } else if (xError < -HORIZONTAL_DEADBAND) {
                rawTurnBias = (float)(xError + HORIZONTAL_DEADBAND) * TURN_KP;
            }

            turnBias = constrain_float(rawTurnBias, -TURN_MAX, TURN_MAX);

            // --- Distance: proportional + speed governor + smoothing ---
            int sizeError = w - TARGET_WIDTH;
            float targetTilt = 0.0;

            if (sizeError < -DISTANCE_DEADBAND) {
                int adjustedError = -sizeError - DISTANCE_DEADBAND;
                float proportionalTilt = (float)adjustedError * FORWARD_KP;
                proportionalTilt = constrain_float(proportionalTilt, 0.0, TRACK_MAX_TILT);

                float speedRatio = avgVelocity / MAX_FORWARD_SPEED;
                speedRatio = constrain_float(speedRatio, 0.0, 1.0);
                float governor = 1.0 - speedRatio;

                targetTilt = proportionalTilt * governor;
            }
            // else: targetTilt stays 0.0 -- too close or good distance

            // Blend toward target -- SAME rate whether engaging or
            // releasing. This is the actual jitter fix.
            TrackingTilt = TrackingTilt + (targetTilt - TrackingTilt) * TILT_SMOOTHING_ALPHA;

            lastXError    = xError;
            lastSizeError = sizeError;
        }
    }

    bool lostNow = !objectVisible &&
                   haveEverSeenObject &&
                   (now - lastObjectSeenTime > OBJECT_LOST_TIMEOUT_MS);

    if (lostNow || !haveEverSeenObject) {
        turnBias     = 0.0;
        TrackingTilt = TrackingTilt * (1.0 - TILT_SMOOTHING_ALPHA);
    }

    TrackingTilt = constrain_float(TrackingTilt, -TRACK_MAX_TILT, TRACK_MAX_TILT);
    turnBias      = constrain_float(turnBias,      -TURN_MAX, TURN_MAX);

    isActivelyTracking = (fabs(TrackingTilt) > 0.5);
}

static uint32_t lastPrint       = 0;
static uint32_t lastPrintFallen = 0;

// ================================================================
//  SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("balancebot_PD_encoder_husky_track_v1 starting...");

    HuskylensSerial.begin(115200, SERIAL_8N1, HUSKY_RX_PIN, HUSKY_TX_PIN);
    Serial.println("Connecting to HuskyLens...");
    while (!huskylens.begin(HuskylensSerial)) {
        Serial.println("Huskylens not detected!");
        delay(1000);
    }
    huskylens.switchAlgorithm(ALGORITHM_OBJECT_TRACKING);
    Serial.println("HuskyLens connected. Learn an object on the device.");

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
    Serial.println("Balancing active. Drift correction + object tracking enabled.");

    LoopTimer = micros();
    lastVelocityUpdate = millis();
    lastTrackingUpdate = millis();
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
        TrackingTilt    = 0.0;
        turnBias        = 0.0;

        if (millis() - lastPrintFallen > 500) {
            lastPrintFallen = millis();
            Serial.print("FALLEN | angle=");
            Serial.println(KalmanAnglePitch);
        }
        while (micros() - LoopTimer < (uint32_t)(dt * 1000000));
        LoopTimer = micros();
        return;
    }

    update_object_tracking();
    update_velocity_and_drift();

    float effectiveTarget = DesiredAnglePitch + AutoAngleOffset + TrackingTilt;
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
    int pwmLeft  = -motorCmd + (int)turnBias;
    int pwmRight =  motorCmd + (int)turnBias;
    set_motor_left(pwmLeft);
    set_motor_right(pwmRight);

    if (millis() - lastPrint > 100) {
        lastPrint = millis();
        Serial.print("Angle: ");     Serial.print(KalmanAnglePitch, 2);
        Serial.print(" | Target: "); Serial.print(effectiveTarget, 2);
        Serial.print(" | Vel: ");    Serial.print(avgVelocity, 0);
        Serial.print(" | AutoOff: ");Serial.print(AutoAngleOffset, 3);
        Serial.print(" | TrackTilt: "); Serial.print(TrackingTilt, 2);
        Serial.print(" | TurnBias: ");  Serial.print(turnBias, 1);
        Serial.print(" | SizeErr: ");   Serial.print(lastSizeError);
        Serial.print(" | XErr: ");      Serial.print(lastXError);
        Serial.print(" | PWM_L: ");  Serial.print(pwmLeft);
        Serial.print(" | PWM_R: ");  Serial.println(pwmRight);
    }

    while (micros() - LoopTimer < (uint32_t)(dt * 1000000));
    LoopTimer = micros();
}