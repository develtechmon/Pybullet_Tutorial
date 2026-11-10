/*  ============================================================
    balancebot_stepper_v2_balance_only.ino

    GOAL: just balance. No forward/backward/turn sequence yet.

    Board  : ESP32-S3
    IMU    : MPU6050 on GPIO6 (SDA) / GPIO7 (SCL)
    Right  : STEP=5  DIR=4   (A4988, 1/16 microstep, MS1/2/3 = HIGH)
    Left   : STEP=9  DIR=8

    ─── HOW THE MOTORS ARE DRIVEN ─────────────────────────────
    Analogy: it's like a drummer keeping the beat.
    A hardware timer fires an interrupt 20,000 times per second.
    Each motor has a number "periodX" that means:
    "fire one step pulse every periodX ticks".
      periodX = 10  ->  20000/10 = 2000 steps per second
      periodX = 0   ->  motor stopped (still holds position)
    The 200Hz balance loop ONLY updates periodX. The interrupt
    (the drummer) does the actual stepping in the background,
    while the main loop (the conductor) changes the tempo.

    ─── PID OUTPUT MEANING ────────────────────────────────────
    Different from DC version! This bit trips people up:
      DC bot   : PID output = PWM  (-255..255)  = torque
      This bot : PID output = SPEED in steps/s  (about -6000..6000)
    Your old gains from the DC bot do NOT apply here.
    ============================================================ */

#include <Wire.h>
#include <math.h>

// ─── Pins ───────────────────────────────────────────────────────
#define CUSTOM_SDA_PIN  7
#define CUSTOM_SCL_PIN  6

#define STEP_PIN_R      5
#define DIR_PIN_R       4
#define STEP_PIN_L      9
#define DIR_PIN_L       8

#define MPU6050_ADDRESS 0x68

// Motors are usually mounted mirrored (facing opposite ways).
// If one wheel spins the wrong way in the tilt test, flip its flag.
const bool INVERT_LEFT  = true;
const bool INVERT_RIGHT = false;

// ─── Timing ─────────────────────────────────────────────────────
const float dt = 0.005;          // 200Hz balance loop (same as DC bot)
uint32_t LoopTimer;

// ─── Stepper settings ───────────────────────────────────────────
#define TICK_HZ  20000           // metronome speed: 20,000 ticks/s
                                 // = one ISR fire every 50µs

// MAX_SPEED_SPS = 6000 sps at 1/16 microstepping
//   Math: 6000 / 3200 microsteps per rev = 1.875 rev/s = 112 RPM
//   NEMA 17 stop-start rate from datasheet is 400-700 full-steps/s.
//   Converted to microsteps: 6400-11200 sps.
//   Our 6000 sits at the safe low end — motor can reach this from
//   rest without needing a ramp. Higher values (~10000) possible
//   but require more careful acceleration control.
const float MAX_SPEED_SPS  = 6000.0;

// MIN_SPEED_SPS = 15 sps threshold. Below this: stop the motor.
// Very slow pulses cause resonance vibration and waste energy.
// Better to stop entirely — stepper still holds position via
// coil current, unlike DC motors which spin down.
const float MIN_SPEED_SPS  = 15.0;

// MAX_ACCEL_SPS2 = 40,000 sps² (steps per second, per second)
//   = how much speed can CHANGE in one full second
//   = 40,000 × dt = 40,000 × 0.005 = 200 sps CHANGE per 5ms loop
//
// Analogy: like a car's throttle response limit. Even if the driver
// (PID) slams the pedal to the floor, the engine won't jump from
// 0 to 6000 RPM instantly. This value says "increase speed by max
// 200 sps per loop tick."
//
// Chosen empirically — community-proven value for NEMA 17 balancing
// robots (matches zjor's Hackaday build at 40kHz). Theoretical max
// is much higher (~13 million sps²) but load, friction, and coil
// current buildup drop practical values to 20k-80k.
//
// TUNING GUIDE:
//   Motor grinding + going limp mid-correction = skipped steps
//     → LOWER this value (try 30000, then 20000)
//   Robot feels sluggish, can't catch tilts fast enough
//     → RAISE this (try 50000, watch for stalls)
const float MAX_ACCEL_SPS2 = 40000;

// ─── IMU (unchanged from DC bot) ────────────────────────────────
int16_t AccXLSB, AccYLSB, AccZLSB;
int16_t GyroXLSB, GyroYLSB, GyroZLSB;
float AccX, AccY, AccZ;
float RateRoll, RatePitch, RateYaw;
float AnglePitch;

// ─── Calibration — RE-RUN these on the NEW chassis! ─────────────
// Every chassis has its own IMU mounting orientation and slight
// gyro drift. Values below are for THIS specific robot.
float RateCalibrationRoll  = -4.93;
float RateCalibrationPitch =  0.93;
float RateCalibrationYaw   = -0.70;
float AccXCalibration      =  0.04;
float AccYCalibration      = -0.04;
float AccZCalibration      =  0.05;

// ─── Kalman filter (unchanged from DC bot) ──────────────────────
// Analogy: it's like combining two witnesses' statements about an
// event. The accelerometer is a reliable but jittery witness (gives
// absolute angle but noisy). The gyroscope is a steady but drifting
// witness (smooth changes but accumulates error over time). Kalman
// weighs them intelligently to get the best estimate.
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

// ─── PID — output is now steps/s ────────────────────────────────
// Starting points (NOT your DC gains — different units entirely):
//   P_gain : steps/s per degree of tilt error
//   I_gain : steady bias correction (drift compensation)
//   D_gain : steps/s per deg/s of gyro rate (damping — brakes)
//
// Your gains below are HIGH because the slew limiter caps most
// of the output anyway. What actually reaches the motor is the
// slew-limited version, not the raw PID output.
float P_gain = 1800.0;
float I_gain = 1000;
float D_gain = 5;

// Mechanical balance point of THIS chassis. Your DC bot liked
// -1.3 to -2.3 deg. This one is different because of stepper motor
// mounting, wheel size, and weight distribution. Tune until the
// robot stops creeping in one direction while balanced.
float AngleOffset = 0.0;

float Error       = 0;
float PrevError   = 0;
float PrevIterm   = 0;
float MotorOutput = 0;    // PID result, in steps/s
float SpeedCmd    = 0;    // slew-limited speed actually sent to motors
float filteredDterm = 0;
const float DTERM_FILTER_ALPHA = 0.5;

// ─── Safety ─────────────────────────────────────────────────────
const float FALL_ANGLE_DEG = 35.0;
static uint32_t lastPrint       = 0;
static uint32_t lastPrintFallen = 0;

// ================================================================
//  THE METRONOME — 20kHz timer interrupt
// ================================================================
// volatile = "this variable is shared between the interrupt and
// the main loop, never cache it" — like a whiteboard both the
// drummer and the conductor can see.
hw_timer_t* stepTimer = NULL;

volatile uint32_t periodL = 0;   // ticks between steps, 0 = stopped
volatile uint32_t periodR = 0;
volatile uint32_t tickCountL = 0;
volatile uint32_t tickCountR = 0;
volatile bool stepPinHighL = false;
volatile bool stepPinHighR = false;

void IRAM_ATTR onStepTick()
{
    // ---- LEFT motor ----
    // If we raised the pin last tick, lower it now.
    // (This makes every step pulse 50µs wide — the A4988 only
    //  needs 1µs, so plenty of margin.)
    if (stepPinHighL) {
        digitalWrite(STEP_PIN_L, LOW);
        stepPinHighL = false;
    }
    if (periodL > 0) {
        tickCountL = tickCountL + 1;
        if (tickCountL >= periodL) {
            tickCountL = 0;
            digitalWrite(STEP_PIN_L, HIGH);
            stepPinHighL = true;
        }
    } else {
        tickCountL = 0;
    }

    // ---- RIGHT motor ----
    if (stepPinHighR) {
        digitalWrite(STEP_PIN_R, LOW);
        stepPinHighR = false;
    }
    if (periodR > 0) {
        tickCountR = tickCountR + 1;
        if (tickCountR >= periodR) {
            tickCountR = 0;
            digitalWrite(STEP_PIN_R, HIGH);
            stepPinHighR = true;
        }
    } else {
        tickCountR = 0;
    }
}

// ================================================================
//  SPEED SETTERS — replace set_motor_left / set_motor_right
//  Input: speed in steps/s. Positive = robot forward.
//
//  Analogy: like a translator between two languages.
//  The main loop speaks "steps per second" (easy for humans).
//  The metronome speaks "ticks between pulses" (easy for timers).
//  This function converts one to the other.
// ================================================================
float constrain_float(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

void set_speed_left(float sps)
{
    bool forward;
    float speed;

    if (sps >= 0) {
        forward = true;
        speed = sps;
    } else {
        forward = false;
        speed = -sps;
    }

    if (INVERT_LEFT) {
        forward = !forward;   // flip because motor is mounted mirrored
    }

    if (speed < MIN_SPEED_SPS) {
        periodL = 0;          // stop (stepper still holds position)
        return;
    }
    if (speed > MAX_SPEED_SPS) {
        speed = MAX_SPEED_SPS;
    }

    if (forward) {
        digitalWrite(DIR_PIN_L, HIGH);
    } else {
        digitalWrite(DIR_PIN_L, LOW);
    }

    // speed -> ticks between steps. Example: 2000 sps -> 10 ticks.
    // Math: 20,000 ticks per second / 2000 steps per second = 10.
    uint32_t p = (uint32_t)(TICK_HZ / speed);
    if (p < 2) {
        p = 2;                // never faster than 1 step per 2 ticks
    }
    periodL = p;
}

void set_speed_right(float sps)
{
    bool forward;
    float speed;

    if (sps >= 0) {
        forward = true;
        speed = sps;
    } else {
        forward = false;
        speed = -sps;
    }

    if (INVERT_RIGHT) {
        forward = !forward;
    }

    if (speed < MIN_SPEED_SPS) {
        periodR = 0;
        return;
    }
    if (speed > MAX_SPEED_SPS) {
        speed = MAX_SPEED_SPS;
    }

    if (forward) {
        digitalWrite(DIR_PIN_R, HIGH);
    } else {
        digitalWrite(DIR_PIN_R, LOW);
    }

    uint32_t p = (uint32_t)(TICK_HZ / speed);
    if (p < 2) {
        p = 2;
    }
    periodR = p;
}

void motors_stop()
{
    periodL = 0;
    periodR = 0;
}

// ================================================================
//  IMU READ — unchanged from DC bot
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
//  SETUP
// ================================================================
void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("balancebot_stepper_v2 (balance only) starting...");

    pinMode(STEP_PIN_R, OUTPUT);
    pinMode(DIR_PIN_R, OUTPUT);
    pinMode(STEP_PIN_L, OUTPUT);
    pinMode(DIR_PIN_L, OUTPUT);
    digitalWrite(STEP_PIN_R, LOW);
    digitalWrite(STEP_PIN_L, LOW);

    Wire.setClock(400000);
    Wire.begin(CUSTOM_SDA_PIN, CUSTOM_SCL_PIN);
    delay(250);

    // Wake up MPU6050
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(100);

    // Set accelerometer range: ±8g
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();

    // Set gyro range: ±500°/s
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();
    delay(100);

    // Start the 20kHz metronome (ESP32 Arduino core 3.x API)
    stepTimer = timerBegin(1000000);               // timer counts in microseconds
    timerAttachInterrupt(stepTimer, &onStepTick);
    timerAlarm(stepTimer, 50, true, 0);            // fire every 50µs = 20kHz

    motors_stop();
    Serial.println("Hold robot upright. Balancing starts in 3s...");
    delay(3000);

    LoopTimer = micros();
}

// ================================================================
//  MAIN LOOP — 200Hz, same skeleton as your DC bot
// ================================================================
void loop()
{
    // ── 1. IMU ──────────────────────────────────────────────────
    read_imu();

    // ── 2. Kalman — unchanged ───────────────────────────────────
    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch,
              RatePitch, AnglePitch);
    KalmanAnglePitch            = Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
    KalmanAnglePitch = constrain_float(KalmanAnglePitch, -45, 45);

    // ── 3. Safety — same as DC bot ──────────────────────────────
    // If we've fallen past recovery angle, stop everything and
    // reset PID state so we don't lurch when picked back up.
    if (fabs(KalmanAnglePitch) > FALL_ANGLE_DEG) {
        motors_stop();
        PrevError = 0;
        PrevIterm = 0;
        filteredDterm = 0;
        SpeedCmd = 0;
        if (millis() - lastPrintFallen > 500) {
            lastPrintFallen = millis();
            Serial.print("FALLEN | angle=");
            Serial.println(KalmanAnglePitch);
        }
        while (micros() - LoopTimer < (uint32_t)(dt * 1000000));
        LoopTimer = micros();
        return;
    }

    // ── 4. PID — same structure, output is now steps/s ──────────
    Error = AngleOffset - KalmanAnglePitch;

    float Pterm = P_gain * Error;

    // Trapezoidal integration, clamped to prevent windup
    float Iterm = PrevIterm + I_gain * (Error + PrevError) * dt / 2.0;
    Iterm = constrain_float(Iterm, -500, 500);

    // D term with low-pass filter to reduce gyro noise sensitivity
    // Analogy: it's like adding shock absorbers to a car — smooths
    // out the bumpy derivative signal so PID doesn't chase noise.
    float DtermRaw = D_gain * RatePitch;
    filteredDterm  = DTERM_FILTER_ALPHA * DtermRaw
                   + (1.0 - DTERM_FILTER_ALPHA) * filteredDterm;

    MotorOutput = constrain_float(Pterm + Iterm - filteredDterm,
                                  -MAX_SPEED_SPS, MAX_SPEED_SPS);

    PrevError = Error;
    PrevIterm = Iterm;

    // ── 5. Slew limit — the stepper's stall insurance ───────────
    // Analogy: a car's traction control. Even if the driver (PID)
    // stomps the throttle, the ECU (this code) limits how fast
    // wheel speed can change to prevent wheelspin. Same idea here:
    // PID may demand big speed changes, but we ramp them smoothly
    // so the stepper doesn't skip steps.
    //
    // Math for THIS code with MAX_ACCEL_SPS2 = 40000:
    //   maxDelta = 40,000 × 0.005 = 200 sps CHANGE per 5ms loop
    //
    // Example scenario: robot suddenly tilts, PID panics.
    //   SpeedCmd = 0 (currently stopped)
    //   MotorOutput = 6000 (PID: "GO MAX SPEED NOW!")
    //   diff = 6000 - 0 = 6000
    //   6000 > 200, so SpeedCmd becomes 0 + 200 = 200
    //   Next loop: SpeedCmd = 200 + 200 = 400
    //   Next loop: SpeedCmd = 400 + 200 = 600
    //   ... ramps smoothly at 200 sps per 5ms tick ...
    //   After 30 loops (150ms), SpeedCmd = 6000. Motor happy.
    //
    // Without this? PID commands 6000 sps to a resting motor,
    // motor tries to jump instantly, skips steps, robot falls.
    float maxDelta = MAX_ACCEL_SPS2 * dt;   // = 200 sps per 5ms loop
    float diff = MotorOutput - SpeedCmd;

    if (diff > maxDelta) {
        // PID wants big INCREASE — cap the change to +maxDelta
        SpeedCmd = SpeedCmd + maxDelta;
    } else if (diff < -maxDelta) {
        // PID wants big DECREASE — cap the change to -maxDelta
        SpeedCmd = SpeedCmd - maxDelta;
    } else {
        // Small change requested — take PID output as-is
        SpeedCmd = MotorOutput;
    }

    // ── 6. Send speed to both motors ────────────────────────────
    // SIGN TEST before trusting: hold robot in the air, tilt it
    // forward — wheels must spin as if driving forward (chasing
    // the fall). If they chase backward, change these two lines to
    // set_speed_left(-SpeedCmd) and set_speed_right(-SpeedCmd).
    // (Same check as your DC bot's pwmCmd = -MotorOutput finding.)
    set_speed_left(SpeedCmd);
    set_speed_right(-SpeedCmd);

    // ── 7. Debug ────────────────────────────────────────────────
    // Print every 100ms — Serial.print is slow, don't do it every
    // loop or you'll break the 200Hz timing.
    if (millis() - lastPrint > 100) {
        lastPrint = millis();
        Serial.print("Angle: ");
        Serial.print(KalmanAnglePitch, 2);
        Serial.print(" | Speed: ");
        Serial.print(SpeedCmd, 0);
        Serial.print(" | periodL: ");
        Serial.print(periodL);
        Serial.print(" | periodR: ");
        Serial.println(periodR);
    }

    // ── 8. Loop timing — busy-wait to hold exactly 5ms ──────────
    // Analogy: like a metronome for the main loop itself.
    // Ensures dt is truly 0.005s regardless of how fast the code ran.
    while (micros() - LoopTimer < (uint32_t)(dt * 1000000));
    LoopTimer = micros();
}
