/*  ============================================================
    encoder_pid_test.ino  —  FINAL VERSION

    STANDALONE TEST — not for balancing.
    Purpose: verify encoders count correctly + PID can drive motor
    to move exactly 1 revolution based on encoder feedback.

    Motor spec: JGB37-520, 333 RPM, 30:1 gearbox, 11 PPR encoder
    Expected pulses per output revolution:
      11 PPR × 30 ratio × 2 (both edges of channel A) = 660

    Pin assignments (new PCB pinout):
    Left motor  : M1A=8, M1B=9
    Right motor : M2A=4, M2B=5
    Left enc    : A=10, B=11
    Right enc   : A=2,  B=3

    ─── HOW TO USE ────────────────────────────────────────────
    Open Serial Monitor at 115200 baud.

    Phase 1: Calibration (motors off)
      Rotate each wheel ONE full turn by hand.
      Watch EncL / EncR values in Serial Monitor.
      Should read ~660 (both should match).

    Phase 2: Position PID test
      Type 'L' + Enter → left motor drives 1 revolution and stops
      Type 'R' + Enter → right motor drives 1 revolution and stops
      Type 'C' + Enter → clear counters and stop
      Type 'S' + Enter → emergency stop

    ─── WHAT SUCCESS LOOKS LIKE ───────────────────────────────
    - Hand-rotation shows ~660 pulses per revolution (consistent)
    - Position PID stops within ±10 pulses of target (1.5% accuracy)
    - No oscillation once stopped
    - Both motors behave identically

    ─── TROUBLESHOOTING ───────────────────────────────────────
    Encoder count goes DOWN when you rotate forward:
      → ISR direction inverted. Swap ++ and -- in ISR.

    Motor spins the wrong way (encoder counts away from target):
      → Motor wires reversed. Swap M1A/M1B or M2A/M2B in your
        hardware wiring, OR swap in set_motor_left/right.

    Motor stalls, doesn't move (timeout):
      → MIN_MOVING_PWM too low for motor + load.
      → Try MIN_MOVING_PWM = 70 first.

    Motor overshoots badly:
      → Kd too low. Increase to 0.15.

    Motor hunts (jitters back and forth at target):
      → Deadband too tight, or Kp too high.
      → Increase DEADBAND to 15 pulses.
    ============================================================ */

#include <math.h>

// ─── Motor pins (updated for new PCB pinout) ───────────────────
#define PIN_M1A         8
#define PIN_M1B         9
#define PIN_M2A         4
#define PIN_M2B         5

#define PWM_FREQ        20000
#define PWM_RESOLUTION  8

// ─── Encoder pins (updated for new PCB pinout) ─────────────────
// Note: GPIO3 is a strapping pin on ESP32-S3 (JTAG signal select
// at boot). Using it as ENC_R_A is common and generally safe, but
// if you ever see odd boot behavior, this pin is the first suspect.
#define ENC_L_A         10
#define ENC_L_B         11
#define ENC_R_A         2
#define ENC_R_B         3

// ─── Loop timing ───────────────────────────────────────────────
// Fixed 5ms loop for consistent PID timing.
// D term calculations require known dt to be meaningful.
const float dt = 0.005;   // 200Hz
uint32_t loopTimer = 0;

// ================================================================
//  ENCODER READING
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
//  POSITION PID
//
//  Math for tuning:
//  Max PWM = 255. Max encoder error at start = 660 pulses (1 rev).
//  We want P term to saturate PWM early, then taper as we approach.
//
//  Kp = 0.3: at start (error=660), P = 0.3 × 660 = 198 → saturates
//  When error drops to 100 pulses: P = 0.3 × 100 = 30 PWM (gentle)
//
//  D term uses time-normalized derivative (per second, not per loop).
//  This makes Kd values meaningful regardless of loop rate.
//
//  Analogy:
//  P is the accelerator: how hard to push toward target.
//  D is the brake: how much to slow down when approaching fast.
//  Too much accelerator → fly past the target.
//  Too little brake → also fly past.
//  Balance: enough acceleration to get there, enough brake to stop.
// ================================================================
float Kp = 0.3;    // PWM per pulse of error
float Kd = 0.05;   // PWM per (pulse per second) of error rate

// Deadband — stop when this close to target.
// 10 pulses out of 660 = ~1.5% precision. Tight enough for balance.
const long DEADBAND = 10;

// Minimum PWM to overcome static friction.
// Motors buzz below this without moving. Only applied when we
// still have significant error (outside deadband).
const int MIN_MOVING_PWM = 50;

// Test state
bool testActive = false;
char activeMotor = 'N';
long targetCount = 0;
long prevError = 0;
unsigned long testStartTime = 0;
const unsigned long TEST_TIMEOUT_MS = 5000;

// Config
long targetRevolutions = 1;
long pulsesPerRev = 660;   // 11 PPR × 30 ratio × 2 edges

// ================================================================
//  RUN POSITION PID (called every 5ms while test active)
// ================================================================
void run_position_pid() {
    if (!testActive) return;

    // Read current count for active motor (atomic read)
    long currentCount;
    noInterrupts();
    if (activeMotor == 'L') {
        currentCount = encoderCountL;
    } else {
        currentCount = encoderCountR;
    }
    interrupts();

    long error = targetCount - currentCount;

    // Check if arrived at target
    if (abs(error) < DEADBAND) {
        motors_stop();
        testActive = false;

        Serial.println();
        Serial.println("─── TEST RESULT ─────────────────────────");
        Serial.print("Motor tested  : "); Serial.println(activeMotor);
        Serial.print("Target count  : "); Serial.println(targetCount);
        Serial.print("Actual count  : "); Serial.println(currentCount);
        Serial.print("Error         : "); Serial.print(error);
        Serial.println(" pulses");
        Serial.print("Accuracy      : ");
        Serial.print(100.0 - (100.0 * abs(error) / targetCount), 2);
        Serial.println(" %");
        Serial.print("Time taken    : ");
        Serial.print(millis() - testStartTime); Serial.println(" ms");
        Serial.println("─────────────────────────────────────────");
        Serial.println();
        Serial.println("Ready. Send L, R, or C.");
        return;
    }

    // Check timeout — safety guard
    if (millis() - testStartTime > TEST_TIMEOUT_MS) {
        motors_stop();
        testActive = false;
        Serial.println();
        Serial.println("!!! TIMEOUT !!!");
        Serial.print("Reached count: "); Serial.println(currentCount);
        Serial.print("Target was:    "); Serial.println(targetCount);
        Serial.println("Possible causes:");
        Serial.println(" - Motor direction wrong (running away from target)");
        Serial.println(" - MIN_MOVING_PWM too low for motor + load");
        Serial.println(" - Encoder wiring or ISR sign inverted");
        Serial.println();
        return;
    }

    // ── Compute PID ─────────────────────────────────────────
    // P term: proportional to how far away we are
    float Pterm = Kp * error;

    // D term: how fast the error is changing (per second)
    // Divide by dt so Kd stays meaningful regardless of loop rate.
    float errorRate = (float)(error - prevError) / dt;
    float Dterm = Kd * errorRate;
    prevError = error;

    // Combined PID output
    float pidOutput = Pterm + Dterm;

    // Clamp to PWM range
    int pwm = (int)pidOutput;
    if (pwm > 255)  pwm = 255;
    if (pwm < -255) pwm = -255;

    // Apply minimum-moving-PWM to overcome static friction.
    // Without this, motor buzzes but doesn't move for small errors.
    if (pwm > 0 && pwm < MIN_MOVING_PWM) pwm = MIN_MOVING_PWM;
    if (pwm < 0 && pwm > -MIN_MOVING_PWM) pwm = -MIN_MOVING_PWM;

    // Drive the active motor
    if (activeMotor == 'L') {
        set_motor_left(pwm);
    } else {
        set_motor_right(pwm);
    }
}

// ================================================================
//  CALIBRATION DISPLAY — shows encoder counts when idle
// ================================================================
unsigned long lastDisplayTime = 0;
long lastDisplayL = 0;
long lastDisplayR = 0;

void calibration_display() {
    if (testActive) return;

    unsigned long now = millis();
    if (now - lastDisplayTime < 200) return;  // 5Hz update
    lastDisplayTime = now;

    noInterrupts();
    long currentL = encoderCountL;
    long currentR = encoderCountR;
    interrupts();

    // Only print if something changed (reduces spam)
    if (currentL != lastDisplayL || currentR != lastDisplayR) {
        Serial.print("EncL: "); Serial.print(currentL);
        Serial.print(" | EncR: "); Serial.println(currentR);
        lastDisplayL = currentL;
        lastDisplayR = currentR;
    }
}

// ================================================================
//  START A MOTOR TEST
//  Clears state cleanly so we don't carry over from previous test.
// ================================================================
void start_test(char motor) {
    if (testActive) {
        Serial.println("Test already running. Wait or send S to stop.");
        return;
    }

    // Zero the correct encoder
    noInterrupts();
    if (motor == 'L') {
        encoderCountL = 0;
    } else {
        encoderCountR = 0;
    }
    interrupts();

    // Reset PID state
    targetCount = pulsesPerRev * targetRevolutions;
    activeMotor = motor;
    testActive = true;
    prevError = targetCount;  // avoid huge derivative spike on first loop
    testStartTime = millis();

    Serial.println();
    Serial.print("─── STARTING ");
    Serial.print(motor == 'L' ? "LEFT" : "RIGHT");
    Serial.println(" MOTOR TEST ────────────");
    Serial.print("Target: "); Serial.print(targetCount);
    Serial.print(" pulses ("); Serial.print(targetRevolutions);
    Serial.println(" revolution)");
    Serial.print("Kp="); Serial.print(Kp, 3);
    Serial.print(" Kd="); Serial.println(Kd, 3);
    Serial.println("─────────────────────────────────────────");
}

// ================================================================
//  SERIAL COMMAND HANDLER
// ================================================================
void process_serial() {
    if (!Serial.available()) return;

    char c = Serial.read();
    // Discard rest of line (newline chars etc.)
    while (Serial.available()) Serial.read();

    if (c == 'L' || c == 'l') {
        start_test('L');
    }
    else if (c == 'R' || c == 'r') {
        start_test('R');
    }
    else if (c == 'C' || c == 'c') {
        noInterrupts();
        encoderCountL = 0;
        encoderCountR = 0;
        interrupts();
        motors_stop();
        testActive = false;
        Serial.println();
        Serial.println("Counters cleared. Motors stopped.");
        Serial.println("Rotate wheels by hand to test encoders.");
    }
    else if (c == 'S' || c == 's') {
        motors_stop();
        testActive = false;
        Serial.println("EMERGENCY STOP.");
    }
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println();
    Serial.println("╔═══════════════════════════════════════════╗");
    Serial.println("║  Encoder + Position PID Test              ║");
    Serial.println("╚═══════════════════════════════════════════╝");
    Serial.println();

    // Motor PWM
    ledcAttach(PIN_M1A, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_M1B, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_M2A, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_M2B, PWM_FREQ, PWM_RESOLUTION);
    motors_stop();

    // Encoder pins with pullups
    pinMode(ENC_L_A, INPUT_PULLUP);
    pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP);
    pinMode(ENC_R_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC_L_A), encoderISR_L, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), encoderISR_R, CHANGE);

    Serial.println("SETUP:");
    Serial.print("  Pulses per revolution: "); Serial.println(pulsesPerRev);
    Serial.print("  Target revolutions   : "); Serial.println(targetRevolutions);
    Serial.print("  Target pulse count   : ");
    Serial.println(pulsesPerRev * targetRevolutions);
    Serial.print("  Kp = "); Serial.print(Kp, 3);
    Serial.print("   Kd = "); Serial.println(Kd, 3);
    Serial.print("  Deadband = "); Serial.print(DEADBAND);
    Serial.println(" pulses");
    Serial.println();
    Serial.println("PHASE 1 — CALIBRATION (motors are off):");
    Serial.println("  Rotate each wheel exactly ONE turn by hand.");
    Serial.println("  Check EncL and EncR match expected PPR.");
    Serial.println();
    Serial.println("PHASE 2 — POSITION TEST (via Serial commands):");
    Serial.println("  L = test LEFT motor  (drives 1 revolution)");
    Serial.println("  R = test RIGHT motor (drives 1 revolution)");
    Serial.println("  C = clear counters and stop");
    Serial.println("  S = emergency stop");
    Serial.println();
    Serial.println("Rotate wheels now to see encoder counts:");
    Serial.println();

    loopTimer = micros();
}

// ================================================================
//  MAIN LOOP — fixed 5ms tick for consistent PID timing
// ================================================================
void loop() {
    process_serial();

    if (testActive) {
        run_position_pid();
    } else {
        calibration_display();
    }

    // Hold exactly 5ms per loop for consistent D term timing
    while (micros() - loopTimer < (uint32_t)(dt * 1000000));
    loopTimer = micros();
}
