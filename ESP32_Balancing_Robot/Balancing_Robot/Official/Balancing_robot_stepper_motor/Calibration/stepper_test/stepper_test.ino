// ============================================================
// Stepper Motor Test - Back and Forth
// Board   : ESP32-S3
// Driver  : A4988 (Full Step - MS1/MS2/MS3 all LOW or disconnected)
// Motor   : NEMA17 17HS4023 (0.7A/phase, Vref = 0.56V)
// STEP    : GPIO 7
// DIR     : GPIO 6
// ============================================================

// --- Pin definitions ---
// Right Stepper Motor
// #define STEP_PIN  5   // STEP signal to A4988
// #define DIR_PIN   4   // DIR signal to A4988

// Left Stepper Motor
#define STEP_PIN  9   // STEP signal to A4988
#define DIR_PIN   8   // DIR signal to A4988

// --- Motion parameters ---
#define STEPS_PER_REV  3200  // 200 × 16 = 3200 steps per revolution
// #define STEPS_PER_REV     200    // Full step = 200 steps per revolution

#define STEPS_TO_MOVE  6400  // 2 full rotations × 3200 steps
// #define STEPS_TO_MOVE     400    // How many steps per direction (2 full rotations)

#define STEP_DELAY_US  500   // can go faster with microstepping
// #define STEP_DELAY_US     1500   // Delay between steps in microseconds (controls speed)
//                                  // Lower = faster, but too low = motor misses steps
//                                  // 1500us = safe starting point for full step

// --- Direction constants ---
#define DIR_FORWARD   HIGH  // Clockwise
#define DIR_BACKWARD  LOW   // Counter-clockwise

// --- Global state ---
int currentDirection = DIR_FORWARD;  // Start by going forward

// ============================================================
void setup()
{
    // Set both pins as output
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);

    // Make sure STEP starts LOW
    digitalWrite(STEP_PIN, LOW);

    // Set initial direction
    digitalWrite(DIR_PIN, currentDirection);

    Serial.begin(115200);
    Serial.println("Stepper test started - back and forth");
}

// ============================================================
void loop()
{
    // Move STEPS_TO_MOVE steps in the current direction
    moveSteps(STEPS_TO_MOVE, currentDirection);

    // Pause briefly before reversing
    // This lets the motor settle before direction change
    delay(500);

    // Flip direction for next move
    if (currentDirection == DIR_FORWARD)
    {
        currentDirection = DIR_BACKWARD;
        Serial.println("Reversing - going backward");
    }
    else
    {
        currentDirection = DIR_FORWARD;
        Serial.println("Reversing - going forward");
    }
}

// ============================================================
// moveSteps() - sends 'numSteps' pulses to the A4988
// Each pulse = one full step (since MS1/MS2/MS3 are all LOW)
// ============================================================
void moveSteps(int numSteps, int direction)
{
    // Set direction pin BEFORE stepping
    // A4988 needs DIR to be stable before STEP pulses arrive
    digitalWrite(DIR_PIN, direction);

    // Small settle time after setting direction
    // A4988 datasheet requires DIR to be set at least 200ns before STEP
    // 1ms here is very safe
    delay(1);

    for (int i = 0; i < numSteps; i++)
    {
        // Pull STEP pin HIGH - rising edge triggers one step
        digitalWrite(STEP_PIN, HIGH);

        // A4988 minimum STEP pulse width is 1us
        // delayMicroseconds here controls your motor speed
        delayMicroseconds(STEP_DELAY_US);

        // Pull STEP pin LOW to complete the pulse
        digitalWrite(STEP_PIN, LOW);

        // Same delay on LOW side keeps the waveform symmetric
        delayMicroseconds(STEP_DELAY_US);
    }
}
