// ============================================================
// Dual Stepper Motor - Synchronized Forward/Backward, 1 Rev Each
// Board   : ESP32-S3
// Driver  : A4988 x2 (Full Step - MS1/MS2/MS3 all LOW or disconnected)
// Motor   : NEMA17 17HS4023 (0.7A/phase, Vref = 0.56V)
// ============================================================

// --- Pin definitions ---
// Right Stepper Motor
#define STEP_PIN_R  5
#define DIR_PIN_R   4

// Left Stepper Motor
#define STEP_PIN_L  9
#define DIR_PIN_L   8

// --- Motion parameters ---
#define STEPS_PER_REV  3200   // 200 x 16 microstepping = 3200 steps/rev
#define STEPS_TO_MOVE  STEPS_PER_REV   // 1 full rotation, as requested
#define STEP_DELAY_US  500    // pulse half-period; lower = faster

// --- Direction constants ---
#define DIR_FORWARD   HIGH
#define DIR_BACKWARD  LOW

// If your motors are mirrored (facing opposite ways on the chassis),
// uncomment this and use it for the LEFT motor's direction instead:
// #define DIR_FORWARD_L   LOW
// #define DIR_BACKWARD_L  HIGH

int currentDirection = DIR_FORWARD;

// ============================================================
void setup()
{
    pinMode(STEP_PIN_R, OUTPUT);
    pinMode(DIR_PIN_R, OUTPUT);
    pinMode(STEP_PIN_L, OUTPUT);
    pinMode(DIR_PIN_L, OUTPUT);

    digitalWrite(STEP_PIN_R, LOW);
    digitalWrite(STEP_PIN_L, LOW);

    digitalWrite(DIR_PIN_R, currentDirection);
    digitalWrite(DIR_PIN_L, currentDirection);

    Serial.begin(115200);
    Serial.println("Dual stepper test started - forward 1 rev, backward 1 rev");
}

// ============================================================
void loop()
{
    moveStepsBoth(STEPS_TO_MOVE, currentDirection);

    delay(500);  // let both motors settle before reversing

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
// moveStepsBoth() - pulses BOTH motors' STEP pins in lockstep
// so they rotate together, not one after the other.
// Think of it like two drummers hitting the same beat at once
// instead of taking turns - that's what keeps L/R in sync.
// ============================================================
void moveStepsBoth(int numSteps, int direction)
{
    digitalWrite(DIR_PIN_R, direction);
    digitalWrite(DIR_PIN_L, direction);  // flip this if motors are mirrored

    delay(1);  // A4988 needs DIR stable before STEP pulses (datasheet min 200ns)

    for (int i = 0; i < numSteps; i++)
    {
        digitalWrite(STEP_PIN_R, HIGH);
        digitalWrite(STEP_PIN_L, HIGH);
        delayMicroseconds(STEP_DELAY_US);

        digitalWrite(STEP_PIN_R, LOW);
        digitalWrite(STEP_PIN_L, LOW);
        delayMicroseconds(STEP_DELAY_US);
    }
}