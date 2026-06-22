/*
  test_motors_mdd3a.ino — XIAO ESP32-C3 + Cytron MDD3A
  ========================================================

  PURPOSE:
    Test the Cytron MDD3A motor driver in isolation BEFORE
    running the full balance robot code. No IMU, no RPi UART,
    no PPO policy — just direct motor control to confirm:

      1. Both motors spin in the correct direction
      2. PWM speed control works at different duty cycles
      3. Wiring (M1A/M1B/M2A/M2B) is correct
      4. Left and right motors are not swapped

  WIRING (Cytron MDD3A, confirmed GPIO numbers):
    M1A = GPIO5    left motor forward PWM
    M1B = GPIO10   left motor backward PWM
    M2A = GPIO3    right motor forward PWM
    M2B = GPIO4    right motor backward PWM
    GND = shared with battery negative
    VCC (battery+) = 3S LiPo positive, 4-16V range

  HOW TO USE:
    1. Wire up MDD3A per above, motors connected to M1/M2 outputs
    2. Upload this sketch
    3. Open Serial Monitor at 115200 baud
    4. Robot will run through a fixed test sequence automatically
    5. Watch physically which wheel moves and in which direction
    6. Note results, then send single-letter commands via
       Serial Monitor to test manually:

  MANUAL COMMANDS (type in Serial Monitor, press Enter):
    f = both motors forward  (slow, 30% PWM)
    b = both motors backward (slow, 30% PWM)
    l = left motor only, forward
    r = right motor only, forward
    j = turn LEFT  (pivot — left back, right forward)
    k = turn RIGHT (pivot — left forward, right back)
    1 = test 25% PWM both forward
    2 = test 50% PWM both forward
    3 = test 75% PWM both forward
    4 = test 100% PWM both forward
    s = stop all motors

  WHAT TO CHECK:
    - Does "left motor forward" actually spin the LEFT wheel
      in the direction that would drive the robot FORWARD?
    - Does "right motor forward" spin the RIGHT wheel forward?
    - If a wheel spins backward when commanded forward, swap
      M_A and M_B wires for that motor (do not change code —
      easier to physically swap two wires than re-flash).
    - TURN LEFT: left wheel back, right wheel forward — robot
      should pivot counter-clockwise (turning left) in place.
    - TURN RIGHT: left wheel forward, right wheel back — robot
      should pivot clockwise (turning right) in place.
    - Does motor speed visibly increase from 25% to 100% PWM?
    - Does the motor respond promptly, no major delay or stutter?

  SAFETY:
    Lift the robot off the ground or remove wheels before testing
    so it does not drive off the table while you are watching pins.
*/

// ── Motor pins — Cytron MDD3A ─────────────────────────────────
// Pin assignment matches actual physical wiring on this robot
#define PIN_M1A   5    // Left  motor forward PWM
#define PIN_M1B   10   // Left  motor backward PWM
#define PIN_M2A   3    // Right motor forward PWM
#define PIN_M2B   4    // Right motor backward PWM

#define PWM_FREQ       20000
#define PWM_RESOLUTION 8

// ─────────────────────────────────────────────────────────────

void motors_stop() {
    ledcWrite(PIN_M1A, 0);
    ledcWrite(PIN_M1B, 0);
    ledcWrite(PIN_M2A, 0);
    ledcWrite(PIN_M2B, 0);
    Serial.println("  [STOP] All motors stopped");
}

void set_left(int pwm_signed) {
    // pwm_signed: -255 to +255. Positive = forward, negative = backward
    pwm_signed = constrain(pwm_signed, -255, 255);
    uint32_t duty = (uint32_t)abs(pwm_signed);
    if (pwm_signed > 0) {
        ledcWrite(PIN_M1A, duty);
        ledcWrite(PIN_M1B, 0);
    } else if (pwm_signed < 0) {
        ledcWrite(PIN_M1A, 0);
        ledcWrite(PIN_M1B, duty);
    } else {
        ledcWrite(PIN_M1A, 0);
        ledcWrite(PIN_M1B, 0);
    }
}

void set_right(int pwm_signed) {
    pwm_signed = constrain(pwm_signed, -255, 255);
    uint32_t duty = (uint32_t)abs(pwm_signed);
    if (pwm_signed > 0) {
        ledcWrite(PIN_M2A, duty);
        ledcWrite(PIN_M2B, 0);
    } else if (pwm_signed < 0) {
        ledcWrite(PIN_M2A, 0);
        ledcWrite(PIN_M2B, duty);
    } else {
        ledcWrite(PIN_M2A, 0);
        ledcWrite(PIN_M2B, 0);
    }
}

// ─────────────────────────────────────────────────────────────

void run_auto_sequence() {
    Serial.println("\n========================================");
    Serial.println("  AUTO TEST SEQUENCE STARTING");
    Serial.println("========================================\n");

    // ── Test 1: Left motor forward ─────────────────────────
    Serial.println("[1/9] LEFT motor FORWARD (50% PWM) - 2 sec");
    Serial.println("      Watch: LEFT wheel should spin forward");
    set_left(128);
    set_right(0);
    delay(2000);
    motors_stop();
    delay(1000);

    // ── Test 2: Left motor backward ────────────────────────
    Serial.println("\n[2/9] LEFT motor BACKWARD (50% PWM) - 2 sec");
    Serial.println("      Watch: LEFT wheel should spin backward");
    set_left(-128);
    set_right(0);
    delay(2000);
    motors_stop();
    delay(1000);

    // ── Test 3: Right motor forward ────────────────────────
    Serial.println("\n[3/9] RIGHT motor FORWARD (50% PWM) - 2 sec");
    Serial.println("      Watch: RIGHT wheel should spin forward");
    set_left(0);
    set_right(128);
    delay(2000);
    motors_stop();
    delay(1000);

    // ── Test 4: Right motor backward ───────────────────────
    Serial.println("\n[4/9] RIGHT motor BACKWARD (50% PWM) - 2 sec");
    Serial.println("      Watch: RIGHT wheel should spin backward");
    set_left(0);
    set_right(-128);
    delay(2000);
    motors_stop();
    delay(1000);

    // ── Test 5: Both forward (robot should drive straight) ─
    Serial.println("\n[5/9] BOTH motors FORWARD (50% PWM) - 2 sec");
    Serial.println("      Watch: robot would drive straight forward");
    set_left(128);
    set_right(128);
    delay(2000);
    motors_stop();
    delay(1000);

    // ── Test 6: Both backward ──────────────────────────────
    Serial.println("\n[6/9] BOTH motors BACKWARD (50% PWM) - 2 sec");
    Serial.println("      Watch: robot would drive straight backward");
    set_left(-128);
    set_right(-128);
    delay(2000);
    motors_stop();
    delay(1000);

    // ── Test 7: PWM ramp — confirm speed control works ────
    Serial.println("\n[7/9] PWM RAMP TEST - both motors forward");
    Serial.println("      Watch: speed should visibly increase");
    int ramp_values[] = {64, 128, 192, 255};
    const char* labels[] = {"25%", "50%", "75%", "100%"};
    for (int i = 0; i < 4; i++) {
        Serial.print("      PWM = ");
        Serial.print(labels[i]);
        Serial.print(" (");
        Serial.print(ramp_values[i]);
        Serial.println("/255)");
        set_left(ramp_values[i]);
        set_right(ramp_values[i]);
        delay(1500);
    }
    motors_stop();
    delay(1000);

    // ── Test 8: Turn LEFT (left wheel back, right wheel forward) ──
    Serial.println("\n[8/9] TURN LEFT - left back, right forward");
    Serial.println("      Watch: robot would pivot LEFT in place");
    set_left(-100);
    set_right(100);
    delay(2000);
    motors_stop();
    delay(1000);

    // ── Test 9: Turn RIGHT (left wheel forward, right wheel back) ──
    Serial.println("\n[9/9] TURN RIGHT - left forward, right back");
    Serial.println("      Watch: robot would pivot RIGHT in place");
    set_left(100);
    set_right(-100);
    delay(2000);
    motors_stop();

    Serial.println("\n========================================");
    Serial.println("  AUTO SEQUENCE COMPLETE");
    Serial.println("========================================");
    Serial.println("\nIf any direction was wrong, swap the two");
    Serial.println("motor wires (M_A <-> M_B) for that motor.");
    Serial.println("\nEntering manual mode. Commands:");
    print_menu();
}

void print_menu() {
    Serial.println("\n--- Manual Commands ---");
    Serial.println("  f = both forward (30%)   b = both backward (30%)");
    Serial.println("  l = left only forward     r = right only forward");
    Serial.println("  j = turn LEFT  (pivot)    k = turn RIGHT (pivot)");
    Serial.println("  1 = 25% PWM   2 = 50% PWM   3 = 75% PWM   4 = 100% PWM");
    Serial.println("  s = stop all motors");
    Serial.println("------------------------\n");
}

// ─────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) { ; }   // wait briefly for USB

    ledcAttach(PIN_M1A, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_M1B, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_M2A, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_M2B, PWM_FREQ, PWM_RESOLUTION);
    motors_stop();

    delay(1000);
    Serial.println("XIAO ESP32-C3 + Cytron MDD3A Motor Test");
    Serial.println("Lift robot off ground before testing!\n");
    delay(2000);

    run_auto_sequence();
}

// ─────────────────────────────────────────────────────────────

void loop() {
    if (Serial.available()) {
        char cmd = Serial.read();

        switch (cmd) {
            case 'f':
                Serial.println("Both FORWARD (30%)");
                set_left(76); set_right(76);
                break;
            case 'b':
                Serial.println("Both BACKWARD (30%)");
                set_left(-76); set_right(-76);
                break;
            case 'l':
                Serial.println("LEFT only FORWARD (50%)");
                set_left(128); set_right(0);
                break;
            case 'r':
                Serial.println("RIGHT only FORWARD (50%)");
                set_left(0); set_right(128);
                break;
            case 'j':
                Serial.println("TURN LEFT (pivot, 40% PWM)");
                set_left(-100); set_right(100);
                break;
            case 'k':
                Serial.println("TURN RIGHT (pivot, 40% PWM)");
                set_left(100); set_right(-100);
                break;
            case '1':
                Serial.println("25% PWM both forward");
                set_left(64); set_right(64);
                break;
            case '2':
                Serial.println("50% PWM both forward");
                set_left(128); set_right(128);
                break;
            case '3':
                Serial.println("75% PWM both forward");
                set_left(192); set_right(192);
                break;
            case '4':
                Serial.println("100% PWM both forward");
                set_left(255); set_right(255);
                break;
            case 's':
                motors_stop();
                break;
            case '\n':
            case '\r':
                break;   // ignore newline/carriage return
            default:
                Serial.print("Unknown command: ");
                Serial.println(cmd);
                print_menu();
        }
    }
}
