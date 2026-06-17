// ============================================================
// XIAO ESP32-C3 + L298N Motor Driver
// ESP32 Arduino Core v3.x API
// ============================================================

// --- Pin definitions (GPIO numbers) ---
#define ENA 3    // Left motor PWM  (D1)
#define IN1 4    // Left motor dir A (D2)
#define IN2 5    // Left motor dir B (D3)
#define ENB 10   // Right motor PWM  (D10)
#define IN3 2    // Right motor dir A (D0)
#define IN4 8    // Right motor dir B (D8)

// --- PWM config ---
#define PWM_FREQ    1000   // 1kHz
#define PWM_RES     8      // 8-bit (0-255)

// --- Speed constants ---
#define SPEED_NORMAL  180
#define SPEED_TURN    150
#define SPEED_STOP    0

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  // Direction pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // v3.x API — ledcAttach replaces ledcSetup + ledcAttachPin
  ledcAttach(ENA, PWM_FREQ, PWM_RES);
  ledcAttach(ENB, PWM_FREQ, PWM_RES);

  motorStop();
  Serial.println("Ready.");
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  Serial.println("Forward");
  motorForward(SPEED_NORMAL);
  delay(2000);

  Serial.println("Stop");
  motorStop();
  delay(1000);

  Serial.println("Backward");
  motorBackward(SPEED_NORMAL);
  delay(2000);

  Serial.println("Stop");
  motorStop();
  delay(1000);

  Serial.println("Turn Left");
  turnLeft(SPEED_TURN);
  delay(1000);

  Serial.println("Stop");
  motorStop();
  delay(1000);

  Serial.println("Turn Right");
  turnRight(SPEED_TURN);
  delay(1000);

  Serial.println("Stop");
  motorStop();
  delay(3000);
}

// ============================================================
// MOTOR PRIMITIVES
// ============================================================

void setLeft(int speed) {
  if (speed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  ledcWrite(ENA, abs(speed));  // v3.x: pass GPIO pin, not channel
}

void setRight(int speed) {
  if (speed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  ledcWrite(ENB, abs(speed));  // v3.x: pass GPIO pin, not channel
}

// ============================================================
// HIGH LEVEL COMMANDS
// ============================================================

void motorForward(int speed) {
  setLeft(speed);
  setRight(speed);
}

void motorBackward(int speed) {
  setLeft(-speed);
  setRight(-speed);
}

void turnLeft(int speed) {
  setLeft(-speed);
  setRight(speed);
}

void turnRight(int speed) {
  setLeft(speed);
  setRight(-speed);
}

void motorStop() {
  setLeft(SPEED_STOP);
  setRight(SPEED_STOP);
}