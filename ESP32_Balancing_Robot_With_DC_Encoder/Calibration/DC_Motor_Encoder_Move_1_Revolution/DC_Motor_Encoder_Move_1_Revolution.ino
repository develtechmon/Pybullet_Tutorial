// ESP32-S3 + Encoder + Cytron Maker Drive
// Move motor 1 output-shaft revolution

// Encoder pins
int encoderA = 8;
int encoderB = 9;

// Motor driver pins
int motorM2A = 7;
int motorM2B = 6;

// Encoder counting
volatile long pulseCount = 0;

// Motor setting
int motorSpeed = 50;   // 0 to 255
long targetPulse = 330; // 11 pulses x 30 gearbox = 330 pulses

bool motorRunning = false;

// This function runs automatically when encoder A detects a pulse
void IRAM_ATTR readEncoder() {
  int B = digitalRead(encoderB);

  // If direction is forward, count up
  // If your count goes negative, reverse this logic
  if (B == HIGH) {
    pulseCount++;
  } else {
    pulseCount--;
  }
}

// Motor forward
void motorForward(int speedValue) {
  analogWrite(motorM2A, speedValue);
  analogWrite(motorM2B, 0);
}


// Motor reverse
void motorReverse(int speedValue) {
  analogWrite(motorM2A, 0);
  analogWrite(motorM2B, speedValue);
}


// Stop motor
void stopMotor() {
  analogWrite(motorM2A, 0);
  analogWrite(motorM2B, 0);
}

void setup() {
  Serial.begin(115200);

  // Encoder input
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);

  // Motor output
  pinMode(motorM2A, OUTPUT);
  pinMode(motorM2B, OUTPUT);

  // Make sure motor is stopped at start
  stopMotor();

  // Interrupt on encoder A rising edge
  attachInterrupt(
    digitalPinToInterrupt(encoderA),readEncoder,RISING);

  delay(1000);

  Serial.println("Motor 1 revolution test");
  Serial.println("Starting motor...");

  // Reset encoder count
  pulseCount = 0;

  // Start motor forward
  motorRunning = true;
  motorForward(motorSpeed);
}


void loop() {
  long currentPulse;

  // Safely copy pulseCount
  noInterrupts();
  currentPulse = pulseCount;
  interrupts();

  // Print pulse count
  Serial.print("Pulse: ");
  Serial.println(currentPulse);

  // Stop when target reached
  if (motorRunning == true && currentPulse >= targetPulse) {
    stopMotor();
    motorRunning = false;

    Serial.println("Target reached!");
    Serial.println("Motor stopped.");
  }

  delay(100);
}


