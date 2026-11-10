// ESP32-S3 + Encoder + Cytron Maker Drive
// PID position control: move motor 1 revolution

// -------------------- Pin setup --------------------
int encoderA = 8;
int encoderB = 9;

int motorM2A = 7;
int motorM2B = 6;


// -------------------- Encoder setup --------------------
volatile long pulseCount = 0;


// -------------------- Target setup --------------------
long targetPulse = 330;   // 11 pulses x 30 gearbox = 330 pulses


// -------------------- PID values --------------------
// Start with these values first
float Kp = 1.2;
float Ki = 0.0;
float Kd = 0.08;


// -------------------- PID variables --------------------
float error = 0;
float previousError = 0;
float integral = 0;
float derivative = 0;

int motorPWM = 0;


// -------------------- Motor settings --------------------
int maxPWM = 130;     // maximum motor speed
int minPWM = 45;      // minimum PWM to make motor move

int stopRange = 2;    // acceptable pulse error
bool targetReached = false;


// -------------------- Timing --------------------
unsigned long lastPIDTime = 0;
unsigned long pidInterval = 20;   // PID runs every 20 ms

unsigned long lastPrintTime = 0;

// -------------------- Encoder interrupt --------------------
void IRAM_ATTR readEncoder() {
  int B = digitalRead(encoderB);

  // If count goes negative while motor moves forward,
  // swap ++ and -- here.
  if (B == HIGH) {
    pulseCount++;
  } else {
    pulseCount--;
  }
}


// -------------------- PID control --------------------
void runPID() {
  long currentPulse;

  // Safely read pulseCount
  noInterrupts();
  currentPulse = pulseCount;
  interrupts();

  // Calculate error
  error = targetPulse - currentPulse;

  // If close enough to target, stop motor
  if (abs(error) <= stopRange) {
    stopMotor();
    targetReached = true;

    // Reset PID memory when stopped
    integral = 0;
    previousError = error;

    return;
  }

  targetReached = false;

  // Integral adds error over time
  integral = integral + error;

  // Limit integral to prevent too much buildup
  integral = constrain(integral, -300, 300);

  // Derivative checks how fast error is changing
  derivative = error - previousError;

  // PID formula
  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  previousError = error;

  // Limit output
  output = constrain(output, -maxPWM, maxPWM);

  // Convert PID output to motor PWM
  motorPWM = abs(output);

  // Make sure PWM is strong enough to move motor
  if (motorPWM > 0 && motorPWM < minPWM) {
    motorPWM = minPWM;
  }

  // Drive motor direction based on PID output
  if (output > 0) {
    motorForward(motorPWM);
  } else {
    motorReverse(motorPWM);
  }
}


// -------------------- Motor functions --------------------
void motorForward(int speedValue) {
  analogWrite(motorM2A, speedValue);
  analogWrite(motorM2B, 0);
}


void motorReverse(int speedValue) {
  analogWrite(motorM2A, 0);
  analogWrite(motorM2B, speedValue);
}


void stopMotor() {
  analogWrite(motorM2A, 0);
  analogWrite(motorM2B, 0);
}

// -------------------- Serial monitor display --------------------
void printStatus() {
  long currentPulse;

  noInterrupts();
  currentPulse = pulseCount;
  interrupts();

  Serial.print("Pulse: ");
  Serial.print(currentPulse);

  Serial.print(" | Target: ");
  Serial.print(targetPulse);

  Serial.print(" | Error: ");
  Serial.print(error);

  Serial.print(" | PWM: ");
  Serial.print(motorPWM);

  Serial.print(" | Status: ");

  if (targetReached == true) {
    Serial.println("Reached");
  } else {
    Serial.println("Moving");
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);

  pinMode(motorM2A, OUTPUT);
  pinMode(motorM2B, OUTPUT);

  stopMotor();

  attachInterrupt(
    digitalPinToInterrupt(encoderA),
    readEncoder,
    RISING
  );

  delay(1000);

  Serial.println("PID motor position control started");
  Serial.println("Target: 1 revolution = 330 pulses");

  // Start from zero position
  noInterrupts();
  pulseCount = 0;
  interrupts();

  lastPIDTime = millis();
}


void loop() {
  unsigned long now = millis();

  // Run PID every 20 ms
  if (now - lastPIDTime >= pidInterval) {
    runPID();
    lastPIDTime = now;
  }

  // Print information every 100 ms
  if (now - lastPrintTime >= 100) {
    printStatus();
    lastPrintTime = now;
  }
}

