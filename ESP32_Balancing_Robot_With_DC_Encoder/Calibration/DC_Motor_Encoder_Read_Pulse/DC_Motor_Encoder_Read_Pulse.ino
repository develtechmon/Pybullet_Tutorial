// ESP32-S3 Encoder Pulse Test
// Encoder A connected to GPIO8
// Encoder B connected to GPIO9

#define ENCODER_A 8
#define ENCODER_B 9

volatile long encoderCount = 0;

volatile bool lastA = 0;
volatile bool lastB = 0;

unsigned long lastPrintTime = 0;
long lastCount = 0;

void IRAM_ATTR readEncoder() {
  bool A = digitalRead(ENCODER_A);
  bool B = digitalRead(ENCODER_B);

  // Quadrature decoding
  if (A != lastA) {
    if (A == B) {
      encoderCount++;
    } else {
      encoderCount--;
    }
  }

  if (B != lastB) {
    if (A != B) {
      encoderCount++;
    } else {
      encoderCount--;
    }
  }

  lastA = A;
  lastB = B;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  lastA = digitalRead(ENCODER_A);
  lastB = digitalRead(ENCODER_B);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), readEncoder, CHANGE);

  Serial.println("ESP32-S3 Encoder Test Started");
  Serial.println("Rotate motor shaft by hand and watch count change.");
}

void loop() {
  unsigned long now = millis();

  if (now - lastPrintTime >= 500) {
    noInterrupts();
    long countNow = encoderCount;
    interrupts();

    long pulseDiff = countNow - lastCount;
    float pulsesPerSecond = pulseDiff * 2.0; // because printing every 0.5 sec

    Serial.print("Count: ");
    Serial.print(countNow);

    Serial.print(" | Pulses/sec: ");
    Serial.print(pulsesPerSecond);

    Serial.print(" | Direction: ");

    if (pulseDiff > 0) {
      Serial.println("Forward");
    } else if (pulseDiff < 0) {
      Serial.println("Reverse");
    } else {
      Serial.println("Stopped");
    }

    lastCount = countNow;
    lastPrintTime = now;
  }
}