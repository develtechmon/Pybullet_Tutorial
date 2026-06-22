/*
  handshake_esp32c3.ino — XIAO ESP32-C3
  ========================================

  PURPOSE:
    Minimal sketch to verify UART wiring between XIAO ESP32-C3
    and RPi Zero 2W works correctly, BEFORE adding MPU6050
    or motor control into the mix.

  WHAT IT DOES:
    Every 1 second, sends "PING\n" to the RPi over UART.
    Listens for "PONG\n" reply from the RPi.
    Blinks the onboard LED on every successful PONG received.

  WIRING (same as full robot sketch):
    XIAO ESP32-C3 GPIO21 (D6, TX) -> RPi GPIO15 / pin 10 (RX)
    XIAO ESP32-C3 GPIO20 (D7, RX) -> RPi GPIO14 / pin 8  (TX)
    XIAO ESP32-C3 GND             -> RPi GND / pin 6
    Both devices 3.3V — direct connection, no divider needed.

  PIN NOTE — D-MACROS DO NOT COMPILE ON THIS BOARD:
    D6 and D7 macros are not declared in this board's variant
    files and fail to compile. Use direct GPIO numbers instead:
      D6 -> GPIO21
      D7 -> GPIO20
    This matches the same fix already applied to the motor
    pins in the full robot sketch (balancebot_esp32c3.ino).

  EXPECTED SERIAL MONITOR OUTPUT (USB, 115200 baud):
    Sent: PING
    Received: PONG
    Sent: PING
    Received: PONG
    ...

  IF NO PONG RECEIVED:
    Check wiring — TX/RX are commonly swapped by accident.
    Check RPi script is running.
    Check RPi UART is enabled (raspi-config) and Bluetooth
    disabled (dtoverlay=disable-bt in /boot/firmware/config.txt).
    Check common GND is connected.

  NOTE ON LED:
    XIAO ESP32-C3 onboard LED is on GPIO10 (active LOW on most
    XIAO boards — LOW = on). If your specific board does not
    blink, this is just cosmetic — the serial handshake is the
    real test, ignore the LED.
*/

#define RPI_SERIAL    Serial1
#define RPI_BAUD      115200
#define RPI_TX_PIN    21    // D6 — use direct GPIO number, D6 macro fails
#define RPI_RX_PIN    20    // D7 — use direct GPIO number, D7 macro fails

#define LED_PIN       10    // XIAO ESP32-C3 onboard LED (active LOW)

String rx_buf = "";
unsigned long last_ping_ms = 0;
const unsigned long PING_INTERVAL_MS = 1000;

void setup() {
    Serial.begin(115200);          // USB debug output
    while (!Serial && millis() < 3000) { ; }   // wait briefly for USB

    RPI_SERIAL.begin(RPI_BAUD, SERIAL_8N1, RPI_RX_PIN, RPI_TX_PIN);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);   // off (active LOW)

    Serial.println("XIAO ESP32-C3 handshake test starting...");
    Serial.println("Sending PING every 1 second, listening for PONG\n");
}

void loop() {
    unsigned long now = millis();

    // Send PING every second
    if (now - last_ping_ms >= PING_INTERVAL_MS) {
        last_ping_ms = now;
        RPI_SERIAL.println("PING");
        Serial.println("Sent: PING");
    }

    // Listen for PONG reply
    while (RPI_SERIAL.available()) {
        char c = (char)RPI_SERIAL.read();
        if (c == '\n') {
            rx_buf.trim();
            if (rx_buf == "PONG") {
                Serial.println("Received: PONG");
                // Blink LED briefly to confirm visually
                digitalWrite(LED_PIN, LOW);    // on
                delay(50);
                digitalWrite(LED_PIN, HIGH);   // off
            } else if (rx_buf.length() > 0) {
                Serial.print("Received (unexpected): ");
                Serial.println(rx_buf);
            }
            rx_buf = "";
        } else {
            rx_buf += c;
            if (rx_buf.length() > 32) rx_buf = "";   // guard runaway
        }
    }
}
