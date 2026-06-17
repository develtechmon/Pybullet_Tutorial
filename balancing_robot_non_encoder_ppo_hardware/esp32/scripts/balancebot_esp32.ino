/*
  balancebot_esp32.ino — ESP32-S3
  =================================

  WHAT THIS SKETCH DOES:
    Acts as the low-level hardware controller for the balance robot.
    The RPi Zero 2W handles the AI policy (high-level brain).
    The ESP32-S3 handles sensors and motors (low-level body).

    Every 10ms (100Hz) this sketch:
      1. Reads raw accelerometer + gyroscope from MPU6050 via I2C
      2. Computes pitch, pitch_rate, yaw, yaw_rate using a
         complementary filter (fuses gyro + accel)
      3. Sends the 4 sensor values to RPi over UART
      4. Reads left/right PWM commands from RPi over UART
      5. Drives L298N motor driver with received commands

  WHY ESP32-S3 INSTEAD OF ARDUINO:
    Arduino Uno/Nano uses 5V GPIO — needs voltage divider for RPi.
    ESP32-S3 uses 3.3V GPIO — direct connection to RPi GPIO safe.
    ESP32-S3 also has LEDC hardware PWM peripheral which gives
    cleaner motor control than Arduino's analogWrite().
    Smaller footprint — saves space on the robot frame.

  COMPLEMENTARY FILTER EXPLAINED:
    MPU6050 has two sensors:
      Gyroscope:     measures rotation rate (rad/s)
                     good short-term, drifts long-term
      Accelerometer: measures tilt angle (via gravity)
                     good long-term, noisy short-term

    Complementary filter combines both:
      pitch = 0.98 * (pitch + gyro * dt) + 0.02 * accel_pitch
              ^^^^ gyro dominates ^^^^     ^^ accel corrects ^^

    Result: accurate pitch with no drift, low noise.
    This matches what the simulation assumed during training.

  WATCHDOG SAFETY:
    If the RPi stops sending commands (crash, disconnect),
    the ESP32 stops the motors after 500ms automatically.
    This prevents the robot from running away uncontrolled.

  SERIAL PROTOCOL (115200 baud, 8N1):
    ESP32-S3 -> RPi : "pitch,pitch_rate,yaw,yaw_rate\n"
      sent every 10ms (100Hz)
      values in radians and radians/second
      6 decimal places of precision

    RPi -> ESP32-S3 : "left,right\n"
      left, right are integers in range [-255, 255]
      positive = forward wheel rotation
      negative = backward wheel rotation
      0 = stop (also applied when within 10% deadband)

  WIRING — MPU6050 (I2C):
    VCC  -> 3.3V
    GND  -> GND
    SDA  -> GPIO 8
    SCL  -> GPIO 9
    AD0  -> GND   (sets I2C address to 0x68)

  WIRING — L298N MOTOR DRIVER:
    ENA  -> GPIO 5  (LEDC PWM channel 0)  left motor speed
    IN1  -> GPIO 6                         left motor direction A
    IN2  -> GPIO 7                         left motor direction B
    ENB  -> GPIO 15 (LEDC PWM channel 1)  right motor speed
    IN3  -> GPIO 16                        right motor direction A
    IN4  -> GPIO 17                        right motor direction B
    GND  -> GND (shared with ESP32-S3 and battery negative)
    12V  -> battery positive (GNB 1350mAh 11.1V 3S LiPo)

  WIRING — UART to RPi Zero 2W (GPIO UART):
    ESP32-S3 TX (GPIO43) -> RPi RX (GPIO15 / pin 10)
    ESP32-S3 RX (GPIO44) -> RPi TX (GPIO14 / pin 8)
    GND                  -> RPi GND (pin 6)

    VOLTAGE COMPATIBILITY:
      ESP32-S3 GPIO = 3.3V
      RPi GPIO      = 3.3V
      Direct connection is safe — NO voltage divider needed.
      This is the key advantage over Arduino (5V) + RPi (3.3V).

  LEDC PWM (ESP32-S3 specific):
    Unlike Arduino, ESP32-S3 does NOT support analogWrite().
    Instead use the LEDC (LED Control) peripheral:
      ledcAttach(pin, frequency, resolution)
      ledcWrite(channel, duty)
    We use 1000Hz frequency, 8-bit resolution (0-255).

  INSTALL LIBRARIES (Arduino IDE):
    Tools -> Manage Libraries -> Search:
      "MPU6050" by Electronic Cats
      (I2Cdev is included automatically)

  BOARD SETUP (Arduino IDE):
    File -> Preferences -> Additional Board Manager URLs:
      https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
    Tools -> Board -> ESP32 Arduino -> ESP32S3 Dev Module

  MPU6050 CALIBRATION (do this once before final deployment):
    1. Place robot perfectly upright and still
    2. Upload the MPU6050_calibration example sketch
       (File -> Examples -> MPU6050 -> MPU6050_calibration)
    3. Open Serial Monitor at 115200 baud
    4. Note the 6 offset values printed
    5. Paste into gx_off, gy_off, gz_off, ax_off, ay_off, az_off
    6. Re-upload this sketch
    Without calibration the IMU has a DC bias that causes
    the robot to think it is always slightly tilted.
*/

#include <Wire.h>
#include <MPU6050.h>

// ── Motor pins ────────────────────────────────────────────────
#define PIN_ENA   5    // Left  motor PWM  (LEDC)
#define PIN_IN1   6    // Left  motor direction A
#define PIN_IN2   7    // Left  motor direction B
#define PIN_ENB   15   // Right motor PWM  (LEDC)
#define PIN_IN3   16   // Right motor direction A
#define PIN_IN4   17   // Right motor direction B

// ── LEDC PWM config (ESP32-S3) ────────────────────────────────
#define PWM_FREQ       1000   // Hz
#define PWM_RESOLUTION 8      // bits — 0 to 255
#define LEDC_CH_LEFT   0      // LEDC channel for left motor
#define LEDC_CH_RIGHT  1      // LEDC channel for right motor

// ── UART to RPi ───────────────────────────────────────────────
// ESP32-S3 default Serial0 = USB, Serial1 = GPIO43/44
#define RPI_SERIAL      Serial1
#define RPI_BAUD        115200
#define RPI_TX_PIN      43
#define RPI_RX_PIN      44

// ── MPU6050 ───────────────────────────────────────────────────
MPU6050 mpu;

// Paste calibration values here after running calibration sketch
int16_t gx_off = 0, gy_off = 0, gz_off = 0;
int16_t ax_off = 0, ay_off = 0, az_off = 0;

// ── Sensor state ─────────────────────────────────────────────
float pitch      = 0.0f;
float yaw        = 0.0f;
float pitch_rate = 0.0f;
float yaw_rate   = 0.0f;

// Complementary filter coefficient
// 98% gyro integration, 2% accelerometer correction
const float ALPHA = 0.98f;

// ── Timing ───────────────────────────────────────────────────
const uint32_t LOOP_US = 10000;   // 10ms = 100Hz
uint32_t last_us       = 0;
float dt               = 0.01f;

// ── Motor commands ────────────────────────────────────────────
int left_pwm  = 0;
int right_pwm = 0;

// Watchdog — stop motors if RPi goes silent
uint32_t last_cmd_ms        = 0;
const uint32_t CMD_TIMEOUT  = 500;   // ms

// ── Serial receive buffer ─────────────────────────────────────
String rx_buf = "";

// ─────────────────────────────────────────────────────────────

void motors_stop() {
    ledcWrite(LEDC_CH_LEFT,  0);
    ledcWrite(LEDC_CH_RIGHT, 0);
    digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW);
}

void set_motor_left(int pwm_val) {
    // 10% deadband matches balance_env.py MOTOR_DEADBAND = 0.10
    // action [-1,1] * 255 = [-255,255], 10% = abs < 26
    if (abs(pwm_val) < 26) {
        ledcWrite(LEDC_CH_LEFT, 0);
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, LOW);
        return;
    }
    ledcWrite(LEDC_CH_LEFT, (uint32_t)constrain(abs(pwm_val), 0, 255));
    if (pwm_val > 0) {
        digitalWrite(PIN_IN1, HIGH);
        digitalWrite(PIN_IN2, LOW);
    } else {
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, HIGH);
    }
}

void set_motor_right(int pwm_val) {
    if (abs(pwm_val) < 26) {
        ledcWrite(LEDC_CH_RIGHT, 0);
        digitalWrite(PIN_IN3, LOW);
        digitalWrite(PIN_IN4, LOW);
        return;
    }
    ledcWrite(LEDC_CH_RIGHT, (uint32_t)constrain(abs(pwm_val), 0, 255));
    if (pwm_val > 0) {
        digitalWrite(PIN_IN3, HIGH);
        digitalWrite(PIN_IN4, LOW);
    } else {
        digitalWrite(PIN_IN3, LOW);
        digitalWrite(PIN_IN4, HIGH);
    }
}

// ─────────────────────────────────────────────────────────────

void read_imu() {
    int16_t ax_r, ay_r, az_r, gx_r, gy_r, gz_r;
    mpu.getMotion6(&ax_r, &ay_r, &az_r, &gx_r, &gy_r, &gz_r);

    // Accelerometer: LSB/g = 16384 for ±2g range
    float ax = ax_r / 16384.0f;
    float az = az_r / 16384.0f;

    // Gyroscope: LSB/(deg/s) = 131 for ±250deg/s -> convert to rad/s
    float gy = (gy_r / 131.0f) * (PI / 180.0f);
    float gz = (gz_r / 131.0f) * (PI / 180.0f);

    pitch_rate = gy;
    yaw_rate   = gz;

    // Accelerometer pitch estimate (forward tilt angle)
    float accel_pitch = atan2f(ax, az);

    // Complementary filter
    // Gyro integrates well short-term but drifts long-term
    // Accelerometer drifts short-term but is stable long-term
    // Combining both gives best of both worlds
    pitch = ALPHA * (pitch + gy * dt) + (1.0f - ALPHA) * accel_pitch;

    // Yaw by gyro integration only (no magnetometer available)
    yaw += gz * dt;

    // Wrap yaw to [-pi, pi]
    if (yaw >  PI) yaw -= 2.0f * PI;
    if (yaw < -PI) yaw += 2.0f * PI;
}

// ─────────────────────────────────────────────────────────────

void read_serial() {
    while (RPI_SERIAL.available()) {
        char c = (char)RPI_SERIAL.read();
        if (c == '\n') {
            // Parse "left,right\n" from RPi
            int comma = rx_buf.indexOf(',');
            if (comma > 0) {
                left_pwm  = rx_buf.substring(0, comma).toInt();
                right_pwm = rx_buf.substring(comma + 1).toInt();
                left_pwm  = constrain(left_pwm,  -255, 255);
                right_pwm = constrain(right_pwm, -255, 255);
                last_cmd_ms = millis();
            }
            rx_buf = "";
        } else {
            rx_buf += c;
            // Guard against runaway buffer
            if (rx_buf.length() > 24) rx_buf = "";
        }
    }
}

// ─────────────────────────────────────────────────────────────

void setup() {
    // USB Serial for debug (optional)
    Serial.begin(115200);

    // UART to RPi on GPIO43/44
    RPI_SERIAL.begin(RPI_BAUD, SERIAL_8N1, RPI_RX_PIN, RPI_TX_PIN);

    // Motor direction pins
    pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);

    // LEDC PWM setup for ESP32-S3
    ledcAttach(PIN_ENA, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_ENB, PWM_FREQ, PWM_RESOLUTION);
    motors_stop();

    // I2C for MPU6050
    Wire.begin(8, 9);          // SDA=GPIO8, SCL=GPIO9
    Wire.setClock(400000);     // 400kHz fast mode

    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("ERR:MPU6050 not found");
        RPI_SERIAL.println("ERR:MPU6050");
        while (1) { delay(1000); }
    }

    // Apply calibration offsets
    mpu.setXGyroOffset(gx_off); mpu.setYGyroOffset(gy_off);
    mpu.setZGyroOffset(gz_off); mpu.setXAccelOffset(ax_off);
    mpu.setYAccelOffset(ay_off); mpu.setZAccelOffset(az_off);

    // ±250 deg/s gyro range, ±2g accel range
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

    last_us     = micros();
    last_cmd_ms = millis();

    Serial.println("ESP32-S3 BalanceBot ready");
    RPI_SERIAL.println("READY");
}

// ─────────────────────────────────────────────────────────────

void loop() {
    uint32_t now_us = micros();
    if (now_us - last_us < LOOP_US) return;

    // Update actual dt — important for accurate integration
    dt      = (now_us - last_us) / 1000000.0f;
    last_us = now_us;

    // Watchdog — stop motors if RPi silent for 500ms
    if (millis() - last_cmd_ms > CMD_TIMEOUT) {
        left_pwm  = 0;
        right_pwm = 0;
    }

    // 1. Read IMU
    read_imu();

    // 2. Read motor commands from RPi
    read_serial();

    // 3. Drive motors
    set_motor_left(left_pwm);
    set_motor_right(right_pwm);

    // 4. Send sensor data to RPi
    // Format: "pitch,pitch_rate,yaw,yaw_rate\n"
    RPI_SERIAL.print(pitch,      6); RPI_SERIAL.print(',');
    RPI_SERIAL.print(pitch_rate, 6); RPI_SERIAL.print(',');
    RPI_SERIAL.print(yaw,        6); RPI_SERIAL.print(',');
    RPI_SERIAL.print(yaw_rate,   6); RPI_SERIAL.print('\n');
}
