/*
 * AD Reactor Wand — Wired USB (single-board setup)
 * Board : Nice!Nano v2 (nRF52840, Pro Micro footprint)
 * IMU   : External MPU-6050 (I2C, address 0x68)
 * Data  : USB Serial at 115200 baud → Python dashboard
 *
 * No BLE required — the USB cable carries both power and data.
 *
 * ── Libraries (install via Arduino IDE Library Manager) ──────────────────────
 *   - Adafruit MPU6050
 *   - Adafruit Unified Sensor   (auto-installed as dependency)
 *   - Adafruit BusIO            (auto-installed as dependency)
 *
 * ── Board setup ──────────────────────────────────────────────────────────────
 *   Board manager URL:
 *     https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
 *   Install  : "Adafruit nRF52" board package
 *   Board    : Tools > Board > "Adafruit Feather nRF52840 Express"
 *   USB Stack: Tools > USB Stack > TinyUSB          ← required for Serial
 *   Flash    : double-tap RST → "NRF52BOOT" drive appears → Upload normally
 *
 * ── MPU-6050 wiring ──────────────────────────────────────────────────────────
 *   MPU-6050 pin  →  Nice!Nano pin
 *   VCC           →  3.3V
 *   GND           →  GND
 *   SDA           →  D4  (SDA)
 *   SCL           →  D5  (SCL)
 *   AD0           →  GND  (fixes I2C address at 0x68)
 *   INT           →  not connected
 *
 * ── Physical switch wiring ───────────────────────────────────────────────────
 *   One leg  →  D1  (internal pull-up enabled)
 *   Other leg →  GND
 *   Open  = HIGH = AI / Digitalisation mode  (SwitchState = 1)
 *   Closed = LOW = Manual mode               (SwitchState = 0)
 *
 * ── Output format (USB Serial, 115200 baud) ──────────────────────────────────
 *   "Pitch,Roll,Yaw,SwitchState\n"
 *   e.g.  "37.5,6.9,12.3,1"
 *   Same format as the BLE setup — the Python dashboard is unchanged.
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// ── IMU ───────────────────────────────────────────────────────────────────────
Adafruit_MPU6050 mpu;

// ── Pins ──────────────────────────────────────────────────────────────────────
const int SWITCH_PIN = D1;

// ── Yaw integration state ─────────────────────────────────────────────────────
static float    yaw    = 0.0f;
static uint32_t lastMs = 0;

const uint32_t LOOP_PERIOD_MS = 50;   // 20 Hz

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // Wait up to 5 s for the USB CDC serial port to enumerate on the PC.
  unsigned long t = millis();
  while (!Serial && millis() - t < 5000);

  Serial.println("=== AD Wand — Wired USB ===");

  pinMode(SWITCH_PIN, INPUT_PULLUP);

  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("MPU-6050 not found — check wiring (SDA=D4, SCL=D5) and AD0→GND");
    while (true) delay(100);   // halt — blink an LED here if preferred
  }

  // ±2 g accelerometer range, ±250 °/s gyro range, 21 Hz low-pass filter
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU-6050 OK — streaming at 20 Hz over USB...");
  lastMs = millis();
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop() {
  uint32_t now = millis();
  if (now - lastMs < LOOP_PERIOD_MS) return;

  float dt = (now - lastMs) / 1000.0f;
  lastMs   = now;

  // Read all IMU axes in one call
  sensors_event_t accel_evt, gyro_evt, temp_evt;
  mpu.getEvent(&accel_evt, &gyro_evt, &temp_evt);

  // Convert m/s² → g  (Adafruit library returns SI units)
  float ax = accel_evt.acceleration.x / 9.80665f;
  float ay = accel_evt.acceleration.y / 9.80665f;
  float az = accel_evt.acceleration.z / 9.80665f;

  // Gyro Z in °/s  (library returns rad/s)
  float gz = gyro_evt.gyro.z * (180.0f / PI);

  // ── Tilt angles (accelerometer — no drift on pitch/roll) ──────────────────
  float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / PI);
  float roll  = atan2f( ay, az)                        * (180.0f / PI);

  // ── Yaw (gyro Z integration — drifts slowly, fine for a museum demo) ──────
  yaw += gz * dt;
  if (yaw >  180.0f) yaw -= 360.0f;
  if (yaw < -180.0f) yaw += 360.0f;

  // ── Switch ────────────────────────────────────────────────────────────────
  int sw = (digitalRead(SWITCH_PIN) == LOW) ? 0 : 1;

  // ── Send CSV line to Python dashboard ────────────────────────────────────
  char buf[32];
  snprintf(buf, sizeof(buf), "%.1f,%.1f,%.1f,%d", pitch, roll, yaw, sw);
  Serial.println(buf);
}
