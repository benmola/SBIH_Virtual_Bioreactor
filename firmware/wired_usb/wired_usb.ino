/*
 * AD Reactor Wand — Wired USB (single-board setup)
 * Board : Nice!Nano v2 (nRF52840, Pro Micro footprint)
 * IMU   : MPU-6500 via I2C (GY-6500 module)
 * Data  : USB Serial at 115200 baud → Python dashboard
 *
 * ── Library (install via Arduino IDE Library Manager) ────────────────────────
 *   "FastIMU" by LiquidCGS
 *
 * ── Board setup ──────────────────────────────────────────────────────────────
 *   Board manager URL:
 *     https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
 *   Install  : "Adafruit nRF52" board package
 *   Board    : Tools > Board > "Adafruit Feather nRF52840 Express"
 *   USB Stack: Tools > USB Stack > TinyUSB          ← required for Serial
 *   Flash    : double-tap RST → "NRF52BOOT" drive → Upload normally
 *
 * ── WHY NOT D2/D3? ───────────────────────────────────────────────────────────
 *   D2 (P0.17) and D3 (P0.20) are the QSPI flash pins in the Adafruit Feather
 *   board package.  The BSP initialises the QSPI peripheral at startup and
 *   claims those GPIOs — Wire.setPins() on them causes a silent hang.
 *   D16 (P0.10) and D18 (P1.15) are clean GPIOs with no BSP conflicts.
 *
 * ── Feather Arduino# derivation (from g_ADigitalPinMap in variant.cpp) ───────
 *   Nice!Nano pad   nRF52840 GPIO   Feather Arduino#   BSP note
 *   D16             P0.10           2                  NFC2 — exposed as GPIO
 *   D18             P1.15           3                  LED1 on Feather; GPIO here
 *   D1              P0.06           11                 Switch
 *
 * ── GY-6500 → Nice!Nano v2 wiring ────────────────────────────────────────────
 *   GY-6500 pin    Nice!Nano pad    Notes
 *   VCC            3V3              —
 *   GND            GND              —
 *   SDA            D16              I2C SDA  (P0.10, Feather index 2)
 *   SCL            D18              I2C SCL  (P1.15, Feather index 3)
 *   AD0/SDO        GND              Pulls I2C address to 0x68
 *   INT, FSYNC     —                not connected
 *
 * ── Switch wiring ────────────────────────────────────────────────────────────
 *   Switch leg 1   D1               P0.06 (Feather index 11)
 *   Switch leg 2   GND
 *   Open  = HIGH = AI mode   (SwitchState = 1)
 *   Closed = LOW = Manual    (SwitchState = 0)
 *
 * ── Output format (USB Serial, 115200 baud) ──────────────────────────────────
 *   "Pitch,Roll,Yaw,SwitchState\n"   e.g.  "37.5,6.9,12.3,1"
 */

#include <Wire.h>
#include <FastIMU.h>
#include <math.h>

// ── I2C pins — Feather Arduino indices from g_ADigitalPinMap ─────────────────
//   Feather 2 = P0.10 = Nice!Nano D16  (SDA)
//   Feather 3 = P1.15 = Nice!Nano D18  (SCL)
#define PIN_SDA    2
#define PIN_SCL    3

// ── Switch ───────────────────────────────────────────────────────────────────
//   Feather 11 = P0.06 = Nice!Nano D1
#define SWITCH_PIN 11

// ── MPU-6500 I2C address (AD0/SDO tied to GND) ───────────────────────────────
#define MPU6500_ADDR  0x68

// ── FastIMU objects (MPU6500 constructor defaults to global Wire) ─────────────
MPU6500   IMU;
calData   calibrationData;
AccelData accel;
GyroData  gyro;

// ── Yaw integration state ─────────────────────────────────────────────────────
static float    yaw    = 0.0f;
static uint32_t lastMs = 0;

const uint32_t LOOP_PERIOD_MS = 50;   // 20 Hz

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  unsigned long t = millis();
  while (!Serial && millis() - t < 5000);

  Serial.println("=== AD Wand — Wired USB (MPU-6500 I2C / FastIMU) ===");

  pinMode(SWITCH_PIN, INPUT_PULLUP);

  // Remap I2C to the Nice!Nano's exposed pads BEFORE Wire.begin()
  Wire.setPins(PIN_SDA, PIN_SCL);
  Wire.begin();

  int err = IMU.init(calibrationData, MPU6500_ADDR);
  if (err != 0) {
    Serial.print("MPU-6500 init error: ");
    Serial.println(err);
    Serial.println("Check: SDA->D2, SCL->D3, AD0->GND, VCC->3V3");
    while (true) delay(100);
  }

  // Auto-zero gyro bias — keep sensor completely still for ~3 s.
  Serial.println("Calibrating — keep sensor still for 3 s...");
  delay(1000);
  IMU.calibrateAccelGyro(&calibrationData);

  Serial.println("MPU-6500 OK — streaming at 20 Hz over USB...");
  lastMs = millis();
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop() {
  uint32_t now = millis();
  if (now - lastMs < LOOP_PERIOD_MS) return;

  float dt = (now - lastMs) / 1000.0f;
  lastMs   = now;

  IMU.update();
  IMU.getAccel(&accel);
  IMU.getGyro(&gyro);

  // FastIMU returns m/s² and rad/s — convert to g and °/s
  float ax = accel.accelX / 9.80665f;
  float ay = accel.accelY / 9.80665f;
  float az = accel.accelZ / 9.80665f;
  float gz = gyro.gyroZ   * (180.0f / PI);

  // ── Tilt angles (accelerometer — no drift on pitch/roll) ──────────────────
  float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / PI);
  float roll  = atan2f( ay, az)                        * (180.0f / PI);

  // ── Yaw (gyro Z integration — slow drift, fine for museum demo) ───────────
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
