/*
 * AD Reactor Wand — Transmitter
 * Board : Seeed Studio XIAO nRF52840 Sense (nRF52840 + built-in IMU)
 * IMU   : Built-in LSM6DS3TR-C (I2C, address 0x6A)
 * BLE   : Peripheral (GATT server) — Adafruit Bluefruit library
 *
 * Required libraries (Arduino IDE Library Manager):
 *   - Seeed_Arduino_LSM6DS3
 *   - Adafruit_TinyUSB_Library  (for USB Serial)
 *   Note: bluefruit.h is included automatically with the Seeed/Adafruit nRF52 board package
 *
 * Board package:
 *   Add https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
 *   to Arduino > Preferences > Additional boards manager URLs,
 *   then install "Seeed nRF52 Boards".
 *   Select board : Tools > Board > "Seeed XIAO nRF52840 Sense"
 *   Select USB   : Tools > USB Stack > TinyUSB          ← required for Serial
 *
 * BLE operates in the 2.4 GHz ISM band — same band as WiFi.
 *
 * Data format notified over BLE (string, ≤ 32 bytes):
 *   "Pitch,Roll,Yaw,SwitchState"
 *   e.g.  "37.5,6.9,12.3,1"
 *
 * Physical switch wiring:
 *   One leg → D1 pin   (internal pull-up enabled)
 *   Other leg → GND
 *   Switch open  = HIGH = AI Mode     (SwitchState = 1)
 *   Switch closed = LOW = Manual Mode (SwitchState = 0)
 *
 * LEDs on XIAO nRF52840 are active-LOW (LOW = on, HIGH = off).
 */

#include <bluefruit.h>
#include <LSM6DS3.h>
#include <Wire.h>
#include <math.h>

// ── BLE service + characteristic ─────────────────────────────────────────────
BLEService        adService("AD00");
BLECharacteristic adChar("AD01");

// ── IMU ───────────────────────────────────────────────────────────────────────
LSM6DS3 imu(I2C_MODE, 0x6A);

// ── Pins ──────────────────────────────────────────────────────────────────────
const int SWITCH_PIN = D1;

// ── Yaw integration state ─────────────────────────────────────────────────────
static float    yaw    = 0.0f;
static uint32_t lastMs = 0;

const uint32_t LOOP_PERIOD_MS = 50;   // 20 Hz

// ── BLE callbacks ─────────────────────────────────────────────────────────────
void onConnect(uint16_t conn_handle) {
  (void) conn_handle;
  digitalWrite(LED_RED,   HIGH);   // red off
  digitalWrite(LED_GREEN, LOW);    // green on  (active-LOW)
  Serial.println("Central connected");
}

void onDisconnect(uint16_t conn_handle, uint8_t reason) {
  (void) conn_handle; (void) reason;
  digitalWrite(LED_GREEN, HIGH);   // green off
  Serial.println("Central disconnected — resuming advertising");
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
  // LEDs first so we have visual feedback before Serial is ready
  pinMode(LED_RED,   OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE,  OUTPUT);
  digitalWrite(LED_RED,   HIGH);   // off
  digitalWrite(LED_GREEN, HIGH);   // off
  digitalWrite(LED_BLUE,  LOW);    // blue on = booting

  Serial.begin(115200);
  // Wait up to 5 s for Serial Monitor to connect before printing startup messages.
  // This is needed because USB CDC re-enumerates after reset.
  unsigned long t = millis();
  while (!Serial && millis() - t < 5000);

  digitalWrite(LED_BLUE, HIGH);    // blue off

  pinMode(SWITCH_PIN, INPUT_PULLUP);

  // P1.08 controls the power transistor for the IMU on XIAO nRF52840 Sense.
  // The library handles this via PIN_LSM6DS3TR_C_POWER if the board package
  // defines it, but we set it explicitly here as a reliable fallback.
  const int IMU_PWR = 32 + 8;   // P1.08 in Arduino pin numbering
  pinMode(IMU_PWR, OUTPUT);
  digitalWrite(IMU_PWR, HIGH);
  delay(50);                     // allow IMU VCC to stabilise

  if (imu.begin() != 0) {
    // Fast-flash red = IMU failure
    Serial.println("IMU init failed — check I2C address / library");
    while (true) {
      digitalWrite(LED_RED, LOW);  delay(100);
      digitalWrite(LED_RED, HIGH); delay(100);
    }
  }

  Bluefruit.begin();
  Bluefruit.setName("AD-Wand");
  Bluefruit.setTxPower(4);
  Bluefruit.Periph.setConnectCallback(onConnect);
  Bluefruit.Periph.setDisconnectCallback(onDisconnect);

  // Register service first, then characteristic
  adService.begin();

  adChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  adChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  adChar.setMaxLen(32);
  adChar.begin();

  // Start advertising
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(adService);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);   // units of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);               // 0 = advertise forever

  lastMs = millis();
  digitalWrite(LED_RED, HIGH);   // red off — init complete
  Serial.println("AD-Wand ready — advertising at 2.4 GHz BLE...");
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop() {
  // Blink red while waiting for a central to connect
  if (!Bluefruit.connected()) {
    static uint32_t blinkMs = 0;
    if (millis() - blinkMs > 500) {
      blinkMs = millis();
      digitalWrite(LED_RED, !digitalRead(LED_RED));
    }
    return;
  }

  uint32_t now = millis();
  if (now - lastMs < LOOP_PERIOD_MS) return;

  float dt = (now - lastMs) / 1000.0f;
  lastMs   = now;

  // Read IMU (accelerometer in g, gyroscope in °/s)
  float ax = imu.readFloatAccelX();
  float ay = imu.readFloatAccelY();
  float az = imu.readFloatAccelZ();
  float gz = imu.readFloatGyroZ();

  // Tilt angles — accelerometer only, so no drift on pitch/roll
  float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;
  float roll  = atan2f( ay, az)                       * 180.0f / PI;

  // Yaw from gyro Z integration (drifts slowly — fine for a museum demo)
  yaw += gz * dt;
  if (yaw >  180.0f) yaw -= 360.0f;
  if (yaw < -180.0f) yaw += 360.0f;

  int sw = (digitalRead(SWITCH_PIN) == LOW) ? 0 : 1;

  char buf[32];
  snprintf(buf, sizeof(buf), "%.1f,%.1f,%.1f,%d", pitch, roll, yaw, sw);

  adChar.notify(buf, strlen(buf));   // push to subscribed central
  Serial.println(buf);               // mirror to USB Serial for debugging
}
