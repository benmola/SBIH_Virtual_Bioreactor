/*
 * AD Reactor Wand — Receiver / Serial Bridge
 * Board : Nice!Nano v2 (nRF52840, Pro Micro footprint)
 * BLE   : Central (GATT client) — Adafruit Bluefruit library
 *
 * Required libraries:
 *   - Adafruit_TinyUSB_Library  (install via Library Manager — needed for USB Serial)
 *   Note: bluefruit.h is included with the Adafruit nRF52 board package
 *
 * Board package:
 *   Add https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
 *   to Arduino > Preferences > Additional boards manager URLs,
 *   then install "Adafruit nRF52".
 *   Select board : Tools > Board > "Adafruit Feather nRF52840 Express"
 *   Select USB   : Tools > USB Stack > TinyUSB          ← required for Serial
 *
 * Flashing the Nice!Nano:
 *   Double-tap RST pin → drive "NRF52BOOT" appears → then Upload normally.
 *
 * WHY ArduinoBLE was removed:
 *   ArduinoBLE has no HCITransport implementation for the Adafruit nRF52 board
 *   package. The correct library for this platform is the built-in Bluefruit API.
 *   BLE still operates at 2.4 GHz — same radio, cleaner driver.
 *
 * This board forwards every BLE notification from the transmitter to the PC:
 *   USB Serial (115200 baud) → Python dashboard reads via pyserial
 *   Format: "Pitch,Roll,Yaw,SwitchState\n"
 */

#include <bluefruit.h>

// Client-side service and characteristic mirrors the transmitter's GATT layout
BLEClientService        adService("AD00");
BLEClientCharacteristic adChar("AD01");

// ── Notify callback — called every time the transmitter sends new data ────────
void notifyCallback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) {
  // Forward raw bytes straight to the PC as a newline-terminated string
  char buf[33] = {0};
  memcpy(buf, data, min((int)len, 32));
  Serial.println(buf);
}

// ── Scan callback — called only for advertisements that match our UUID filter ─
// No name check needed: Bluefruit.Scanner.filterUuid() below ensures only
// AD00 advertisers reach this callback.
void scanCallback(ble_gap_evt_adv_report_t* report) {
  Serial.print("Found AD-Wand — connecting... ");
  Bluefruit.Central.connect(report);
}

// ── Connect callback ──────────────────────────────────────────────────────────
void connectCallback(uint16_t conn_handle) {
  Serial.println("Connected");

  if (!adService.discover(conn_handle)) {
    Serial.println("Service AD00 not found — disconnecting");
    Bluefruit.disconnect(conn_handle);
    return;
  }

  if (!adChar.discover()) {
    Serial.println("Characteristic AD01 not found — disconnecting");
    Bluefruit.disconnect(conn_handle);
    return;
  }

  if (adChar.enableNotify()) {
    Serial.println("Subscribed — forwarding data to PC...");
  } else {
    Serial.println("enableNotify failed");
  }
}

// ── Disconnect callback ───────────────────────────────────────────────────────
void disconnectCallback(uint16_t conn_handle, uint8_t reason) {
  (void) conn_handle; (void) reason;
  Serial.println("Disconnected — resuming scan...");
  // Scanner restarts automatically (restartOnDisconnect set below)
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);   // wait for USB CDC to enumerate

  Bluefruit.begin(0, 1);   // 0 peripheral roles, 1 central role
  Bluefruit.setName("AD-Bridge");

  Bluefruit.Central.setConnectCallback(connectCallback);
  Bluefruit.Central.setDisconnectCallback(disconnectCallback);

  // Initialise the client service and characteristic
  adService.begin();

  adChar.setNotifyCallback(notifyCallback);
  adChar.begin();

  // Start scanning for the transmitter
  Bluefruit.Scanner.setRxCallback(scanCallback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.filterUuid(adService.uuid);
  Bluefruit.Scanner.setInterval(160, 80);   // units of 0.625 ms
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0);              // 0 = scan forever

  Serial.println("AD Bridge ready — scanning for AD-Wand at 2.4 GHz...");
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop() {
  // All data flow is callback-driven — nothing needed here
}
