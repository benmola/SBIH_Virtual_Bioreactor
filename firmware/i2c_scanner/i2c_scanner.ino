/*
 * I2C Scanner — XIAO nRF52840 Sense diagnostic
 * Upload this to the XIAO Sense to find the IMU I2C address.
 * Board  : Seeed XIAO nRF52840 Sense
 * USB    : TinyUSB
 */

#include <Wire.h>

const int IMU_PWR = 32 + 8;   // P1.08 — IMU power transistor

void scanBus(TwoWire &bus, const char* busName) {
  Serial.print("\nScanning ");
  Serial.print(busName);
  Serial.println("...");

  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    bus.beginTransmission(addr);
    uint8_t err = bus.endTransmission();
    if (err == 0) {
      Serial.print("  Device found at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (found == 0) Serial.println("  No devices found.");
}

void setup() {
  Serial.begin(115200);
  unsigned long t = millis();
  while (!Serial && millis() - t < 5000);

  Serial.println("=== I2C Scanner ===");

  // Power on the IMU via its transistor
  pinMode(IMU_PWR, OUTPUT);
  digitalWrite(IMU_PWR, HIGH);
  delay(100);
  Serial.println("IMU power pin P1.08 set HIGH");

  // Scan default Wire bus (P0.04 SDA / P0.05 SCL)
  Wire.begin();
  scanBus(Wire, "Wire (P0.04/P0.05)");

  // Scan Wire1 bus if available (some board variants route IMU differently)
#if defined(PIN_WIRE1_SDA)
  Wire1.begin();
  scanBus(Wire1, "Wire1");
#else
  Serial.println("\nWire1 not defined on this board variant.");
#endif
}

void loop() {
  // Re-scan every 5 seconds in case device powers up late
  delay(5000);
  Wire.begin();
  scanBus(Wire, "Wire (re-scan)");
}
