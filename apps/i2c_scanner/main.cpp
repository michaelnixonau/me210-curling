#include <Arduino.h>
#include <Wire.h>

static const uint8_t PCA9548A_ADDR = 0x70;
static const uint32_t BAUD = 115200;
static const uint16_t SCAN_DELAY_MS = 1200;

static bool selectMuxChannel(uint8_t channel) {
  if (channel > 7) return false;
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  return Wire.endTransmission() == 0;
}

static void printAddress(uint8_t address) {
  if (address < 0x10) Serial.print('0');
  Serial.print(address, HEX);
}

static uint8_t scanBus(bool hideMuxAddress) {
  uint8_t foundCount = 0;

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      if (hideMuxAddress && address == PCA9548A_ADDR) continue;
      Serial.print(F("  0x"));
      printAddress(address);
      Serial.println();
      foundCount++;
    } else if (error == 4) {
      Serial.print(F("  Unknown error at 0x"));
      printAddress(address);
      Serial.println();
    }
  }

  return foundCount;
}

void setup() {
  Serial.begin(BAUD);
  Wire.begin();
  Wire.setClock(400000);

  Serial.println(F("I2C scanner via PCA9548A multiplexer"));
  Serial.print(F("Mux address: 0x"));
  printAddress(PCA9548A_ADDR);
  Serial.println();
}

void loop() {
  static uint32_t pass = 0;
  pass++;

  Serial.println();
  Serial.print(F("=== Scan pass "));
  Serial.print(pass);
  Serial.println(F(" ==="));

  Serial.println(F("Base bus (mux visible):"));
  uint8_t baseFound = scanBus(false);
  if (baseFound == 0) Serial.println(F("  (none)"));

  for (uint8_t channel = 0; channel < 8; channel++) {
    Serial.print(F("Mux ch"));
    Serial.print(channel);
    Serial.println(F(":"));

    if (!selectMuxChannel(channel)) {
      Serial.println(F("  Failed to select channel"));
      continue;
    }

    uint8_t foundOnChannel = scanBus(true);
    if (foundOnChannel == 0) Serial.println(F("  (none)"));
  }

  // Disable all channels between passes.
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();

  delay(SCAN_DELAY_MS);
}
