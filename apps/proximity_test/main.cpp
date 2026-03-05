#include <Wire.h>
#include <VL53L1X.h>

static const uint8_t PCA9548A_ADDR = 0x70;
static const uint8_t SENSOR_CHANNELS[] = {0, 1, 2, 3};
static const uint8_t SENSOR_COUNT = sizeof(SENSOR_CHANNELS) / sizeof(SENSOR_CHANNELS[0]);
static const uint16_t SENSOR_PERIOD_MS[] = {47, 53, 59, 67};
static const uint16_t SENSOR_PHASE_DELAY_MS[] = {0, 12, 24, 36};

static const uint32_t BAUD = 115200;
static const uint32_t PRINT_EVERY_MS = 500; // 2 Hz

VL53L1X sensors[SENSOR_COUNT];
bool sensorReady[SENSOR_COUNT] = {false, false, false, false};

static bool selectMuxChannel(uint8_t channel) {
  if (channel > 7) return false;

  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  uint8_t err = Wire.endTransmission();
  return err == 0 && !Wire.getWireTimeoutFlag();
}

void setup() {
  Serial.begin(BAUD);
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(5000, true); // 5 ms timeout; auto-reset bus if stuck

  Serial.println(F("Starting VL53L1X proximity test via PCA9548A..."));

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    const uint8_t channel = SENSOR_CHANNELS[i];

    Serial.print(F("Selecting mux ch"));
    Serial.println(channel);
    if (!selectMuxChannel(channel)) {
      Serial.print(F("Mux select FAIL on ch"));
      Serial.println(channel);
      continue;
    }

    sensors[i].setTimeout(60);

    Serial.print(F("Initialising sensor on ch"));
    Serial.println(channel);
    if (!sensors[i].init()) {
      Serial.print(F("VL53L1X init FAIL on ch"));
      Serial.println(channel);
      continue;
    }

    sensors[i].setDistanceMode(VL53L1X::Long);
    sensors[i].setMeasurementTimingBudget(50000);

    if (SENSOR_PHASE_DELAY_MS[i] > 0) {
      delay(SENSOR_PHASE_DELAY_MS[i]);
    }

    sensors[i].startContinuous(SENSOR_PERIOD_MS[i]);
    sensorReady[i] = true;

    Serial.print(F("VL53L1X OK on ch"));
    Serial.println(channel);
  }

  Serial.println(F("t_ms, ch0_mm, ch1_mm, ch2_mm, ch4_mm"));
}

void loop() {
  static uint32_t lastPrintMs = 0;

  uint32_t nowMs = millis();
  if (nowMs - lastPrintMs < PRINT_EVERY_MS) return;
  lastPrintMs = nowMs;

  int16_t distances[SENSOR_COUNT] = {-1, -1, -1, -1};

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (!sensorReady[i]) continue;

    const uint8_t channel = SENSOR_CHANNELS[i];
    if (!selectMuxChannel(channel)) continue;

    uint16_t mm = sensors[i].read();
    if (!sensors[i].timeoutOccurred()) {
      distances[i] = (int16_t)mm;
    }
  }

  Serial.print(nowMs);
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(F(", "));
    Serial.print(distances[i]);
  }
  Serial.println();
}
