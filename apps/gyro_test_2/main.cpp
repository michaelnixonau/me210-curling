#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>

Adafruit_LSM6DSOX lsm6dsox;

// === I2C topology ===
static const uint8_t PCA9548A_ADDR = 0x70;
static const uint8_t IMU_MUX_CHANNEL = 4; // zero-indexed port 4

// === Settings ===
static const uint32_t BAUD = 115200;
static const uint16_t CALIB_SAMPLES = 800;     // keep still during this
static const uint16_t CALIB_DELAY_MS = 2;      // 800*2ms ~ 1.6s
static const uint32_t PRINT_EVERY_MS = 1000;

// Bias (deg/s)
float gbx = 0.0f, gby = 0.0f, gbz = 0.0f;

// Total integrated angles (degrees, unbounded)
double totalX = 0, totalY = 0, totalZ = 0;

// Turn counters (integer revolutions)
long turnsX = 0, turnsY = 0, turnsZ = 0;

// For turn counting we track a wrapped angle in [-180, 180)
float wrapX = 0, wrapY = 0, wrapZ = 0;

uint32_t lastUs = 0;
uint32_t lastPrintMs = 0;

static bool selectMuxChannel(uint8_t channel) {
  if (channel > 7) return false;

  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  return Wire.endTransmission() == 0;
}

static float wrap180(float angleDeg) {
  while (angleDeg >= 180.0f) angleDeg -= 360.0f;
  while (angleDeg < -180.0f) angleDeg += 360.0f;
  return angleDeg;
}

void calibrateGyroBias() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  Serial.println(F("Calibrating gyro bias: keep the board STILL..."));
  float sx = 0.0f, sy = 0.0f, sz = 0.0f;

  for (uint16_t i = 0; i < CALIB_SAMPLES; i++) {
    lsm6dsox.getEvent(&accel, &gyro, &temp);
    sx += gyro.gyro.x;
    sy += gyro.gyro.y;
    sz += gyro.gyro.z;
    delay(CALIB_DELAY_MS);
  }

  // gyro.* is in rad/s from Adafruit Unified Sensor.
  // Convert average bias to deg/s for consistency with printout/integration.
  const float radToDeg = 180.0f / PI;
  gbx = (sx / CALIB_SAMPLES) * radToDeg;
  gby = (sy / CALIB_SAMPLES) * radToDeg;
  gbz = (sz / CALIB_SAMPLES) * radToDeg;

  Serial.print(F("Bias (deg/s): "));
  Serial.print(gbx, 4); Serial.print(F(", "));
  Serial.print(gby, 4); Serial.print(F(", "));
  Serial.println(gbz, 4);
}

void setup() {
  Serial.begin(BAUD);
  Wire.begin();
  Wire.setClock(400000);

  if (!selectMuxChannel(IMU_MUX_CHANNEL)) {
    Serial.println(F("PCA9548A select FAIL"));
    while (1) delay(10);
  }

  if (!lsm6dsox.begin_I2C()) {
    Serial.println(F("LSM6DSOX FAIL"));
    while (1) delay(10);
  }
  Serial.println(F("LSM6DSOX OK (via PCA9548A ch4)"));

  // Keep ranges moderate for stable turn-rate integration.
  lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  lsm6dsox.setGyroDataRate(LSM6DS_RATE_104_HZ);
  lsm6dsox.setAccelDataRate(LSM6DS_RATE_104_HZ);

  calibrateGyroBias();

  lastUs = micros();
  lastPrintMs = millis();

  Serial.println(F("t_ms, wx_dps,wy_dps,wz_dps, totalX_deg,totalY_deg,totalZ_deg, turnsX,turnsY,turnsZ"));
}

void loop() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  // dt
  uint32_t nowUs = micros();
  float dt = (nowUs - lastUs) / 1000000.0f;
  lastUs = nowUs;

  // Read in selected mux channel
  lsm6dsox.getEvent(&accel, &gyro, &temp);

  // Convert from rad/s to deg/s, then remove bias
  const float radToDeg = 180.0f / PI;
  float wx = gyro.gyro.x * radToDeg - gbx;
  float wy = gyro.gyro.y * radToDeg - gby;
  float wz = gyro.gyro.z * radToDeg - gbz;

  // Integrate totals (unbounded)
  totalX += (double)wx * dt;
  totalY += (double)wy * dt;
  totalZ += (double)wz * dt;

  // Turn counting using wrapped angles
  float prevWrapX = wrapX, prevWrapY = wrapY, prevWrapZ = wrapZ;

  wrapX = wrap180(wrapX + wx * dt);
  wrapY = wrap180(wrapY + wy * dt);
  wrapZ = wrap180(wrapZ + wz * dt);

  // Detect wrap-around jumps to count revolutions
  if (wrapX - prevWrapX < -180.0f) turnsX++;
  if (wrapX - prevWrapX >  180.0f) turnsX--;

  if (wrapY - prevWrapY < -180.0f) turnsY++;
  if (wrapY - prevWrapY >  180.0f) turnsY--;

  if (wrapZ - prevWrapZ < -180.0f) turnsZ++;
  if (wrapZ - prevWrapZ >  180.0f) turnsZ--;

  // Print
  uint32_t nowMs = millis();
  if (nowMs - lastPrintMs >= PRINT_EVERY_MS) {
    lastPrintMs = nowMs;

    Serial.print(nowMs); Serial.print(F(", "));
    Serial.print(wx, 3); Serial.print(F(", "));
    Serial.print(wy, 3); Serial.print(F(", "));
    Serial.print(wz, 3); Serial.print(F(", "));

    Serial.print(totalX, 2); Serial.print(F(", "));
    Serial.print(totalY, 2); Serial.print(F(", "));
    Serial.print(totalZ, 2); Serial.print(F(", "));

    Serial.print(turnsX); Serial.print(F(", "));
    Serial.print(turnsY); Serial.print(F(", "));
    Serial.println(turnsZ);
  }
}