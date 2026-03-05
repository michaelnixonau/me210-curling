#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>

MPU6050 mpu;

// === Settings ===
static const uint32_t BAUD = 115200;
static const uint16_t CALIB_SAMPLES = 800;     // keep still during this
static const uint16_t CALIB_DELAY_MS = 2;      // 800*2ms ~ 1.6s
static const uint32_t PRINT_EVERY_MS = 1000;

// If you change gyro full-scale range, you MUST change the scale factor below.
static const uint8_t GYRO_RANGE = MPU6050_GYRO_FS_250; // 250, 500, 1000, 2000 dps
static float gyroLsbPerDps(uint8_t range) {
  switch (range) {
    case MPU6050_GYRO_FS_250:  return 131.0f;
    case MPU6050_GYRO_FS_500:  return 65.5f;
    case MPU6050_GYRO_FS_1000: return 32.8f;
    case MPU6050_GYRO_FS_2000: return 16.4f;
    default: return 131.0f;
  }
}

int16_t ax, ay, az, gx, gy, gz;

// Bias (raw LSB)
float gbx = 0, gby = 0, gbz = 0;

// Total integrated angles (degrees, unbounded)
double totalX = 0, totalY = 0, totalZ = 0;

// Turn counters (integer revolutions)
long turnsX = 0, turnsY = 0, turnsZ = 0;

// For turn counting we track a wrapped angle in [-180, 180)
float wrapX = 0, wrapY = 0, wrapZ = 0;

uint32_t lastUs = 0;
uint32_t lastPrintMs = 0;

static float wrap180(float a) {
  while (a >= 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

void calibrateGyroBias() {
  Serial.println(F("Calibrating gyro bias: keep the board STILL..."));
  long sx = 0, sy = 0, sz = 0;

  for (uint16_t i = 0; i < CALIB_SAMPLES; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sx += gx; sy += gy; sz += gz;
    delay(CALIB_DELAY_MS);
  }

  gbx = (float)sx / CALIB_SAMPLES;
  gby = (float)sy / CALIB_SAMPLES;
  gbz = (float)sz / CALIB_SAMPLES;

  Serial.print(F("Bias (raw LSB): "));
  Serial.print(gbx, 2); Serial.print(F(", "));
  Serial.print(gby, 2); Serial.print(F(", "));
  Serial.println(gbz, 2);
}

void setup() {
  Serial.begin(BAUD);
  Wire.begin();
  Wire.setClock(400000);

  mpu.initialize();
  Serial.println(mpu.testConnection() ? F("MPU6050 OK") : F("MPU6050 FAIL"));

  mpu.setFullScaleGyroRange(GYRO_RANGE);
  mpu.setDLPFMode(3);     // moderate filtering
  mpu.setRate(0);         // fastest

  calibrateGyroBias();

  lastUs = micros();
  lastPrintMs = millis();

  Serial.println(F("t_ms, wx_dps,wy_dps,wz_dps, totalX_deg,totalY_deg,totalZ_deg, turnsX,turnsY,turnsZ"));
}

void loop() {
  // dt
  uint32_t nowUs = micros();
  float dt = (nowUs - lastUs) / 1000000.0f;
  lastUs = nowUs;

  // Read
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert to deg/s (bias removed)
  const float s = gyroLsbPerDps(GYRO_RANGE);
  float wx = (gx - gbx) / s;
  float wy = (gy - gby) / s;
  float wz = (gz - gbz) / s;

  // Integrate totals (unbounded)
  totalX += (double)wx * dt;
  totalY += (double)wy * dt;
  totalZ += (double)wz * dt;

  // Optional: turn counting using wrapped angles
  // Update wrapped angles (bounded representation)
  float prevWrapX = wrapX, prevWrapY = wrapY, prevWrapZ = wrapZ;

  wrapX = wrap180(wrapX + wx * dt);
  wrapY = wrap180(wrapY + wy * dt);
  wrapZ = wrap180(wrapZ + wz * dt);

  // Detect wrap-around jumps to count revolutions
  // If we jumped from +179 to -179, that's +1 turn. Opposite is -1.
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