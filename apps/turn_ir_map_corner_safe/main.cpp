#include <Arduino.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <L298N.h>

namespace {
constexpr uint8_t LEFT_IN1  = 8;
constexpr uint8_t LEFT_IN2  = 7;
constexpr uint8_t LEFT_PWM  = 6;
constexpr uint8_t RIGHT_IN1 = 5;
constexpr uint8_t RIGHT_IN2 = 4;
constexpr uint8_t RIGHT_PWM = 3;
constexpr uint8_t IR_PIN    = A0;

L298NMotor leftMotor(LEFT_IN1, LEFT_IN2, LEFT_PWM);
L298NMotor rightMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_PWM);

MPU6050 mpu;

constexpr uint8_t GYRO_RANGE = MPU6050_GYRO_FS_250;
constexpr uint16_t CALIB_SAMPLES = 800;
constexpr uint16_t CALIB_DELAY_MS = 2;

constexpr unsigned long IR_SAMPLE_WINDOW_US = 10400;
constexpr float MAP_INTERVAL_DEG = 8.0f;
constexpr uint8_t MAX_MAP_SAMPLES = 64;
constexpr float SCAN_TARGET_DEG = 360.0f;
constexpr float MIN_PEAK_SEPARATION_DEG = 40.0f;
constexpr float TURN_TOLERANCE_DEG = 2.0f;

constexpr uint8_t LEFT_SCAN_SPEED = 180;
constexpr uint8_t RIGHT_SCAN_SPEED = 120;
constexpr uint8_t LEFT_TURN_SPEED = 180;
constexpr uint8_t RIGHT_TURN_SPEED = 120;

constexpr float CAUGHT_WZ_THRESHOLD_DPS = 10.0f;
constexpr uint16_t CAUGHT_TIME_MS = 450;
constexpr float CORNER_HIT_DELTA_ACC_LSB = 9000.0f;

float gbz = 0.0f;
float bax = 0.0f;
float bay = 0.0f;
float baz = 0.0f;

double totalZ = 0.0;
uint32_t lastUs = 0;

float latestWzDps = 0.0f;
float latestAccelDeltaMag = 0.0f;

bool turningCommandActive = false;
uint32_t lowYawStartMs = 0;

bool safetyTriggered = false;

double scanStartZ = 0.0;
float nextSampleAngleDeg = 0.0f;

struct MapSample {
  float angleDeg;
  int peakToPeak;
};

MapSample mapSamples[MAX_MAP_SAMPLES];
uint8_t sampleCount = 0;
float midpointAngleDeg = 0.0f;

enum class State : uint8_t {
  SCANNING,
  TURNING_TO_MIDPOINT,
  DONE,
  SAFETY_STOP
};

State state = State::SCANNING;

float gyroLsbPerDps(uint8_t range) {
  switch (range) {
    case MPU6050_GYRO_FS_250: return 131.0f;
    case MPU6050_GYRO_FS_500: return 65.5f;
    case MPU6050_GYRO_FS_1000: return 32.8f;
    case MPU6050_GYRO_FS_2000: return 16.4f;
    default: return 131.0f;
  }
}

float wrap360(float angle) {
  while (angle >= 360.0f) angle -= 360.0f;
  while (angle < 0.0f) angle += 360.0f;
  return angle;
}

float wrap180(float angle) {
  while (angle >= 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float absf(float v) {
  return (v < 0.0f) ? -v : v;
}

float sqrtNewton(float x) {
  if (x <= 0.0f) return 0.0f;
  float guess = x;
  for (uint8_t i = 0; i < 6; i++) {
    guess = 0.5f * (guess + x / guess);
  }
  return guess;
}

void stopMotors() {
  leftMotor.drive(L298NMotor::Direction::Brake, 0);
  rightMotor.drive(L298NMotor::Direction::Brake, 0);
  turningCommandActive = false;
  lowYawStartMs = 0;
}

void startTurnBySign(float signedAngleDeg) {
  if (signedAngleDeg >= 0.0f) {
    leftMotor.drive(L298NMotor::Direction::Backward, LEFT_TURN_SPEED);
    rightMotor.drive(L298NMotor::Direction::Forward, RIGHT_TURN_SPEED);
  } else {
    leftMotor.drive(L298NMotor::Direction::Forward, LEFT_TURN_SPEED);
    rightMotor.drive(L298NMotor::Direction::Backward, RIGHT_TURN_SPEED);
  }
  turningCommandActive = true;
  lowYawStartMs = 0;
}

void startScan() {
  scanStartZ = totalZ;
  nextSampleAngleDeg = 0.0f;
  sampleCount = 0;

  leftMotor.drive(L298NMotor::Direction::Backward, LEFT_SCAN_SPEED);
  rightMotor.drive(L298NMotor::Direction::Forward, RIGHT_SCAN_SPEED);
  turningCommandActive = true;
  lowYawStartMs = 0;

  Serial.println(F("SCAN_START"));
  Serial.println(F("MAP,angle_deg,peak_to_peak"));
}

void calibrateImuBias() {
  Serial.println(F("Calibrating MPU6050 (keep robot still)..."));
  long sumGz = 0;
  long sumAx = 0;
  long sumAy = 0;
  long sumAz = 0;
  int16_t ax, ay, az, gx, gy, gz;

  for (uint16_t i = 0; i < CALIB_SAMPLES; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sumGz += gz;
    sumAx += ax;
    sumAy += ay;
    sumAz += az;
    delay(CALIB_DELAY_MS);
  }

  gbz = static_cast<float>(sumGz) / CALIB_SAMPLES;
  bax = static_cast<float>(sumAx) / CALIB_SAMPLES;
  bay = static_cast<float>(sumAy) / CALIB_SAMPLES;
  baz = static_cast<float>(sumAz) / CALIB_SAMPLES;

  Serial.print(F("GYRO_BIAS_Z,"));
  Serial.println(gbz, 2);
  Serial.print(F("ACC_BIAS,"));
  Serial.print(bax, 1);
  Serial.print(F(","));
  Serial.print(bay, 1);
  Serial.print(F(","));
  Serial.println(baz, 1);
}

void updateImu() {
  uint32_t nowUs = micros();
  float dt = (nowUs - lastUs) / 1000000.0f;
  lastUs = nowUs;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  latestWzDps = (gz - gbz) / gyroLsbPerDps(GYRO_RANGE);
  totalZ += static_cast<double>(latestWzDps) * dt;

  float dax = static_cast<float>(ax) - bax;
  float day = static_cast<float>(ay) - bay;
  float daz = static_cast<float>(az) - baz;
  latestAccelDeltaMag = sqrtNewton(dax * dax + day * day + daz * daz);
}

void triggerSafetyStop(const __FlashStringHelper *reason) {
  if (safetyTriggered) return;
  safetyTriggered = true;
  stopMotors();
  state = State::SAFETY_STOP;
  Serial.print(F("SAFETY_STOP,"));
  Serial.print(reason);
  Serial.print(F(",wz_dps="));
  Serial.print(latestWzDps, 2);
  Serial.print(F(",acc_delta="));
  Serial.println(latestAccelDeltaMag, 1);
}

void checkRotationSafety() {
  if (!turningCommandActive || safetyTriggered) return;

  if (latestAccelDeltaMag >= CORNER_HIT_DELTA_ACC_LSB) {
    triggerSafetyStop(F("CORNER_HIT"));
    return;
  }

  if (absf(latestWzDps) <= CAUGHT_WZ_THRESHOLD_DPS) {
    if (lowYawStartMs == 0) {
      lowYawStartMs = millis();
    } else if (millis() - lowYawStartMs >= CAUGHT_TIME_MS) {
      triggerSafetyStop(F("CAUGHT_STALL"));
      return;
    }
  } else {
    lowYawStartMs = 0;
  }
}

int sampleIrPeakToPeak() {
  unsigned long startUs = micros();
  int minVal = 1023;
  int maxVal = 0;

  while (micros() - startUs < IR_SAMPLE_WINDOW_US) {
    int sample = analogRead(IR_PIN);
    if (sample < minVal) minVal = sample;
    if (sample > maxVal) maxVal = sample;
  }

  return maxVal - minVal;
}

float circularBinDistance(uint8_t i, uint8_t j, uint8_t n) {
  int d = static_cast<int>(i) - static_cast<int>(j);
  if (d < 0) d = -d;
  int wrapped = n - d;
  return static_cast<float>(d < wrapped ? d : wrapped);
}

bool computeMidpointFromTwoPeaks(float &peak1Deg, float &peak2Deg, float &midDeg) {
  if (sampleCount < 2) return false;

  int smoothed[MAX_MAP_SAMPLES];
  for (uint8_t i = 0; i < sampleCount; i++) {
    uint8_t prev = (i == 0) ? (sampleCount - 1) : (i - 1);
    uint8_t next = (i + 1 == sampleCount) ? 0 : (i + 1);
    smoothed[i] = (mapSamples[prev].peakToPeak + mapSamples[i].peakToPeak + mapSamples[next].peakToPeak) / 3;
  }

  uint8_t bestIdx = 0;
  for (uint8_t i = 1; i < sampleCount; i++) {
    if (smoothed[i] > smoothed[bestIdx]) bestIdx = i;
  }

  uint8_t minSepBins = static_cast<uint8_t>((MIN_PEAK_SEPARATION_DEG / MAP_INTERVAL_DEG) + 0.5f);
  if (minSepBins < 1) minSepBins = 1;

  bool foundSecond = false;
  uint8_t secondIdx = 0;

  for (uint8_t i = 0; i < sampleCount; i++) {
    if (i == bestIdx) continue;
    if (circularBinDistance(i, bestIdx, sampleCount) < minSepBins) continue;

    if (!foundSecond || smoothed[i] > smoothed[secondIdx]) {
      secondIdx = i;
      foundSecond = true;
    }
  }

  if (!foundSecond) {
    for (uint8_t i = 0; i < sampleCount; i++) {
      if (i == bestIdx) continue;
      if (!foundSecond || smoothed[i] > smoothed[secondIdx]) {
        secondIdx = i;
        foundSecond = true;
      }
    }
  }

  if (!foundSecond) return false;

  peak1Deg = mapSamples[bestIdx].angleDeg;
  peak2Deg = mapSamples[secondIdx].angleDeg;

  float delta = wrap180(peak2Deg - peak1Deg);
  midDeg = wrap360(peak1Deg + 0.5f * delta);
  return true;
}

void scanStep() {
  double traveled = totalZ - scanStartZ;

  while (nextSampleAngleDeg <= SCAN_TARGET_DEG && traveled >= nextSampleAngleDeg) {
    if (sampleCount >= MAX_MAP_SAMPLES) break;

    int peakToPeak = sampleIrPeakToPeak();
    mapSamples[sampleCount].angleDeg = nextSampleAngleDeg;
    mapSamples[sampleCount].peakToPeak = peakToPeak;

    Serial.print(F("MAP,"));
    Serial.print(nextSampleAngleDeg, 1);
    Serial.print(F(","));
    Serial.println(peakToPeak);

    sampleCount++;
    nextSampleAngleDeg += MAP_INTERVAL_DEG;
  }

  if (traveled >= SCAN_TARGET_DEG || sampleCount >= MAX_MAP_SAMPLES) {
    stopMotors();

    float peakA = 0.0f;
    float peakB = 0.0f;
    if (computeMidpointFromTwoPeaks(peakA, peakB, midpointAngleDeg)) {
      Serial.print(F("PEAK_A_DEG,"));
      Serial.println(peakA, 1);
      Serial.print(F("PEAK_B_DEG,"));
      Serial.println(peakB, 1);
      Serial.print(F("MIDPOINT_DEG,"));
      Serial.println(midpointAngleDeg, 1);

      state = State::TURNING_TO_MIDPOINT;
      Serial.println(F("TURN_TO_MIDPOINT_START"));
    } else {
      Serial.println(F("ERROR,Not enough map samples to identify two peaks"));
      state = State::DONE;
    }
  }
}

void turnToMidpointStep() {
  float currentAngle = wrap360(static_cast<float>(totalZ - scanStartZ));
  float remaining = wrap180(midpointAngleDeg - currentAngle);

  if (absf(remaining) <= TURN_TOLERANCE_DEG) {
    stopMotors();
    Serial.println(F("TURN_TO_MIDPOINT_DONE"));
    Serial.print(F("FINAL_ANGLE_DEG,"));
    Serial.println(currentAngle, 1);
    state = State::DONE;
    return;
  }

  startTurnBySign(remaining);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  pinMode(IR_PIN, INPUT);

  leftMotor.begin();
  rightMotor.begin();

  Wire.begin();
  Wire.setClock(400000);
  mpu.initialize();
  mpu.setFullScaleGyroRange(GYRO_RANGE);
  mpu.setDLPFMode(3);
  mpu.setRate(0);

  Serial.println(mpu.testConnection() ? F("MPU6050 OK") : F("MPU6050 FAIL"));

  calibrateImuBias();
  lastUs = micros();
  startScan();
}

void loop() {
  updateImu();
  checkRotationSafety();

  switch (state) {
    case State::SCANNING:
      scanStep();
      break;

    case State::TURNING_TO_MIDPOINT:
      turnToMidpointStep();
      break;

    case State::DONE:
      stopMotors();
      delay(50);
      break;

    case State::SAFETY_STOP:
      stopMotors();
      delay(50);
      break;
  }
}
