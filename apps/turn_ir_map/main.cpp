#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>
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

Adafruit_LSM6DSOX lsm6dsox;

constexpr uint8_t PCA9548A_ADDR = 0x70;
constexpr uint8_t IMU_MUX_CHANNEL = 4;

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
constexpr uint8_t STOP_PULSE_SPEED = 130;
constexpr uint8_t STOP_PULSE_MS = 40;

float gbz = 0.0f;
double totalZ = 0.0;
uint32_t lastUs = 0;

bool selectMuxChannel(uint8_t channel) {
  if (channel > 7) return false;

  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  return Wire.endTransmission() == 0;
}

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
  DONE
};

enum class MotionDir : uint8_t {
  NONE,
  POSITIVE,
  NEGATIVE
};

State state = State::SCANNING;
MotionDir motionDir = MotionDir::NONE;

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

void stopMotors() {
  if (motionDir == MotionDir::POSITIVE) {
    leftMotor.drive(L298NMotor::Direction::Forward, STOP_PULSE_SPEED);
    rightMotor.drive(L298NMotor::Direction::Backward, STOP_PULSE_SPEED);
    delay(STOP_PULSE_MS);
  } else if (motionDir == MotionDir::NEGATIVE) {
    leftMotor.drive(L298NMotor::Direction::Backward, STOP_PULSE_SPEED);
    rightMotor.drive(L298NMotor::Direction::Forward, STOP_PULSE_SPEED);
    delay(STOP_PULSE_MS);
  }

  leftMotor.drive(L298NMotor::Direction::Brake, 0);
  rightMotor.drive(L298NMotor::Direction::Brake, 0);
  motionDir = MotionDir::NONE;
}

void startTurnBySign(float signedAngleDeg) {
  if (signedAngleDeg >= 0.0f) {
    leftMotor.drive(L298NMotor::Direction::Backward, LEFT_TURN_SPEED);
    rightMotor.drive(L298NMotor::Direction::Forward, RIGHT_TURN_SPEED);
    motionDir = MotionDir::POSITIVE;
  } else {
    leftMotor.drive(L298NMotor::Direction::Forward, LEFT_TURN_SPEED);
    rightMotor.drive(L298NMotor::Direction::Backward, RIGHT_TURN_SPEED);
    motionDir = MotionDir::NEGATIVE;
  }
}

void startScan() {
  scanStartZ = totalZ;
  nextSampleAngleDeg = 0.0f;
  sampleCount = 0;

  leftMotor.drive(L298NMotor::Direction::Backward, LEFT_SCAN_SPEED);
  rightMotor.drive(L298NMotor::Direction::Forward, RIGHT_SCAN_SPEED);
  motionDir = MotionDir::POSITIVE;

  Serial.println(F("SCAN_START"));
  Serial.println(F("MAP,angle_deg,peak_to_peak"));
}

void calibrateGyroBias() {
  Serial.println(F("Calibrating gyro (keep robot still)..."));
  float sumZ = 0.0f;

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  for (uint16_t i = 0; i < CALIB_SAMPLES; i++) {
    lsm6dsox.getEvent(&accel, &gyro, &temp);
    sumZ += gyro.gyro.z;
    delay(CALIB_DELAY_MS);
  }

  const float radToDeg = 180.0f / PI;
  gbz = (sumZ / CALIB_SAMPLES) * radToDeg;
  Serial.print(F("GYRO_BIAS_Z,"));
  Serial.println(gbz, 2);
}

void updateGyro() {
  uint32_t nowUs = micros();
  float dt = (nowUs - lastUs) / 1000000.0f;
  lastUs = nowUs;

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm6dsox.getEvent(&accel, &gyro, &temp);

  const float radToDeg = 180.0f / PI;
  float wz = (gyro.gyro.z * radToDeg) - gbz;
  totalZ += static_cast<double>(wz) * dt;
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

  if (!selectMuxChannel(IMU_MUX_CHANNEL)) {
    Serial.println(F("PCA9548A select FAIL"));
    while (1) delay(10);
  }

  if (!lsm6dsox.begin_I2C()) {
    Serial.println(F("LSM6DSOX FAIL"));
    while (1) delay(10);
  }

  lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  lsm6dsox.setGyroDataRate(LSM6DS_RATE_104_HZ);
  lsm6dsox.setAccelDataRate(LSM6DS_RATE_104_HZ);

  Serial.println(F("LSM6DSOX OK (via PCA9548A ch4)"));

  calibrateGyroBias();
  lastUs = micros();
  startScan();
}

void loop() {
  updateGyro();

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
  }
}
