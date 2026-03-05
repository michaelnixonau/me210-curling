#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_QMC5883P.h>
#include <Adafruit_Sensor.h>
#include <CompassCalibration.h>
#include <L298N.h> // NEW: Motor library

namespace {

constexpr uint32_t BAUD = 115200;

constexpr uint8_t PCA9548A_ADDR = 0x70;
constexpr uint8_t IMU_MUX_CHANNEL = 4;
constexpr uint8_t COMPASS_MUX_CHANNEL = 7;

constexpr float FT_TO_MM = 304.8f;
constexpr float ARENA_WIDTH_MM = 4.0f * FT_TO_MM;    // 4'
constexpr float ARENA_LENGTH_MM = 16.0f * FT_TO_MM;  // 16'
constexpr float ARENA_HALF_LENGTH_MM = ARENA_LENGTH_MM * 0.5f;

// Arena +Y (long axis) points at this magnetic bearing.
constexpr float ARENA_LONG_AXIS_BEARING_DEG = 7.0f;
constexpr float COMPASS_HEADING_OFFSET_DEG = 63.0f; 

enum class HalfConstraint : uint8_t {
  Auto = 0,
  South = 1,
  North = 2,
};

constexpr HalfConstraint START_HALF_CONSTRAINT = HalfConstraint::Auto;

enum SensorIndex : uint8_t {
  SENSOR_SOUTH = 0,
  SENSOR_WEST = 1,
  SENSOR_NORTH = 2,
  SENSOR_EAST = 3,
  SENSOR_COUNT = 4,
};

constexpr uint8_t TOF_MUX_CHANNEL[SENSOR_COUNT] = {0, 1, 2, 3};
constexpr uint16_t TOF_PERIOD_MS[SENSOR_COUNT] = {47, 53, 59, 67};
constexpr uint16_t TOF_PHASE_MS[SENSOR_COUNT] = {0, 12, 24, 36};
constexpr int16_t TOF_MIN_VALID_MM = 40;
constexpr int16_t TOF_MAX_VALID_MM = 3800;

// Physical sensor offsets from the center of rotation
constexpr float TOF_BIAS_MM[SENSOR_COUNT] = {76.2f, 139.7f, 228.6f, 139.7f};

constexpr uint16_t GYRO_BIAS_SAMPLES = 250;
constexpr uint16_t GYRO_BIAS_DELAY_MS = 2;
constexpr float GYRO_BEARING_SIGN = -1.0f;

constexpr float ACCEL_NOMINAL_MPS2 = 9.80665f;
constexpr float STATIONARY_ACCEL_THRESH_MPS2 = 0.35f;
constexpr float STATIONARY_GYRO_THRESH_DPS = 0.8f;
constexpr float HEADING_CORR_GAIN_STABLE = 0.04f;
constexpr float HEADING_CORR_GAIN_MOVING = 0.015f;
constexpr float BIAS_ADAPT_GAIN = 0.003f;
constexpr float MAG_LPF_GAIN = 0.20f;
constexpr float MAG_OUTLIER_REJECT_DEG = 35.0f;

constexpr uint32_t SOLVE_EVERY_MS = 60;
constexpr uint32_t PRINT_EVERY_MS = 100;
constexpr float POSITION_FILTER_GAIN_MOVING = 0.35f;
constexpr float POSITION_FILTER_GAIN_STATIONARY = 0.12f;
constexpr float HALF_CONTINUITY_PENALTY_GAIN = 0.05f;
constexpr float HALF_SWITCH_MARGIN_MM = 80.0f;

constexpr float GRID_X_COARSE_MM = 40.0f;
constexpr float GRID_Y_COARSE_MM = 80.0f;
constexpr float GRID_FINE_MM = 20.0f;
constexpr float GRID_FINE_WINDOW_MM = 80.0f;
constexpr float RESIDUAL_CLAMP_MM = 700.0f;

// Local Tracking & Angle Rejection Constants
constexpr float LOCAL_SEARCH_WINDOW_MM = 150.0f; 
constexpr float MAX_LOCAL_RMSE_MM = 150.0f;       
constexpr float MIN_COS_INCIDENCE = 0.819f;       

// NEW: Motor Pins and Recovery Constants
constexpr uint8_t RIGHT_IN1 = 7;
constexpr uint8_t RIGHT_IN2 = 8;
constexpr uint8_t RIGHT_PWM = 6;
constexpr uint8_t LEFT_IN1 = 4;
constexpr uint8_t LEFT_IN2 = 5;
constexpr uint8_t LEFT_PWM = 3;

constexpr uint32_t WIGGLE_TRIGGER_DELAY_MS = 3000; // Wait 3s lost before wiggling
constexpr uint32_t WIGGLE_DURATION_MS = 200;       // Very short pulse to perturb position
constexpr uint8_t WIGGLE_SPEED = 180;

L298NMotor leftMotor(LEFT_IN1, LEFT_IN2, LEFT_PWM);
L298NMotor rightMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_PWM);

uint32_t lastValidPoseMs = 0;
bool isWiggling = false;
uint32_t wiggleStartMs = 0;

VL53L1X tofSensors[SENSOR_COUNT];
bool tofReady[SENSOR_COUNT] = {false, false, false, false};

Adafruit_LSM6DSOX imu;
Adafruit_QMC5883P compass;

struct RangeFrame {
  int16_t mm[SENSOR_COUNT];
  uint8_t validCount;
};

struct PoseEstimate {
  bool valid;
  float xMm;
  float yMm;
  float rmseMm;
  uint8_t usedRanges;
};

HalfConstraint activeHalf = START_HALF_CONSTRAINT;

float gyroBiasZDps = 0.0f;
float gyroZDps = 0.0f;
float accelNormMps2 = ACCEL_NOMINAL_MPS2;
bool isRobotStationary = false;

float magHeadingGlobalDeg = 0.0f;
float magHeadingArenaDeg = 0.0f;
bool magHeadingReady = false;
float headingArenaDeg = 0.0f;
bool headingReady = false;

uint32_t lastImuUs = 0;
uint32_t lastSolveMs = 0;
uint32_t lastPrintMs = 0;

RangeFrame lastRanges = {{-1, -1, -1, -1}, 0};
PoseEstimate lastSolvePose = {false, 0.0f, 0.0f, 0.0f, 0};

bool filteredPoseValid = false;
float filteredXmm = 0.0f;
float filteredYmm = 0.0f;
float filteredRmseMm = 0.0f;
uint8_t filteredUsedRanges = 0;

float wrap360(float angleDeg) {
  while (angleDeg >= 360.0f) angleDeg -= 360.0f;
  while (angleDeg < 0.0f) angleDeg += 360.0f;
  return angleDeg;
}

float wrap180(float angleDeg) {
  while (angleDeg >= 180.0f) angleDeg -= 360.0f;
  while (angleDeg < -180.0f) angleDeg += 360.0f;
  return angleDeg;
}

float absf(float value) {
  return (value < 0.0f) ? -value : value;
}

float clampf(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

const char* halfToString(HalfConstraint half) {
  switch (half) {
    case HalfConstraint::South: return "south";
    case HalfConstraint::North: return "north";
    case HalfConstraint::Auto: default: return "auto";
  }
}

bool selectMuxChannel(uint8_t channel) {
  if (channel > 7) return false;
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  return Wire.endTransmission() == 0;
}

bool initTofSensors() {
  bool anyReady = false;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (!selectMuxChannel(TOF_MUX_CHANNEL[i])) continue;
    tofSensors[i].setTimeout(60);
    if (!tofSensors[i].init()) continue;
    tofSensors[i].setDistanceMode(VL53L1X::Long);
    tofSensors[i].setMeasurementTimingBudget(50000);
    if (TOF_PHASE_MS[i] > 0) delay(TOF_PHASE_MS[i]);
    tofSensors[i].startContinuous(TOF_PERIOD_MS[i]);
    tofReady[i] = true;
    anyReady = true;
  }
  return anyReady;
}

bool initImu() {
  if (!selectMuxChannel(IMU_MUX_CHANNEL)) return false;
  if (!imu.begin_I2C()) return false;
  imu.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  imu.setGyroDataRate(LSM6DS_RATE_104_HZ);
  imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
  return true;
}

bool initCompass() {
  if (!selectMuxChannel(COMPASS_MUX_CHANNEL)) return false;
  delay(10);
  if (!compass.begin()) return false;
  compass.setMode(QMC5883P_MODE_NORMAL);
  compass.setODR(QMC5883P_ODR_50HZ);
  compass.setOSR(QMC5883P_OSR_4);
  compass.setDSR(QMC5883P_DSR_2);
  compass.setRange(QMC5883P_RANGE_8G);
  compass.setSetResetMode(QMC5883P_SETRESET_ON);
  return true;
}

void calibrateGyroBias() {
  float sumZ = 0.0f;
  uint16_t samplesTaken = 0;
  sensors_event_t accelEvent, gyroEvent, tempEvent;

  for (uint16_t i = 0; i < GYRO_BIAS_SAMPLES; i++) {
    if (!selectMuxChannel(IMU_MUX_CHANNEL)) continue;
    imu.getEvent(&accelEvent, &gyroEvent, &tempEvent);
    sumZ += gyroEvent.gyro.z * 180.0f / PI;
    samplesTaken++;
    delay(GYRO_BIAS_DELAY_MS);
  }
  gyroBiasZDps = (samplesTaken == 0) ? 0.0f : (sumZ / samplesTaken);
}

bool readCompassHeadingGlobal(float& headingDeg) {
  if (!selectMuxChannel(COMPASS_MUX_CHANNEL)) return false;
  if (!compass.isDataReady()) return false;
  int16_t x = 0, y = 0, z = 0;
  if (!compass.getRawMagnetic(&x, &y, &z)) return false;

  applyCompassCalibration(x, y);
  float heading = atan2(static_cast<float>(y), static_cast<float>(x)) * 180.0f / PI;
  headingDeg = wrap360(heading + COMPASS_HEADING_OFFSET_DEG);
  return true;
}

void initializeHeadingFromMagnetometer() {
  float sumSin = 0.0f, sumCos = 0.0f;
  uint8_t valid = 0;

  for (uint8_t i = 0; i < 30; i++) {
    float globalHeading = 0.0f;
    if (readCompassHeadingGlobal(globalHeading)) {
      const float arenaHeading = wrap360(globalHeading - ARENA_LONG_AXIS_BEARING_DEG);
      const float rad = arenaHeading * PI / 180.0f;
      sumSin += sin(rad);
      sumCos += cos(rad);
      valid++;
    }
    delay(10);
  }

  if (valid > 0) {
    float meanHeading = atan2(sumSin, sumCos) * 180.0f / PI;
    headingArenaDeg = wrap360(meanHeading);
    magHeadingArenaDeg = headingArenaDeg;
    magHeadingGlobalDeg = wrap360(headingArenaDeg + ARENA_LONG_AXIS_BEARING_DEG);
    magHeadingReady = true;
    headingReady = true;
  }
}

void updateHeadingFusion() {
  if (!selectMuxChannel(IMU_MUX_CHANNEL)) return;
  sensors_event_t accelEvent, gyroEvent, tempEvent;
  imu.getEvent(&accelEvent, &gyroEvent, &tempEvent);

  const uint32_t nowUs = micros();
  float dt = (nowUs - lastImuUs) / 1000000.0f;
  lastImuUs = nowUs;

  if (dt < 0.0f || dt > 0.2f) dt = 0.0f;

  const float rawGyroZDps = gyroEvent.gyro.z * 180.0f / PI;
  gyroZDps = rawGyroZDps - gyroBiasZDps;

  accelNormMps2 = sqrt(accelEvent.acceleration.x * accelEvent.acceleration.x +
                       accelEvent.acceleration.y * accelEvent.acceleration.y +
                       accelEvent.acceleration.z * accelEvent.acceleration.z);

  if (headingReady && dt > 0.0f) {
    headingArenaDeg = wrap360(headingArenaDeg + GYRO_BEARING_SIGN * gyroZDps * dt);
  }

  isRobotStationary = absf(gyroZDps) < STATIONARY_GYRO_THRESH_DPS &&
                      absf(accelNormMps2 - ACCEL_NOMINAL_MPS2) < STATIONARY_ACCEL_THRESH_MPS2;

  if (isRobotStationary && !isWiggling) {
    gyroBiasZDps = (1.0f - BIAS_ADAPT_GAIN) * gyroBiasZDps + BIAS_ADAPT_GAIN * rawGyroZDps;
  }

  float compassGlobalRawDeg = 0.0f;
  if (!readCompassHeadingGlobal(compassGlobalRawDeg)) return;

  const float magHeadingArenaRawDeg = wrap360(compassGlobalRawDeg - ARENA_LONG_AXIS_BEARING_DEG);
  if (!magHeadingReady) {
    magHeadingArenaDeg = magHeadingArenaRawDeg;
    magHeadingReady = true;
  } else {
    const float magJumpDeg = wrap180(magHeadingArenaRawDeg - magHeadingArenaDeg);
    if (absf(magJumpDeg) <= MAG_OUTLIER_REJECT_DEG) {
      magHeadingArenaDeg = wrap360(magHeadingArenaDeg + MAG_LPF_GAIN * magJumpDeg);
    }
  }
  magHeadingGlobalDeg = wrap360(magHeadingArenaDeg + ARENA_LONG_AXIS_BEARING_DEG);

  if (!headingReady) {
    headingArenaDeg = magHeadingArenaDeg;
    headingReady = true;
    return;
  }

  const float headingErrorDeg = wrap180(magHeadingArenaDeg - headingArenaDeg);
  const float corrGain = isRobotStationary ? HEADING_CORR_GAIN_STABLE : HEADING_CORR_GAIN_MOVING;
  headingArenaDeg = wrap360(headingArenaDeg + corrGain * headingErrorDeg);
}

RangeFrame readRanges() {
  RangeFrame frame = {{-1, -1, -1, -1}, 0};
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (!tofReady[i] || !selectMuxChannel(TOF_MUX_CHANNEL[i])) continue;
    const uint16_t rawMm = tofSensors[i].read();
    if (tofSensors[i].timeoutOccurred()) continue;

    const float correctedMm = rawMm + TOF_BIAS_MM[i];
    const int16_t correctedInt = static_cast<int16_t>(correctedMm + 0.5f);
    if (correctedInt >= TOF_MIN_VALID_MM && correctedInt <= TOF_MAX_VALID_MM) {
      frame.mm[i] = correctedInt;
      frame.validCount++;
    }
  }
  return frame;
}

void getHalfBounds(HalfConstraint half, float& yMin, float& yMax) {
  yMin = 0.0f;
  yMax = ARENA_LENGTH_MM;
  if (half == HalfConstraint::South) yMax = ARENA_HALF_LENGTH_MM;
  else if (half == HalfConstraint::North) yMin = ARENA_HALF_LENGTH_MM;
}

float rayDistanceToBoundaryMm(float xMm, float yMm, float dx, float dy, float invDx, float invDy, bool& hitXWall) {
  float tMin = 1.0e9f;
  hitXWall = false;

  if (invDx != 0.0f) {
    if (dx > 0.0f) {
      float tx = (ARENA_WIDTH_MM - xMm) * invDx;
      if (tx >= 0.0f && tx < tMin) { tMin = tx; hitXWall = true; }
    } else {
      float tx = (0.0f - xMm) * invDx;
      if (tx >= 0.0f && tx < tMin) { tMin = tx; hitXWall = true; }
    }
  }

  if (invDy != 0.0f) {
    if (dy > 0.0f) {
      float ty = (ARENA_LENGTH_MM - yMm) * invDy;
      if (ty >= 0.0f && ty < tMin) { tMin = ty; hitXWall = false; }
    } else {
      float ty = (0.0f - yMm) * invDy;
      if (ty >= 0.0f && ty < tMin) { tMin = ty; hitXWall = false; }
    }
  }
  return (tMin > 9000.0f) ? -1.0f : tMin;
}

bool computeCost(float xMm, float yMm, const float dx[SENSOR_COUNT], const float dy[SENSOR_COUNT], 
                 const float invDx[SENSOR_COUNT], const float invDy[SENSOR_COUNT],
                 const RangeFrame& ranges, float& costOut, uint8_t& usedOut) {
  float totalCost = 0.0f;
  uint8_t used = 0;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (ranges.mm[i] < 0) continue;

    bool hitXWall = false;
    const float predictedMm = rayDistanceToBoundaryMm(xMm, yMm, dx[i], dy[i], invDx[i], invDy[i], hitXWall);
    if (predictedMm < 0.0f) continue;

    const float cosIncidence = hitXWall ? absf(dx[i]) : absf(dy[i]);
    if (cosIncidence < MIN_COS_INCIDENCE) continue;

    float residual = static_cast<float>(ranges.mm[i]) - predictedMm;
    residual = clampf(residual, -RESIDUAL_CLAMP_MM, RESIDUAL_CLAMP_MM);
    totalCost += residual * residual;
    used++;
  }

  if (used < 2) return false;
  costOut = totalCost;
  usedOut = used;
  return true;
}

PoseEstimate solvePoseForHalf(const RangeFrame& ranges, float headingDeg, HalfConstraint half, bool useLocalSearch, float localX, float localY) {
  PoseEstimate result = {false, 0.0f, 0.0f, 0.0f, 0};
  if (ranges.validCount < 2) return result;

  float bearings[SENSOR_COUNT];
  bearings[SENSOR_SOUTH] = wrap360(headingDeg + 180.0f);
  bearings[SENSOR_WEST] = wrap360(headingDeg + 270.0f);
  bearings[SENSOR_NORTH] = wrap360(headingDeg);
  bearings[SENSOR_EAST] = wrap360(headingDeg + 90.0f);

  float dx[SENSOR_COUNT], dy[SENSOR_COUNT], invDx[SENSOR_COUNT], invDy[SENSOR_COUNT];
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    const float rad = bearings[i] * PI / 180.0f;
    dx[i] = sin(rad); dy[i] = cos(rad);
    invDx[i] = (absf(dx[i]) > 1.0e-5f) ? (1.0f / dx[i]) : 0.0f;
    invDy[i] = (absf(dy[i]) > 1.0e-5f) ? (1.0f / dy[i]) : 0.0f;
  }

  float yMinHalf = 0.0f;
  float yMaxHalf = ARENA_LENGTH_MM;
  getHalfBounds(half, yMinHalf, yMaxHalf);

  bool bestFound = false;
  float bestCost = 0.0f, bestX = 0.0f, bestY = 0.0f;
  uint8_t bestUsed = 0;

  float xStart, xEnd, yStart, yEnd;

  if (!useLocalSearch) {
    for (float y = yMinHalf; y <= yMaxHalf + 0.5f; y += GRID_Y_COARSE_MM) {
      for (float x = 0.0f; x <= ARENA_WIDTH_MM + 0.5f; x += GRID_X_COARSE_MM) {
        float candidateCost = 0.0f; uint8_t used = 0;
        if (!computeCost(x, y, dx, dy, invDx, invDy, ranges, candidateCost, used)) continue;
        if (!bestFound || candidateCost < bestCost) {
          bestFound = true; bestCost = candidateCost; bestX = x; bestY = y; bestUsed = used;
        }
      }
    }
    if (!bestFound) return result;
    xStart = clampf(bestX - GRID_FINE_WINDOW_MM, 0.0f, ARENA_WIDTH_MM);
    xEnd = clampf(bestX + GRID_FINE_WINDOW_MM, 0.0f, ARENA_WIDTH_MM);
    yStart = clampf(bestY - GRID_FINE_WINDOW_MM, yMinHalf, yMaxHalf);
    yEnd = clampf(bestY + GRID_FINE_WINDOW_MM, yMinHalf, yMaxHalf);
  } else {
    xStart = clampf(localX - LOCAL_SEARCH_WINDOW_MM, 0.0f, ARENA_WIDTH_MM);
    xEnd = clampf(localX + LOCAL_SEARCH_WINDOW_MM, 0.0f, ARENA_WIDTH_MM);
    yStart = clampf(localY - LOCAL_SEARCH_WINDOW_MM, yMinHalf, yMaxHalf);
    yEnd = clampf(localY + LOCAL_SEARCH_WINDOW_MM, yMinHalf, yMaxHalf);
  }

  for (float y = yStart; y <= yEnd + 0.5f; y += GRID_FINE_MM) {
    for (float x = xStart; x <= xEnd + 0.5f; x += GRID_FINE_MM) {
      float candidateCost = 0.0f; uint8_t used = 0;
      if (!computeCost(x, y, dx, dy, invDx, invDy, ranges, candidateCost, used)) continue;
      if (!bestFound || candidateCost < bestCost) {
        bestFound = true; bestCost = candidateCost; bestX = x; bestY = y; bestUsed = used;
      }
    }
  }

  if (bestFound) {
    result.valid = true; result.xMm = bestX; result.yMm = bestY;
    result.usedRanges = bestUsed; result.rmseMm = sqrt(bestCost / static_cast<float>(bestUsed));
  }
  return result;
}

float halfSelectionScore(const PoseEstimate& pose) {
  if (!pose.valid) return 1.0e9f;
  float score = pose.rmseMm;
  if (filteredPoseValid) {
    const float continuity = absf(pose.xMm - filteredXmm) + absf(pose.yMm - filteredYmm);
    score += HALF_CONTINUITY_PENALTY_GAIN * continuity;
  }
  return score;
}

PoseEstimate solvePose(const RangeFrame& ranges, float headingDeg) {
  bool useLocal = filteredPoseValid; 
  float localX = filteredXmm;
  float localY = filteredYmm;

  PoseEstimate chosen = {false, 0.0f, 0.0f, 0.0f, 0};

  if (START_HALF_CONSTRAINT != HalfConstraint::Auto) {
    activeHalf = START_HALF_CONSTRAINT;
    chosen = solvePoseForHalf(ranges, headingDeg, START_HALF_CONSTRAINT, useLocal, localX, localY);
  } else {
    const PoseEstimate south = solvePoseForHalf(ranges, headingDeg, HalfConstraint::South, useLocal, localX, localY);
    const PoseEstimate north = solvePoseForHalf(ranges, headingDeg, HalfConstraint::North, useLocal, localX, localY);

    if (!south.valid && !north.valid) return chosen;

    const float southScore = halfSelectionScore(south);
    const float northScore = halfSelectionScore(north);

    HalfConstraint chosenHalf = (northScore < southScore) ? HalfConstraint::North : HalfConstraint::South;
    chosen = (chosenHalf == HalfConstraint::North) ? north : south;

    if (activeHalf == HalfConstraint::South && south.valid && !(north.valid && (northScore + HALF_SWITCH_MARGIN_MM < southScore))) {
      chosenHalf = HalfConstraint::South; chosen = south;
    } else if (activeHalf == HalfConstraint::North && north.valid && !(south.valid && (southScore + HALF_SWITCH_MARGIN_MM < northScore))) {
      chosenHalf = HalfConstraint::North; chosen = north;
    }
    activeHalf = chosenHalf;
  }

  if (useLocal && (!chosen.valid || chosen.rmseMm > MAX_LOCAL_RMSE_MM)) {
    Serial.println(F("Local tracking lost. Triggering global search next frame."));
    filteredPoseValid = false; 
    chosen.valid = false;
  }

  return chosen;
}

void updateFilteredPose(const PoseEstimate& pose) {
  if (!pose.valid) return;

  if (!filteredPoseValid) {
    filteredXmm = pose.xMm;
    filteredYmm = pose.yMm;
    filteredPoseValid = true;
  } else {
    const float gain = isRobotStationary ? POSITION_FILTER_GAIN_STATIONARY : POSITION_FILTER_GAIN_MOVING;
    filteredXmm += gain * (pose.xMm - filteredXmm);
    filteredYmm += gain * (pose.yMm - filteredYmm);
  }

  filteredRmseMm = pose.rmseMm;
  filteredUsedRanges = pose.usedRanges;
}

void printTelemetry(uint32_t nowMs) {
  if (nowMs - lastPrintMs < PRINT_EVERY_MS) return;
  lastPrintMs = nowMs;

  const float xFt = filteredPoseValid ? (filteredXmm / FT_TO_MM) : -1.0f;
  const float yFt = filteredPoseValid ? (filteredYmm / FT_TO_MM) : -1.0f;

  Serial.print(F("POSE,")); Serial.print(nowMs); Serial.print(F(","));
  Serial.print(filteredPoseValid ? filteredXmm : -1.0f, 1); Serial.print(F(","));
  Serial.print(filteredPoseValid ? filteredYmm : -1.0f, 1); Serial.print(F(","));
  Serial.print(xFt, 3); Serial.print(F(",")); Serial.print(yFt, 3); Serial.print(F(","));
  Serial.print(headingArenaDeg, 2); Serial.print(F(","));
  Serial.print(magHeadingArenaDeg, 2); Serial.print(F(","));
  Serial.print(magHeadingGlobalDeg, 2); Serial.print(F(","));
  Serial.print(gyroZDps, 2); Serial.print(F(","));
  Serial.print(accelNormMps2, 2); Serial.print(F(","));
  Serial.print(filteredPoseValid ? filteredRmseMm : -1.0f, 1); Serial.print(F(","));
  Serial.print(filteredPoseValid ? filteredUsedRanges : 0); Serial.print(F(","));
  Serial.print(halfToString(activeHalf)); Serial.print(F(","));
  Serial.print(lastRanges.mm[SENSOR_NORTH]); Serial.print(F(","));
  Serial.print(lastRanges.mm[SENSOR_SOUTH]); Serial.print(F(","));
  Serial.print(lastRanges.mm[SENSOR_EAST]); Serial.print(F(","));
  Serial.print(lastRanges.mm[SENSOR_WEST]); Serial.print(F(","));
  Serial.print(wrap180(magHeadingArenaDeg - headingArenaDeg), 2); Serial.print(F(","));
  Serial.println(isRobotStationary ? 1 : 0);
}

}  // namespace

void setup() {
  Serial.begin(BAUD);
  Wire.begin();
  Wire.setClock(400000);

  // Initialize motors
  leftMotor.begin();
  rightMotor.begin();

  Serial.println(F("robot_localisation starting..."));
  
  const bool tofOk = initTofSensors();
  const bool imuOk = initImu();
  const bool compassOk = initCompass();

  if (!tofOk) { Serial.println(F("No TOF sensors initialized")); while (1) delay(10); }
  if (!imuOk || !compassOk) { Serial.println(F("IMU/compass init failed")); while (1) delay(10); }

  calibrateGyroBias();
  initializeHeadingFromMagnetometer();

  if (!headingReady) { Serial.println(F("Heading init failed")); while (1) delay(10); }

  lastValidPoseMs = millis(); // Give it a buffer right on boot
  lastImuUs = micros();
  lastSolveMs = millis();
  lastPrintMs = 0;

  Serial.println(
      F("POSE,t_ms,x_mm,y_mm,x_ft,y_ft,heading_arena_deg,heading_mag_arena_deg,"
        "heading_mag_global_deg,gyro_z_dps,accel_norm_mps2,rmse_mm,used_ranges,"
        "half,n_mm,s_mm,e_mm,w_mm,heading_err_deg,stationary"));
}

void loop() {
  updateHeadingFusion();

  const uint32_t nowMs = millis();
  
  // 1. Check if we should trigger a recovery wiggle
  if (filteredPoseValid) {
    lastValidPoseMs = nowMs; 
  }

  // If we've been lost for 3+ seconds and aren't currently wiggling...
  if (!filteredPoseValid && (nowMs - lastValidPoseMs > WIGGLE_TRIGGER_DELAY_MS)) {
    if (!isWiggling) {
      isWiggling = true;
      wiggleStartMs = nowMs;
      Serial.println(F("Initiating recovery wiggle..."));
      // Pulse left motor forward, right motor brake to spin slightly
      leftMotor.drive(L298NMotor::Direction::Forward, WIGGLE_SPEED);
      rightMotor.drive(L298NMotor::Direction::Brake, 0); 
    }
  }

  // 2. Check if we should stop the recovery wiggle
  if (isWiggling && (nowMs - wiggleStartMs > WIGGLE_DURATION_MS)) {
    isWiggling = false;
    leftMotor.drive(L298NMotor::Direction::Brake, 0);
    rightMotor.drive(L298NMotor::Direction::Brake, 0);
    // Reset timer to give sensors a full 3 seconds to catch the new angle before trying again
    lastValidPoseMs = nowMs; 
    Serial.println(F("Wiggle complete, settling..."));
  }

  // 3. Normal localization routine (runs even while wiggling)
  if (nowMs - lastSolveMs >= SOLVE_EVERY_MS) {
    lastSolveMs = nowMs;

    lastRanges = readRanges();
    lastSolvePose = solvePose(lastRanges, headingArenaDeg);
    updateFilteredPose(lastSolvePose);
  }

  printTelemetry(nowMs);
}