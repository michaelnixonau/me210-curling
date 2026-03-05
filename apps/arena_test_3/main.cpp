#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>
#include <L298N.h>
#include <Servo.h>

namespace {

constexpr uint32_t BAUD = 115200;

// Motor wiring (same baseline as arena_test_1 / line_follow).
constexpr uint8_t RIGHT_IN1 = 7;
constexpr uint8_t RIGHT_IN2 = 8;
constexpr uint8_t RIGHT_PWM = 6;
constexpr uint8_t LEFT_IN1 = 4;
constexpr uint8_t LEFT_IN2 = 5;
constexpr uint8_t LEFT_PWM = 3;

constexpr uint8_t IR_BEACON_PIN = A0;
constexpr uint8_t IR_EMITTER_PIN = 9;

// Physical left->right sensor order is A3, A2, A1.
constexpr uint8_t LINE_SENSOR_PINS[3] = {A3, A2, A1};
constexpr float LINE_SENSOR_WEIGHTS[3] = {-1.0f, 0.0f, 1.0f};

// line_follow calibration (left->right: A3, A2, A1).
constexpr int SENSOR_WHITE[3] = {434, 473, 519};
constexpr int SENSOR_BLACK[3] = {979, 987, 990};
constexpr bool LINE_IS_DARK = false;

constexpr uint8_t SERVO_PIN = 11;
constexpr uint8_t SERVO_MIN_ANGLE_DEG = 73;
constexpr uint8_t SERVO_MAX_ANGLE_DEG = 157;

constexpr uint8_t PCA9548A_ADDR = 0x70;
constexpr uint8_t REAR_MUX_CHANNEL = 0;     // VL53L1X rear
constexpr uint8_t IMU_MUX_CHANNEL = 4;      // LSM6DSOX

constexpr uint16_t TOF_TIMEOUT_MS = 60;
constexpr uint16_t TOF_PERIOD_REAR_MS = 47;

constexpr uint16_t FORWARD_TARGET_MM = 1800;
constexpr uint16_t REVERSE_TARGET_MM = 150;
constexpr unsigned long OPEN_HOLD_MS = 2000;
constexpr unsigned long WAIT_MS = 5000;

// Beacon scan/align (lifted from arena_test_2 style flow).
constexpr float GYRO_SIGN = -1.0f;
constexpr uint16_t GYRO_BIAS_SAMPLES = 300;
constexpr uint16_t GYRO_BIAS_DELAY_MS = 2;


constexpr uint8_t ALIGN_LEFT_PWM_BASE = 144;
constexpr uint8_t ALIGN_RIGHT_PWM_BASE = 216;
constexpr float ALIGN_TOLERANCE_DEG = 4.0f;
constexpr uint16_t ALIGN_HOLD_MS = 300;
constexpr uint16_t ALIGN_TIMEOUT_MS = 6500;
constexpr float ALIGN_KP = 2.2f;
constexpr float ALIGN_KD = 0.45f;
constexpr uint8_t ALIGN_MIN_PWM = 40;
constexpr uint8_t ALIGN_MAX_PWM = 145;
constexpr float ALIGN_CONTROL_DEADBAND_DEG = 1.8f;
constexpr float ALIGN_ERROR_FILTER_GAIN = 0.22f;

constexpr unsigned long IR_SAMPLE_WINDOW_US = 10400;
constexpr float IR_MAP_INTERVAL_DEG = 8.0f;
constexpr float IR_SCAN_TARGET_DEG = 360.0f;
constexpr float IR_MIN_PEAK_SEP_DEG = 40.0f;
constexpr uint8_t MAX_IR_MAP_SAMPLES = 64;
constexpr uint8_t SCAN_LEFT_PWM = 144;
constexpr uint8_t SCAN_RIGHT_PWM = 216;

// line_follow control (same algorithm, adapted for forward/reverse).
constexpr uint8_t BASE_SPEED_LEFT = 155;
constexpr uint8_t BASE_SPEED_RIGHT = 165;
constexpr uint8_t MIN_DRIVE_SPEED = 85;
constexpr uint8_t MAX_DRIVE_SPEED = 230;
constexpr float KP = 24.0f;
constexpr float KD = 55.0f;
constexpr float ERROR_SMOOTH_ALPHA = 0.55f;
constexpr float SENSOR_ACTIVE_MIN = 0.18f;
constexpr float TRACK_CONFIDENCE_MIN = 0.65f;
constexpr uint8_t SEARCH_SPEED = 120;
constexpr unsigned long LOOP_DT_MS = 8;
constexpr uint16_t FIND_LINE_LOCK_MS = 150;

L298NMotor leftMotor(LEFT_IN1, LEFT_IN2, LEFT_PWM);
L298NMotor rightMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_PWM);
VL53L1X rearSensor;
Adafruit_LSM6DSOX imu;
Servo dumpServo;
bool rearSensorReady = false;
bool imuReady = false;

float gyroBiasZDps = 0.0f;
float gyroZDps = 0.0f;
uint32_t lastImuUs = 0;

bool headingReady = false;
float fusedHeadingDeg = 0.0f;

float travelHeadingDeg = 0.0f;
bool travelHeadingReady = false;

unsigned long alignInToleranceSinceMs = 0;
unsigned long alignStartMs = 0;
bool alignErrFilterReady = false;
float alignErrFilteredDeg = 0.0f;

float filteredError = 0.0f;
float prevFilteredError = 0.0f;
float lastSeenError = 0.0f;
unsigned long lastLineLoopMs = 0;
unsigned long lineConfidenceSinceMs = 0;

struct IrMapSample {
  float headingDeg;
  int peakToPeak;
};

IrMapSample irMap[MAX_IR_MAP_SAMPLES];
uint8_t irMapCount = 0;
float irScanAccumDeg = 0.0f;
float irScanPrevHeadingDeg = 0.0f;
float irNextSampleDeg = 0.0f;
bool irScanActive = false;

enum class RunState : uint8_t {
  ScanIr,
  AlignForward,
  FindLineForward,
  FollowLineForward,
  OpenHold,
  AlignReverse,
  FindLineReverse,
  FollowLineReverse,
  Wait,
};

RunState state = RunState::ScanIr;
unsigned long phaseStartMs = 0;

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

float absf(float v) {
  return (v < 0.0f) ? -v : v;
}

float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

uint8_t clampPwm(float v) {
  if (v < 0.0f) return 0;
  if (v > 255.0f) return 255;
  return static_cast<uint8_t>(v + 0.5f);
}

bool selectMuxChannel(uint8_t channel) {
  if (channel > 7) return false;
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  uint8_t err = Wire.endTransmission();
  return err == 0 && !Wire.getWireTimeoutFlag();
}

void brakeMotors() {
  leftMotor.drive(L298NMotor::Direction::Brake, 0);
  rightMotor.drive(L298NMotor::Direction::Brake, 0);
}

void driveScanRotation() {
  leftMotor.drive(L298NMotor::Direction::Backward, SCAN_LEFT_PWM);
  rightMotor.drive(L298NMotor::Direction::Forward, SCAN_RIGHT_PWM);
}

void turnInPlace(float signedPwm) {
  float scale = absf(signedPwm) / ALIGN_MAX_PWM;
  scale = clampf(scale, 0.25f, 1.0f);
  const uint8_t leftPwm = clampPwm(ALIGN_LEFT_PWM_BASE * scale);
  const uint8_t rightPwm = clampPwm(ALIGN_RIGHT_PWM_BASE * scale);

  if (signedPwm >= 0.0f) {
    leftMotor.drive(L298NMotor::Direction::Backward, leftPwm);
    rightMotor.drive(L298NMotor::Direction::Forward, rightPwm);
  } else {
    leftMotor.drive(L298NMotor::Direction::Forward, leftPwm);
    rightMotor.drive(L298NMotor::Direction::Backward, rightPwm);
  }
}

bool initTofRear() {
  if (!selectMuxChannel(REAR_MUX_CHANNEL)) return false;
  rearSensor.setTimeout(TOF_TIMEOUT_MS);
  if (!rearSensor.init()) return false;
  rearSensor.setDistanceMode(VL53L1X::Long);
  rearSensor.setMeasurementTimingBudget(50000);
  rearSensor.startContinuous(TOF_PERIOD_REAR_MS);
  return true;
}

bool readRearMm(uint16_t &mmOut) {
  if (!rearSensorReady) return false;
  if (!selectMuxChannel(REAR_MUX_CHANNEL)) return false;
  const uint16_t mm = rearSensor.read();
  if (rearSensor.timeoutOccurred()) return false;
  mmOut = mm;
  return true;
}

bool initImu() {
  if (!selectMuxChannel(IMU_MUX_CHANNEL)) return false;
  if (!imu.begin_I2C()) return false;
  imu.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  imu.setGyroDataRate(LSM6DS_RATE_104_HZ);
  imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
  return true;
}

void calibrateGyroBias() {
  float sumZ = 0.0f;
  uint16_t valid = 0;

  sensors_event_t accelEvent;
  sensors_event_t gyroEvent;
  sensors_event_t tempEvent;

  for (uint16_t i = 0; i < GYRO_BIAS_SAMPLES; i++) {
    if (!selectMuxChannel(IMU_MUX_CHANNEL)) {
      delay(GYRO_BIAS_DELAY_MS);
      continue;
    }

    imu.getEvent(&accelEvent, &gyroEvent, &tempEvent);
    sumZ += gyroEvent.gyro.z * 180.0f / PI;
    valid++;
    delay(GYRO_BIAS_DELAY_MS);
  }

  gyroBiasZDps = (valid > 0) ? (sumZ / valid) : 0.0f;
}

void initializeHeadingEstimate() {
  fusedHeadingDeg = 0.0f;
  headingReady = true;
}

void updateHeadingFusion() {
  if (!imuReady) return;
  if (!selectMuxChannel(IMU_MUX_CHANNEL)) return;

  sensors_event_t accelEvent;
  sensors_event_t gyroEvent;
  sensors_event_t tempEvent;
  imu.getEvent(&accelEvent, &gyroEvent, &tempEvent);

  const uint32_t nowUs = micros();
  float dt = (nowUs - lastImuUs) / 1000000.0f;
  lastImuUs = nowUs;
  if (dt < 0.0f || dt > 0.2f) dt = 0.0f;

  const float rawGyroDps = gyroEvent.gyro.z * 180.0f / PI;
  gyroZDps = rawGyroDps - gyroBiasZDps;

  if (headingReady && dt > 0.0f) {
    fusedHeadingDeg = wrap360(fusedHeadingDeg + (GYRO_SIGN * gyroZDps * dt));
  }
}

int sampleIrPeakToPeak() {
  const unsigned long startUs = micros();
  int minVal = 1023;
  int maxVal = 0;

  while (micros() - startUs < IR_SAMPLE_WINDOW_US) {
    const int sample = analogRead(IR_BEACON_PIN);
    if (sample < minVal) minVal = sample;
    if (sample > maxVal) maxVal = sample;
  }

  return maxVal - minVal;
}

float circularHeadingDistance(float aDeg, float bDeg) {
  return absf(wrap180(aDeg - bDeg));
}

bool computeIrMidpoint(float &peakAHeadingDeg, float &peakBHeadingDeg, float &midHeadingDeg) {
  if (irMapCount < 2) return false;

  int smoothed[MAX_IR_MAP_SAMPLES];
  for (uint8_t i = 0; i < irMapCount; i++) {
    const uint8_t prev = (i == 0) ? (irMapCount - 1) : (i - 1);
    const uint8_t next = (i + 1 == irMapCount) ? 0 : (i + 1);
    smoothed[i] = (irMap[prev].peakToPeak + irMap[i].peakToPeak + irMap[next].peakToPeak) / 3;
  }

  uint8_t bestIdx = 0;
  for (uint8_t i = 1; i < irMapCount; i++) {
    if (smoothed[i] > smoothed[bestIdx]) bestIdx = i;
  }

  bool foundSecond = false;
  uint8_t secondIdx = 0;
  for (uint8_t i = 0; i < irMapCount; i++) {
    if (i == bestIdx) continue;
    if (circularHeadingDistance(irMap[i].headingDeg, irMap[bestIdx].headingDeg) < IR_MIN_PEAK_SEP_DEG) continue;
    if (!foundSecond || smoothed[i] > smoothed[secondIdx]) {
      secondIdx = i;
      foundSecond = true;
    }
  }

  if (!foundSecond) {
    for (uint8_t i = 0; i < irMapCount; i++) {
      if (i == bestIdx) continue;
      if (!foundSecond || smoothed[i] > smoothed[secondIdx]) {
        secondIdx = i;
        foundSecond = true;
      }
    }
  }

  if (!foundSecond) return false;

  peakAHeadingDeg = irMap[bestIdx].headingDeg;
  peakBHeadingDeg = irMap[secondIdx].headingDeg;
  const float delta = wrap180(peakBHeadingDeg - peakAHeadingDeg);
  midHeadingDeg = wrap360(peakAHeadingDeg + 0.5f * delta);
  return true;
}

void startIrScan() {
  irMapCount = 0;
  irScanAccumDeg = 0.0f;
  irNextSampleDeg = 0.0f;
  irScanPrevHeadingDeg = fusedHeadingDeg;
  irScanActive = true;
  driveScanRotation();

  Serial.println(F("IR_SCAN_START"));
  Serial.println(F("IR_MAP,heading_deg,peak_to_peak"));
}

void finishIrScan() {
  brakeMotors();
  irScanActive = false;

  float peakA = 0.0f;
  float peakB = 0.0f;
  float mid = 0.0f;
  if (computeIrMidpoint(peakA, peakB, mid)) {
    travelHeadingDeg = mid;
    travelHeadingReady = true;
    Serial.print(F("IR_PEAK_A_HEADING_DEG,"));
    Serial.println(peakA, 1);
    Serial.print(F("IR_PEAK_B_HEADING_DEG,"));
    Serial.println(peakB, 1);
    Serial.print(F("TRAVEL_HEADING_DEG,"));
    Serial.println(travelHeadingDeg, 1);
  } else {
    travelHeadingDeg = fusedHeadingDeg;
    travelHeadingReady = true;
    Serial.println(F("IR midpoint failed, fallback to current heading"));
  }

  alignInToleranceSinceMs = 0;
  alignStartMs = 0;
  alignErrFilterReady = false;
  alignErrFilteredDeg = 0.0f;
  state = RunState::AlignForward;
  Serial.println(F("IR_SCAN_DONE -> ALIGN_FORWARD"));
}

void scanIrStep() {
  if (!headingReady) {
    brakeMotors();
    return;
  }

  if (!irScanActive) {
    startIrScan();
  }

  const float deltaDeg = wrap180(fusedHeadingDeg - irScanPrevHeadingDeg);
  irScanPrevHeadingDeg = fusedHeadingDeg;
  irScanAccumDeg += absf(deltaDeg);

  while (irNextSampleDeg <= IR_SCAN_TARGET_DEG && irScanAccumDeg >= irNextSampleDeg && irMapCount < MAX_IR_MAP_SAMPLES) {
    const int p2p = sampleIrPeakToPeak();
    irMap[irMapCount].headingDeg = fusedHeadingDeg;
    irMap[irMapCount].peakToPeak = p2p;

    Serial.print(F("IR_MAP,"));
    Serial.print(fusedHeadingDeg, 1);
    Serial.print(F(","));
    Serial.println(p2p);

    irMapCount++;
    irNextSampleDeg += IR_MAP_INTERVAL_DEG;
  }

  if (irScanAccumDeg >= IR_SCAN_TARGET_DEG || irMapCount >= MAX_IR_MAP_SAMPLES) {
    finishIrScan();
  } else {
    driveScanRotation();
  }
}

void resetLineControl() {
  filteredError = 0.0f;
  prevFilteredError = 0.0f;
  lastSeenError = 0.0f;
  lastLineLoopMs = 0;
  lineConfidenceSinceMs = 0;
}

float normalize01(int raw, int whiteVal, int blackVal) {
  const int span = blackVal - whiteVal;
  if (span < 25) return raw / 1023.0f;
  const float norm = static_cast<float>(raw - whiteVal) / static_cast<float>(span);
  return clampf(norm, 0.0f, 1.0f);
}

float readLineErrorAndStrength(float &sumStrengthOut) {
  float weighted = 0.0f;
  float sum = 0.0f;

  for (int i = 0; i < 3; i++) {
    const int raw = analogRead(LINE_SENSOR_PINS[i]);
    const float norm = normalize01(raw, SENSOR_WHITE[i], SENSOR_BLACK[i]);
    const float lineResponse = LINE_IS_DARK ? (1.0f - norm) : norm;
    const float strength = (lineResponse > SENSOR_ACTIVE_MIN) ? lineResponse : 0.0f;

    weighted += strength * LINE_SENSOR_WEIGHTS[i];
    sum += strength;
  }

  sumStrengthOut = sum;
  if (sum <= 0.0f) return lastSeenError;
  return weighted / sum;
}

void driveLineFollow(bool forward, bool allowTracking) {
  const unsigned long now = millis();
  if (lastLineLoopMs != 0 && now - lastLineLoopMs < LOOP_DT_MS) {
    return;
  }
  lastLineLoopMs = now;

  float sumStrength = 0.0f;
  const float error = readLineErrorAndStrength(sumStrength);

  if (allowTracking && sumStrength >= TRACK_CONFIDENCE_MIN) {
    filteredError = ERROR_SMOOTH_ALPHA * filteredError + (1.0f - ERROR_SMOOTH_ALPHA) * error;
    const float dError = filteredError - prevFilteredError;
    prevFilteredError = filteredError;
    lastSeenError = filteredError;

    const float correction = KP * filteredError + KD * dError;
    float leftCmd = 0.0f;
    float rightCmd = 0.0f;

    if (forward) {
      leftCmd = BASE_SPEED_LEFT + correction;
      rightCmd = BASE_SPEED_RIGHT - correction;
      leftCmd = clampf(leftCmd, MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
      rightCmd = clampf(rightCmd, MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
      leftMotor.drive(L298NMotor::Direction::Forward, clampPwm(leftCmd));
      rightMotor.drive(L298NMotor::Direction::Forward, clampPwm(rightCmd));
    } else {
      // Reverse needs opposite diff sign to bend toward the same sensed line side.
      leftCmd = BASE_SPEED_LEFT - correction;
      rightCmd = BASE_SPEED_RIGHT + correction;
      leftCmd = clampf(leftCmd, MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
      rightCmd = clampf(rightCmd, MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
      leftMotor.drive(L298NMotor::Direction::Backward, clampPwm(leftCmd));
      rightMotor.drive(L298NMotor::Direction::Backward, clampPwm(rightCmd));
    }
  } else {
    // Search for line if confidence is too low.
    if (lastSeenError >= 0.0f) {
      leftMotor.drive(L298NMotor::Direction::Forward, SEARCH_SPEED);
      rightMotor.drive(L298NMotor::Direction::Backward, SEARCH_SPEED);
    } else {
      leftMotor.drive(L298NMotor::Direction::Backward, SEARCH_SPEED);
      rightMotor.drive(L298NMotor::Direction::Forward, SEARCH_SPEED);
    }
  }
}

void beginFindLine(bool forward) {
  resetLineControl();
  state = forward ? RunState::FindLineForward : RunState::FindLineReverse;
  Serial.println(forward ? F("FIND_LINE_FORWARD_START") : F("FIND_LINE_REVERSE_START"));
}

void beginFollowLine(bool forward) {
  state = forward ? RunState::FollowLineForward : RunState::FollowLineReverse;
  Serial.println(forward ? F("FOLLOW_LINE_FORWARD_START") : F("FOLLOW_LINE_REVERSE_START"));
}

void findLineStep(bool forward) {
  float sumStrength = 0.0f;
  (void)readLineErrorAndStrength(sumStrength);

  driveLineFollow(forward, false);

  const unsigned long now = millis();
  if (sumStrength >= TRACK_CONFIDENCE_MIN) {
    if (lineConfidenceSinceMs == 0) {
      lineConfidenceSinceMs = now;
    }

    if (now - lineConfidenceSinceMs >= FIND_LINE_LOCK_MS) {
      resetLineControl();
      beginFollowLine(forward);
    }
  } else {
    lineConfidenceSinceMs = 0;
  }
}

void alignToHeadingStep(float targetHeadingDeg, RunState nextState) {
  if (!headingReady) {
    brakeMotors();
    return;
  }

  const float headingErr = wrap180(targetHeadingDeg - fusedHeadingDeg);

  if (alignStartMs == 0) {
    alignStartMs = millis();
  }

  if (!alignErrFilterReady) {
    alignErrFilteredDeg = headingErr;
    alignErrFilterReady = true;
  } else {
    const float eDelta = wrap180(headingErr - alignErrFilteredDeg);
    alignErrFilteredDeg = wrap180(alignErrFilteredDeg + ALIGN_ERROR_FILTER_GAIN * eDelta);
  }

  if (absf(alignErrFilteredDeg) <= ALIGN_TOLERANCE_DEG) {
    if (alignInToleranceSinceMs == 0) {
      alignInToleranceSinceMs = millis();
    }

    if (millis() - alignInToleranceSinceMs >= ALIGN_HOLD_MS) {
      brakeMotors();
      alignStartMs = 0;
      alignInToleranceSinceMs = 0;
      alignErrFilterReady = false;
      beginFindLine(nextState == RunState::FindLineForward);
      return;
    }

    brakeMotors();
    return;
  }

  if (millis() - alignStartMs > ALIGN_TIMEOUT_MS) {
    Serial.println(F("ALIGN_TIMEOUT"));
    brakeMotors();
    alignStartMs = 0;
    alignInToleranceSinceMs = 0;
    alignErrFilterReady = false;
    beginFindLine(nextState == RunState::FindLineForward);
    return;
  }

  alignInToleranceSinceMs = 0;

  float controlErr = alignErrFilteredDeg;
  if (absf(controlErr) < ALIGN_CONTROL_DEADBAND_DEG) {
    controlErr = 0.0f;
  }

  float turnCmd = ALIGN_KP * controlErr - ALIGN_KD * (GYRO_SIGN * gyroZDps);

  const float errForMin = absf(controlErr);
  const float minBlend = clampf((errForMin - ALIGN_CONTROL_DEADBAND_DEG) / 25.0f, 0.0f, 1.0f);
  const float dynMinPwm = ALIGN_MIN_PWM * minBlend;
  const float mag = clampf(absf(turnCmd), dynMinPwm, ALIGN_MAX_PWM);
  turnCmd = (turnCmd >= 0.0f) ? mag : -mag;

  turnInPlace(turnCmd);
}

void followLineForwardStep() {
  driveLineFollow(true, true);

  uint16_t rearMm = 0;
  if (!readRearMm(rearMm)) return;
  if (rearMm < FORWARD_TARGET_MM) return;

  Serial.print(F("FORWARD_TARGET_REACHED_MM,"));
  Serial.println(rearMm);

  brakeMotors();
  dumpServo.write(SERVO_MIN_ANGLE_DEG);
  phaseStartMs = millis();
  state = RunState::OpenHold;
  Serial.println(F("SERVO_OPEN_HOLD_START"));
}

void followLineReverseStep() {
  driveLineFollow(false, true);

  uint16_t rearMm = 0;
  if (!readRearMm(rearMm)) return;
  if (rearMm > REVERSE_TARGET_MM) return;

  Serial.print(F("REVERSE_TARGET_REACHED_MM,"));
  Serial.println(rearMm);

  brakeMotors();
  phaseStartMs = millis();
  state = RunState::Wait;
  Serial.println(F("CYCLE_WAIT_START"));
}

}  // namespace

void setup() {
  Serial.begin(BAUD);

  pinMode(IR_BEACON_PIN, INPUT);
  pinMode(IR_EMITTER_PIN, OUTPUT);
  digitalWrite(IR_EMITTER_PIN, HIGH);

  leftMotor.begin();
  rightMotor.begin();
  brakeMotors();

  dumpServo.attach(SERVO_PIN);
  dumpServo.write(SERVO_MAX_ANGLE_DEG);

  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(5000, true); // 5 ms timeout; auto-reset bus if stuck

  rearSensorReady = initTofRear();
  if (!rearSensorReady) {
    Serial.println(F("WARN: VL53L1X rear init failed; distance stop disabled"));
  }

  imuReady = initImu();
  if (!imuReady) {
    Serial.println(F("WARN: LSM6DSOX init failed; heading held at zero"));
  }

  Serial.println(F("Calibrating gyro bias (keep still)..."));
  if (imuReady) {
    calibrateGyroBias();
  }
  initializeHeadingEstimate();

  lastImuUs = micros();

  state = RunState::ScanIr;
  Serial.println(F("arena_test_3 ready"));
  Serial.println(F("FLOW: scan IR -> align -> find line -> follow forward -> open -> align reverse -> find line -> follow reverse"));
}

void loop() {
  updateHeadingFusion();

  switch (state) {
    case RunState::ScanIr:
      scanIrStep();
      break;

    case RunState::AlignForward:
      if (!travelHeadingReady) {
        brakeMotors();
        break;
      }
      alignToHeadingStep(travelHeadingDeg, RunState::FindLineForward);
      break;

    case RunState::FindLineForward:
      findLineStep(true);
      break;

    case RunState::FollowLineForward:
      followLineForwardStep();
      break;

    case RunState::OpenHold:
      if (millis() - phaseStartMs >= OPEN_HOLD_MS) {
        dumpServo.write(SERVO_MAX_ANGLE_DEG);
        alignStartMs = 0;
        alignInToleranceSinceMs = 0;
        alignErrFilterReady = false;
        state = RunState::AlignReverse;
        Serial.println(F("ALIGN_REVERSE_START"));
      }
      break;

    case RunState::AlignReverse:
      if (!travelHeadingReady) {
        brakeMotors();
        break;
      }
      alignToHeadingStep(wrap360(travelHeadingDeg + 180.0f), RunState::FindLineReverse);
      break;

    case RunState::FindLineReverse:
      findLineStep(false);
      break;

    case RunState::FollowLineReverse:
      followLineReverseStep();
      break;

    case RunState::Wait:
      if (millis() - phaseStartMs >= WAIT_MS) {
        state = RunState::AlignForward;
        alignStartMs = 0;
        alignInToleranceSinceMs = 0;
        alignErrFilterReady = false;
        Serial.println(F("CYCLE_RESTART_ALIGN_FORWARD"));
      }
      break;
  }
}
