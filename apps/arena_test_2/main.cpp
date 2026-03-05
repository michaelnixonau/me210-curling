#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <Adafruit_QMC5883P.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>
#include <CompassCalibration.h>
#include <L298N.h>

namespace {

constexpr uint32_t BAUD = 115200;

constexpr uint8_t RIGHT_IN1 = 7;
constexpr uint8_t RIGHT_IN2 = 8;
constexpr uint8_t RIGHT_PWM = 6;
constexpr uint8_t LEFT_IN1 = 4;
constexpr uint8_t LEFT_IN2 = 5;
constexpr uint8_t LEFT_PWM = 3;
constexpr uint8_t IR_PIN = A0;

constexpr uint8_t PCA9548A_ADDR = 0x70;
constexpr uint8_t REAR_MUX_CHANNEL = 0;     // VL53L1X rear
constexpr uint8_t LEFT_MUX_CHANNEL = 1;     // VL53L1X left
constexpr uint8_t RIGHT_MUX_CHANNEL = 3;    // VL53L1X right
constexpr uint8_t IMU_MUX_CHANNEL = 4;      // LSM6DSOX
constexpr uint8_t COMPASS_MUX_CHANNEL = 7;  // QMC5883P

constexpr uint16_t TOF_TIMEOUT_MS = 60;
constexpr uint16_t TOF_PERIOD_REAR_MS = 47;
constexpr uint16_t TOF_PERIOD_LEFT_MS = 53;
constexpr uint16_t TOF_PERIOD_RIGHT_MS = 59;

constexpr uint16_t TARGET_REAR_MM = 1800;  // 180 cm
constexpr uint16_t MIN_VALID_MM = 60;
constexpr uint16_t MAX_VALID_MM = 3800;

constexpr float ARENA_WIDTH_MM = 4.0f * 12.0f * 25.4f;  // 4 ft
constexpr float ROBOT_WIDTH_MM = 12.5f * 25.4f;         // 12.5 in
constexpr float TARGET_SIDE_GAP_MM = (ARENA_WIDTH_MM - ROBOT_WIDTH_MM) * 0.5f;

constexpr float GYRO_SIGN = -1.0f;  // matched to robot_localisation convention
constexpr uint16_t GYRO_BIAS_SAMPLES = 300;
constexpr uint16_t GYRO_BIAS_DELAY_MS = 2;

constexpr uint8_t DRIVE_LEFT_PWM_BASE = 144;   // arena_test_1 baseline
constexpr uint8_t DRIVE_RIGHT_PWM_BASE = 190;  // arena_test_1 baseline
constexpr uint8_t TURN_LEFT_PWM_BASE = 144;    // arena_test_1 baseline
constexpr uint8_t TURN_RIGHT_PWM_BASE = 216;   // arena_test_1 baseline
constexpr uint8_t SCAN_LEFT_PWM = 144;         // arena_test_1 baseline
constexpr uint8_t SCAN_RIGHT_PWM = 216;        // arena_test_1 baseline

constexpr float ALIGN_TOLERANCE_DEG = 4.0f;
constexpr uint16_t ALIGN_HOLD_MS = 300;
constexpr uint16_t ALIGN_TIMEOUT_MS = 6500;
constexpr float ALIGN_KP = 2.2f;
constexpr float ALIGN_KD = 0.45f;
constexpr uint8_t ALIGN_MIN_PWM = 40;
constexpr uint8_t ALIGN_MAX_PWM = 145;
constexpr float ALIGN_CONTROL_DEADBAND_DEG = 1.8f;
constexpr float ALIGN_ERROR_FILTER_GAIN = 0.22f;

constexpr float DRIVE_HEADING_KP = 1.10f;
constexpr float DRIVE_HEADING_KD = 0.28f;
constexpr float DRIVE_HEADING_MAX = 18.0f;

constexpr float CENTER_KP = 0.028f;
constexpr float CENTER_MAX = 14.0f;
constexpr float CENTER_DEADBAND_MM = 16.0f;

constexpr float MAG_LPF_GAIN = 0.22f;
constexpr float FUSE_GAIN = 0.04f;
constexpr float FUSE_GAIN_MOVING = 0.02f;
constexpr float GYRO_MOVING_THRESH_DPS = 10.0f;
constexpr float MAG_OUTLIER_REJECT_DEG = 35.0f;
constexpr float COMMAND_LPF_GAIN = 0.30f;

constexpr uint8_t COMPASS_SAMPLES_PER_READ = 7;
constexpr uint16_t COMPASS_SAMPLE_WAIT_US = 3500;
constexpr uint16_t COMPASS_INTER_SAMPLE_DELAY_US = 700;

constexpr unsigned long IR_SAMPLE_WINDOW_US = 10400;
constexpr float IR_MAP_INTERVAL_DEG = 8.0f;
constexpr float IR_SCAN_TARGET_DEG = 360.0f;
constexpr float IR_MIN_PEAK_SEP_DEG = 40.0f;
constexpr uint8_t MAX_IR_MAP_SAMPLES = 64;
constexpr uint32_t LOG_PERIOD_MS = 120;

L298NMotor leftMotor(LEFT_IN1, LEFT_IN2, LEFT_PWM);
L298NMotor rightMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_PWM);
VL53L1X rearSensor;
VL53L1X leftSensor;
VL53L1X rightSensor;
Adafruit_LSM6DSOX imu;
Adafruit_QMC5883P compass;

float gyroBiasZDps = 0.0f;
float gyroZDps = 0.0f;
uint32_t lastImuUs = 0;

bool magReady = false;
float magHeadingDeg = 0.0f;

bool headingReady = false;
float fusedHeadingDeg = 0.0f;

bool travelHeadingReady = false;
float travelHeadingDeg = 0.0f;

float headingCorrFiltered = 0.0f;
float centerCorrFiltered = 0.0f;
float steerSign = 1.0f;
bool steerSignLocked = false;

bool steerProbeActive = false;
unsigned long steerProbeStartMs = 0;
float steerProbeStartHeadingDeg = 0.0f;
float steerProbeCmdSign = 0.0f;

bool rearDirectionIncreasing = true;
uint16_t rearStartMm = 0;

unsigned long alignInToleranceSinceMs = 0;
unsigned long alignStartMs = 0;
bool alignErrFilterReady = false;
float alignErrFilteredDeg = 0.0f;
unsigned long lastLogMs = 0;

struct IrMapSample {
  float headingDeg;
  int peakToPeak;
};

IrMapSample irMap[MAX_IR_MAP_SAMPLES];
uint8_t irMapCount = 0;
bool irScanActive = false;
float irScanAccumDeg = 0.0f;
float irScanPrevHeadingDeg = 0.0f;
float irNextSampleDeg = 0.0f;

enum class RunState : uint8_t {
  ScanIr,
  AlignToTravel,
  DriveTravel,
  Done,
};

RunState state = RunState::ScanIr;

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

float clampf(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

uint8_t clampPwm(float value) {
  if (value < 0.0f) return 0;
  if (value > 255.0f) return 255;
  return static_cast<uint8_t>(value + 0.5f);
}

bool selectMuxChannel(uint8_t channel) {
  if (channel > 7) return false;
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  return Wire.endTransmission() == 0;
}

void brakeMotors() {
  leftMotor.drive(L298NMotor::Direction::Brake, 0);
  rightMotor.drive(L298NMotor::Direction::Brake, 0);
}

void turnInPlace(float signedPwm) {
  // Preserve L/R asymmetry but allow much gentler near-target turning.
  float scale = absf(signedPwm) / ALIGN_MAX_PWM;
  scale = clampf(scale, 0.25f, 1.0f);
  const uint8_t leftPwm = clampPwm(TURN_LEFT_PWM_BASE * scale);
  const uint8_t rightPwm = clampPwm(TURN_RIGHT_PWM_BASE * scale);
  if (signedPwm >= 0.0f) {
    leftMotor.drive(L298NMotor::Direction::Backward, leftPwm);
    rightMotor.drive(L298NMotor::Direction::Forward, rightPwm);
  } else {
    leftMotor.drive(L298NMotor::Direction::Forward, leftPwm);
    rightMotor.drive(L298NMotor::Direction::Backward, rightPwm);
  }
}

void driveForwardWithDiff(float diff) {
  const float left = DRIVE_LEFT_PWM_BASE - diff;
  const float right = DRIVE_RIGHT_PWM_BASE + diff;
  leftMotor.drive(L298NMotor::Direction::Forward, clampPwm(left));
  rightMotor.drive(L298NMotor::Direction::Forward, clampPwm(right));
}

void driveScanRotation() {
  leftMotor.drive(L298NMotor::Direction::Backward, SCAN_LEFT_PWM);
  rightMotor.drive(L298NMotor::Direction::Forward, SCAN_RIGHT_PWM);
}

bool initTof(VL53L1X &sensor, uint8_t muxChannel, uint16_t periodMs) {
  if (!selectMuxChannel(muxChannel)) return false;

  sensor.setTimeout(TOF_TIMEOUT_MS);
  if (!sensor.init()) return false;

  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);
  sensor.startContinuous(periodMs);
  return true;
}

bool readTofMm(VL53L1X &sensor, uint8_t muxChannel, uint16_t &mmOut) {
  if (!selectMuxChannel(muxChannel)) return false;
  const uint16_t raw = sensor.read();
  if (sensor.timeoutOccurred()) return false;
  if (raw < MIN_VALID_MM || raw > MAX_VALID_MM) return false;
  mmOut = raw;
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

bool readCompassHeading(float &headingDegOut) {
  if (!selectMuxChannel(COMPASS_MUX_CHANNEL)) return false;

  float sumSin = 0.0f;
  float sumCos = 0.0f;
  uint8_t valid = 0;

  for (uint8_t i = 0; i < COMPASS_SAMPLES_PER_READ; i++) {
    const uint32_t waitStartUs = micros();
    while (!compass.isDataReady()) {
      if (micros() - waitStartUs > COMPASS_SAMPLE_WAIT_US) break;
    }

    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
    if (!compass.getRawMagnetic(&x, &y, &z)) {
      delayMicroseconds(COMPASS_INTER_SAMPLE_DELAY_US);
      continue;
    }

    applyCompassCalibration(x, y);

    const float heading = atan2(static_cast<float>(y), static_cast<float>(x)) * 180.0f / PI;
    const float rad = heading * PI / 180.0f;
    sumSin += sin(rad);
    sumCos += cos(rad);
    valid++;

    delayMicroseconds(COMPASS_INTER_SAMPLE_DELAY_US);
  }

  if (valid == 0) return false;
  headingDegOut = wrap360(atan2(sumSin, sumCos) * 180.0f / PI);
  return true;
}

void initializeHeadingEstimate() {
  float sumSin = 0.0f;
  float sumCos = 0.0f;
  uint8_t valid = 0;

  for (uint8_t i = 0; i < 40; i++) {
    float h = 0.0f;
    if (readCompassHeading(h)) {
      const float rad = h * PI / 180.0f;
      sumSin += sin(rad);
      sumCos += cos(rad);
      valid++;
    }
    delay(8);
  }

  if (valid == 0) {
    headingReady = false;
    magReady = false;
    fusedHeadingDeg = 0.0f;
    return;
  }

  const float meanDeg = wrap360(atan2(sumSin, sumCos) * 180.0f / PI);
  magHeadingDeg = meanDeg;
  fusedHeadingDeg = meanDeg;
  magReady = true;
  headingReady = true;
}

void updateHeadingFusion() {
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
    fusedHeadingDeg = wrap360(fusedHeadingDeg + GYRO_SIGN * gyroZDps * dt);
  }

  float rawMagDeg = 0.0f;
  if (!readCompassHeading(rawMagDeg)) return;

  if (!magReady) {
    magHeadingDeg = rawMagDeg;
    magReady = true;
  } else {
    const float jump = wrap180(rawMagDeg - magHeadingDeg);
    if (absf(jump) <= MAG_OUTLIER_REJECT_DEG) {
      magHeadingDeg = wrap360(magHeadingDeg + MAG_LPF_GAIN * jump);
    }
  }

  if (!headingReady) {
    fusedHeadingDeg = magHeadingDeg;
    headingReady = true;
    return;
  }

  const float headingErr = wrap180(magHeadingDeg - fusedHeadingDeg);
  const float fuseGain =
      (absf(gyroZDps) > GYRO_MOVING_THRESH_DPS) ? FUSE_GAIN_MOVING : FUSE_GAIN;
  fusedHeadingDeg = wrap360(fusedHeadingDeg + fuseGain * headingErr);
}

int sampleIrPeakToPeak() {
  const unsigned long startUs = micros();
  int minVal = 1023;
  int maxVal = 0;

  while (micros() - startUs < IR_SAMPLE_WINDOW_US) {
    const int sample = analogRead(IR_PIN);
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
    Serial.println(F("Compass zero set to IR midpoint (travel heading)."));
  } else {
    travelHeadingDeg = fusedHeadingDeg;
    travelHeadingReady = true;
    Serial.println(F("IR midpoint failed, fallback to current heading."));
  }

  alignInToleranceSinceMs = 0;
  alignStartMs = 0;
  alignErrFilterReady = false;
  alignErrFilteredDeg = 0.0f;
  steerSignLocked = false;
  steerProbeActive = false;
  headingCorrFiltered = 0.0f;
  centerCorrFiltered = 0.0f;
  state = RunState::AlignToTravel;
  Serial.println(F("IR_SCAN_DONE -> ALIGN"));
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

  while (irNextSampleDeg <= IR_SCAN_TARGET_DEG &&
         irScanAccumDeg >= irNextSampleDeg &&
         irMapCount < MAX_IR_MAP_SAMPLES) {
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

bool rearTargetReached(uint16_t rearMm) {
  if (rearDirectionIncreasing) {
    return rearMm >= TARGET_REAR_MM;
  }
  return rearMm <= TARGET_REAR_MM;
}

void logStatus(bool rearOk,
               uint16_t rearMm,
               bool leftOk,
               uint16_t leftMm,
               bool rightOk,
               uint16_t rightMm,
               float headingErr,
               float headingCmd,
               float centerCmd,
               float combinedCmd) {
  const unsigned long nowMs = millis();
  if (nowMs - lastLogMs < LOG_PERIOD_MS) return;
  lastLogMs = nowMs;

  Serial.print(F("state="));
  if (state == RunState::ScanIr) Serial.print(F("SCAN"));
  else if (state == RunState::AlignToTravel) Serial.print(F("ALIGN"));
  else if (state == RunState::DriveTravel) Serial.print(F("DRIVE"));
  else Serial.print(F("DONE"));

  Serial.print(F(" rear="));
  if (rearOk) Serial.print(rearMm); else Serial.print(F("NA"));

  Serial.print(F(" left="));
  if (leftOk) Serial.print(leftMm); else Serial.print(F("NA"));

  Serial.print(F(" right="));
  if (rightOk) Serial.print(rightMm); else Serial.print(F("NA"));

  Serial.print(F(" head="));
  if (headingReady) Serial.print(fusedHeadingDeg, 1); else Serial.print(F("NA"));

  Serial.print(F(" zeroed="));
  if (headingReady && travelHeadingReady) Serial.print(wrap180(fusedHeadingDeg - travelHeadingDeg), 1);
  else Serial.print(F("NA"));

  Serial.print(F(" tgt="));
  if (travelHeadingReady) Serial.print(travelHeadingDeg, 1); else Serial.print(F("NA"));

  Serial.print(F(" e="));
  Serial.print(headingErr, 1);

  Serial.print(F(" gz="));
  Serial.print(gyroZDps, 1);

  Serial.print(F(" hCmd="));
  Serial.print(headingCmd, 1);

  Serial.print(F(" cCmd="));
  Serial.print(centerCmd, 1);

  Serial.print(F(" cmd="));
  Serial.println(combinedCmd, 1);
}

void updateSteerSignProbe(float desiredTurnCmd) {
  if (steerSignLocked || !headingReady) return;

  const float cmdSign = (desiredTurnCmd > 0.0f) ? 1.0f : (desiredTurnCmd < 0.0f ? -1.0f : 0.0f);
  if (cmdSign == 0.0f) {
    steerProbeActive = false;
    return;
  }

  if (!steerProbeActive) {
    steerProbeActive = true;
    steerProbeStartMs = millis();
    steerProbeStartHeadingDeg = fusedHeadingDeg;
    steerProbeCmdSign = cmdSign;
    return;
  }

  if (millis() - steerProbeStartMs < 180) return;

  const float headingDelta = wrap180(fusedHeadingDeg - steerProbeStartHeadingDeg);
  if (absf(headingDelta) < 2.0f) {
    steerProbeActive = false;
    return;
  }

  const float expectedSign = steerSign * steerProbeCmdSign;
  if (headingDelta * expectedSign < 0.0f) {
    steerSign = -steerSign;
    Serial.println(F("STEER_SIGN_FLIPPED"));
  }

  steerSignLocked = true;
  steerProbeActive = false;
}

}  // namespace

void setup() {
  Serial.begin(BAUD);
  pinMode(IR_PIN, INPUT);

  leftMotor.begin();
  rightMotor.begin();
  brakeMotors();

  Wire.begin();
  Wire.setClock(400000);

  if (!initTof(rearSensor, REAR_MUX_CHANNEL, TOF_PERIOD_REAR_MS)) {
    Serial.println(F("VL53L1X rear (ch0) init failed"));
    while (1) delay(10);
  }
  if (!initTof(leftSensor, LEFT_MUX_CHANNEL, TOF_PERIOD_LEFT_MS)) {
    Serial.println(F("VL53L1X left (ch1) init failed"));
    while (1) delay(10);
  }
  if (!initTof(rightSensor, RIGHT_MUX_CHANNEL, TOF_PERIOD_RIGHT_MS)) {
    Serial.println(F("VL53L1X right (ch3) init failed"));
    while (1) delay(10);
  }

  if (!initImu()) {
    Serial.println(F("LSM6DSOX init failed"));
    while (1) delay(10);
  }

  if (!initCompass()) {
    Serial.println(F("QMC5883P init failed"));
    while (1) delay(10);
  }

  Serial.println(F("Calibrating gyro bias (keep still)..."));
  calibrateGyroBias();
  initializeHeadingEstimate();
  lastImuUs = micros();

  uint16_t rearMm = 0;
  if (readTofMm(rearSensor, REAR_MUX_CHANNEL, rearMm)) {
    rearStartMm = rearMm;
    rearDirectionIncreasing = rearStartMm < TARGET_REAR_MM;
  } else {
    rearStartMm = 0;
    rearDirectionIncreasing = true;
  }

  Serial.println(F("arena_test_2 ready"));
  Serial.print(F("target_side_gap_mm="));
  Serial.println(TARGET_SIDE_GAP_MM, 1);
  Serial.print(F("rear_start_mm="));
  Serial.print(rearStartMm);
  Serial.print(F(" rear_target_mm="));
  Serial.print(TARGET_REAR_MM);
  Serial.print(F(" expect_direction="));
  Serial.println(rearDirectionIncreasing ? F("increasing") : F("decreasing"));
  Serial.println(F("SCAN IR -> ALIGN TO MIDPOINT HEADING -> DRIVE"));
}

void loop() {
  updateHeadingFusion();

  uint16_t rearMm = 0;
  const bool rearOk = readTofMm(rearSensor, REAR_MUX_CHANNEL, rearMm);

  uint16_t leftMm = 0;
  const bool leftOk = readTofMm(leftSensor, LEFT_MUX_CHANNEL, leftMm);

  uint16_t rightMm = 0;
  const bool rightOk = readTofMm(rightSensor, RIGHT_MUX_CHANNEL, rightMm);

  if (state == RunState::Done) {
    brakeMotors();
    logStatus(rearOk, rearMm, leftOk, leftMm, rightOk, rightMm, 0.0f, 0.0f, 0.0f, 0.0f);
    return;
  }

  if (state == RunState::ScanIr) {
    scanIrStep();
    logStatus(rearOk, rearMm, leftOk, leftMm, rightOk, rightMm, 0.0f, 0.0f, 0.0f, 0.0f);
    return;
  }

  if (!headingReady || !travelHeadingReady) {
    brakeMotors();
    return;
  }

  const float headingErr = wrap180(travelHeadingDeg - fusedHeadingDeg);

  if (state == RunState::AlignToTravel) {
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
        state = RunState::DriveTravel;
        headingCorrFiltered = 0.0f;
        centerCorrFiltered = 0.0f;
        alignStartMs = 0;
        alignErrFilterReady = false;
        Serial.println(F("ALIGN_DONE -> DRIVE"));
      }

      brakeMotors();
      logStatus(
          rearOk, rearMm, leftOk, leftMm, rightOk, rightMm, alignErrFilteredDeg, 0.0f, 0.0f, 0.0f);
      return;
    }

    if (millis() - alignStartMs > ALIGN_TIMEOUT_MS) {
      state = RunState::DriveTravel;
      headingCorrFiltered = 0.0f;
      centerCorrFiltered = 0.0f;
      alignStartMs = 0;
      alignErrFilterReady = false;
      Serial.println(F("ALIGN_TIMEOUT -> DRIVE"));
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

    updateSteerSignProbe(turnCmd);
    const float appliedTurnCmd = steerSign * turnCmd;
    turnInPlace(appliedTurnCmd);
    logStatus(rearOk,
              rearMm,
              leftOk,
              leftMm,
              rightOk,
              rightMm,
              alignErrFilteredDeg,
              appliedTurnCmd,
              0.0f,
              appliedTurnCmd);
    return;
  }

  if (rearOk && rearTargetReached(rearMm)) {
    brakeMotors();
    state = RunState::Done;
    Serial.print(F("TARGET_REACHED rear_mm="));
    Serial.println(rearMm);
    return;
  }

  float headingCmd = DRIVE_HEADING_KP * headingErr - DRIVE_HEADING_KD * (GYRO_SIGN * gyroZDps);
  headingCmd = clampf(headingCmd, -DRIVE_HEADING_MAX, DRIVE_HEADING_MAX);

  float centerCmd = 0.0f;
  if (leftOk && rightOk) {
    float centerErrorMm = static_cast<float>(leftMm) - static_cast<float>(rightMm);
    if (absf(centerErrorMm) < CENTER_DEADBAND_MM) centerErrorMm = 0.0f;
    centerCmd = clampf(CENTER_KP * centerErrorMm, -CENTER_MAX, CENTER_MAX);
  }

  headingCorrFiltered += COMMAND_LPF_GAIN * (headingCmd - headingCorrFiltered);
  centerCorrFiltered += COMMAND_LPF_GAIN * (centerCmd - centerCorrFiltered);

  float combinedCmd = headingCorrFiltered + centerCorrFiltered;
  combinedCmd = clampf(combinedCmd, -30.0f, 30.0f);
  combinedCmd *= steerSign;

  driveForwardWithDiff(combinedCmd);
  logStatus(rearOk,
            rearMm,
            leftOk,
            leftMm,
            rightOk,
            rightMm,
            headingErr,
            headingCorrFiltered,
            centerCorrFiltered,
            combinedCmd);
}
