/**
 * ARENA TEST 1.1
 * IMPROVEMENTS: Adds better beacon finding.
 * (1) 360 scan to find beacons
 * (2) Drive forward to the hog line
 * (3) Release puck (servo)
 * (4) Return to bay
 * (5) Manual reload
 * (6) Repeat (2) to (5) for next pucks
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>
#include <VL53L1X.h>
#include <L298N.h>
#include <Servo.h>

namespace {
constexpr uint8_t RIGHT_IN1 = 7;
constexpr uint8_t RIGHT_IN2 = 8;
constexpr uint8_t RIGHT_PWM = 6;
constexpr uint8_t LEFT_IN1 = 4;
constexpr uint8_t LEFT_IN2 = 5;
constexpr uint8_t LEFT_PWM = 3;
constexpr uint8_t IR_PIN    = A0;
constexpr uint8_t LINE_SENSOR_PIN = A1;

constexpr uint8_t SERVO_PIN = 11;
constexpr uint8_t SERVO_MIN_ANGLE_DEG = 73;
constexpr uint8_t SERVO_MAX_ANGLE_DEG = 157;

constexpr uint8_t PCA9548A_ADDR = 0x70;
constexpr uint8_t IMU_MUX_CHANNEL = 4;
constexpr uint8_t PROX_MUX_CHANNEL = 0;

constexpr uint16_t CALIB_SAMPLES = 800;
constexpr uint16_t CALIB_DELAY_MS = 2;

constexpr unsigned long IR_SAMPLE_WINDOW_US = 10400;
constexpr float MAP_INTERVAL_DEG = 8.0f;
constexpr uint8_t MAX_MAP_SAMPLES = 64;
constexpr float SCAN_TARGET_DEG = 360.0f;
constexpr float MIN_PEAK_SEPARATION_DEG = 40.0f;
constexpr float TURN_TOLERANCE_DEG = 2.0f;

// constexpr uint8_t LEFT_SCAN_SPEED = 120;
// constexpr uint8_t RIGHT_SCAN_SPEED = 180;

// constexpr uint8_t LEFT_TURN_SPEED = 120;
// constexpr uint8_t RIGHT_TURN_SPEED = 180;

// constexpr uint8_t STOP_PULSE_SPEED = 130;
// constexpr uint8_t STOP_PULSE_MS = 40;

// constexpr uint8_t DRIVE_LEFT_SPEED = 120;
// constexpr uint8_t DRIVE_RIGHT_SPEED = 180;

// ------

constexpr uint8_t LEFT_SCAN_SPEED = 120;
constexpr uint8_t RIGHT_SCAN_SPEED = 180; 

constexpr uint8_t LEFT_TURN_SPEED = 144;
constexpr uint8_t RIGHT_TURN_SPEED = 216;

constexpr uint8_t STOP_PULSE_SPEED = 200;
constexpr uint8_t STOP_PULSE_MS = 40;

constexpr uint8_t DRIVE_LEFT_SPEED = 144;
constexpr uint8_t DRIVE_RIGHT_SPEED = 190;
constexpr float DRIVE_HEADING_KP = 2.0f;
constexpr float DRIVE_HEADING_CORR_MAX = 35.0f;

// ------

constexpr unsigned long OPEN_HOLD_MS = 2000;
constexpr unsigned long WAIT_MS = 5000;
constexpr unsigned long MIDPOINT_BRAKE_HOLD_MS = 150;
constexpr uint16_t PROX_PERIOD_MS = 47;
constexpr uint16_t PROX_TIMEOUT_MS = 60;
constexpr uint16_t FORWARD_TARGET_MM = 1800;
constexpr uint16_t REVERSE_TARGET_MM = 150;

L298NMotor leftMotor(LEFT_IN1, LEFT_IN2, LEFT_PWM);
L298NMotor rightMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_PWM);
Adafruit_LSM6DSOX lsm6dsox;
VL53L1X proximity0;
Servo dumpServo;

float gbz = 0.0f;
double totalZ = 0.0;
uint32_t lastUs = 0;

double scanStartZ = 0.0;
float nextSampleAngleDeg = 0.0f;

struct MapSample {
  float angleDeg;
  int peakToPeak;
};

MapSample mapSamples[MAX_MAP_SAMPLES];
uint8_t sampleCount = 0;
float midpointAngleDeg = 0.0f;
double driveHeadingTargetZ = 0.0;

unsigned long phaseStartMs = 0;

enum class State : uint8_t {
  SCANNING,
  TURNING_TO_MIDPOINT,
  FORWARD,
  OPEN_HOLD,
  REVERSE,
  WAIT
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

uint8_t clampPwm(float value) {
  if (value < 0.0f) return 0;
  if (value > 255.0f) return 255;
  return static_cast<uint8_t>(value + 0.5f);
}

float clampf(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

bool selectMuxChannel(uint8_t channel) {
  if (channel > 7) return false;

  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  return Wire.endTransmission() == 0;
}

bool readProximity0Mm(uint16_t &mmOut) {
  if (!selectMuxChannel(PROX_MUX_CHANNEL)) return false;

  uint16_t mm = proximity0.read();
  if (proximity0.timeoutOccurred()) return false;

  mmOut = mm;
  return true;
}

void brakeMotors() {
  leftMotor.drive(L298NMotor::Direction::Brake, 0);
  rightMotor.drive(L298NMotor::Direction::Brake, 0);
  motionDir = MotionDir::NONE;
}

void stopTurnMotors() {
  if (motionDir == MotionDir::POSITIVE) {
    leftMotor.drive(L298NMotor::Direction::Forward, STOP_PULSE_SPEED);
    rightMotor.drive(L298NMotor::Direction::Backward, STOP_PULSE_SPEED);
    delay(STOP_PULSE_MS);
  } else if (motionDir == MotionDir::NEGATIVE) {
    leftMotor.drive(L298NMotor::Direction::Backward, STOP_PULSE_SPEED);
    rightMotor.drive(L298NMotor::Direction::Forward, STOP_PULSE_SPEED);
    delay(STOP_PULSE_MS);
  }

  brakeMotors();
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

void driveForward() {
  leftMotor.drive(L298NMotor::Direction::Forward, DRIVE_LEFT_SPEED);
  rightMotor.drive(L298NMotor::Direction::Forward, DRIVE_RIGHT_SPEED);
  motionDir = MotionDir::NONE;
}

void driveBackward() {
  leftMotor.drive(L298NMotor::Direction::Backward, DRIVE_LEFT_SPEED);
  rightMotor.drive(L298NMotor::Direction::Backward, DRIVE_RIGHT_SPEED);
  motionDir = MotionDir::NONE;
}

void driveWithHeadingHold(bool forward) {
  float errorDeg = wrap180(static_cast<float>(driveHeadingTargetZ - totalZ));
  float correction = clampf(DRIVE_HEADING_KP * errorDeg, -DRIVE_HEADING_CORR_MAX, DRIVE_HEADING_CORR_MAX);

  float leftSpeed = DRIVE_LEFT_SPEED;
  float rightSpeed = DRIVE_RIGHT_SPEED;

  if (forward) {
    leftSpeed -= correction;
    rightSpeed += correction;
    leftMotor.drive(L298NMotor::Direction::Forward, clampPwm(leftSpeed));
    rightMotor.drive(L298NMotor::Direction::Forward, clampPwm(rightSpeed));
  } else {
    leftSpeed += correction;
    rightSpeed -= correction;
    leftMotor.drive(L298NMotor::Direction::Backward, clampPwm(leftSpeed));
    rightMotor.drive(L298NMotor::Direction::Backward, clampPwm(rightSpeed));
  }
}

void calibrateGyroBias() {
  Serial.println(F("Calibrating gyro (keep robot still)..."));
  if (!selectMuxChannel(IMU_MUX_CHANNEL)) {
    Serial.println(F("PCA9548A IMU ch select FAIL"));
    return;
  }

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

  if (!selectMuxChannel(IMU_MUX_CHANNEL)) return;

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

  // 1. Find min and max IR readings to determine a dynamic threshold
  int minVal = mapSamples[0].peakToPeak;
  int maxVal = mapSamples[0].peakToPeak;
  for (uint8_t i = 1; i < sampleCount; i++) {
    if (mapSamples[i].peakToPeak < minVal) minVal = mapSamples[i].peakToPeak;
    if (mapSamples[i].peakToPeak > maxVal) maxVal = mapSamples[i].peakToPeak;
  }

  // Optional: Reject scan if the difference is too small (e.g., beacons are off/ambient noise)
  if ((maxVal - minVal) < 50) return false;

  // Set threshold at 50% between min and max (adjust this ratio if needed)
  int threshold = minVal + (maxVal - minVal) / 2;

  // 2. Find a definitive 'low' spot so we don't start looping in the middle of a spike
  int startIdx = -1;
  for (uint8_t i = 0; i < sampleCount; i++) {
    if (mapSamples[i].peakToPeak < threshold) {
      startIdx = i;
      break;
    }
  }

  if (startIdx == -1) return false; // Signal never dropped below threshold

  float centers[MAX_MAP_SAMPLES];
  uint8_t centerCount = 0;
  bool inSpike = false;
  float spikeStartDeg = 0.0f;
  float spikeEndDeg = 0.0f;

  // 3. Sweep through the circular array to find the bounds of each spike
  // Stepping up to sampleCount (inclusive) guarantees we check the startIdx again 
  // at the very end to cleanly close out any spike that wraps around the 360 mark.
  for (uint8_t step = 0; step <= sampleCount; step++) {
    uint8_t i = (startIdx + step) % sampleCount;
    bool overThresh = (mapSamples[i].peakToPeak >= threshold);

    if (overThresh && !inSpike) {
      // Just entered a new spike
      inSpike = true;
      spikeStartDeg = mapSamples[i].angleDeg;
      spikeEndDeg = spikeStartDeg; 
    } else if (overThresh && inSpike) {
      // Continuing inside the spike
      spikeEndDeg = mapSamples[i].angleDeg;
    } else if (!overThresh && inSpike) {
      // Just exited the spike; calculate its true center
      inSpike = false;
      float delta = wrap180(spikeEndDeg - spikeStartDeg);
      centers[centerCount] = wrap360(spikeStartDeg + delta / 2.0f);
      centerCount++;
    }
  }

  if (centerCount < 2) {
    return false; // Did not find at least two distinct beacons
  }

  // 4. Calculate the midpoint between the first two detected beacon centers
  // (Assuming a clean environment. If reflections exist, you might need to sort by spike width)
  peak1Deg = centers[0];
  peak2Deg = centers[1];

  float deltaCenters = wrap180(peak2Deg - peak1Deg);
  midDeg = wrap360(peak1Deg + 0.5f * deltaCenters);

  return true;
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

void beginForwardPhase() {
  dumpServo.write(SERVO_MAX_ANGLE_DEG);
  driveHeadingTargetZ = totalZ;
  driveForward();
  phaseStartMs = millis();
  state = State::FORWARD;
  Serial.println(F("CYCLE_FORWARD_START"));
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
    stopTurnMotors();

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
      beginForwardPhase();
    }
  }
}

void turnToMidpointStep() {
  float currentAngle = wrap360(static_cast<float>(totalZ - scanStartZ));
  float remaining = wrap180(midpointAngleDeg - currentAngle);

  if (absf(remaining) <= TURN_TOLERANCE_DEG) {
    stopTurnMotors();
    brakeMotors();
    delay(MIDPOINT_BRAKE_HOLD_MS);
    Serial.println(F("TURN_TO_MIDPOINT_DONE"));
    Serial.print(F("FINAL_ANGLE_DEG,"));
    Serial.println(currentAngle, 1);
    beginForwardPhase();
    return;
  }

  startTurnBySign(remaining);
}

void forwardStep() {
  driveWithHeadingHold(true);

  uint16_t distanceMm = 0;
  if (!readProximity0Mm(distanceMm)) return;
  if (distanceMm < FORWARD_TARGET_MM) return;

  Serial.print(F("FORWARD_TARGET_REACHED_MM,"));
  Serial.println(distanceMm);
  brakeMotors();
  dumpServo.write(SERVO_MIN_ANGLE_DEG);
  phaseStartMs = millis();
  state = State::OPEN_HOLD;
  Serial.println(F("SERVO_OPEN_HOLD_START"));
}

void openHoldStep() {
  if (millis() - phaseStartMs < OPEN_HOLD_MS) return;

  dumpServo.write(SERVO_MAX_ANGLE_DEG);
  driveHeadingTargetZ = totalZ;
  driveBackward();
  phaseStartMs = millis();
  state = State::REVERSE;
  Serial.println(F("CYCLE_REVERSE_START"));
}

void reverseStep() {
  driveWithHeadingHold(false);

  uint16_t distanceMm = 0;
  if (!readProximity0Mm(distanceMm)) return;
  if (distanceMm > REVERSE_TARGET_MM) return;

  Serial.print(F("REVERSE_TARGET_REACHED_MM,"));
  Serial.println(distanceMm);

  brakeMotors();
  phaseStartMs = millis();
  state = State::WAIT;
  Serial.println(F("CYCLE_WAIT_START"));
}

void waitStep() {
  if (millis() - phaseStartMs < WAIT_MS) return;

  beginForwardPhase();
}

}  // namespace

void setup() {
  Serial.begin(115200);
  pinMode(IR_PIN, INPUT);
  pinMode(LINE_SENSOR_PIN, INPUT);

  leftMotor.begin();
  rightMotor.begin();

  dumpServo.attach(SERVO_PIN);
  dumpServo.write(SERVO_MAX_ANGLE_DEG);

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

  if (!selectMuxChannel(PROX_MUX_CHANNEL)) {
    Serial.println(F("PCA9548A ch0 select FAIL"));
    while (1) delay(10);
  }

  proximity0.setTimeout(PROX_TIMEOUT_MS);
  if (!proximity0.init()) {
    Serial.println(F("VL53L1X ch0 FAIL"));
    while (1) delay(10);
  }
  proximity0.setDistanceMode(VL53L1X::Long);
  proximity0.setMeasurementTimingBudget(50000);
  proximity0.startContinuous(PROX_PERIOD_MS);
  Serial.println(F("VL53L1X OK (via PCA9548A ch0)"));

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

    case State::FORWARD:
      forwardStep();
      break;

    case State::OPEN_HOLD:
      openHoldStep();
      break;

    case State::REVERSE:
      reverseStep();
      break;

    case State::WAIT:
      waitStep();
      break;
  }
}
