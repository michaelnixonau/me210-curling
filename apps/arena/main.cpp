#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>
#include <VL53L1X.h>
#include <L298N.h>
#include <SoftwareSerial.h>
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
constexpr uint8_t SENSOR_CHANNELS[] = {0, 1, 2, 3};
constexpr uint8_t SENSOR_COUNT = sizeof(SENSOR_CHANNELS) / sizeof(SENSOR_CHANNELS[0]);

constexpr uint16_t CALIB_SAMPLES = 800;
constexpr uint16_t CALIB_DELAY_MS = 2;

constexpr unsigned long IR_SAMPLE_WINDOW_US = 10400;
constexpr float MAP_INTERVAL_DEG = 8.0f;
constexpr uint8_t MAX_MAP_SAMPLES = 64;
constexpr float SCAN_TARGET_DEG = 360.0f;
constexpr float MIN_PEAK_SEPARATION_DEG = 40.0f;
constexpr float TURN_TOLERANCE_DEG = 2.0f;

constexpr uint8_t SAMPLES_PER_SENSOR = 10;
constexpr uint16_t LOOP_DELAY_MS = 200;

// constexpr uint8_t LEFT_SCAN_SPEED = 120;
// constexpr uint8_t RIGHT_SCAN_SPEED = 180; 
constexpr uint8_t LEFT_SCAN_SPEED = 145;
constexpr uint8_t RIGHT_SCAN_SPEED = 205; 

constexpr uint8_t LEFT_TURN_SPEED = 144;
constexpr uint8_t RIGHT_TURN_SPEED = 216;

constexpr uint8_t STOP_PULSE_SPEED = 200;
constexpr uint8_t STOP_PULSE_MS = 40;

constexpr uint8_t DRIVE_LEFT_SPEED = 144;
constexpr uint8_t DRIVE_RIGHT_SPEED = 190;
constexpr float DRIVE_HEADING_KP = 2.0f;
constexpr float DRIVE_HEADING_CORR_MAX = 35.0f;

constexpr uint8_t LEFT_DRIVE_SPEED = 180;
constexpr uint8_t RIGHT_DRIVE_SPEED = 210;

constexpr unsigned long OPEN_HOLD_MS = 2000;
constexpr unsigned long WAIT_MS = 5000;
constexpr unsigned long MIDPOINT_BRAKE_HOLD_MS = 150;
constexpr unsigned long FORWARD_DRIVE_MS = 1000;
constexpr uint16_t PROX_PERIOD_MS = 47;
constexpr uint16_t PROX_TIMEOUT_MS = 60;
constexpr uint16_t FORWARD_TARGET_MM = 1800;
constexpr uint16_t REVERSE_TARGET_MM = 150;

L298NMotor leftMotor(LEFT_IN1, LEFT_IN2, LEFT_PWM);
L298NMotor rightMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_PWM);
Adafruit_LSM6DSOX lsm6dsox;
VL53L1X proximity0;
VL53L1X tofSensors[SENSOR_COUNT];
bool tofReady[SENSOR_COUNT] = {false, false, false, false};
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

namespace lineFollow3 {
constexpr uint8_t LINE_UART_RX_PIN = 10;
constexpr uint8_t LINE_UART_TX_PIN = 9;
constexpr long LINE_UART_BAUD = 19200;

constexpr uint8_t SENSOR_COUNT = 6;
constexpr uint8_t RAW_PACKET_SIZE = 8;
constexpr uint8_t FRAME_SYNC_0 = 0xA5;
constexpr uint8_t FRAME_SYNC_1 = 0x5A;
// Physical order is right->left: A0, A1, A2, A3, A4, A5.
// Negative error means line is on the right; positive means line is on the left.
constexpr float SENSOR_WEIGHTS[SENSOR_COUNT] = {-5.0f, -3.0f, -1.0f, 1.0f, 3.0f, 5.0f};
constexpr uint16_t SENSOR_THRESHOLDS[SENSOR_COUNT] = {900, 900, 900, 900, 900, 900};
constexpr bool DARK_IS_BELOW_THRESHOLD = false;
constexpr char CMD_READ_ONCE = 'R';

// --- TUNING PARAMETERS ---
constexpr uint8_t BASE_SPEED_LEFT = 216;
constexpr uint8_t BASE_SPEED_RIGHT = 230;
// Calculates universal multiplier to ensure the right motor always matches the left proportionately
constexpr float RIGHT_MOTOR_SCALAR = static_cast<float>(BASE_SPEED_RIGHT) / static_cast<float>(BASE_SPEED_LEFT); 

constexpr float KP = 25.0f; // Increased to ensure it bites the edge without hardcoding limits
constexpr float KD = 65.0f;
constexpr float ERROR_SMOOTH_ALPHA = 0.45f;

// The secret to smooth tracking/latching: Slows down forward speed proportionately as error increases
constexpr float TURN_DECELERATION = 18.0f; 

constexpr uint8_t SEARCH_SPIN_SPEED_LEFT = 145;
constexpr uint8_t SEARCH_SPIN_SPEED_RIGHT =
  static_cast<uint8_t>(SEARCH_SPIN_SPEED_LEFT * RIGHT_MOTOR_SCALAR + 0.5f);
constexpr unsigned long LOOP_DT_MS = 8;
constexpr unsigned long LINE_RX_TIMEOUT_US = 12000;
constexpr unsigned long LINE_STALE_MS = 120;
constexpr uint8_t TRACK_EXIT_FRAMES = 3; // Reduced to enter search mode quicker if lost

SoftwareSerial lineSerial(LINE_UART_RX_PIN, LINE_UART_TX_PIN);

uint16_t latestRaw[SENSOR_COUNT] = {0};
bool lineFresh = false;
unsigned long lastRawMs = 0;
unsigned long lastLoopMs = 0;
float filteredError = 0.0f;
float prevFilteredError = 0.0f;
float lastSeenError = 0.0f;
uint8_t lostLineFrames = 0;
bool hasLineNow = false;

enum class FollowMode : uint8_t { Search, Track };
FollowMode mode = FollowMode::Search;

// Helpers
float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

uint8_t clampPwm(float speed) {
  if (speed < 0.0f) return 0;
  if (speed > 255.0f) return 255;
  return static_cast<uint8_t>(speed + 0.5f);
}

// Unified motor control that handles discrepancy, clamps, and allows for reverse pivoting
void driveMotors(float leftCmd, float rightCmd) {
  // Apply motor age/strength difference automatically to ALL movements
  rightCmd *= RIGHT_MOTOR_SCALAR;

  L298NMotor::Direction leftDir = leftCmd >= 0.0f ? L298NMotor::Direction::Forward : L298NMotor::Direction::Backward;
  L298NMotor::Direction rightDir = rightCmd >= 0.0f ? L298NMotor::Direction::Forward : L298NMotor::Direction::Backward;

  leftMotor.drive(leftDir, clampPwm(abs(leftCmd)));
  rightMotor.drive(rightDir, clampPwm(abs(rightCmd)));
}

void stopMotors() {
  leftMotor.stop();
  rightMotor.stop();
}

void driveSearchSpin(bool spinLeft) {
  const L298NMotor::Direction leftDir =
      spinLeft ? L298NMotor::Direction::Backward : L298NMotor::Direction::Forward;
  const L298NMotor::Direction rightDir =
      spinLeft ? L298NMotor::Direction::Forward : L298NMotor::Direction::Backward;
  leftMotor.drive(leftDir, SEARCH_SPIN_SPEED_LEFT);
  rightMotor.drive(rightDir, SEARCH_SPIN_SPEED_RIGHT);
}

// UART Code remains untouched
void unpackRawPacket(const uint8_t packet[RAW_PACKET_SIZE], uint16_t rawOut[SENSOR_COUNT]) {
  uint64_t packed = 0;
  for (uint8_t i = 0; i < RAW_PACKET_SIZE; i++) {
    packed |= (static_cast<uint64_t>(packet[i]) << (8u * i));
  }
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    rawOut[i] = static_cast<uint16_t>((packed >> (10u * i)) & 0x03FFu);
  }
}

uint8_t checksumRaw(const uint8_t packet[RAW_PACKET_SIZE]) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < RAW_PACKET_SIZE; i++) {
    crc ^= packet[i];
  }
  return crc;
}

bool requestLatestRaw(uint16_t rawOut[SENSOR_COUNT]) {
  while (lineSerial.available() > 0) lineSerial.read();
  lineSerial.write(static_cast<uint8_t>(CMD_READ_ONCE));

  uint8_t packet[RAW_PACKET_SIZE];
  uint8_t got = 0;
  bool gotSync0 = false;
  bool gotSync1 = false;
  const unsigned long start = micros();
  while (micros() - start < LINE_RX_TIMEOUT_US) {
    while (lineSerial.available() > 0) {
      const uint8_t b = static_cast<uint8_t>(lineSerial.read());

      if (!gotSync0) { gotSync0 = (b == FRAME_SYNC_0); continue; }
      if (!gotSync1) {
        gotSync1 = (b == FRAME_SYNC_1);
        if (!gotSync1) gotSync0 = (b == FRAME_SYNC_0);
        continue;
      }

      if (got < RAW_PACKET_SIZE) { packet[got++] = b; continue; }

      if (b != checksumRaw(packet)) { gotSync0 = false; gotSync1 = false; got = 0; continue; }

      unpackRawPacket(packet, rawOut);
      return true;
    }
  }
  return false;
}

bool isDark(uint16_t raw, uint16_t threshold) {
  return DARK_IS_BELOW_THRESHOLD ? (raw < threshold) : (raw > threshold);
}

bool computeErrorFromRaw(const uint16_t raw[SENSOR_COUNT], float &errorOut, bool activeOut[SENSOR_COUNT]) {
  float weighted = 0.0f;
  float sum = 0.0f;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    const bool active = isDark(raw[i], SENSOR_THRESHOLDS[i]);
    activeOut[i] = active;
    if (!active) continue;
    weighted += SENSOR_WEIGHTS[i];
    sum += 1.0f;
  }
  if (sum <= 0.0f) return false;
  errorOut = weighted / sum;
  return true;
}

void begin() {
  lineSerial.begin(LINE_UART_BAUD);
}

void reset() {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    latestRaw[i] = 0;
  }
  lineFresh = false;
  lastRawMs = 0;
  lastLoopMs = 0;
  filteredError = 0.0f;
  prevFilteredError = 0.0f;
  lastSeenError = 0.0f;
  lostLineFrames = 0;
  hasLineNow = false;
  mode = FollowMode::Search;
}

bool isTrackingLine() {
  return mode == FollowMode::Track && hasLineNow;
}

void step() {
  const unsigned long now = millis();
  if (now - lastLoopMs < LOOP_DT_MS) return;
  lastLoopMs = now;

  uint16_t newRaw[SENSOR_COUNT] = {0};
  if (requestLatestRaw(newRaw)) {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      latestRaw[i] = newRaw[i];
    }
    lineFresh = true;
    lastRawMs = now;
  }

  if (!lineFresh || (now - lastRawMs > LINE_STALE_MS)) {
    mode = FollowMode::Search;
    lostLineFrames = 0;
    hasLineNow = false;
    stopMotors();
    return;
  }

  bool active[SENSOR_COUNT] = {false};
  float error = 0.0f;
  const bool hasLine = computeErrorFromRaw(latestRaw, error, active);
  hasLineNow = hasLine;

  // State Transitions
  if (hasLine) {
    mode = FollowMode::Track; // Jump to track instantly to catch the line
    lostLineFrames = 0;
    lastSeenError = error;
  } else {
    if (lostLineFrames < 255) lostLineFrames++;
    if (lostLineFrames >= TRACK_EXIT_FRAMES) mode = FollowMode::Search;
  }

  // --- CONTROL LOGIC ---

  if (mode == FollowMode::Search) {
    // If we lost the line, pivot in place towards the last known location.
    // This is mathematically superior to guessing left/right sweeps.
    driveSearchSpin(lastSeenError > 0.0f);
  } 
  
  else if (mode == FollowMode::Track) {
    filteredError = ERROR_SMOOTH_ALPHA * filteredError + (1.0f - ERROR_SMOOTH_ALPHA) * error;
    const float dError = filteredError - prevFilteredError;
    prevFilteredError = filteredError;

    // Standard PD correction
    const float correction = KP * filteredError + KD * dError;

    // DYNAMIC DECELERATION: This fixes the "running over the line" issue.
    // As the error gets larger (approaching edges), we pull down the forward speed. 
    // This allows the outer wheels to smoothly reverse if necessary, pivoting the robot.
    float dynamicBase = static_cast<float>(BASE_SPEED_LEFT) - (abs(filteredError) * TURN_DECELERATION);

    // Apply correction
    float leftCmd = dynamicBase - correction;
    float rightCmd = dynamicBase + correction;

    // Clamping is handled safely inside driveMotors()
    driveMotors(leftCmd, rightCmd);
  }

  // Debug Output
  static unsigned long lastPrintMs = 0;
  if (now - lastPrintMs >= 100) {
    lastPrintMs = now;
    Serial.print(mode == FollowMode::Search ? F("SEARCH ") : F("TRACK "));
    Serial.print(F("raw:"));
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      Serial.print(latestRaw[i]);
      if (i < SENSOR_COUNT - 1) Serial.print(',');
    }
    Serial.print(F(" lost="));
    Serial.print(lostLineFrames);
    Serial.print(F(" err="));
    Serial.println(filteredError, 3);
  }
}
}  // namespace lineFollow3

enum class State : uint8_t {
  CORNER_START,
  SCANNING,
  TURNING_TO_MIDPOINT,
  FORWARD,
  LINE_FOLLOW,
  OPEN_HOLD,
  REVERSE,
  WAIT
};

enum class MotionDir : uint8_t {
  NONE,
  POSITIVE,
  NEGATIVE
};

enum class NextAction : uint8_t {
  Idle = 0,
  LeftForward, 
  LeftReverse,
  RightForward,
  RightReverse,
};

struct SensorSnapshot {
  int16_t tofAvgMm[SENSOR_COUNT];
  int16_t irAvg;
  int16_t lineAvg;
};

State state = State::CORNER_START;
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

void beginLineFollowPhase() {
  lineFollow3::reset();
  state = State::LINE_FOLLOW;
  Serial.println(F("CYCLE_LINE_FOLLOW_START"));
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

  if (millis() - phaseStartMs < FORWARD_DRIVE_MS) return;

  beginLineFollowPhase();
}

void lineFollowStep() {
  lineFollow3::step();

  // Do not trigger line-follow completion while still searching for the line.
  if (!lineFollow3::isTrackingLine()) return;

  bool allDark = true;
    for (uint8_t i = 0; i < lineFollow3::SENSOR_COUNT; i++) {
        bool dark = lineFollow3::DARK_IS_BELOW_THRESHOLD
            ? (lineFollow3::latestRaw[i] < lineFollow3::SENSOR_THRESHOLDS[i])
            : (lineFollow3::latestRaw[i] >= lineFollow3::SENSOR_THRESHOLDS[i]);
        if (!dark) { allDark = false; break; }
    }

  uint16_t distanceMm = 0;
  unsigned long diff = millis() - phaseStartMs;
  if (!readProximity0Mm(distanceMm)) return;
  if (diff < 1000) return; // Give it a moment to find the line
  if (distanceMm <= FORWARD_TARGET_MM && !allDark) return;
//   if (!allDark) return;

  // Either distance target reached or hog line reached

  Serial.print(F("LINE_FOLLOW_TARGET_REACHED_MM,"));
  Serial.println(distanceMm);
  dumpServo.write(SERVO_MIN_ANGLE_DEG);
  // brake rapidly
  leftMotor.drive(L298NMotor::Direction::Backward, -255);
  rightMotor.drive(L298NMotor::Direction::Backward, -255);
  delay(250);
  brakeMotors();
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

bool initTofSensors() {
  bool anyReady = false;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    const uint8_t channel = SENSOR_CHANNELS[i];

    if (!selectMuxChannel(channel)) {
      Serial.print(F("Mux select failed on channel "));
      Serial.println(channel);
      continue;
    }

    tofSensors[i].setTimeout(60);
    if (!tofSensors[i].init()) {
      Serial.print(F("TOF init failed on channel "));
      Serial.println(channel);
      continue;
    }

    tofSensors[i].setDistanceMode(VL53L1X::Long);
    tofSensors[i].setMeasurementTimingBudget(50000);
    tofSensors[i].startContinuous(50);

    tofReady[i] = true;
    anyReady = true;

    Serial.print(F("TOF ready on channel "));
    Serial.println(channel);
  }

  return anyReady;
}

int16_t readAveragedTof(uint8_t sensorIndex) {
  if (sensorIndex >= SENSOR_COUNT || !tofReady[sensorIndex]) return -1;

  const uint8_t channel = SENSOR_CHANNELS[sensorIndex];
  int32_t sum = 0;
  uint8_t validCount = 0;

  for (uint8_t sample = 0; sample < SAMPLES_PER_SENSOR; sample++) {
    if (!selectMuxChannel(channel)) continue;

    uint16_t mm = tofSensors[sensorIndex].read();
    if (!tofSensors[sensorIndex].timeoutOccurred()) {
      sum += mm;
      validCount++;
    }
  }

  if (validCount == 0) return -1;
  return static_cast<int16_t>(sum / validCount);
}

int16_t readAveragedAnalog(uint8_t pin) {
  int32_t sum = 0;
  for (uint8_t sample = 0; sample < SAMPLES_PER_SENSOR; sample++) {
    sum += analogRead(pin);
  }

  return static_cast<int16_t>(sum / SAMPLES_PER_SENSOR);
}

SensorSnapshot takeMeasurements() {
  SensorSnapshot snapshot{};

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    snapshot.tofAvgMm[i] = readAveragedTof(i);
  }

  snapshot.irAvg = readAveragedAnalog(IR_PIN);
  snapshot.lineAvg = readAveragedAnalog(LINE_SENSOR_PIN);

  return snapshot;
}

NextAction decideNextAction(const SensorSnapshot &snapshot) {
    const int16_t backDist = snapshot.tofAvgMm[0];
    const int16_t leftDist = snapshot.tofAvgMm[1];
    const int16_t frontDist = snapshot.tofAvgMm[2];
    const int16_t rightDist = snapshot.tofAvgMm[3];

    if (backDist > 254) {
        if (leftDist > rightDist) {
            return NextAction::LeftReverse;
        } else {
            return NextAction::RightReverse;
        }
    } else if (frontDist > 254) {
        if (leftDist > rightDist) {
            return NextAction::RightForward;
        } else {
            return NextAction::LeftForward;
        }
    }

//   const int16_t frontDistanceMm = snapshot.tofAvgMm[2];
//   const int16_t 

//   if (frontDistanceMm > 0 && frontDistanceMm < 250) {
//     return NextAction::Reverse;
//   }

//   if (snapshot.lineAvg < 300) {
//     return NextAction::TurnRight;
//   }

//   if (snapshot.irAvg > 700) {
//     return NextAction::TurnLeft;
//   }

  return NextAction::Idle;
}

void handleNextAction(NextAction action, const SensorSnapshot &snapshot) {
  Serial.print(F("TOF avg mm: "));
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(snapshot.tofAvgMm[i]);
    if (i + 1 < SENSOR_COUNT) Serial.print(F(", "));
  }

  Serial.print(F(" | IR avg: "));
  Serial.print(snapshot.irAvg);
  Serial.print(F(" | Line avg: "));
  Serial.print(snapshot.lineAvg);
  Serial.print(F(" | Action: "));

  switch (action) {
    case NextAction::Idle:
      Serial.println(F("Idle"));
      break;
    case NextAction::LeftForward:
      Serial.println(F("LeftForward"));
      // move left motor forward, right motor stopped
      leftMotor.drive(L298NMotor::Direction::Forward, LEFT_DRIVE_SPEED);
      rightMotor.drive(L298NMotor::Direction::Brake, 0);
      break;
    case NextAction::LeftReverse:
      Serial.println(F("LeftReverse"));
        // move left motor backward, right motor stopped
        leftMotor.drive(L298NMotor::Direction::Backward, LEFT_DRIVE_SPEED);
        rightMotor.drive(L298NMotor::Direction::Brake, 0);
      break;
    case NextAction::RightForward:
      Serial.println(F("RightForward"));
        // move right motor forward, left motor stopped
        rightMotor.drive(L298NMotor::Direction::Forward, RIGHT_DRIVE_SPEED);
        leftMotor.drive(L298NMotor::Direction::Brake, 0);
      break;
    case NextAction::RightReverse:
      Serial.println(F("RightReverse"));
        // move right motor backward, left motor stopped
        rightMotor.drive(L298NMotor::Direction::Backward, RIGHT_DRIVE_SPEED);
        leftMotor.drive(L298NMotor::Direction::Brake, 0);
      break;
  }

  delay(750);

  // stop all motors
  leftMotor.stop();
  rightMotor.stop();

  delay(150);

  SensorSnapshot snap = takeMeasurements();

  if (snap.tofAvgMm[0] < snap.tofAvgMm[2]) {
    leftMotor.drive(L298NMotor::Direction::Forward, LEFT_DRIVE_SPEED);
    rightMotor.drive(L298NMotor::Direction::Forward, RIGHT_DRIVE_SPEED);
  } else {
    leftMotor.drive(L298NMotor::Direction::Backward, LEFT_DRIVE_SPEED);
    rightMotor.drive(L298NMotor::Direction::Backward, RIGHT_DRIVE_SPEED);
  }

  delay(750);

    // stop all motors
  leftMotor.stop();
  rightMotor.stop();

  delay(150);

  // Transition to arena test
  Serial.println(F("Transitioning to arena test scan phase"));
  startScan();
  state = State::SCANNING;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  lineFollow3::begin();
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

  Serial.println(F("corner_start_1 boilerplate starting..."));

  if (!initTofSensors()) {
    Serial.println(F("Warning: no TOF sensors initialized."));
  }

  calibrateGyroBias();
  lastUs = micros();
  // Do not start scan yet, first do corner start
}

void loop() {
  updateGyro();

  switch (state) {
    case State::CORNER_START:
      {
        SensorSnapshot snapshot = takeMeasurements();
        NextAction action = decideNextAction(snapshot);
        handleNextAction(action, snapshot);
      }
      break;

    case State::SCANNING:
      scanStep();
      break;

    case State::TURNING_TO_MIDPOINT:
      turnToMidpointStep();
      break;

    case State::FORWARD:
      forwardStep();
      break;

    case State::LINE_FOLLOW:
      lineFollowStep();
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
