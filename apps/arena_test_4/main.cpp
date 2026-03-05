#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>
#include <VL53L1X.h>
#include <L298N.h>
#include <Servo.h>
#include <SoftwareSerial.h>

namespace {
constexpr uint32_t BAUD = 115200;

constexpr uint8_t RIGHT_IN1 = 7;
constexpr uint8_t RIGHT_IN2 = 8;
constexpr uint8_t RIGHT_PWM = 6;
constexpr uint8_t LEFT_IN1 = 4;
constexpr uint8_t LEFT_IN2 = 5;
constexpr uint8_t LEFT_PWM = 3;

constexpr uint8_t IR_PIN = A0;

// RGB status LED pins (user mapping: A1 red, A2 green, A3 blue).
constexpr uint8_t STATUS_LED_RED_PIN = A1;
constexpr uint8_t STATUS_LED_GREEN_PIN = A2;
constexpr uint8_t STATUS_LED_BLUE_PIN = A3;
constexpr bool STATUS_LED_ACTIVE_HIGH = true;

constexpr uint8_t SERVO_PIN = 11;
constexpr uint8_t SERVO_MIN_ANGLE_DEG = 73;
constexpr uint8_t SERVO_MAX_ANGLE_DEG = 157;
constexpr uint8_t SERVO_CLOSED_ANGLE_DEG = SERVO_MAX_ANGLE_DEG; // arena_test_1 default
constexpr uint8_t SERVO_OPEN_ANGLE_DEG = SERVO_MIN_ANGLE_DEG;   // arena_test_1 deploy
constexpr unsigned long SERVO_COMMAND_SETTLE_MS = 180;

constexpr uint8_t PCA9548A_ADDR = 0x70;
constexpr uint8_t IMU_MUX_CHANNEL = 4;

constexpr uint8_t TOF_SENSOR_CHANNELS[] = {0, 1, 2, 3};
constexpr uint8_t TOF_SENSOR_COUNT = sizeof(TOF_SENSOR_CHANNELS) / sizeof(TOF_SENSOR_CHANNELS[0]);
constexpr uint8_t REAR_SENSOR_INDEX = 0;
constexpr uint8_t LEFT_SENSOR_INDEX = 1;
constexpr uint8_t FRONT_SENSOR_INDEX = 2;
constexpr uint8_t RIGHT_SENSOR_INDEX = 3;

constexpr uint16_t TOF_TIMEOUT_MS = 60;
constexpr uint16_t TOF_PERIOD_MS = 50;

// corner_start_1 behavior constants.
constexpr uint8_t CORNER_SAMPLES_PER_SENSOR = 10;
constexpr int16_t CORNER_OPEN_THRESHOLD_MM = 254;
constexpr unsigned long CORNER_ACTION_MS = 500;
constexpr unsigned long CORNER_SETTLE_MS = 250;
constexpr uint8_t CORNER_LEFT_DRIVE_SPEED = 180;
constexpr uint8_t CORNER_RIGHT_DRIVE_SPEED = 210;

// arena_test_1 scan/turn/drive constants.
constexpr uint16_t CALIB_SAMPLES = 800;
constexpr uint16_t CALIB_DELAY_MS = 2;
constexpr unsigned long IR_SAMPLE_WINDOW_US = 10400;
constexpr float MAP_INTERVAL_DEG = 8.0f;
constexpr uint8_t MAX_MAP_SAMPLES = 64;
constexpr float SCAN_TARGET_DEG = 360.0f;
constexpr float MIN_PEAK_SEPARATION_DEG = 40.0f;
constexpr float TURN_TOLERANCE_DEG = 2.0f;

constexpr uint8_t LEFT_SCAN_SPEED = 144;
constexpr uint8_t RIGHT_SCAN_SPEED = 216;

constexpr uint8_t LEFT_TURN_SPEED = 144;
constexpr uint8_t RIGHT_TURN_SPEED = 216;

constexpr uint8_t STOP_PULSE_SPEED = 200;
constexpr uint8_t STOP_PULSE_MS = 40;

constexpr uint8_t DRIVE_LEFT_SPEED = 144;
constexpr uint8_t DRIVE_RIGHT_SPEED = 190;
constexpr float DRIVE_HEADING_KP = 2.0f;
constexpr float DRIVE_HEADING_CORR_MAX = 35.0f;

constexpr unsigned long TIMED_FORWARD_MS = 1500;
constexpr unsigned long OPEN_HOLD_MS = 2000;
constexpr unsigned long SERVO_CLOSE_SETTLE_MS = 300;
constexpr unsigned long MIDPOINT_BRAKE_HOLD_MS = 150;

constexpr uint16_t FORWARD_TARGET_MM = 1800;       // 180 cm
constexpr uint16_t FRONT_STOP_TARGET_MM = 300;     // 30 cm

// line_follow_3 UART constants.
constexpr uint8_t LINE_UART_RX_PIN = 10;
constexpr uint8_t LINE_UART_TX_PIN = 9;
constexpr long LINE_UART_BAUD = 19200;

constexpr uint8_t LINE_SENSOR_COUNT = 6;
constexpr uint8_t RAW_PACKET_SIZE = 8;
constexpr uint8_t FRAME_SYNC_0 = 0xA5;
constexpr uint8_t FRAME_SYNC_1 = 0x5A;
constexpr float SENSOR_WEIGHTS[LINE_SENSOR_COUNT] = {-5.0f, -3.0f, -1.0f, 1.0f, 3.0f, 5.0f};
constexpr uint16_t SENSOR_THRESHOLDS[LINE_SENSOR_COUNT] = {900, 900, 900, 900, 900, 900};
constexpr bool DARK_IS_BELOW_THRESHOLD = false;

constexpr char CMD_READ_ONCE = 'R';
constexpr char CMD_STREAM_OFF = 'P';
constexpr uint8_t BASE_SPEED_LEFT = 150;
constexpr uint8_t BASE_SPEED_RIGHT = 160;
constexpr uint8_t MIN_DRIVE_SPEED = 90;
constexpr uint8_t MAX_DRIVE_SPEED = 235;
constexpr uint8_t SEARCH_SWEEP_FAST = 120;
constexpr uint8_t SEARCH_SWEEP_SLOW = 82;
constexpr uint8_t SEARCH_DRIVE_SPEED = 115;
constexpr uint8_t LATCH_HARD_FAST = 210;
constexpr uint8_t LATCH_HARD_SLOW = 90;
constexpr uint8_t LATCH_MED_FAST = 185;
constexpr uint8_t LATCH_MED_SLOW = 105;
constexpr uint8_t LATCH_SOFT_FAST = 165;
constexpr uint8_t LATCH_SOFT_SLOW = 125;
constexpr unsigned long LOOP_DT_MS = 8;
constexpr unsigned long LINE_RX_TIMEOUT_US = 20000;
constexpr unsigned long LINE_STALE_MS = 250;
constexpr float KP = 18.0f;
constexpr float KD = 45.0f;
constexpr float ERROR_SMOOTH_ALPHA = 0.45f;
constexpr uint8_t TRACK_ENTER_FRAMES = 2;
constexpr uint8_t TRACK_EXIT_FRAMES = 5;
constexpr unsigned long SWEEP_FLIP_MS = 260;

L298NMotor leftMotor(LEFT_IN1, LEFT_IN2, LEFT_PWM);
L298NMotor rightMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_PWM);
Adafruit_LSM6DSOX lsm6dsox;
VL53L1X tofSensors[TOF_SENSOR_COUNT];
bool tofReady[TOF_SENSOR_COUNT] = {false, false, false, false};
Servo dumpServo;
SoftwareSerial lineSerial(LINE_UART_RX_PIN, LINE_UART_TX_PIN);
bool dumpServoAttached = false;
int16_t dumpServoLastAngle = -1;

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

enum class MotionDir : uint8_t {
  NONE,
  POSITIVE,
  NEGATIVE
};

MotionDir motionDir = MotionDir::NONE;

enum class NextAction : uint8_t {
  Idle = 0,
  LeftForward,
  LeftReverse,
  RightForward,
  RightReverse,
};

struct CornerSnapshot {
  int16_t tofAvgMm[TOF_SENSOR_COUNT];
};

enum class FollowMode : uint8_t {
  Search,
  Track
};

uint16_t latestRaw[LINE_SENSOR_COUNT] = {0};
bool lineFresh = false;
unsigned long lastRawMs = 0;
unsigned long lastLineLoopMs = 0;
float filteredError = 0.0f;
float prevFilteredError = 0.0f;
float lastSeenError = 0.0f;
uint8_t seenLineFrames = 0;
uint8_t lostLineFrames = 0;
bool sweepToLeft = true;
unsigned long lastSweepFlipMs = 0;
unsigned long lastLinePrintMs = 0;
FollowMode lineMode = FollowMode::Search;

enum class State : uint8_t {
  CORNER_ESCAPE,
  SCANNING,
  TURNING_TO_MIDPOINT,
  TIMED_FORWARD,
  FIND_LINE_FORWARD,
  FOLLOW_LINE_FORWARD,
  OPEN_HOLD,
  CLOSE_SETTLE,
  FIND_LINE_RETURN,
  FOLLOW_LINE_RETURN,
  RELATIVE_TURN,
  FORWARD_TO_FRONT_WALL,
};

enum class StartMode : uint8_t {
  FullCycle,
  ScanOnly,
  FindLineForward,
  FollowLineForward,
  FindLineReturn,
  FollowLineReturn,
};

// Debug convenience: set this to jump directly into a specific phase.
constexpr StartMode START_MODE = StartMode::FullCycle;

State state = State::CORNER_ESCAPE;
State transitionPrevState = State::CORNER_ESCAPE;
unsigned long phaseStartMs = 0;
uint16_t lastRearMm = 0;
bool lastRearValid = false;
unsigned long lastRearReadMs = 0;
uint16_t lastFrontMm = 0;
bool lastFrontValid = false;
unsigned long lastFrontReadMs = 0;
unsigned long lastStatusPrintMs = 0;
uint32_t lineReqCount = 0;
uint32_t lineReqOkCount = 0;
uint32_t lineReqTimeoutCount = 0;
uint32_t lineReqBadCrcCount = 0;

double relativeTurnStartZ = 0.0;
float relativeTurnDeltaDeg = 0.0f;
State relativeTurnNextState = State::FORWARD_TO_FRONT_WALL;

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

const __FlashStringHelper *stateName(State s) {
  switch (s) {
    case State::CORNER_ESCAPE: return F("CORNER_ESCAPE");
    case State::SCANNING: return F("SCANNING");
    case State::TURNING_TO_MIDPOINT: return F("TURN_TO_MID");
    case State::TIMED_FORWARD: return F("TIMED_FORWARD");
    case State::FIND_LINE_FORWARD: return F("FIND_LINE_FWD");
    case State::FOLLOW_LINE_FORWARD: return F("FOLLOW_LINE_FWD");
    case State::OPEN_HOLD: return F("OPEN_HOLD");
    case State::CLOSE_SETTLE: return F("CLOSE_SETTLE");
    case State::FIND_LINE_RETURN: return F("FIND_LINE_RET");
    case State::FOLLOW_LINE_RETURN: return F("FOLLOW_LINE_RET");
    case State::RELATIVE_TURN: return F("RELATIVE_TURN");
    case State::FORWARD_TO_FRONT_WALL: return F("FWD_TO_FRONT_30");
  }
  return F("UNKNOWN");
}

const __FlashStringHelper *lineModeName(FollowMode m) {
  return (m == FollowMode::Search) ? F("SEARCH") : F("TRACK");
}

const __FlashStringHelper *startModeName(StartMode m) {
  switch (m) {
    case StartMode::FullCycle: return F("FULL_CYCLE");
    case StartMode::ScanOnly: return F("SCAN_ONLY");
    case StartMode::FindLineForward: return F("FIND_LINE_FWD");
    case StartMode::FollowLineForward: return F("FOLLOW_LINE_FWD");
    case StartMode::FindLineReturn: return F("FIND_LINE_RET");
    case StartMode::FollowLineReturn: return F("FOLLOW_LINE_RET");
  }
  return F("UNKNOWN");
}

void printStatusHeartbeat() {
  const unsigned long now = millis();
  if (now - lastStatusPrintMs < 250) return;
  lastStatusPrintMs = now;

  Serial.print(F("STATUS,t_ms="));
  Serial.print(now);
  Serial.print(F(",state="));
  Serial.print(stateName(state));
  Serial.print(F(",line_mode="));
  Serial.print(lineModeName(lineMode));
  Serial.print(F(",rear_mm="));
  if (lastRearValid) {
    Serial.print(lastRearMm);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(",rear_age_ms="));
  if (lastRearValid) {
    Serial.print(now - lastRearReadMs);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(",front_mm="));
  if (lastFrontValid) {
    Serial.print(lastFrontMm);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(",front_age_ms="));
  if (lastFrontValid) {
    Serial.print(now - lastFrontReadMs);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(",heading_deg="));
  Serial.print(totalZ, 1);
  Serial.print(F(",line_req="));
  Serial.print(lineReqCount);
  Serial.print(F(",line_ok="));
  Serial.print(lineReqOkCount);
  Serial.print(F(",line_to="));
  Serial.print(lineReqTimeoutCount);
  Serial.print(F(",line_crc="));
  Serial.println(lineReqBadCrcCount);
}

void writeStatusLedChannel(uint8_t pin, bool on) {
  const uint8_t level = STATUS_LED_ACTIVE_HIGH ? (on ? HIGH : LOW) : (on ? LOW : HIGH);
  digitalWrite(pin, level);
}

void setStatusLed(bool redOn, bool greenOn, bool blueOn) {
  writeStatusLedChannel(STATUS_LED_RED_PIN, redOn);
  writeStatusLedChannel(STATUS_LED_GREEN_PIN, greenOn);
  writeStatusLedChannel(STATUS_LED_BLUE_PIN, blueOn);
}

void updateStatusLedForState() {
  bool red = false;
  bool green = false;
  bool blue = false;
  unsigned long blinkMs = 0;

  switch (state) {
    case State::CORNER_ESCAPE:
      red = true;  // red solid
      break;

    case State::SCANNING:
      blue = true;  // blue blink (searching for beacons)
      blinkMs = 220;
      break;

    case State::TURNING_TO_MIDPOINT:
      blue = true;  // blue solid (beacon heading locked)
      break;

    case State::TIMED_FORWARD:
      green = true;  // green blink (pre-line forward)
      blinkMs = 280;
      break;

    case State::FIND_LINE_FORWARD:
      red = true;
      green = true;  // yellow blink (find line)
      blinkMs = 140;
      break;

    case State::FOLLOW_LINE_FORWARD:
      red = true;
      green = true;  // yellow solid (line locked)
      break;

    case State::OPEN_HOLD:
      red = true;
      blue = true;  // magenta solid
      break;

    case State::CLOSE_SETTLE:
      red = true;
      blue = true;  // magenta blink
      blinkMs = 120;
      break;

    case State::FIND_LINE_RETURN:
      green = true;
      blue = true;  // cyan: blink while searching, solid when tracking
      blinkMs = 140;
      break;

    case State::FOLLOW_LINE_RETURN:
      green = true;
      blue = true;  // cyan solid
      break;

    case State::RELATIVE_TURN:
      red = true;
      green = true;
      blue = true;  // white fast blink
      blinkMs = 100;
      break;

    case State::FORWARD_TO_FRONT_WALL:
      green = true;  // green solid (direct back drive)
      break;
  }

  if (blinkMs > 0) {
    const bool onPhase = ((millis() / blinkMs) % 2u) == 0u;
    setStatusLed(onPhase && red, onPhase && green, onPhase && blue);
  } else {
    setStatusLed(red, green, blue);
  }
}

bool initTofSensors() {
  bool anyReady = false;

  for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
    if (!selectMuxChannel(TOF_SENSOR_CHANNELS[i])) {
      Serial.print(F("TOF mux select failed on ch "));
      Serial.println(TOF_SENSOR_CHANNELS[i]);
      continue;
    }

    tofSensors[i].setTimeout(TOF_TIMEOUT_MS);
    if (!tofSensors[i].init()) {
      Serial.print(F("TOF init failed on ch "));
      Serial.println(TOF_SENSOR_CHANNELS[i]);
      continue;
    }

    tofSensors[i].setDistanceMode(VL53L1X::Long);
    tofSensors[i].setMeasurementTimingBudget(50000);
    tofSensors[i].startContinuous(TOF_PERIOD_MS);

    tofReady[i] = true;
    anyReady = true;

    Serial.print(F("TOF ready on ch "));
    Serial.println(TOF_SENSOR_CHANNELS[i]);
  }

  return anyReady;
}

bool readTofMm(uint8_t sensorIndex, uint16_t &mmOut) {
  if (sensorIndex >= TOF_SENSOR_COUNT || !tofReady[sensorIndex]) return false;
  if (!selectMuxChannel(TOF_SENSOR_CHANNELS[sensorIndex])) return false;

  const uint16_t mm = tofSensors[sensorIndex].read();
  if (tofSensors[sensorIndex].timeoutOccurred()) return false;

  mmOut = mm;
  return true;
}

int16_t readAveragedTof(uint8_t sensorIndex) {
  if (sensorIndex >= TOF_SENSOR_COUNT || !tofReady[sensorIndex]) return -1;

  const uint8_t muxChannel = TOF_SENSOR_CHANNELS[sensorIndex];
  int32_t sum = 0;
  uint8_t validCount = 0;

  for (uint8_t sample = 0; sample < CORNER_SAMPLES_PER_SENSOR; sample++) {
    if (!selectMuxChannel(muxChannel)) continue;

    uint16_t mm = tofSensors[sensorIndex].read();
    if (!tofSensors[sensorIndex].timeoutOccurred()) {
      sum += mm;
      validCount++;
    }
  }

  if (validCount == 0) return -1;
  return static_cast<int16_t>(sum / validCount);
}

bool readRearMm(uint16_t &mmOut) {
  const bool ok = readTofMm(REAR_SENSOR_INDEX, mmOut);
  if (ok) {
    lastRearValid = true;
    lastRearMm = mmOut;
    lastRearReadMs = millis();
  }
  return ok;
}

bool readFrontMm(uint16_t &mmOut) {
  const bool ok = readTofMm(FRONT_SENSOR_INDEX, mmOut);
  if (ok) {
    lastFrontValid = true;
    lastFrontMm = mmOut;
    lastFrontReadMs = millis();
  }
  return ok;
}

void brakeMotors() {
  leftMotor.drive(L298NMotor::Direction::Brake, 0);
  rightMotor.drive(L298NMotor::Direction::Brake, 0);
  motionDir = MotionDir::NONE;
}

void attachDumpServoIfNeeded() {
  if (dumpServoAttached) return;
  dumpServo.attach(SERVO_PIN);
  dumpServoAttached = true;
  delay(12);
}

void detachDumpServoIfNeeded() {
  if (!dumpServoAttached) return;
  dumpServo.detach();
  dumpServoAttached = false;
}

void commandServo(uint8_t angleDeg, const __FlashStringHelper *tag, bool keepAttached) {
  // Avoid repeatedly re-commanding same angle (prevents unnecessary twitch/current spikes).
  if (dumpServoLastAngle == static_cast<int16_t>(angleDeg)) {
    if (!keepAttached) {
      detachDumpServoIfNeeded();
    }
    return;
  }

  attachDumpServoIfNeeded();
  dumpServo.write(angleDeg);
  delay(SERVO_COMMAND_SETTLE_MS);
  dumpServo.write(angleDeg);
  dumpServoLastAngle = angleDeg;
  Serial.print(F("SERVO_CMD,"));
  Serial.println(tag);
  if (!keepAttached) {
    detachDumpServoIfNeeded();
  }
}

void openDumpServo() {
  commandServo(SERVO_OPEN_ANGLE_DEG, F("OPEN"), true);
}

void closeDumpServo() {
  commandServo(SERVO_CLOSED_ANGLE_DEG, F("CLOSE"), true);
}

void closeDumpServoAndDetach() {
  commandServo(SERVO_CLOSED_ANGLE_DEG, F("CLOSE"), false);
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
  const unsigned long startUs = micros();
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

CornerSnapshot takeCornerMeasurements() {
  CornerSnapshot snapshot{};
  for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
    snapshot.tofAvgMm[i] = readAveragedTof(i);
  }
  return snapshot;
}

NextAction decideCornerAction(const CornerSnapshot &snapshot) {
  const int16_t backDist = snapshot.tofAvgMm[REAR_SENSOR_INDEX];
  const int16_t leftDist = snapshot.tofAvgMm[LEFT_SENSOR_INDEX];
  const int16_t frontDist = snapshot.tofAvgMm[FRONT_SENSOR_INDEX];
  const int16_t rightDist = snapshot.tofAvgMm[RIGHT_SENSOR_INDEX];

  if (backDist > CORNER_OPEN_THRESHOLD_MM) {
    return (leftDist > rightDist) ? NextAction::LeftReverse : NextAction::RightReverse;
  }

  if (frontDist > CORNER_OPEN_THRESHOLD_MM) {
    return (leftDist > rightDist) ? NextAction::RightForward : NextAction::LeftForward;
  }

  return NextAction::Idle;
}

void applyCornerAction(NextAction action) {
  switch (action) {
    case NextAction::Idle:
      Serial.println(F("CORNER_ACTION,Idle"));
      brakeMotors();
      break;

    case NextAction::LeftForward:
      Serial.println(F("CORNER_ACTION,LeftForward"));
      leftMotor.drive(L298NMotor::Direction::Forward, CORNER_LEFT_DRIVE_SPEED);
      rightMotor.drive(L298NMotor::Direction::Brake, 0);
      break;

    case NextAction::LeftReverse:
      Serial.println(F("CORNER_ACTION,LeftReverse"));
      leftMotor.drive(L298NMotor::Direction::Backward, CORNER_LEFT_DRIVE_SPEED);
      rightMotor.drive(L298NMotor::Direction::Brake, 0);
      break;

    case NextAction::RightForward:
      Serial.println(F("CORNER_ACTION,RightForward"));
      rightMotor.drive(L298NMotor::Direction::Forward, CORNER_RIGHT_DRIVE_SPEED);
      leftMotor.drive(L298NMotor::Direction::Brake, 0);
      break;

    case NextAction::RightReverse:
      Serial.println(F("CORNER_ACTION,RightReverse"));
      rightMotor.drive(L298NMotor::Direction::Backward, CORNER_RIGHT_DRIVE_SPEED);
      leftMotor.drive(L298NMotor::Direction::Brake, 0);
      break;
  }
}

void runCornerEscapeRoutine() {
  const CornerSnapshot snapshot = takeCornerMeasurements();
  const NextAction action = decideCornerAction(snapshot);

  Serial.print(F("CORNER_TOF_MM,"));
  for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
    Serial.print(snapshot.tofAvgMm[i]);
    if (i + 1 < TOF_SENSOR_COUNT) Serial.print(',');
  }
  Serial.println();

  applyCornerAction(action);
  delay(CORNER_ACTION_MS);

  brakeMotors();
  delay(CORNER_SETTLE_MS);

  CornerSnapshot postTurnSnapshot = takeCornerMeasurements();

  if (postTurnSnapshot.tofAvgMm[REAR_SENSOR_INDEX] < postTurnSnapshot.tofAvgMm[FRONT_SENSOR_INDEX]) {
    leftMotor.drive(L298NMotor::Direction::Forward, CORNER_LEFT_DRIVE_SPEED);
    rightMotor.drive(L298NMotor::Direction::Forward, CORNER_RIGHT_DRIVE_SPEED);
    Serial.println(F("CORNER_EXIT_DRIVE,Forward"));
  } else {
    leftMotor.drive(L298NMotor::Direction::Backward, CORNER_LEFT_DRIVE_SPEED);
    rightMotor.drive(L298NMotor::Direction::Backward, CORNER_RIGHT_DRIVE_SPEED);
    Serial.println(F("CORNER_EXIT_DRIVE,Backward"));
  }

  delay(CORNER_ACTION_MS);
  brakeMotors();
  delay(CORNER_SETTLE_MS);
}

void startScan() {
  scanStartZ = totalZ;
  nextSampleAngleDeg = 0.0f;
  sampleCount = 0;

  leftMotor.drive(L298NMotor::Direction::Backward, LEFT_SCAN_SPEED);
  rightMotor.drive(L298NMotor::Direction::Forward, RIGHT_SCAN_SPEED);
  motionDir = MotionDir::POSITIVE;

  state = State::SCANNING;

  Serial.println(F("SCAN_START"));
  Serial.println(F("MAP,angle_deg,peak_to_peak"));
}

void beginTimedForwardPhase() {
  closeDumpServoAndDetach();
  driveHeadingTargetZ = totalZ;
  phaseStartMs = millis();
  state = State::TIMED_FORWARD;
  Serial.println(F("TIMED_FORWARD_START"));
}

void beginFindLineForward() {
  lineMode = FollowMode::Search;
  lineFresh = false;
  lastRawMs = 0;
  lastLineLoopMs = 0;
  filteredError = 0.0f;
  prevFilteredError = 0.0f;
  lastSeenError = 0.0f;
  seenLineFrames = 0;
  lostLineFrames = 0;
  sweepToLeft = true;
  lastSweepFlipMs = millis();
  state = State::FIND_LINE_FORWARD;
  Serial.println(F("FIND_LINE_FORWARD_START"));
}

void beginFindLineReturn() {
  lineMode = FollowMode::Search;
  lineFresh = false;
  lastRawMs = 0;
  lastLineLoopMs = 0;
  filteredError = 0.0f;
  prevFilteredError = 0.0f;
  lastSeenError = 0.0f;
  seenLineFrames = 0;
  lostLineFrames = 0;
  sweepToLeft = true;
  lastSweepFlipMs = millis();
  state = State::FIND_LINE_RETURN;
  Serial.println(F("FIND_LINE_RETURN_START"));
}

void beginFollowLineReturn() {
  state = State::FOLLOW_LINE_RETURN;
  Serial.println(F("FOLLOW_LINE_RETURN_START"));
}

void beginRelativeTurn(float deltaDeg, State nextState, const __FlashStringHelper *label) {
  relativeTurnStartZ = totalZ;
  relativeTurnDeltaDeg = deltaDeg;
  relativeTurnNextState = nextState;
  state = State::RELATIVE_TURN;
  Serial.println(label);
}

void applyStartMode() {
  Serial.print(F("START_MODE,"));
  Serial.println(startModeName(START_MODE));

  switch (START_MODE) {
    case StartMode::FullCycle:
      state = State::CORNER_ESCAPE;
      break;

    case StartMode::ScanOnly:
      startScan();
      break;

    case StartMode::FindLineForward:
      beginFindLineForward();
      break;

    case StartMode::FollowLineForward:
      beginFindLineForward();
      state = State::FOLLOW_LINE_FORWARD;
      Serial.println(F("START_OVERRIDE,FOLLOW_LINE_FORWARD"));
      break;

    case StartMode::FindLineReturn:
      beginFindLineReturn();
      Serial.println(F("START_OVERRIDE,FIND_LINE_RETURN"));
      break;

    case StartMode::FollowLineReturn:
      beginFindLineReturn();
      state = State::FOLLOW_LINE_RETURN;
      Serial.println(F("START_OVERRIDE,FOLLOW_LINE_RETURN"));
      break;
  }
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
      beginTimedForwardPhase();
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
    beginTimedForwardPhase();
    return;
  }

  startTurnBySign(remaining);
}

void timedForwardStep() {
  driveWithHeadingHold(true);

  if (millis() - phaseStartMs < TIMED_FORWARD_MS) return;

  brakeMotors();
  beginFindLineForward();
}

void unpackRawPacket(const uint8_t packet[RAW_PACKET_SIZE], uint16_t rawOut[LINE_SENSOR_COUNT]) {
  uint64_t packed = 0;
  for (uint8_t i = 0; i < RAW_PACKET_SIZE; i++) {
    packed |= (static_cast<uint64_t>(packet[i]) << (8u * i));
  }
  for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) {
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

bool requestLatestRaw(uint16_t rawOut[LINE_SENSOR_COUNT]) {
  lineReqCount++;

  while (lineSerial.available() > 0) {
    lineSerial.read();
  }

  lineSerial.write(static_cast<uint8_t>(CMD_READ_ONCE));

  uint8_t packet[RAW_PACKET_SIZE];
  uint8_t got = 0;
  bool gotSync0 = false;
  bool gotSync1 = false;
  const unsigned long start = micros();

  while (micros() - start < LINE_RX_TIMEOUT_US) {
    while (lineSerial.available() > 0) {
      const int rx = lineSerial.read();
      if (rx < 0) continue;
      const uint8_t b = static_cast<uint8_t>(rx);

      if (!gotSync0) {
        gotSync0 = (b == FRAME_SYNC_0);
        continue;
      }
      if (!gotSync1) {
        gotSync1 = (b == FRAME_SYNC_1);
        if (!gotSync1) {
          gotSync0 = (b == FRAME_SYNC_0);
        }
        continue;
      }

      if (got < RAW_PACKET_SIZE) {
        packet[got++] = b;
        continue;
      }

      if (b != checksumRaw(packet)) {
        lineReqBadCrcCount++;
        gotSync0 = false;
        gotSync1 = false;
        got = 0;
        continue;
      }

      unpackRawPacket(packet, rawOut);
      lineReqOkCount++;
      return true;
    }
  }

  lineReqTimeoutCount++;
  return false;
}

bool isDark(uint16_t raw, uint16_t threshold) {
  return DARK_IS_BELOW_THRESHOLD ? (raw < threshold) : (raw > threshold);
}

bool computeErrorFromRaw(const uint16_t raw[LINE_SENSOR_COUNT], float &errorOut, bool activeOut[LINE_SENSOR_COUNT]) {
  float weighted = 0.0f;
  float sum = 0.0f;

  for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) {
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

void driveForwardArc(uint8_t leftPwm, uint8_t rightPwm) {
  leftMotor.drive(L298NMotor::Direction::Forward, leftPwm);
  rightMotor.drive(L298NMotor::Direction::Forward, rightPwm);
}

void driveBackwardArc(uint8_t leftPwm, uint8_t rightPwm) {
  leftMotor.drive(L298NMotor::Direction::Backward, leftPwm);
  rightMotor.drive(L298NMotor::Direction::Backward, rightPwm);
}

void driveSearchTurnLeft(bool forward) {
  if (forward) {
    driveForwardArc(SEARCH_SWEEP_SLOW, SEARCH_SWEEP_FAST);
  } else {
    driveBackwardArc(SEARCH_SWEEP_FAST, SEARCH_SWEEP_SLOW);
  }
}

void driveSearchTurnRight(bool forward) {
  if (forward) {
    driveForwardArc(SEARCH_SWEEP_FAST, SEARCH_SWEEP_SLOW);
  } else {
    driveBackwardArc(SEARCH_SWEEP_SLOW, SEARCH_SWEEP_FAST);
  }
}

void driveLatchRight(bool forward, uint8_t fastPwm, uint8_t slowPwm) {
  if (forward) {
    driveForwardArc(fastPwm, slowPwm);
  } else {
    driveBackwardArc(slowPwm, fastPwm);
  }
}

void driveLatchLeft(bool forward, uint8_t fastPwm, uint8_t slowPwm) {
  if (forward) {
    driveForwardArc(slowPwm, fastPwm);
  } else {
    driveBackwardArc(fastPwm, slowPwm);
  }
}

void runLineFollowerStep(bool forward) {
  const unsigned long now = millis();
  if (now - lastLineLoopMs < LOOP_DT_MS) return;
  lastLineLoopMs = now;

  uint16_t newRaw[LINE_SENSOR_COUNT] = {0};
  if (requestLatestRaw(newRaw)) {
    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) {
      latestRaw[i] = newRaw[i];
    }
    lineFresh = true;
    lastRawMs = now;
  }

  if (!lineFresh || (now - lastRawMs > LINE_STALE_MS)) {
    lineMode = FollowMode::Search;
    seenLineFrames = 0;
    lostLineFrames = 0;
    if (now - lastSweepFlipMs >= SWEEP_FLIP_MS) {
      lastSweepFlipMs = now;
      sweepToLeft = !sweepToLeft;
    }
    if (lastSeenError > 0.2f) {
      driveSearchTurnLeft(forward);
    } else if (lastSeenError < -0.2f) {
      driveSearchTurnRight(forward);
    } else {
      if (sweepToLeft) driveSearchTurnLeft(forward);
      else driveSearchTurnRight(forward);
    }
    return;
  }

  bool active[LINE_SENSOR_COUNT] = {false};
  float error = 0.0f;
  const bool hasLine = computeErrorFromRaw(latestRaw, error, active);
  const bool rightOuterActive = active[0] || active[1];
  const bool leftOuterActive = active[4] || active[5];

  if (hasLine) {
    if (seenLineFrames < 255) seenLineFrames++;
    lostLineFrames = 0;
    lastSeenError = error;
  } else {
    seenLineFrames = 0;
    if (lostLineFrames < 255) lostLineFrames++;
  }

  if (lineMode == FollowMode::Search) {
    if (seenLineFrames >= TRACK_ENTER_FRAMES) {
      lineMode = FollowMode::Track;
      filteredError = error;
      prevFilteredError = error;
    } else {
      const bool rightMostOnly = active[0] && !active[1] && !active[2];
      const bool leftMostOnly = active[5] && !active[4] && !active[3];
      const bool rightEdge = (active[0] && active[1]) || (active[1] && !active[2]);
      const bool leftEdge = (active[5] && active[4]) || (active[4] && !active[3]);
      const bool rightNearCenter = active[2] && !active[3];
      const bool leftNearCenter = active[3] && !active[2];

      if (rightMostOnly) {
        driveLatchRight(forward, LATCH_HARD_FAST, LATCH_HARD_SLOW);
      } else if (leftMostOnly) {
        driveLatchLeft(forward, LATCH_HARD_FAST, LATCH_HARD_SLOW);
      } else if (rightEdge && !leftEdge) {
        driveLatchRight(forward, LATCH_MED_FAST, LATCH_MED_SLOW);
      } else if (leftEdge && !rightEdge) {
        driveLatchLeft(forward, LATCH_MED_FAST, LATCH_MED_SLOW);
      } else if (rightNearCenter && !leftNearCenter) {
        driveLatchRight(forward, LATCH_SOFT_FAST, LATCH_SOFT_SLOW);
      } else if (leftNearCenter && !rightNearCenter) {
        driveLatchLeft(forward, LATCH_SOFT_FAST, LATCH_SOFT_SLOW);
      } else if (hasLine) {
        if (forward) driveForwardArc(SEARCH_DRIVE_SPEED, SEARCH_DRIVE_SPEED);
        else driveBackwardArc(SEARCH_DRIVE_SPEED, SEARCH_DRIVE_SPEED);
      } else {
        if (now - lastSweepFlipMs >= SWEEP_FLIP_MS) {
          lastSweepFlipMs = now;
          sweepToLeft = !sweepToLeft;
        }

        if (lastSeenError > 0.2f) {
          driveSearchTurnLeft(forward);
        } else if (lastSeenError < -0.2f) {
          driveSearchTurnRight(forward);
        } else {
          if (sweepToLeft) driveSearchTurnLeft(forward);
          else driveSearchTurnRight(forward);
        }
      }
    }
  }

  if (lineMode == FollowMode::Track) {
    if (lostLineFrames >= TRACK_EXIT_FRAMES) {
      lineMode = FollowMode::Search;
      return;
    }

    filteredError = ERROR_SMOOTH_ALPHA * filteredError + (1.0f - ERROR_SMOOTH_ALPHA) * error;
    const float dError = filteredError - prevFilteredError;
    prevFilteredError = filteredError;

    const float correction = KP * filteredError + KD * dError;
    float leftCmd = 0.0f;
    float rightCmd = 0.0f;

    if (forward) {
      leftCmd = BASE_SPEED_LEFT - correction;
      rightCmd = BASE_SPEED_RIGHT + correction;
    } else {
      leftCmd = BASE_SPEED_LEFT + correction;
      rightCmd = BASE_SPEED_RIGHT - correction;
    }

    leftCmd = clampf(leftCmd, MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
    rightCmd = clampf(rightCmd, MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);

    if (forward) {
      if (leftOuterActive && !rightOuterActive && filteredError > 1.5f) {
        leftCmd = MIN_DRIVE_SPEED;
        rightCmd = MAX_DRIVE_SPEED;
      } else if (rightOuterActive && !leftOuterActive && filteredError < -1.5f) {
        leftCmd = MAX_DRIVE_SPEED;
        rightCmd = MIN_DRIVE_SPEED;
      }

      leftMotor.drive(L298NMotor::Direction::Forward, clampPwm(leftCmd));
      rightMotor.drive(L298NMotor::Direction::Forward, clampPwm(rightCmd));
    } else {
      if (leftOuterActive && !rightOuterActive && filteredError > 1.5f) {
        leftCmd = MAX_DRIVE_SPEED;
        rightCmd = MIN_DRIVE_SPEED;
      } else if (rightOuterActive && !leftOuterActive && filteredError < -1.5f) {
        leftCmd = MIN_DRIVE_SPEED;
        rightCmd = MAX_DRIVE_SPEED;
      }

      leftMotor.drive(L298NMotor::Direction::Backward, clampPwm(leftCmd));
      rightMotor.drive(L298NMotor::Direction::Backward, clampPwm(rightCmd));
    }
  }

  if (now - lastLinePrintMs >= 100) {
    lastLinePrintMs = now;
    Serial.print(forward ? F("LINE_FWD ") : F("LINE_REV "));
    Serial.print(lineMode == FollowMode::Search ? F("SEARCH ") : F("TRACK "));
    Serial.print(F("raw:"));
    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) {
      Serial.print(latestRaw[i]);
      if (i < LINE_SENSOR_COUNT - 1) Serial.print(',');
    }
    Serial.print(F(" seen="));
    Serial.print(seenLineFrames);
    Serial.print(F(" lost="));
    Serial.print(lostLineFrames);
    Serial.print(F(" err="));
    Serial.println(filteredError, 3);
  }
}

void findLineForwardStep() {
  runLineFollowerStep(true);

  if (lineMode == FollowMode::Track) {
    state = State::FOLLOW_LINE_FORWARD;
    Serial.println(F("FOLLOW_LINE_FORWARD_START"));
  }
}

void followLineForwardStep() {
  runLineFollowerStep(true);

  // Only evaluate forward stop distance when the line is actively locked.
  if (lineMode != FollowMode::Track) return;

  uint16_t rearMm = 0;
  if (!readRearMm(rearMm)) return;
  if (rearMm < FORWARD_TARGET_MM) return;

  Serial.print(F("FORWARD_TARGET_REACHED_MM,"));
  Serial.println(rearMm);

  brakeMotors();
  openDumpServo();
  phaseStartMs = millis();
  state = State::OPEN_HOLD;
  Serial.println(F("SERVO_OPEN_HOLD_START"));
}

void openHoldStep() {
  if (millis() - phaseStartMs < OPEN_HOLD_MS) return;

  closeDumpServo();
  phaseStartMs = millis();
  state = State::CLOSE_SETTLE;
  Serial.println(F("SERVO_CLOSE_SETTLE_START"));
}

void closeSettleStep() {
  if (millis() - phaseStartMs < SERVO_CLOSE_SETTLE_MS) return;
  detachDumpServoIfNeeded();
  beginRelativeTurn(180.0f, State::FIND_LINE_RETURN, F("TURN_180_FOR_RETURN_START"));
}

void findLineReturnStep() {
  runLineFollowerStep(true);

  if (lineMode == FollowMode::Track) {
    beginFollowLineReturn();
  }
}

void followLineReturnStep() {
  runLineFollowerStep(true);
  if (lineMode != FollowMode::Track) return;

  uint16_t frontMm = 0;
  if (!readFrontMm(frontMm)) return;
  if (frontMm > FRONT_STOP_TARGET_MM) return;

  Serial.print(F("RETURN_LINE_FRONT_TARGET_REACHED_MM,"));
  Serial.println(frontMm);

  brakeMotors();
  beginRelativeTurn(-90.0f, State::FORWARD_TO_FRONT_WALL, F("TURN_RIGHT_AFTER_LINE_START"));
}

void relativeTurnStep() {
  const float turnedDeg = static_cast<float>(totalZ - relativeTurnStartZ);
  const float remainingDeg = relativeTurnDeltaDeg - turnedDeg;

  if (absf(remainingDeg) <= TURN_TOLERANCE_DEG) {
    stopTurnMotors();
    brakeMotors();
    delay(MIDPOINT_BRAKE_HOLD_MS);

    state = relativeTurnNextState;

    if (state == State::FORWARD_TO_FRONT_WALL) {
      driveHeadingTargetZ = totalZ;
      Serial.println(F("FORWARD_TO_FRONT_WALL_START"));
    } else if (state == State::FIND_LINE_RETURN) {
      beginFindLineReturn();
    }

    return;
  }

  startTurnBySign(remainingDeg);
}

void forwardToFrontWallStep() {
  driveWithHeadingHold(true);

  uint16_t frontMm = 0;
  if (!readFrontMm(frontMm)) return;
  if (frontMm > FRONT_STOP_TARGET_MM) return;

  Serial.print(F("FRONT_WALL_TARGET_REACHED_MM,"));
  Serial.println(frontMm);

  brakeMotors();
  Serial.println(F("CYCLE_RESTART -> CORNER_ESCAPE"));
  state = State::CORNER_ESCAPE;
}

}  // namespace

void setup() {
  const uint8_t resetFlags = MCUSR;
  MCUSR = 0;

  Serial.begin(BAUD);
  Serial.print(F("RESET,MCUSR=0x"));
  Serial.println(resetFlags, HEX);
  if (resetFlags & _BV(PORF)) Serial.println(F("RESET_CAUSE,POWER_ON"));
  if (resetFlags & _BV(EXTRF)) Serial.println(F("RESET_CAUSE,EXTERNAL"));
  if (resetFlags & _BV(BORF)) Serial.println(F("RESET_CAUSE,BROWN_OUT"));
  if (resetFlags & _BV(WDRF)) Serial.println(F("RESET_CAUSE,WATCHDOG"));

  pinMode(IR_PIN, INPUT);
  pinMode(STATUS_LED_RED_PIN, OUTPUT);
  pinMode(STATUS_LED_GREEN_PIN, OUTPUT);
  pinMode(STATUS_LED_BLUE_PIN, OUTPUT);
  setStatusLed(false, false, false);

  lineSerial.begin(LINE_UART_BAUD);
  lineSerial.write(static_cast<uint8_t>(CMD_STREAM_OFF));
  delay(20);
  while (lineSerial.available() > 0) {
    lineSerial.read();
  }

  leftMotor.begin();
  rightMotor.begin();
  brakeMotors();

  // Do not move servo at boot; repeated boot actuation can trigger power dips.
  detachDumpServoIfNeeded();

  Wire.begin();
  Wire.setClock(400000);

  if (!initTofSensors()) {
    Serial.println(F("WARN: no TOF sensors initialized"));
  }

  if (!selectMuxChannel(IMU_MUX_CHANNEL)) {
    Serial.println(F("PCA9548A IMU select FAIL"));
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

  applyStartMode();
  transitionPrevState = state;
  updateStatusLedForState();
  Serial.print(F("STATE_ENTER,"));
  Serial.println(stateName(state));
  Serial.println(F("arena_test_4 ready"));
  Serial.println(F("FLOW: corner_escape -> scan -> midpoint -> 1.5s forward -> line forward to 180cm -> servo open/close -> turn 180 -> find/follow line forward -> front(Port2) 30cm -> turn right -> forward to front 30cm -> repeat"));
}

void loop() {
  updateGyro();
  updateStatusLedForState();
  printStatusHeartbeat();

  if (state != transitionPrevState) {
    Serial.print(F("STATE_TRANSITION,from="));
    Serial.print(stateName(transitionPrevState));
    Serial.print(F(",to="));
    Serial.print(stateName(state));
    Serial.print(F(",t_ms="));
    Serial.println(millis());
    transitionPrevState = state;
  }

  switch (state) {
    case State::CORNER_ESCAPE:
      runCornerEscapeRoutine();
      startScan();
      break;

    case State::SCANNING:
      scanStep();
      break;

    case State::TURNING_TO_MIDPOINT:
      turnToMidpointStep();
      break;

    case State::TIMED_FORWARD:
      timedForwardStep();
      break;

    case State::FIND_LINE_FORWARD:
      findLineForwardStep();
      break;

    case State::FOLLOW_LINE_FORWARD:
      followLineForwardStep();
      break;

    case State::OPEN_HOLD:
      openHoldStep();
      break;

    case State::CLOSE_SETTLE:
      closeSettleStep();
      break;

    case State::FIND_LINE_RETURN:
      findLineReturnStep();
      break;

    case State::FOLLOW_LINE_RETURN:
      followLineReturnStep();
      break;

    case State::RELATIVE_TURN:
      relativeTurnStep();
      break;

    case State::FORWARD_TO_FRONT_WALL:
      forwardToFrontWallStep();
      break;
  }
}
