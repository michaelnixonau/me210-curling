/**
 * Arena Test 4.1
 * (1) Corner escape
 * (2) Line follow to end of arena
 */

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <L298N.h>
#include <SoftwareSerial.h>

namespace {
// --- PIN DEFINITIONS ---
constexpr uint8_t RIGHT_IN1 = 7;
constexpr uint8_t RIGHT_IN2 = 8;
constexpr uint8_t RIGHT_PWM = 6;
constexpr uint8_t LEFT_IN1 = 4;
constexpr uint8_t LEFT_IN2 = 5;
constexpr uint8_t LEFT_PWM = 3;

// TOF / Mux Constants
constexpr uint8_t PCA9548A_ADDR = 0x70;
constexpr uint8_t SENSOR_CHANNELS[] = {0, 1, 2, 3};
constexpr uint8_t SENSOR_COUNT_TOF = 4;
constexpr uint8_t SAMPLES_PER_SENSOR = 10;

// Line Sensor UART Constants
constexpr uint8_t LINE_UART_RX_PIN = 10;
constexpr uint8_t LINE_UART_TX_PIN = 9;
constexpr long LINE_UART_BAUD = 19200;
constexpr uint8_t SENSOR_COUNT_LINE = 6;
constexpr uint8_t RAW_PACKET_SIZE = 8;
constexpr uint8_t FRAME_SYNC_0 = 0xA5;
constexpr uint8_t FRAME_SYNC_1 = 0x5A;

// --- TUNING PARAMETERS ---
// Motor calibration from line_follow_3
constexpr uint8_t BASE_SPEED_LEFT = 150;
constexpr uint8_t BASE_SPEED_RIGHT = 160;
constexpr float RIGHT_MOTOR_SCALAR = static_cast<float>(BASE_SPEED_RIGHT) / static_cast<float>(BASE_SPEED_LEFT);

// Corner Start Speeds
constexpr uint8_t CORNER_SPEED_LEFT = 180;
constexpr uint8_t CORNER_SPEED_RIGHT = 210;

// PID / Following Constants
constexpr float KP = 25.0f;
constexpr float KD = 65.0f;
constexpr float ERROR_SMOOTH_ALPHA = 0.45f;
constexpr float TURN_DECELERATION = 18.0f;
constexpr float SENSOR_WEIGHTS[6] = {-5.0f, -3.0f, -1.0f, 1.0f, 3.0f, 5.0f};
constexpr uint16_t SENSOR_THRESHOLDS[6] = {900, 900, 900, 900, 900, 900};

// --- OBJECTS & GLOBALS ---
L298NMotor leftMotor(LEFT_IN1, LEFT_IN2, LEFT_PWM);
L298NMotor rightMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_PWM);
VL53L1X tofSensors[SENSOR_COUNT_TOF];
bool tofReady[SENSOR_COUNT_TOF] = {false};
SoftwareSerial lineSerial(LINE_UART_RX_PIN, LINE_UART_TX_PIN);

enum class State { CORNER_START, LINE_FOLLOW };
State currentState = State::CORNER_START;

// Line Following Variables
uint16_t latestRaw[SENSOR_COUNT_LINE] = {0};
unsigned long lastRawMs = 0;
unsigned long lastLoopMs = 0;
float filteredError = 0.0f;
float prevFilteredError = 0.0f;
float lastSeenError = 0.0f;
uint8_t lostLineFrames = 0;
enum class FollowMode : uint8_t { Search, Track };
FollowMode followMode = FollowMode::Search;

// --- MOTOR HELPER ---
uint8_t clampPwm(float speed) {
  return (speed < 0.0f) ? 0 : (speed > 255.0f) ? 255 : static_cast<uint8_t>(speed + 0.5f);
}

void driveMotors(float leftCmd, float rightCmd) {
  rightCmd *= RIGHT_MOTOR_SCALAR; // Apply the scalar to keep motors balanced
  
  L298NMotor::Direction leftDir = leftCmd >= 0.0f ? L298NMotor::Direction::Forward : L298NMotor::Direction::Backward;
  L298NMotor::Direction rightDir = rightCmd >= 0.0f ? L298NMotor::Direction::Forward : L298NMotor::Direction::Backward;

  leftMotor.drive(leftDir, clampPwm(abs(leftCmd)));
  rightMotor.drive(rightDir, clampPwm(abs(rightCmd)));
}

void stopMotors() {
  leftMotor.stop();
  rightMotor.stop();
}

// --- TOF / MUX HELPERS ---
bool selectMuxChannel(uint8_t channel) {
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  return Wire.endTransmission() == 0;
}

int16_t readAveragedTof(uint8_t index) {
  if (!tofReady[index]) return -1;
  int32_t sum = 0;
  uint8_t valid = 0;
  for (uint8_t s = 0; s < SAMPLES_PER_SENSOR; s++) {
    selectMuxChannel(SENSOR_CHANNELS[index]);
    uint16_t mm = tofSensors[index].read();
    if (!tofSensors[index].timeoutOccurred()) { sum += mm; valid++; }
  }
  return (valid == 0) ? -1 : (int16_t)(sum / valid);
}

// --- LINE SENSOR UART HELPERS ---
void unpackRawPacket(const uint8_t packet[8], uint16_t rawOut[6]) {
  uint64_t packed = 0;
  for (uint8_t i = 0; i < 8; i++) packed |= (static_cast<uint64_t>(packet[i]) << (8u * i));
  for (uint8_t i = 0; i < 6; i++) rawOut[i] = static_cast<uint16_t>((packed >> (10u * i)) & 0x03FFu);
}

bool requestLatestRaw(uint16_t rawOut[6]) {
  while (lineSerial.available() > 0) lineSerial.read();
  lineSerial.write('R');
  uint8_t packet[8], got = 0;
  bool s0 = false, s1 = false;
  unsigned long start = micros();
  while (micros() - start < 12000) {
    if (lineSerial.available() > 0) {
      uint8_t b = lineSerial.read();
      if (!s0) { s0 = (b == FRAME_SYNC_0); continue; }
      if (!s1) { s1 = (b == FRAME_SYNC_1); continue; }
      if (got < 8) { packet[got++] = b; continue; }
      unpackRawPacket(packet, rawOut);
      return true;
    }
  }
  return false;
}

// --- CORNER START LOGIC ---
void runCornerStartManeuver() {
  Serial.println(F("Executing Corner Start..."));
  
  // Initial Snapshot
  int16_t tof[4];
  for(int i=0; i<4; i++) tof[i] = readAveragedTof(i);

  // Decision logic from corner_start
  if (tof[0] > 254) { // Back distance
    if (tof[1] > tof[3]) driveMotors(-CORNER_SPEED_LEFT, 0); // Left Reverse
    else driveMotors(0, -CORNER_SPEED_RIGHT);               // Right Reverse
  } else if (tof[2] > 254) { // Front distance
    if (tof[1] > tof[3]) driveMotors(0, CORNER_SPEED_RIGHT); // Right Forward
    else driveMotors(CORNER_SPEED_LEFT, 0);                 // Left Forward
  }

  delay(500);
  stopMotors();
  delay(250);

  // Second movement phase
  int16_t back = readAveragedTof(0);
  int16_t front = readAveragedTof(2);

  if (back < front) driveMotors(CORNER_SPEED_LEFT, CORNER_SPEED_RIGHT);
  else driveMotors(-CORNER_SPEED_LEFT, -CORNER_SPEED_RIGHT);

  delay(500);
  stopMotors();
  Serial.println(F("Corner Start Complete. Switching to Line Follow."));
}

// --- LINE FOLLOW LOGIC ---
void runLineFollow() {
  const unsigned long now = millis();
  if (now - lastLoopMs < 8) return;
  lastLoopMs = now;

  uint16_t newRaw[6];
  if (requestLatestRaw(newRaw)) {
    for (int i = 0; i < 6; i++) latestRaw[i] = newRaw[i];
    lastRawMs = now;
  }

  if (now - lastRawMs > 120) { stopMotors(); return; }

  float error = 0.0f;
  float weighted = 0.0f, sum = 0.0f;
  bool hasLine = false;
  for (int i = 0; i < 6; i++) {
    if (latestRaw[i] > SENSOR_THRESHOLDS[i]) {
      weighted += SENSOR_WEIGHTS[i];
      sum += 1.0f;
      hasLine = true;
    }
  }

  if (hasLine) {
    error = weighted / sum;
    followMode = FollowMode::Track;
    lostLineFrames = 0;
    lastSeenError = error;
  } else {
    if (lostLineFrames < 255) lostLineFrames++;
    if (lostLineFrames >= 3) followMode = FollowMode::Search;
  }

  if (followMode == FollowMode::Search) {
    if (lastSeenError > 0.0f) driveMotors(-110, 110);
    else driveMotors(110, -110);
  } else {
    filteredError = ERROR_SMOOTH_ALPHA * filteredError + (1.0f - ERROR_SMOOTH_ALPHA) * error;
    float dError = filteredError - prevFilteredError;
    prevFilteredError = filteredError;

    float correction = KP * filteredError + KD * dError;
    float dynamicBase = (float)BASE_SPEED_LEFT - (abs(filteredError) * TURN_DECELERATION);

    driveMotors(dynamicBase - correction, dynamicBase + correction);
  }
}

} // namespace

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  lineSerial.begin(LINE_UART_BAUD);
  
  leftMotor.begin();
  rightMotor.begin();

  // Init TOF Sensors
  for (uint8_t i = 0; i < 4; i++) {
    if (selectMuxChannel(SENSOR_CHANNELS[i])) {
      tofSensors[i].setTimeout(60);
      if (tofSensors[i].init()) {
        tofSensors[i].setDistanceMode(VL53L1X::Long);
        tofSensors[i].setMeasurementTimingBudget(50000);
        tofSensors[i].startContinuous(50);
        tofReady[i] = true;
      }
    }
  }
  Serial.println(F("System Initialized."));
}

void loop() {
  switch (currentState) {
    case State::CORNER_START:
      runCornerStartManeuver();
      currentState = State::LINE_FOLLOW;
      break;

    case State::LINE_FOLLOW:
      runLineFollow();
      break;
  }
}