#include <Arduino.h>
#include <L298N.h>
#include <SoftwareSerial.h>

namespace {
constexpr uint8_t RIGHT_IN1 = 7;
constexpr uint8_t RIGHT_IN2 = 8;
constexpr uint8_t RIGHT_PWM = 6;
constexpr uint8_t LEFT_IN1 = 4;
constexpr uint8_t LEFT_IN2 = 5;
constexpr uint8_t LEFT_PWM = 3;

// SoftwareSerial on the line-follower MCU.
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
constexpr uint8_t BASE_SPEED_LEFT = 150;
constexpr uint8_t BASE_SPEED_RIGHT = 160;
// Calculates universal multiplier to ensure the right motor always matches the left proportionately
constexpr float RIGHT_MOTOR_SCALAR = static_cast<float>(BASE_SPEED_RIGHT) / static_cast<float>(BASE_SPEED_LEFT); 

constexpr float KP = 25.0f; // Increased to ensure it bites the edge without hardcoding limits
constexpr float KD = 65.0f;
constexpr float ERROR_SMOOTH_ALPHA = 0.45f;

// The secret to smooth tracking/latching: Slows down forward speed proportionately as error increases
constexpr float TURN_DECELERATION = 18.0f; 

constexpr uint8_t SEARCH_SPIN_SPEED = 110;
constexpr unsigned long LOOP_DT_MS = 8;
constexpr unsigned long LINE_RX_TIMEOUT_US = 12000;
constexpr unsigned long LINE_STALE_MS = 120;
constexpr uint8_t TRACK_EXIT_FRAMES = 3; // Reduced to enter search mode quicker if lost

L298NMotor leftMotor(LEFT_IN1, LEFT_IN2, LEFT_PWM);
L298NMotor rightMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_PWM);
SoftwareSerial lineSerial(LINE_UART_RX_PIN, LINE_UART_TX_PIN);

uint16_t latestRaw[SENSOR_COUNT] = {0};
bool lineFresh = false;
unsigned long lastRawMs = 0;
unsigned long lastLoopMs = 0;
float filteredError = 0.0f;
float prevFilteredError = 0.0f;
float lastSeenError = 0.0f;
uint8_t lostLineFrames = 0;

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

} // namespace

void setup() {
  Serial.begin(115200);
  lineSerial.begin(LINE_UART_BAUD);

  leftMotor.begin();
  rightMotor.begin();
  stopMotors();

  Serial.println(F("line_follow_3 ready"));
}

void loop() {
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
    stopMotors();
    return;
  }

  bool active[SENSOR_COUNT] = {false};
  float error = 0.0f;
  const bool hasLine = computeErrorFromRaw(latestRaw, error, active);

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
    if (lastSeenError > 0.0f) {
      driveMotors(-SEARCH_SPIN_SPEED, SEARCH_SPIN_SPEED); // Spin Left
    } else {
      driveMotors(SEARCH_SPIN_SPEED, -SEARCH_SPIN_SPEED); // Spin Right
    }
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