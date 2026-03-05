#include <Arduino.h>
#include <L298N.h>

namespace {
// Motor wiring copied from arena_test_1.
constexpr uint8_t RIGHT_IN1 = 7;
constexpr uint8_t RIGHT_IN2 = 8;
constexpr uint8_t RIGHT_PWM = 6;
constexpr uint8_t LEFT_IN1 = 4;
constexpr uint8_t LEFT_IN2 = 5;
constexpr uint8_t LEFT_PWM = 3;

// Line board IR emitter enable (same convention as line_test).
constexpr uint8_t IR_EMITTER_PIN = 9;

// Physical left->right sensor order is A3, A2, A1.
constexpr uint8_t SENSOR_PINS[3] = {A3, A2, A1};
constexpr float SENSOR_WEIGHTS[3] = {-1.0f, 0.0f, 1.0f};

// Calibration points (left->right: A3, A2, A1). Update after running a
// calibration pass in your setup.
constexpr int SENSOR_WHITE[3] = {434, 473, 519};
constexpr int SENSOR_BLACK[3] = {979, 987, 990};

// Black tape produced higher readings in your measurements.
constexpr bool LINE_IS_DARK = false;

constexpr uint8_t BASE_SPEED_LEFT = 155;
constexpr uint8_t BASE_SPEED_RIGHT = 165;
constexpr uint8_t MIN_DRIVE_SPEED = 85;
constexpr uint8_t MAX_DRIVE_SPEED = 230;

constexpr float KP = 24.0f;
constexpr float KD = 55.0f;
constexpr float ERROR_SMOOTH_ALPHA = 0.55f;

// Ignore tiny normalized responses as floor noise.
constexpr float SENSOR_ACTIVE_MIN = 0.18f;
// With a thick line (usually 2 sensors active), this keeps tracking stable.
constexpr float TRACK_CONFIDENCE_MIN = 0.65f;
constexpr uint8_t SEARCH_SPEED = 120;
constexpr unsigned long LOOP_DT_MS = 8;

L298NMotor leftMotor(LEFT_IN1, LEFT_IN2, LEFT_PWM);
L298NMotor rightMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_PWM);

float filteredError = 0.0f;
float prevFilteredError = 0.0f;
float lastSeenError = 0.0f;

unsigned long lastLoopMs = 0;

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

float normalize01(int raw, int whiteVal, int blackVal) {
  int span = blackVal - whiteVal;
  if (span < 25) {
    return raw / 1023.0f;
  }

  float norm = static_cast<float>(raw - whiteVal) / static_cast<float>(span);
  return clampf(norm, 0.0f, 1.0f);
}

void readSensors(float lineStrength[3], int rawOut[3]) {
  for (int i = 0; i < 3; i++) {
    int raw = analogRead(SENSOR_PINS[i]);
    rawOut[i] = raw;

    float norm = normalize01(raw, SENSOR_WHITE[i], SENSOR_BLACK[i]);
    float lineResponse = LINE_IS_DARK ? (1.0f - norm) : norm;
    lineStrength[i] = (lineResponse > SENSOR_ACTIVE_MIN) ? lineResponse : 0.0f;
  }
}

} // namespace

void setup() {
  Serial.begin(115200);

  pinMode(IR_EMITTER_PIN, OUTPUT);
  digitalWrite(IR_EMITTER_PIN, HIGH);

  leftMotor.begin();
  rightMotor.begin();

  Serial.println(F("line_follow_2 ready"));
  Serial.println(F("sensor order left->right: A3 A2 A1"));
}

void loop() {
  unsigned long now = millis();
  if (now - lastLoopMs < LOOP_DT_MS) {
    return;
  }
  lastLoopMs = now;

  float strength[3];
  int raw[3];
  readSensors(strength, raw);

  float weighted = 0.0f;
  float sum = 0.0f;
  for (int i = 0; i < 3; i++) {
    weighted += strength[i] * SENSOR_WEIGHTS[i];
    sum += strength[i];
  }

  if (sum >= TRACK_CONFIDENCE_MIN) {
    float error = weighted / sum;

    filteredError = ERROR_SMOOTH_ALPHA * filteredError + (1.0f - ERROR_SMOOTH_ALPHA) * error;
    float dError = filteredError - prevFilteredError;
    prevFilteredError = filteredError;

    lastSeenError = filteredError;

    float correction = KP * filteredError + KD * dError;

    float leftCmd = BASE_SPEED_LEFT + correction;
    float rightCmd = BASE_SPEED_RIGHT - correction;

    leftCmd = clampf(leftCmd, MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
    rightCmd = clampf(rightCmd, MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);

    leftMotor.drive(L298NMotor::Direction::Forward, clampPwm(leftCmd));
    rightMotor.drive(L298NMotor::Direction::Forward, clampPwm(rightCmd));
  } else {
    // If line is briefly lost, pivot toward the side where it was last seen.
    if (lastSeenError >= 0.0f) {
      leftMotor.drive(L298NMotor::Direction::Forward, SEARCH_SPEED);
      rightMotor.drive(L298NMotor::Direction::Backward, SEARCH_SPEED);
    } else {
      leftMotor.drive(L298NMotor::Direction::Backward, SEARCH_SPEED);
      rightMotor.drive(L298NMotor::Direction::Forward, SEARCH_SPEED);
    }
  }

  static unsigned long lastPrintMs = 0;
  if (now - lastPrintMs >= 100) {
    lastPrintMs = now;
    Serial.print(F("raw:"));
    for (int i = 0; i < 3; i++) {
      Serial.print(raw[i]);
      if (i < 2) Serial.print(',');
    }
    Serial.print(F(" str:"));
    for (int i = 0; i < 3; i++) {
      Serial.print(strength[i], 2);
      if (i < 2) Serial.print(',');
    }
    Serial.print(F(" err:"));
    Serial.println(filteredError, 3);
  }
}
