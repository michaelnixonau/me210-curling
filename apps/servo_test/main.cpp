#include <Arduino.h>
#include <Servo.h>

namespace {
constexpr uint8_t SERVO_PIN = 11;
constexpr uint8_t SERVO_MIN_ANGLE_DEG = 73;
constexpr uint8_t SERVO_MAX_ANGLE_DEG = 157;
constexpr unsigned long SERVO_HOLD_TIME_MS = 1000;

Servo testServo;
} // namespace

void setup() {
  testServo.attach(SERVO_PIN);
  testServo.write(SERVO_MIN_ANGLE_DEG);
}

void loop() {
  static bool atMinAngle = true;

  if (atMinAngle) {
    testServo.write(SERVO_MAX_ANGLE_DEG);
  } else {
    testServo.write(SERVO_MIN_ANGLE_DEG);
  }

  atMinAngle = !atMinAngle;
  delay(SERVO_HOLD_TIME_MS);
}
