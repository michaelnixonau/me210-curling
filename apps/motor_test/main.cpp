#include <Arduino.h>
#include <L298N.h>

namespace {
constexpr uint8_t LEFT_IN1 = 8;
constexpr uint8_t LEFT_IN2 = 7;
constexpr uint8_t LEFT_PWM = 6;
constexpr uint8_t RIGHT_IN1 = 5;
constexpr uint8_t RIGHT_IN2 = 4;
constexpr uint8_t RIGHT_PWM = 3;

constexpr uint8_t TEST_SPEED = 180;
constexpr unsigned long STEP_TIME_MS = 2000;

L298NMotor leftMotor(LEFT_IN1, LEFT_IN2, LEFT_PWM);
L298NMotor rightMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_PWM);

void driveBoth(L298NMotor::Direction direction, uint8_t speed) {
  leftMotor.drive(direction, speed);
  rightMotor.drive(direction, speed);
}
} // namespace

void setup() {
  Serial.begin(9600);

  leftMotor.begin();
  rightMotor.begin();
}

void loop() {
  static uint8_t step = 0;
  static unsigned long stepStartMs = 0;
  unsigned long now = millis();

  if (stepStartMs == 0) {
    stepStartMs = now;
  }

  if (now - stepStartMs >= STEP_TIME_MS) {
    step = (step + 1) % 4;
    stepStartMs = now;
  }

  switch (step) {
    case 0:
      driveBoth(L298NMotor::Direction::Forward, TEST_SPEED);
      break;
    case 1:
      driveBoth(L298NMotor::Direction::Coast, 0);
      break;
    case 2:
      driveBoth(L298NMotor::Direction::Backward, TEST_SPEED);
      break;
    case 3:
    default:
      driveBoth(L298NMotor::Direction::Brake, 0);
      break;
  }
}
