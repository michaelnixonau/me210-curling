#include "L298N.h"

L298NMotor::L298NMotor(uint8_t in1Pin, uint8_t in2Pin, uint8_t enablePin)
    : _in1Pin(in1Pin), _in2Pin(in2Pin), _enablePin(enablePin), _speed(0), _direction(Direction::Coast) {}

void L298NMotor::begin() {
  pinMode(_in1Pin, OUTPUT);
  pinMode(_in2Pin, OUTPUT);
  pinMode(_enablePin, OUTPUT);
  stop(false);
}

void L298NMotor::setSpeed(uint8_t speed) {
  _speed = speed;
  analogWrite(_enablePin, _speed);
}

void L298NMotor::setDirection(Direction direction) {
  _direction = direction;

  switch (_direction) {
    case Direction::Forward:
      digitalWrite(_in1Pin, HIGH);
      digitalWrite(_in2Pin, LOW);
      break;
    case Direction::Backward:
      digitalWrite(_in1Pin, LOW);
      digitalWrite(_in2Pin, HIGH);
      break;
    case Direction::Brake:
      digitalWrite(_in1Pin, HIGH);
      digitalWrite(_in2Pin, HIGH);
      break;
    case Direction::Coast:
    default:
      digitalWrite(_in1Pin, LOW);
      digitalWrite(_in2Pin, LOW);
      break;
  }
}

void L298NMotor::drive(Direction direction, uint8_t speed) {
  setDirection(direction);
  setSpeed(speed);
}

void L298NMotor::stop(bool brake) {
  setSpeed(0);
  setDirection(brake ? Direction::Brake : Direction::Coast);
}

uint8_t L298NMotor::getSpeed() const {
  return _speed;
}

L298NMotor::Direction L298NMotor::getDirection() const {
  return _direction;
}
