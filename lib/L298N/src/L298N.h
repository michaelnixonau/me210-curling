#ifndef L298N_H
#define L298N_H

#include <Arduino.h>

class L298NMotor {
public:
  enum class Direction : uint8_t {
    Coast = 0,
    Forward,
    Backward,
    Brake
  };

  L298NMotor(uint8_t in1Pin, uint8_t in2Pin, uint8_t enablePin);

  void begin();
  void setSpeed(uint8_t speed);
  void setDirection(Direction direction);
  void drive(Direction direction, uint8_t speed);
  void stop(bool brake = false);

  uint8_t getSpeed() const;
  Direction getDirection() const;

private:
  uint8_t _in1Pin;
  uint8_t _in2Pin;
  uint8_t _enablePin;
  uint8_t _speed;
  Direction _direction;
};

#endif
