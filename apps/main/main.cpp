#include <Arduino.h>
#include <SimpleFSM.h>
#include <L298N.h>

SimpleFSM gameFsm;
SimpleFSM fsm;

constexpr int GAME_DURATION_MS = 2 * 60 * 1000; // 2 minutes

// Helper function to convert percentage to 8-bit value (0-255)
// This is done at compile time to prevent doing float math on the MCU
constexpr uint8_t percent(uint8_t p) {
  return (p > 100) ? 255 : (uint8_t)((p * 255UL) / 100);
}

/****************************************************************************** 
 *                              Pin Definitions                               * 
 ******************************************************************************/

struct PinConfig {
  // IR beacon sensor
  const int ir_beacon_receiver = A0;

  // Line following sensors
  const int line_left_emitter = A1;
  const int line_left_receiver = A2;
  const int line_right_emitter = A3;
  const int line_right_receiver = A4;

  // Drive motors (L298N motor driver)
  const int motor_left_in1 = 8;
  const int motor_left_in2 = 7;
  const int motor_left_pwm = 6;
  const int motor_right_in1 = 5;
  const int motor_right_in2 = 4;
  const int motor_right_pwm = 3;
} pins;

L298NMotor leftMotor(
  pins.motor_left_in1,
  pins.motor_left_in2,
  pins.motor_left_pwm
);
L298NMotor rightMotor(
  pins.motor_right_in1,
  pins.motor_right_in2,
  pins.motor_right_pwm
);

using Direction = L298NMotor::Direction;

void setAllPinModes() {
  // IR beacon sensor
  pinMode(pins.ir_beacon_receiver, INPUT);

  // Line following sensors
  pinMode(pins.line_left_emitter, OUTPUT);
  pinMode(pins.line_left_receiver, INPUT);
  pinMode(pins.line_right_emitter, OUTPUT);
  pinMode(pins.line_right_receiver, INPUT);

  // Drive motors
  leftMotor.begin();
  rightMotor.begin();
}

/****************************************************************************** 
 *                         State Callback Procedures                          * 
 ******************************************************************************/

// Game State ------------------------------------------------------------------

void game_end() {
  leftMotor.stop();
  rightMotor.stop();
}

// Homing State ----------------------------------------------------------------

void wall_alignment() {
  // @todo
  leftMotor.setDirection(Direction::Backward);
  leftMotor.setSpeed(percent(30));
  rightMotor.setDirection(Direction::Backward);
  rightMotor.setSpeed(percent(30));
}

void drive_to_front_line() {
  // @todo
  leftMotor.setDirection(Direction::Forward);
  leftMotor.setSpeed(percent(50));
  rightMotor.setDirection(Direction::Forward);
  rightMotor.setSpeed(percent(50));
}

void rotate_90() {
  // @todo
  leftMotor.setDirection(Direction::Forward);
  leftMotor.setSpeed(percent(50));
  rightMotor.setDirection(Direction::Backward);
  rightMotor.setSpeed(percent(50));
}

void drive_to_center_line() {
  // @todo
  rightMotor.setDirection(Direction::Forward);
  rightMotor.setSpeed(percent(50));
}

void rotate_neg_90() {
  // @todo
  leftMotor.setDirection(Direction::Backward);
  leftMotor.setSpeed(percent(50));
}

void drive_to_back_wall() {
  // @todo
  leftMotor.setDirection(Direction::Backward);
  leftMotor.setSpeed(percent(30));
  rightMotor.setDirection(Direction::Backward);
  rightMotor.setSpeed(percent(30));
}

// Launching State -------------------------------------------------------------

void forward() {
  // @todo
  leftMotor.setDirection(Direction::Forward);
  leftMotor.setSpeed(percent(100));
  rightMotor.setDirection(Direction::Forward);
  rightMotor.setSpeed(percent(100));
}

void prime() {
  // @todo
  // @todo: safety gate down
}

void shoot() {
  // @todo
  leftMotor.setDirection(Direction::Brake);
  leftMotor.setSpeed(0);
  rightMotor.setDirection(Direction::Brake);
  rightMotor.setSpeed(0);
}

void backward() {
  // @todo
  leftMotor.setDirection(Direction::Backward);
  leftMotor.setSpeed(percent(100));
  rightMotor.setDirection(Direction::Backward);
  rightMotor.setSpeed(percent(100));
}

void reload() {
  // @todo
  leftMotor.setDirection(Direction::Brake);
  leftMotor.setSpeed(0);
  rightMotor.setDirection(Direction::Brake);
  rightMotor.setSpeed(0);
}

/****************************************************************************** 
 *                             Game State Machine                             * 
 ******************************************************************************/

State gameStates[] = {
  State("GAME START", NULL),
  State("GAME END", game_end)
};

TimedTransition gameTransitions[] = {
  TimedTransition(&gameStates[0], &gameStates[1], GAME_DURATION_MS)
};

int num_game_transitions = sizeof(gameTransitions) / sizeof(gameTransitions[0]);

/****************************************************************************** 
 *                             Main State Machine                             * 
 ******************************************************************************/

// Homing States ---------------------------------------------------------------

State h[] = {
  State("orienting (primed)", NULL),
  State("orienting (primed)", NULL),
  State("wall alignment", wall_alignment),
  State("drive to front line", drive_to_front_line),
  State("rotating (90 deg)", rotate_90),
  State("drive to center line", drive_to_center_line),
  State("rotating (-90 deg)", rotate_neg_90),
  State("drive to back wall", drive_to_back_wall)
};

// Launching States ------------------------------------------------------------

State l[] = {
  State("forward", forward),
  State("prime", prime),
  State("shoot", shoot),
  State("backward", backward),
  State("reload", reload)
};

// State Transitions -----------------------------------------------------------

Transition transitions[] = {
  // Homing transitions
  Transition(&h[0], &h[1], -1),
  Transition(&h[1], &h[2], -1),
  Transition(&h[2], &h[3], -1),
  Transition(&h[3], &h[4], -1),
  Transition(&h[4], &h[3], -1),
  Transition(&h[4], &h[5], -1),
  Transition(&h[5], &h[6], -1),
  Transition(&h[6], &h[7], -1),

  // KEY: Homing to launching transition
  Transition(&h[7], &l[0], -1),

  // Launching transitions
  Transition(&l[0], &l[1], -1),
  Transition(&l[1], &l[2], -1),
  Transition(&l[2], &l[3], -1),
  Transition(&l[3], &l[4], -1),
  Transition(&l[4], &l[0], -1)
};

int num_transitions = sizeof(transitions) / sizeof(transitions[0]);

void setup() {
  setAllPinModes();

  gameFsm.add(gameTransitions, num_game_transitions);
  fsm.add(transitions, num_transitions);
}

void loop() {
  gameFsm.run();
  if (gameFsm.isInState(&gameStates[0])) {
    fsm.run();
  }
}