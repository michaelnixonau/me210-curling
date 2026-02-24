#include <Arduino.h>
#include <SimpleFSM.h>

SimpleFSM gameFsm;
SimpleFSM fsm;

constexpr int GAME_DURATION_MS = 2 * 60 * 1000; // 2 minutes

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

void setAllPinModes() {
  // IR beacon sensor
  pinMode(pins.ir_beacon_receiver, INPUT);

  // Line following sensors
  pinMode(pins.line_left_emitter, OUTPUT);
  pinMode(pins.line_left_receiver, INPUT);
  pinMode(pins.line_right_emitter, OUTPUT);
  pinMode(pins.line_right_receiver, INPUT);

  // Drive motors
  pinMode(pins.motor_left_in1, OUTPUT);
  pinMode(pins.motor_left_in2, OUTPUT);
  pinMode(pins.motor_left_pwm, OUTPUT);
  pinMode(pins.motor_right_in1, OUTPUT);
  pinMode(pins.motor_right_in2, OUTPUT);
  pinMode(pins.motor_right_pwm, OUTPUT);
}

/****************************************************************************** 
 *                         State Callback Procedures                          * 
 ******************************************************************************/

// Game State ------------------------------------------------------------------

void game_end() {
  // Stop the robot
  digitalWrite(pins.motor_left_in1, LOW);
  digitalWrite(pins.motor_left_in2, LOW);
  digitalWrite(pins.motor_left_pwm, LOW);
  digitalWrite(pins.motor_right_in1, LOW);
  digitalWrite(pins.motor_right_in2, LOW);
  digitalWrite(pins.motor_right_pwm, LOW);
}

// Homing State ----------------------------------------------------------------

void wall_alignment() {
  // @todo
}

void drive_to_front_line() {
  // @todo
}

void rotate_90() {
  // @todo
}

void drive_to_center_line() {
  // @todo
}

void rotate_neg_90() {
  // @todo
}

void drive_to_back_wall() {
  // @todo
}

// Launching State -------------------------------------------------------------

void forward() {
  // @todo
}

void prime() {
  // @todo
}

void shoot() {
  // @todo
}

void backward() {
  // @todo
}

void reload() {
  // @todo
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