#include <Arduino.h>
#include <SimpleFSM.h>

SimpleFSM fsm;

/****************************************************************************** 
 *                              Pin Definitions                               * 
 ******************************************************************************/

struct Pins {
  // IR beacon sensor
  const int ir_beacon_receiver = A0;

  // Line following sensors
  const int line_left_emitter = A1;
  const int line_left_receiver = A2;
  const int line_right_emitter = A3;
  const int line_right_receiver = A4;

  // Drive motors
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
 *                             State Definitions                              * 
 ******************************************************************************/

// Homing States ---------------------------------------------------------------

State h[] = {
  State("orienting (primed)", NULL),
  State("orienting (primed)", NULL),
  State("wall alignment", NULL),
  State("drive to front line", NULL),
  State("rotating (90 deg)", NULL),
  State("drive to center line", NULL),
  State("rotating (-90 deg)", NULL),
  State("drive to back wall", NULL)
};

// Launching States ------------------------------------------------------------

State l[] = {
  State("forward", NULL),
  State("prime", NULL),
  State("shoot", NULL),
  State("backward", NULL),
  State("reload", NULL)
};

void setup() {
  setAllPinModes();
}

void loop() {

}