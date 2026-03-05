#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <L298N.h>

namespace {
constexpr uint8_t RIGHT_IN1 = 7;
constexpr uint8_t RIGHT_IN2 = 8;
constexpr uint8_t RIGHT_PWM = 6;
constexpr uint8_t LEFT_IN1 = 4;
constexpr uint8_t LEFT_IN2 = 5;
constexpr uint8_t LEFT_PWM = 3;

constexpr uint8_t PCA9548A_ADDR = 0x70;
constexpr uint8_t SENSOR_CHANNELS[] = {0, 1, 2, 3};
constexpr uint8_t SENSOR_COUNT = sizeof(SENSOR_CHANNELS) / sizeof(SENSOR_CHANNELS[0]);

constexpr uint8_t IR_SENSOR_PIN = A0;
constexpr uint8_t LINE_SENSOR_PIN = A1;

constexpr uint8_t SAMPLES_PER_SENSOR = 10;
constexpr uint16_t LOOP_DELAY_MS = 200;

// constexpr uint8_t LEFT_DRIVE_SPEED = 200;
// constexpr uint8_t RIGHT_DRIVE_SPEED = 230;

constexpr uint8_t LEFT_DRIVE_SPEED = 180;
constexpr uint8_t RIGHT_DRIVE_SPEED = 210;

L298NMotor leftMotor(LEFT_IN1, LEFT_IN2, LEFT_PWM);
L298NMotor rightMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_PWM);

VL53L1X tofSensors[SENSOR_COUNT];
bool tofReady[SENSOR_COUNT] = {false, false, false, false};

enum class NextAction : uint8_t {
  Idle = 0,
  LeftForward, 
  LeftReverse,
  RightForward,
  RightReverse,
};

struct SensorSnapshot {
  int16_t tofAvgMm[SENSOR_COUNT];
  int16_t irAvg;
  int16_t lineAvg;
};

bool selectMuxChannel(uint8_t channel) {
  if (channel > 7) return false;

  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  return Wire.endTransmission() == 0;
}

bool initTofSensors() {
  bool anyReady = false;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    const uint8_t channel = SENSOR_CHANNELS[i];

    if (!selectMuxChannel(channel)) {
      Serial.print(F("Mux select failed on channel "));
      Serial.println(channel);
      continue;
    }

    tofSensors[i].setTimeout(60);
    if (!tofSensors[i].init()) {
      Serial.print(F("TOF init failed on channel "));
      Serial.println(channel);
      continue;
    }

    tofSensors[i].setDistanceMode(VL53L1X::Long);
    tofSensors[i].setMeasurementTimingBudget(50000);
    tofSensors[i].startContinuous(50);

    tofReady[i] = true;
    anyReady = true;

    Serial.print(F("TOF ready on channel "));
    Serial.println(channel);
  }

  return anyReady;
}

int16_t readAveragedTof(uint8_t sensorIndex) {
  if (sensorIndex >= SENSOR_COUNT || !tofReady[sensorIndex]) return -1;

  const uint8_t channel = SENSOR_CHANNELS[sensorIndex];
  int32_t sum = 0;
  uint8_t validCount = 0;

  for (uint8_t sample = 0; sample < SAMPLES_PER_SENSOR; sample++) {
    if (!selectMuxChannel(channel)) continue;

    uint16_t mm = tofSensors[sensorIndex].read();
    if (!tofSensors[sensorIndex].timeoutOccurred()) {
      sum += mm;
      validCount++;
    }
  }

  if (validCount == 0) return -1;
  return static_cast<int16_t>(sum / validCount);
}

int16_t readAveragedAnalog(uint8_t pin) {
  int32_t sum = 0;
  for (uint8_t sample = 0; sample < SAMPLES_PER_SENSOR; sample++) {
    sum += analogRead(pin);
  }

  return static_cast<int16_t>(sum / SAMPLES_PER_SENSOR);
}

SensorSnapshot takeMeasurements() {
  SensorSnapshot snapshot{};

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    snapshot.tofAvgMm[i] = readAveragedTof(i);
  }

  snapshot.irAvg = readAveragedAnalog(IR_SENSOR_PIN);
  snapshot.lineAvg = readAveragedAnalog(LINE_SENSOR_PIN);

  return snapshot;
}

NextAction decideNextAction(const SensorSnapshot &snapshot) {
    const int16_t backDist = snapshot.tofAvgMm[0];
    const int16_t leftDist = snapshot.tofAvgMm[1];
    const int16_t frontDist = snapshot.tofAvgMm[2];
    const int16_t rightDist = snapshot.tofAvgMm[3];

    if (backDist > 254) {
        if (leftDist > rightDist) {
            return NextAction::LeftReverse;
        } else {
            return NextAction::RightReverse;
        }
    } else if (frontDist > 254) {
        if (leftDist > rightDist) {
            return NextAction::RightForward;
        } else {
            return NextAction::LeftForward;
        }
    }

//   const int16_t frontDistanceMm = snapshot.tofAvgMm[2];
//   const int16_t 

//   if (frontDistanceMm > 0 && frontDistanceMm < 250) {
//     return NextAction::Reverse;
//   }

//   if (snapshot.lineAvg < 300) {
//     return NextAction::TurnRight;
//   }

//   if (snapshot.irAvg > 700) {
//     return NextAction::TurnLeft;
//   }

//   return NextAction::MoveForward;
}

void handleNextAction(NextAction action, const SensorSnapshot &snapshot) {
  Serial.print(F("TOF avg mm: "));
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(snapshot.tofAvgMm[i]);
    if (i + 1 < SENSOR_COUNT) Serial.print(F(", "));
  }

  Serial.print(F(" | IR avg: "));
  Serial.print(snapshot.irAvg);
  Serial.print(F(" | Line avg: "));
  Serial.print(snapshot.lineAvg);
  Serial.print(F(" | Action: "));

  switch (action) {
    case NextAction::Idle:
      Serial.println(F("Idle"));
      break;
    case NextAction::LeftForward:
      Serial.println(F("LeftForward"));
      // move left motor forward, right motor stopped
      leftMotor.drive(L298NMotor::Direction::Forward, LEFT_DRIVE_SPEED);
      rightMotor.drive(L298NMotor::Direction::Brake, 0);
      break;
    case NextAction::LeftReverse:
      Serial.println(F("LeftReverse"));
        // move left motor backward, right motor stopped
        leftMotor.drive(L298NMotor::Direction::Backward, LEFT_DRIVE_SPEED);
        rightMotor.drive(L298NMotor::Direction::Brake, 0);
      break;
    case NextAction::RightForward:
      Serial.println(F("RightForward"));
        // move right motor forward, left motor stopped
        rightMotor.drive(L298NMotor::Direction::Forward, RIGHT_DRIVE_SPEED);
        leftMotor.drive(L298NMotor::Direction::Brake, 0);
      break;
    case NextAction::RightReverse:
      Serial.println(F("RightReverse"));
        // move right motor backward, left motor stopped
        rightMotor.drive(L298NMotor::Direction::Backward, RIGHT_DRIVE_SPEED);
        leftMotor.drive(L298NMotor::Direction::Brake, 0);
      break;
  }

  delay(500);

  // stop all motors
  leftMotor.stop();
  rightMotor.stop();

  delay(250);

  SensorSnapshot snap = takeMeasurements();

  if (snap.tofAvgMm[0] < snap.tofAvgMm[2]) {
    leftMotor.drive(L298NMotor::Direction::Forward, LEFT_DRIVE_SPEED);
    rightMotor.drive(L298NMotor::Direction::Forward, RIGHT_DRIVE_SPEED);
  } else {
    leftMotor.drive(L298NMotor::Direction::Backward, LEFT_DRIVE_SPEED);
    rightMotor.drive(L298NMotor::Direction::Backward, RIGHT_DRIVE_SPEED);
  }

  delay(500);

// stop all motors
  leftMotor.stop();
  rightMotor.stop();

  delay(250);

  // Action execution placeholder:
  // - drive motors
  // - set servo state
  // - update FSM state
}
}  // namespace

void setup() {
  Serial.begin(115200);

  leftMotor.begin();
  rightMotor.begin();

  Wire.begin();
  Wire.setClock(400000);

  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(LINE_SENSOR_PIN, INPUT);

  Serial.println(F("corner_start_1 boilerplate starting..."));

  if (!initTofSensors()) {
    Serial.println(F("Warning: no TOF sensors initialized."));
  }
}

void loop() {
  SensorSnapshot snapshot = takeMeasurements();
  NextAction action = decideNextAction(snapshot);
  handleNextAction(action, snapshot);
  
  for (;;) {
    // wait forever
  }

  delay(LOOP_DELAY_MS);
}
