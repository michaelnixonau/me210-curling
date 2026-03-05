#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>
#include <L298N.h>

namespace {
constexpr uint8_t RIGHT_IN1 = 7;
constexpr uint8_t RIGHT_IN2 = 8;
constexpr uint8_t RIGHT_PWM = 6;
constexpr uint8_t LEFT_IN1 = 4;
constexpr uint8_t LEFT_IN2 = 5;
constexpr uint8_t LEFT_PWM = 3;

constexpr uint8_t PCA9548A_ADDR = 0x70;
constexpr uint8_t IMU_MUX_CHANNEL = 4;

constexpr uint8_t DIR_SOUTH = 0;
constexpr uint8_t DIR_WEST = 1;
constexpr uint8_t DIR_NORTH = 2;
constexpr uint8_t DIR_EAST = 3;

constexpr uint8_t SENSOR_CHANNELS[4] = {0, 1, 2, 3};
constexpr uint8_t SENSOR_COUNT = 4;

constexpr uint8_t TURN_SPEED_LEFT = 180;
constexpr uint8_t TURN_SPEED_RIGHT = 180;
constexpr uint8_t DRIVE_SPEED_LEFT = 190;
constexpr uint8_t DRIVE_SPEED_RIGHT = 190;
constexpr uint8_t STOP_PULSE_SPEED = 130;
constexpr uint8_t STOP_PULSE_MS = 40;

constexpr uint16_t ESCAPE_DRIVE_MS = 200;
constexpr uint16_t GYRO_CALIB_SAMPLES = 600;
constexpr uint16_t GYRO_CALIB_DELAY_MS = 2;
constexpr float HEADING_TOLERANCE_DEG = 5.0f;
constexpr uint16_t TURN_SETTLE_MS = 80;
constexpr uint16_t TURN_TIMEOUT_MS = 2500;

constexpr uint16_t BLINK_PERIOD_MS = 80;

L298NMotor leftMotor(LEFT_IN1, LEFT_IN2, LEFT_PWM);
L298NMotor rightMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_PWM);
Adafruit_LSM6DSOX imu;
VL53L1X sensors[SENSOR_COUNT];
bool sensorReady[SENSOR_COUNT] = {false, false, false, false};
int16_t distances[SENSOR_COUNT] = {-1, -1, -1, -1};

float gyroBiasZDegPerSec = 0.0f;
double integratedZDeg = 0.0;
uint32_t lastGyroUs = 0;

enum class MotionDir : uint8_t {
  None = 0,
  Forward,
  Backward,
  CW,
  CCW
};

MotionDir motionDir = MotionDir::None;

enum class Corner : uint8_t {
  Unknown = 0,
  SouthWest,
  NorthWest,
  NorthEast,
  SouthEast
};

const __FlashStringHelper *dirName(uint8_t direction) {
  switch (direction) {
    case DIR_SOUTH: return F("South");
    case DIR_WEST: return F("West");
    case DIR_NORTH: return F("North");
    case DIR_EAST: return F("East");
    default: return F("UnknownDir");
  }
}

const __FlashStringHelper *cornerName(Corner corner) {
  switch (corner) {
    case Corner::SouthWest: return F("SouthWest");
    case Corner::NorthWest: return F("NorthWest");
    case Corner::NorthEast: return F("NorthEast");
    case Corner::SouthEast: return F("SouthEast");
    case Corner::Unknown:
    default:
      return F("Unknown");
  }
}

float targetDeltaDegreesForDirection(uint8_t direction) {
  if (direction == DIR_NORTH) return 0.0f;
  if (direction == DIR_EAST) return -90.0f;
  if (direction == DIR_WEST) return 90.0f;
  if (direction == DIR_SOUTH) return 180.0f;
  return 0.0f;
}

void printDistances() {
  Serial.print(F("Distances(mm) S="));
  Serial.print(distances[DIR_SOUTH]);
  Serial.print(F(" W="));
  Serial.print(distances[DIR_WEST]);
  Serial.print(F(" N="));
  Serial.print(distances[DIR_NORTH]);
  Serial.print(F(" E="));
  Serial.println(distances[DIR_EAST]);
}

void printCornerScoringAndChoice(Corner chosenCorner) {
  struct Candidate {
    Corner corner;
    uint8_t a;
    uint8_t b;
  };

  static const Candidate candidates[] = {
    {Corner::SouthWest, DIR_SOUTH, DIR_WEST},
    {Corner::NorthWest, DIR_WEST, DIR_NORTH},
    {Corner::NorthEast, DIR_NORTH, DIR_EAST},
    {Corner::SouthEast, DIR_EAST, DIR_SOUTH},
  };

  Serial.println(F("Corner scoring (smaller adjacent sum => closer corner):"));
  for (const auto &candidate : candidates) {
    int16_t da = distances[candidate.a];
    int16_t db = distances[candidate.b];
    Serial.print(F("  "));
    Serial.print(cornerName(candidate.corner));
    Serial.print(F(": "));
    Serial.print(dirName(candidate.a));
    Serial.print(F("="));
    Serial.print(da);
    Serial.print(F(" + "));
    Serial.print(dirName(candidate.b));
    Serial.print(F("="));
    Serial.print(db);
    Serial.print(F(" => sum="));
    if (da < 0 || db < 0) {
      Serial.println(F("invalid"));
    } else {
      Serial.println(static_cast<int32_t>(da) + static_cast<int32_t>(db));
    }
  }

  Serial.print(F("Chosen corner: "));
  Serial.println(cornerName(chosenCorner));
}

bool selectMuxChannel(uint8_t channel) {
  if (channel > 7) return false;
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  return Wire.endTransmission() == 0;
}

bool initGyro() {
  if (!selectMuxChannel(IMU_MUX_CHANNEL)) return false;
  if (!imu.begin_I2C()) return false;

  imu.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  imu.setGyroDataRate(LSM6DS_RATE_104_HZ);
  imu.setAccelDataRate(LSM6DS_RATE_104_HZ);

  return true;
}

void calibrateGyroZBias() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  Serial.println(F("Calibrating gyro Z bias (keep robot still)..."));

  float sumZRadPerSec = 0.0f;
  for (uint16_t i = 0; i < GYRO_CALIB_SAMPLES; i++) {
    imu.getEvent(&accel, &gyro, &temp);
    sumZRadPerSec += gyro.gyro.z;
    delay(GYRO_CALIB_DELAY_MS);
  }

  const float radToDeg = 180.0f / PI;
  gyroBiasZDegPerSec = (sumZRadPerSec / GYRO_CALIB_SAMPLES) * radToDeg;

  Serial.print(F("Gyro Z bias (deg/s): "));
  Serial.println(gyroBiasZDegPerSec, 4);

  integratedZDeg = 0.0;
  lastGyroUs = micros();
}

void updateIntegratedHeading() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  uint32_t nowUs = micros();
  float dt = (nowUs - lastGyroUs) / 1000000.0f;
  lastGyroUs = nowUs;

  imu.getEvent(&accel, &gyro, &temp);

  const float radToDeg = 180.0f / PI;
  float wzDegPerSec = gyro.gyro.z * radToDeg - gyroBiasZDegPerSec;
  integratedZDeg += static_cast<double>(wzDegPerSec) * dt;
}

void stopMotors() {
  if (motionDir == MotionDir::Forward) {
    leftMotor.drive(L298NMotor::Direction::Backward, STOP_PULSE_SPEED);
    rightMotor.drive(L298NMotor::Direction::Backward, STOP_PULSE_SPEED);
    delay(STOP_PULSE_MS);
  } else if (motionDir == MotionDir::Backward) {
    leftMotor.drive(L298NMotor::Direction::Forward, STOP_PULSE_SPEED);
    rightMotor.drive(L298NMotor::Direction::Forward, STOP_PULSE_SPEED);
    delay(STOP_PULSE_MS);
  } else if (motionDir == MotionDir::CW) {
    leftMotor.drive(L298NMotor::Direction::Backward, STOP_PULSE_SPEED);
    rightMotor.drive(L298NMotor::Direction::Forward, STOP_PULSE_SPEED);
    delay(STOP_PULSE_MS);
  } else if (motionDir == MotionDir::CCW) {
    leftMotor.drive(L298NMotor::Direction::Forward, STOP_PULSE_SPEED);
    rightMotor.drive(L298NMotor::Direction::Backward, STOP_PULSE_SPEED);
    delay(STOP_PULSE_MS);
  }

  leftMotor.drive(L298NMotor::Direction::Brake, 0);
  rightMotor.drive(L298NMotor::Direction::Brake, 0);
  motionDir = MotionDir::None;
}

void driveForward(uint8_t leftSpeed, uint8_t rightSpeed) {
  leftMotor.drive(L298NMotor::Direction::Forward, leftSpeed);
  rightMotor.drive(L298NMotor::Direction::Forward, rightSpeed);
  motionDir = MotionDir::Forward;
}

void turnCW(uint8_t leftSpeed, uint8_t rightSpeed) {
  leftMotor.drive(L298NMotor::Direction::Forward, leftSpeed);
  rightMotor.drive(L298NMotor::Direction::Backward, rightSpeed);
  motionDir = MotionDir::CW;
}

void turnCCW(uint8_t leftSpeed, uint8_t rightSpeed) {
  leftMotor.drive(L298NMotor::Direction::Backward, leftSpeed);
  rightMotor.drive(L298NMotor::Direction::Forward, rightSpeed);
  motionDir = MotionDir::CCW;
}

bool initSensors() {
  bool anyReady = false;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    uint8_t channel = SENSOR_CHANNELS[i];

    if (!selectMuxChannel(channel)) continue;

    sensors[i].setTimeout(80);
    if (!sensors[i].init()) continue;

    sensors[i].setDistanceMode(VL53L1X::Long);
    sensors[i].setMeasurementTimingBudget(50000);
    sensors[i].startContinuous(50);

    sensorReady[i] = true;
    anyReady = true;
  }

  return anyReady;
}

void readAllDistancesOnce() {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    distances[i] = -1;

    if (!sensorReady[i]) continue;
    if (!selectMuxChannel(SENSOR_CHANNELS[i])) continue;

    uint16_t mm = sensors[i].read();
    if (!sensors[i].timeoutOccurred()) {
      distances[i] = static_cast<int16_t>(mm);
    }
  }
}

Corner inferCorner() {
  struct Candidate {
    Corner corner;
    uint8_t a;
    uint8_t b;
  };

  static const Candidate candidates[] = {
    {Corner::SouthWest, DIR_SOUTH, DIR_WEST},
    {Corner::NorthWest, DIR_WEST, DIR_NORTH},
    {Corner::NorthEast, DIR_NORTH, DIR_EAST},
    {Corner::SouthEast, DIR_EAST, DIR_SOUTH},
  };

  int32_t bestScore = INT32_MAX;
  Corner bestCorner = Corner::Unknown;

  for (const auto &candidate : candidates) {
    int16_t da = distances[candidate.a];
    int16_t db = distances[candidate.b];
    if (da < 0 || db < 0) continue;

    int32_t score = static_cast<int32_t>(da) + static_cast<int32_t>(db);
    if (score < bestScore) {
      bestScore = score;
      bestCorner = candidate.corner;
    }
  }

  return bestCorner;
}

uint8_t chooseEscapeDirection(Corner corner) {
  auto pickFarthest = [](uint8_t a, uint8_t b) {
    int16_t da = distances[a];
    int16_t db = distances[b];
    if (da < 0 && db < 0) return DIR_NORTH;
    if (da < 0) return b;
    if (db < 0) return a;
    return (da >= db) ? a : b;
  };

  switch (corner) {
    case Corner::SouthWest:
      return pickFarthest(DIR_NORTH, DIR_EAST);
    case Corner::NorthWest:
      return pickFarthest(DIR_SOUTH, DIR_EAST);
    case Corner::NorthEast:
      return pickFarthest(DIR_SOUTH, DIR_WEST);
    case Corner::SouthEast:
      return pickFarthest(DIR_NORTH, DIR_WEST);
    case Corner::Unknown:
    default:
      break;
  }

  uint8_t bestDir = DIR_NORTH;
  int16_t bestDist = -1;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (distances[i] > bestDist) {
      bestDist = distances[i];
      bestDir = i;
    }
  }
  return bestDir;
}

void printEscapeChoice(Corner corner, uint8_t escapeDirection) {
  Serial.print(F("Escape planning from "));
  Serial.print(cornerName(corner));
  Serial.print(F(": heading "));
  Serial.print(dirName(escapeDirection));

  Serial.print(F(" -> target turn delta="));
  Serial.print(targetDeltaDegreesForDirection(escapeDirection), 1);
  Serial.print(F(" deg (gyro-closed-loop), then forward "));
  Serial.print(ESCAPE_DRIVE_MS);
  Serial.println(F(" ms"));
}

void turnToHeadingGyro(float targetDeltaDeg) {
  if (targetDeltaDeg > -HEADING_TOLERANCE_DEG && targetDeltaDeg < HEADING_TOLERANCE_DEG) {
    return;
  }

  uint32_t startMs = millis();
  while (true) {
    updateIntegratedHeading();

    float errorDeg = targetDeltaDeg - static_cast<float>(integratedZDeg);
    if (errorDeg >= -HEADING_TOLERANCE_DEG && errorDeg <= HEADING_TOLERANCE_DEG) {
      break;
    }

    if (errorDeg < 0.0f) {
      turnCW(TURN_SPEED_LEFT, TURN_SPEED_RIGHT);
    } else {
      turnCCW(TURN_SPEED_LEFT, TURN_SPEED_RIGHT);
    }

    if (millis() - startMs > TURN_TIMEOUT_MS) {
      Serial.println(F("Gyro turn timeout; stopping turn"));
      break;
    }
  }

  stopMotors();
  delay(TURN_SETTLE_MS);
}

void turnToDirectionAndEscapeGyro(uint8_t direction, bool gyroReady) {
  if (gyroReady) {
    float targetDeltaDeg = targetDeltaDegreesForDirection(direction);
    turnToHeadingGyro(targetDeltaDeg);
    Serial.print(F("Final integrated Z (deg): "));
    Serial.println(integratedZDeg, 2);
  } else {
    Serial.println(F("Gyro unavailable; skipping turn and driving forward"));
  }

  driveForward(DRIVE_SPEED_LEFT, DRIVE_SPEED_RIGHT);
  delay(ESCAPE_DRIVE_MS);
  stopMotors();
}
} // namespace

void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  leftMotor.begin();
  rightMotor.begin();

  Wire.begin();
  Wire.setClock(400000);

  bool gyroOk = initGyro();
  if (gyroOk) {
    Serial.println(F("IMU init OK on PCA9548A ch4"));
    calibrateGyroZBias();
  } else {
    Serial.println(F("IMU init FAILED on PCA9548A ch4"));
  }

  bool sensorsOk = initSensors();
  delay(70);
  readAllDistancesOnce();
  printDistances();

  Corner corner = inferCorner();
  printCornerScoringAndChoice(corner);
  uint8_t escapeDirection = chooseEscapeDirection(corner);
  printEscapeChoice(corner, escapeDirection);

  if (sensorsOk) {
    turnToDirectionAndEscapeGyro(escapeDirection, gyroOk);
  } else {
    Serial.print(F("Sensor init failed; fallback: forward "));
    Serial.print(ESCAPE_DRIVE_MS);
    Serial.println(F(" ms"));
    driveForward(DRIVE_SPEED_LEFT, DRIVE_SPEED_RIGHT);
    delay(ESCAPE_DRIVE_MS);
    stopMotors();
  }
}

void loop() {
  static bool ledOn = false;
  ledOn = !ledOn;
  digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
  delay(BLINK_PERIOD_MS);
}
