#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>
#include <L298N.h>

// ─── Motor pins ───
namespace {
constexpr uint8_t LEFT_IN1  = 8;
constexpr uint8_t LEFT_IN2  = 7;
constexpr uint8_t LEFT_PWM  = 6;
constexpr uint8_t RIGHT_IN1 = 5;
constexpr uint8_t RIGHT_IN2 = 4;
constexpr uint8_t RIGHT_PWM = 3;

L298NMotor leftMotor(LEFT_IN1, LEFT_IN2, LEFT_PWM);
L298NMotor rightMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_PWM);

// ─── Gyro over I2C mux ───
Adafruit_LSM6DSOX lsm6dsox;
static const uint8_t PCA9548A_ADDR   = 0x70;
static const uint8_t IMU_MUX_CHANNEL = 4; // zero-indexed port 4
static const uint16_t CALIB_SAMPLES  = 800;
static const uint16_t CALIB_DELAY_MS = 2;

float gbz = 0.0f;          // Z-axis gyro bias (deg/s)
double totalZ = 0.0;       // integrated Z angle (degrees, unbounded)
uint32_t lastUs = 0;

// ─── Turn sequence ───
// Positive = CCW, Negative = CW (looking down at the robot)
const float TARGET_ANGLES[] = { 90.0f, -90.0f, 180.0f, -180.0f };
const uint8_t NUM_TARGETS = sizeof(TARGET_ANGLES) / sizeof(TARGET_ANGLES[0]);
uint8_t targetIdx = 0;

// ─── Turning parameters ───
constexpr uint8_t  LEFT_TURN_SPEED  = 200;   // PWM 0-255 for left motor
constexpr uint8_t  RIGHT_TURN_SPEED = 133;   // PWM 0-255 for right motor
constexpr float    ANGLE_TOLERANCE  = 2.0f;  // degrees — stop when within this
constexpr uint32_t PAUSE_MS         = 1500;  // pause between turns

// ─── State machine ───
enum class State : uint8_t { IDLE, TURNING, PAUSING };
State state = State::IDLE;
double startAngle = 0;
uint32_t pauseStartMs = 0;

// ─── Helpers ───

bool selectMuxChannel(uint8_t channel) {
  if (channel > 7) return false;
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  return Wire.endTransmission() == 0;
}

void calibrateGyroBias() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  Serial.println(F("Calibrating gyro — keep still..."));
  float sz = 0.0f;

  for (uint16_t i = 0; i < CALIB_SAMPLES; i++) {
    lsm6dsox.getEvent(&accel, &gyro, &temp);
    sz += gyro.gyro.z;
    delay(CALIB_DELAY_MS);
  }

  const float radToDeg = 180.0f / PI;
  gbz = (sz / CALIB_SAMPLES) * radToDeg;
  Serial.print(F("Z bias (deg/s): "));
  Serial.println(gbz, 4);
}

// Integrate gyro Z and update totalZ in degrees
void updateGyro() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  uint32_t nowUs = micros();
  float dt = (nowUs - lastUs) / 1000000.0f;
  lastUs = nowUs;

  lsm6dsox.getEvent(&accel, &gyro, &temp);

  const float radToDeg = 180.0f / PI;
  float wz = gyro.gyro.z * radToDeg - gbz;  // deg/s
  totalZ += (double)wz * dt;
}

// Spin in place: positive angle → left motor backward, right motor forward (CCW)
void startTurn(float angleDeg) {
  startAngle = totalZ;

  Serial.print(F("Turning "));
  Serial.print(angleDeg, 1);
  Serial.println(F(" deg"));

  if (angleDeg > 0) {
    // CCW: left backward, right forward
    leftMotor.drive(L298NMotor::Direction::Backward, LEFT_TURN_SPEED);
    rightMotor.drive(L298NMotor::Direction::Forward, RIGHT_TURN_SPEED);
  } else {
    // CW: left forward, right backward
    leftMotor.drive(L298NMotor::Direction::Forward, LEFT_TURN_SPEED);
    rightMotor.drive(L298NMotor::Direction::Backward, RIGHT_TURN_SPEED);
  }

  state = State::TURNING;
}

void stopMotors() {
  leftMotor.drive(L298NMotor::Direction::Brake, 0);
  rightMotor.drive(L298NMotor::Direction::Brake, 0);
}

} // namespace

void setup() {
  Serial.begin(115200);

  // Motors
  leftMotor.begin();
  rightMotor.begin();

  // Gyro via mux
  Wire.begin();
  Wire.setClock(400000);

  if (!selectMuxChannel(IMU_MUX_CHANNEL)) {
    Serial.println(F("PCA9548A select FAIL"));
    while (1) delay(10);
  }

  if (!lsm6dsox.begin_I2C()) {
    Serial.println(F("LSM6DSOX FAIL"));
    while (1) delay(10);
  }
  Serial.println(F("LSM6DSOX OK (via PCA9548A ch4)"));

  lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  lsm6dsox.setGyroDataRate(LSM6DS_RATE_104_HZ);
  lsm6dsox.setAccelDataRate(LSM6DS_RATE_104_HZ);

  calibrateGyroBias();

  lastUs = micros();
  Serial.println(F("Starting turn sequence..."));
}

void loop() {
  updateGyro();

  switch (state) {
    case State::IDLE: {
      if (targetIdx < NUM_TARGETS) {
        startTurn(TARGET_ANGLES[targetIdx]);
      }
      // else: all turns done — do nothing
      break;
    }

    case State::TURNING: {
      float target = TARGET_ANGLES[targetIdx];
      double traveled = totalZ - startAngle;

      if ((target > 0 && traveled >= target - ANGLE_TOLERANCE) ||
          (target < 0 && traveled <= target + ANGLE_TOLERANCE)) {
        stopMotors();
        Serial.print(F("Done. Traveled: "));
        Serial.print(traveled, 1);
        Serial.print(F(" deg  (totalZ = "));
        Serial.print(totalZ, 1);
        Serial.println(F(" deg)"));

        targetIdx++;
        pauseStartMs = millis();
        state = State::PAUSING;
      }
      break;
    }

    case State::PAUSING: {
      if (millis() - pauseStartMs >= PAUSE_MS) {
        state = State::IDLE;   // will pick up next target
      }
      break;
    }
  }
}