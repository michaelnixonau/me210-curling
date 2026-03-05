#include <Arduino.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
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

// ─── Gyro ───
MPU6050 mpu;
static const uint8_t  GYRO_RANGE      = MPU6050_GYRO_FS_250;
static const uint16_t CALIB_SAMPLES   = 800;
static const uint16_t CALIB_DELAY_MS  = 2;

float gbz = 0;            // Z-axis gyro bias (raw LSB)
double totalZ = 0;         // integrated Z angle (degrees, unbounded)
uint32_t lastUs = 0;

float gyroLsbPerDps(uint8_t range) {
  switch (range) {
    case MPU6050_GYRO_FS_250:  return 131.0f;
    case MPU6050_GYRO_FS_500:  return 65.5f;
    case MPU6050_GYRO_FS_1000: return 32.8f;
    case MPU6050_GYRO_FS_2000: return 16.4f;
    default: return 131.0f;
  }
}

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

void calibrateGyroBias() {
  Serial.println(F("Calibrating gyro — keep still..."));
  long sz = 0;
  int16_t ax, ay, az, gx, gy, gz;
  for (uint16_t i = 0; i < CALIB_SAMPLES; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sz += gz;
    delay(CALIB_DELAY_MS);
  }
  gbz = (float)sz / CALIB_SAMPLES;
  Serial.print(F("Z bias (LSB): ")); Serial.println(gbz, 2);
}

// Integrate gyro Z and return current totalZ in degrees
void updateGyro() {
  uint32_t nowUs = micros();
  float dt = (nowUs - lastUs) / 1000000.0f;
  lastUs = nowUs;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float wz = (gz - gbz) / gyroLsbPerDps(GYRO_RANGE);  // deg/s
  totalZ += (double)wz * dt;
}

// Spin in place: positive angle → left motor backward, right motor forward (CCW)
void startTurn(float angleDeg) {
  startAngle = totalZ;

  Serial.print(F("Turning ")); Serial.print(angleDeg, 1); Serial.println(F(" deg"));

  if (angleDeg > 0) {
    // CCW: left backward, right forward
    leftMotor.drive(L298NMotor::Direction::Backward, LEFT_TURN_SPEED);
    rightMotor.drive(L298NMotor::Direction::Forward,  RIGHT_TURN_SPEED);
  } else {
    // CW: left forward, right backward
    leftMotor.drive(L298NMotor::Direction::Forward,  LEFT_TURN_SPEED);
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

  // Gyro
  Wire.begin();
  Wire.setClock(400000);
  mpu.initialize();
  Serial.println(mpu.testConnection() ? F("MPU6050 OK") : F("MPU6050 FAIL"));
  mpu.setFullScaleGyroRange(GYRO_RANGE);
  mpu.setDLPFMode(3);
  mpu.setRate(0);

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
        Serial.print(F("Done. Traveled: ")); Serial.print(traveled, 1);
        Serial.print(F(" deg  (totalZ = ")); Serial.print(totalZ, 1);
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
