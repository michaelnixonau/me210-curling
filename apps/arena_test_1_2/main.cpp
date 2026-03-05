/**
 * ARENA TEST 1.1 - PATCHED WITH ROBUST LOCALISATION
 * * LOGIC:
 * 1. 360 Scan: Reads 4 VL53L1X sensors to map walls.
 * 2. Analyze: Determines (x,y) and target heading to face +Y.
 * 3. Turn: Rotates to face the hog line.
 * 4. Cycle: Drive Forward -> Release -> Reverse -> Wait.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <VL53L1X.h>
#include <L298N.h>
#include <Servo.h>

namespace {

// ===================== HARDWARE & ARENA CONSTANTS =====================
constexpr uint8_t RIGHT_IN1 = 7, RIGHT_IN2 = 8, RIGHT_PWM = 6;
constexpr uint8_t LEFT_IN1  = 4, LEFT_IN2  = 5, LEFT_PWM  = 3;
constexpr uint8_t PCA9548A_ADDR = 0x70;
constexpr uint8_t IMU_CH = 4;
constexpr uint8_t SERVO_PIN = 11;

const float ARENA_WIDTH  = 1219.2f;   // 4 feet
const float ARENA_LENGTH = 4876.8f;   // 16 feet
const float HALF_SPLIT_Y = ARENA_LENGTH * 0.5f;
const float DIR_MARGIN_MM = 120.0f;

// Offsets: Port 0:S, 1:W, 2:N, 3:E (Distance from center of rotation)
const float OFFSETS[4] = {63.5f, 158.75f, 158.75f, 234.95f}; 

// ===================== MOTOR & PID SETTINGS =====================
constexpr uint8_t SCAN_SPEED_L = 135;
constexpr uint8_t SCAN_SPEED_R = 185;
constexpr uint8_t DRIVE_SPEED_L = 144;
constexpr uint8_t DRIVE_SPEED_R = 190;
constexpr float DRIVE_KP = 2.0f;

constexpr uint16_t FORWARD_TARGET_MM = 4000; // Stop when N-wall is this far (near hog line)
constexpr uint16_t REVERSE_TARGET_MM = 250;  // Stop when S-wall is this close

// ===================== OBJECTS & GLOBALS =====================
L298NMotor leftMotor(LEFT_IN1, LEFT_IN2, LEFT_PWM);
L298NMotor rightMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_PWM);
Adafruit_LSM6DSOX lsm6dsox;
VL53L1X sensors[4];
Servo dumpServo;

float currentHeading = 0.0f;
float gyroBiasZ = 0.0f;
uint32_t lastUs = 0;
float targetHeading = 0.0f;
unsigned long phaseStartMs = 0;

enum class State { LOCALIZE_SCAN, TURN_TO_TARGET, FORWARD, OPEN_HOLD, REVERSE, WAIT };
State state = State::LOCALIZE_SCAN;

struct BestScan {
    float headingDeg = 0.0f;
    float d[4] = {0,0,0,0};
    bool hypothesisA = true; // True = Axis Aligned
    float cost = 1e9f;
} best;

// ===================== HELPERS =====================

bool selectMux(uint8_t ch) {
    Wire.beginTransmission(PCA9548A_ADDR);
    Wire.write(1 << ch);
    return Wire.endTransmission() == 0;
}

float wrap180(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

void brakeMotors() {
    leftMotor.stop(true);
    rightMotor.stop(true);
}

void updateIMU() {
    sensors_event_t accel, gyro, temp;
    selectMux(IMU_CH);
    lsm6dsox.getEvent(&accel, &gyro, &temp);
    unsigned long now = micros();
    float dt = (now - lastUs) / 1000000.0f;
    lastUs = now;
    currentHeading += ((gyro.gyro.z * 180.0f / PI) - gyroBiasZ) * dt;
}

bool readSensorCorrected(uint8_t port, float &out) {
    selectMux(port);
    uint16_t r = sensors[port].read(false);
    if (sensors[port].timeoutOccurred() || r < 40 || r > 4500) return false;
    out = (float)r + OFFSETS[port];
    return true;
}

// ===================== LOCALIZATION LOGIC =====================

void considerScanSample(float heading, const float d[4]) {
    float sumNS = d[2] + d[0];
    float sumEW = d[3] + d[1];
    float cA = fabsf(sumNS - ARENA_LENGTH) + fabsf(sumEW - ARENA_WIDTH);
    float cB = fabsf(sumNS - ARENA_WIDTH) + fabsf(sumEW - ARENA_LENGTH);
    bool hypA = (cA <= cB);
    float c = hypA ? cA : cB;

    if (c < best.cost) {
        best.cost = c;
        best.headingDeg = heading;
        best.hypothesisA = hypA;
        for (int i=0; i<4; i++) best.d[i] = d[i];
    }
}

void solveTargetHeading() {
    float S = best.d[0], W = best.d[1], N = best.d[2], E = best.d[3];
    if (best.hypothesisA) {
        bool facingPlusY = (fabsf(S - N) > DIR_MARGIN_MM) ? (S < N) : (S <= N);
        targetHeading = best.headingDeg + (facingPlusY ? 0.0f : 180.0f);
    } else {
        bool eastPointsPlusY = (fabsf(W - E) > DIR_MARGIN_MM) ? (W < E) : (W <= E);
        targetHeading = best.headingDeg + (eastPointsPlusY ? 90.0f : -90.0f);
    }
    targetHeading = currentHeading + wrap180(targetHeading - currentHeading);
}

} // namespace

// ===================== SETUP & LOOP =====================

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);
    leftMotor.begin();
    rightMotor.begin();
    dumpServo.attach(SERVO_PIN);
    dumpServo.write(157); // Closed

    selectMux(IMU_CH);
    if (!lsm6dsox.begin_I2C()) while(1);

    // Gyro Calibration
    float sum = 0;
    for(int i=0; i<500; i++) {
        sensors_event_t a, g, t;
        lsm6dsox.getEvent(&a, &g, &t);
        sum += g.gyro.z;
        delay(2);
    }
    gyroBiasZ = (sum / 500.0f) * 180.0f / PI;

    for (uint8_t i = 0; i < 4; i++) {
        if (selectMux(i)) {
            sensors[i].setTimeout(60);
            sensors[i].init();
            sensors[i].setDistanceMode(VL53L1X::Long);
            sensors[i].startContinuous(33);
        }
    }
    lastUs = micros();
    Serial.println(F("SYSTEM_READY"));
}

void loop() {
    updateIMU();

    switch (state) {
        case State::LOCALIZE_SCAN: {
            static float startH = currentHeading;
            leftMotor.drive(L298NMotor::Direction::Backward, SCAN_SPEED_L);
            rightMotor.drive(L298NMotor::Direction::Forward, SCAN_SPEED_R);
            
            float d[4];
            bool valid = true;
            for(int i=0; i<4; i++) if(!readSensorCorrected(i, d[i])) valid = false;
            if(valid) considerScanSample(currentHeading, d);

            if (fabsf(currentHeading - startH) >= 360.0f) {
                brakeMotors();
                solveTargetHeading();
                state = State::TURN_TO_TARGET;
                Serial.print(F("SCAN_DONE. TARGET_H: ")); Serial.println(targetHeading);
            }
            break;
        }

        case State::TURN_TO_TARGET: {
            float err = wrap180(targetHeading - currentHeading);
            if (fabsf(err) < 2.5f) {
                brakeMotors();
                delay(200);
                state = State::FORWARD;
                Serial.println(F("FACING_NORTH_START_CYCLE"));
            } else {
                int turnSpd = 160;
                if (err > 0) {
                    leftMotor.drive(L298NMotor::Direction::Backward, turnSpd);
                    rightMotor.drive(L298NMotor::Direction::Forward, turnSpd);
                } else {
                    leftMotor.drive(L298NMotor::Direction::Forward, turnSpd);
                    rightMotor.drive(L298NMotor::Direction::Backward, turnSpd);
                }
            }
            break;
        }

        case State::FORWARD: {
            float err = wrap180(targetHeading - currentHeading);
            float corr = DRIVE_KP * err;
            leftMotor.drive(L298NMotor::Direction::Forward, constrain(DRIVE_SPEED_L - corr, 0, 255));
            rightMotor.drive(L298NMotor::Direction::Forward, constrain(DRIVE_SPEED_R + corr, 0, 255));

            float distN;
            if (readSensorCorrected(2, distN) && distN >= FORWARD_TARGET_MM) {
                brakeMotors();
                dumpServo.write(73); // Open
                phaseStartMs = millis();
                state = State::OPEN_HOLD;
                Serial.println(F("HOG_LINE_REACHED"));
            }
            break;
        }

        case State::OPEN_HOLD: {
            if (millis() - phaseStartMs > 2000) {
                dumpServo.write(157); // Close
                state = State::REVERSE;
            }
            break;
        }

        case State::REVERSE: {
            float err = wrap180(targetHeading - currentHeading);
            float corr = DRIVE_KP * err;
            leftMotor.drive(L298NMotor::Direction::Backward, constrain(DRIVE_SPEED_L + corr, 0, 255));
            rightMotor.drive(L298NMotor::Direction::Backward, constrain(DRIVE_SPEED_R - corr, 0, 255));

            float distS;
            if (readSensorCorrected(0, distS) && distS <= REVERSE_TARGET_MM) {
                brakeMotors();
                phaseStartMs = millis();
                state = State::WAIT;
                Serial.println(F("BACK_IN_BAY"));
            }
            break;
        }

        case State::WAIT: {
            if (millis() - phaseStartMs > 5000) {
                state = State::FORWARD;
                Serial.println(F("RESTART_CYCLE"));
            }
            break;
        }
    }
}