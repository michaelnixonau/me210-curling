#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <VL53L1X.h>
#include <L298N.h>

// ---------------- ARENA CONSTANTS (mm) ----------------
const float ARENA_WIDTH  = 1219.2f;   // 4 feet
const float ARENA_LENGTH = 4876.8f;   // 16 feet

// "We are in one half" assumption used for disambiguation.
// If you are always in the SOUTH half, y should be <= LENGTH/2.
const float HALF_SPLIT_Y = ARENA_LENGTH * 0.5f;

// ---------------- SENSOR OFFSETS (mm) ----------------
// Add these to raw to get distance from robot center of rotation to wall.
const float OFFSET_S = 2.50f * 25.4f; // Port 0 (south)
const float OFFSET_W = 6.25f * 25.4f; // Port 1 (west)
const float OFFSET_N = 6.25f * 25.4f; // Port 2 (north)
const float OFFSET_E = 9.25f * 25.4f; // Port 3 (east)
const float OFFSETS[4] = {OFFSET_S, OFFSET_W, OFFSET_N, OFFSET_E};

// VL53L1X long mode max is ~4 m; keep a conservative validity bound
const float MIN_VALID_MM = 60.0f;      // anything smaller is usually junk
const float MAX_VALID_MM = 4200.0f;    // conservative cap (raw), before offset

// ---------------- HARDWARE PINOUT ----------------
constexpr uint8_t RIGHT_IN1 = 7, RIGHT_IN2 = 8, RIGHT_PWM = 6;
constexpr uint8_t LEFT_IN1  = 4, LEFT_IN2  = 5, LEFT_PWM  = 3;
constexpr uint8_t PCA9548A_ADDR = 0x70;
constexpr uint8_t IMU_CH = 4;

L298NMotor leftMotor(LEFT_IN1, LEFT_IN2, LEFT_PWM);
L298NMotor rightMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_PWM);
Adafruit_LSM6DSOX lsm6dsox;
VL53L1X sensors[4];

// ---------------- IMU HEADING ----------------
float currentHeadingDeg = 0.0f;     // integrated yaw (deg), arbitrary zero at boot
unsigned long lastUs = 0;
float gyroBiasZDeg = 0.0f;

// ---------------- MOTOR TUNING (IMPORTANT) ----------------
// Minimum PWM floors so motors actually move.
const int MIN_PWM_LEFT  = 120;
const int MIN_PWM_RIGHT = 160;

// Turn-in-place base powers (asymmetric to compensate mismatch).
const int BASE_TURN_LEFT  = 135;
const int BASE_TURN_RIGHT = 185;

// Controller gains/thresholds
const float TURN_KP = 2.2f;          // deg -> PWM scale (tune)
const float STOP_DEADBAND_DEG = 2.5f;
const uint16_t SETTLE_MS = 250;

// Scan speed (must be above minimum PWM)
const int SCAN_LEFT_PWM  = -135;
const int SCAN_RIGHT_PWM =  185;

// ---------------- MUX ----------------
bool selectMux(uint8_t ch) {
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << ch);
  return Wire.endTransmission() == 0;
}

// ---------------- ANGLES ----------------
static float wrapDeg180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// ---------------- IMU UPDATE ----------------
void updateIMU() {
  sensors_event_t accel, gyro, temp;
  selectMux(IMU_CH);
  lsm6dsox.getEvent(&accel, &gyro, &temp);

  unsigned long now = micros();
  float dt = (now - lastUs) / 1000000.0f;
  lastUs = now;

  float gzDeg = (gyro.gyro.z * 180.0f / PI) - gyroBiasZDeg;
  currentHeadingDeg += gzDeg * dt;
}

// ---------------- MOTORS ----------------
void setMotors(int left, int right) {
  auto lDir = left  >= 0 ? L298NMotor::Direction::Forward  : L298NMotor::Direction::Backward;
  auto rDir = right >= 0 ? L298NMotor::Direction::Forward : L298NMotor::Direction::Backward;
  leftMotor.drive(lDir,  abs(left));
  rightMotor.drive(rDir, abs(right));
}

void stopMotorsHard() {
  leftMotor.stop(true);
  rightMotor.stop(true);
}

// ---------------- ROBUST SENSOR READ ----------------
// Take multiple reads and median-filter them.
// Returns true if valid, and fills corrected distance in mm (raw + offset).
bool readSensorMedianCorrected(uint8_t port, float &outCorrected) {
  const int N = 5; // median of 5 is robust and still light
  uint16_t vals[N];
  int got = 0;

  for (int i = 0; i < N; i++) {
    selectMux(port);
    uint16_t r = sensors[port].read(false);

    // VL53L1X sometimes returns 0/1/8190-ish garbage depending on timeout/state
    if (r >= MIN_VALID_MM && r <= MAX_VALID_MM && !sensors[port].timeoutOccurred()) {
      vals[got++] = r;
    }
    delayMicroseconds(800); // small spacing; adjust if your bus is busy
  }

  if (got < 3) return false; // require at least 3 good samples

  // insertion sort small array
  for (int i = 1; i < got; i++) {
    uint16_t key = vals[i];
    int j = i - 1;
    while (j >= 0 && vals[j] > key) {
      vals[j + 1] = vals[j];
      j--;
    }
    vals[j + 1] = key;
  }

  uint16_t med = vals[got / 2];
  outCorrected = (float)med + OFFSETS[port];
  return true;
}

// Read all 4 sensors robustly. Returns true if enough sensors are valid.
bool readAllSensors(float d[4]) {
  bool ok[4];
  int good = 0;
  for (uint8_t i = 0; i < 4; i++) {
    ok[i] = readSensorMedianCorrected(i, d[i]);
    if (ok[i]) good++;
  }
  // We really want all four for strong alignment constraints, but allow 3.
  return (good >= 3);
}

// ---------------- LOCALIZATION RECORD ----------------
struct BestScan {
  float headingDeg = 0.0f;
  float d[4] = {0,0,0,0};     // corrected distances at best heading
  bool hypothesisA = true;    // true => (N+S~L and E+W~W), false => swapped
  float cost = 1e9f;
} best;

// Robust cost for a hypothesis
static float costFor(float sumNS, float sumEW, float expectNS, float expectEW) {
  // Use absolute errors; you can optionally Huberize for even more robustness.
  return fabsf(sumNS - expectNS) + fabsf(sumEW - expectEW);
}

// Choose best heading during scan
void considerScanSample(float headingDeg, const float d[4]) {
  // Need N,S and E,W sums ideally; if one is missing (0), degrade confidence.
  float sumNS = d[2] + d[0];
  float sumEW = d[3] + d[1];

  // Hypothesis A: axis-aligned
  float cA = costFor(sumNS, sumEW, ARENA_LENGTH, ARENA_WIDTH);
  // Hypothesis B: rotated 90
  float cB = costFor(sumNS, sumEW, ARENA_WIDTH, ARENA_LENGTH);

  bool hypA = (cA <= cB);
  float c = hypA ? cA : cB;

  // Basic plausibility: distances must be within arena-ish bounds
  // (corrected distances can exceed 4m if offset added, but still bounded by geometry)
  for (int i = 0; i < 4; i++) {
    if (d[i] <= 0.0f || d[i] > 6000.0f) return;
  }

  // Keep best
  if (c < best.cost) {
    best.cost = c;
    best.headingDeg = headingDeg;
    best.hypothesisA = hypA;
    for (int i = 0; i < 4; i++) best.d[i] = d[i];
  }
}

// ---------------- TURN-TO-HEADING ----------------
void turnToHeading(float targetDeg) {
  unsigned long stableStart = 0;

  while (true) {
    updateIMU();
    float err = wrapDeg180(targetDeg - currentHeadingDeg);

    if (fabsf(err) <= STOP_DEADBAND_DEG) {
      if (stableStart == 0) stableStart = millis();
      if (millis() - stableStart >= SETTLE_MS) break;
      setMotors(0, 0);
      delay(10);
      continue;
    } else {
      stableStart = 0;
    }

    // Proportional command with minimum PWM floors and asymmetry
    float mag = fabsf(err);
    int add = (int)(TURN_KP * mag);

    int leftCmd  = BASE_TURN_LEFT  + add;
    int rightCmd = BASE_TURN_RIGHT + add;

    // clamp
    leftCmd  = constrain(leftCmd,  MIN_PWM_LEFT,  255);
    rightCmd = constrain(rightCmd, MIN_PWM_RIGHT, 255);

    if (err > 0) {
      // need CCW (increase heading): left backward, right forward
      setMotors(-leftCmd, rightCmd);
    } else {
      // need CW: left forward, right backward
      setMotors(leftCmd, -rightCmd);
    }

    delay(10);
  }

  // braking pulse then stop
  setMotors(130, -130);
  delay(40);
  stopMotorsHard();
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  leftMotor.begin();
  rightMotor.begin();

  // IMU init
  selectMux(IMU_CH);
  if (!lsm6dsox.begin_I2C()) { while (1) {} }

  // Gyro bias calibration (keep robot still)
  sensors_event_t a, g, t;
  float sum = 0.0f;
  const int CAL_N = 500;
  for (int i = 0; i < CAL_N; i++) {
    lsm6dsox.getEvent(&a, &g, &t);
    sum += g.gyro.z;
    delay(2);
  }
  gyroBiasZDeg = (sum / (float)CAL_N) * 180.0f / PI;

  // VL53L1X init on mux ports 0..3
  for (uint8_t i = 0; i < 4; i++) {
    if (selectMux(i)) {
      sensors[i].setTimeout(60);
      if (sensors[i].init()) {
        sensors[i].setDistanceMode(VL53L1X::Long);
        sensors[i].startContinuous(33);
      }
    }
  }

  lastUs = micros();
  delay(100);
}

// ---------------- LOOP ----------------
void loop() {
  enum State { SCAN_360, ANALYZE, TURN_TO_PLUS_Y, FINISHED };
  static State state = SCAN_360;

  static float startHeading = 0.0f;
  static bool scanStarted = false;

  if (state == SCAN_360) {
    if (!scanStarted) {
      scanStarted = true;
      best = BestScan(); // reset best
      updateIMU();
      startHeading = currentHeadingDeg;
    }

    updateIMU();
    setMotors(SCAN_LEFT_PWM, SCAN_RIGHT_PWM);

    float d[4] = {0,0,0,0};
    if (readAllSensors(d)) {
      considerScanSample(currentHeadingDeg, d);
    }

    // done once we've rotated ~360 from start
    float rotated = fabsf(currentHeadingDeg - startHeading);
    if (rotated >= 360.0f) {
      // stop
      setMotors(130, -130);
      delay(40);
      stopMotorsHard();
      state = ANALYZE;
    }
  }

  else if (state == ANALYZE) {
    // Interpret best scan result.
    // d indices: 0:S, 1:W, 2:N, 3:E (relative to robot)

    float S = best.d[0], W = best.d[1], N = best.d[2], E = best.d[3];

    // Determine if best hypothesis is axis-aligned (A) or swapped (B).
    // If hypothesisA:
    //   N+S ~= LENGTH, E+W ~= WIDTH
    // If hypothesisB:
    //   N+S ~= WIDTH,  E+W ~= LENGTH
    bool hypA = best.hypothesisA;

    // There are 4 possible cardinal orientations (N sensor points +Y, -Y, +X, -X).
    // We use geometry + "we're in south half" assumption to pick the direction.

    // We'll compute two candidate y values for the long axis depending on whether
    // robot faces +Y or -Y when axis-aligned.
    float x = 0.0f, y = 0.0f;
    float targetHeading = best.headingDeg; // will adjust by +0/+90/+180/+270

    // Helper lambdas to pick solution consistent with y <= HALF_SPLIT_Y
    auto pickY = [&](float y1, float y2, bool &useFirst) {
      // Prefer the one that lands in the south half; if both do, choose closer to half.
      bool y1ok = (y1 >= 0.0f && y1 <= HALF_SPLIT_Y);
      bool y2ok = (y2 >= 0.0f && y2 <= HALF_SPLIT_Y);
      if (y1ok && !y2ok) { useFirst = true; return; }
      if (!y1ok && y2ok) { useFirst = false; return; }
      // fallback: choose smaller y (more "southern") to avoid driving outwards
      useFirst = (y1 <= y2);
    };

    // Case 1: Hypothesis A (robot aligned with arena axes)
    if (hypA) {
      // If N points +Y, then y = S and x = W
      float y_plusY = S;
      float x_plusY = W;

      // If N points -Y, then y = ARENA_LENGTH - N and x = ARENA_WIDTH - E? (depends on x-facing)
      // When axis-aligned, W measures distance to x=0 wall if robot isn't mirrored.
      // But if robot is rotated 180, W still points to x=0 wall (world west) and x=W still holds.
      // The ambiguity in x mirror is actually handled by E/W pair; simplest robust is:
      // x = W (distance to west wall), unless we detect swapped x via (E/W).
      // We'll compute x in the standard way and only flip if it violates bounds.
      float y_minusY = ARENA_LENGTH - N;
      float x_common = W;

      bool usePlusY = true;
      pickY(y_plusY, y_minusY, usePlusY);

      if (usePlusY) {
        y = y_plusY;
        x = x_plusY;
        // N already points +Y, so target heading = best.headingDeg
        targetHeading = best.headingDeg;
      } else {
        y = y_minusY;
        x = x_common;
        // Need rotate 180 to make N point +Y
        targetHeading = best.headingDeg + 180.0f;
      }

      // Validate x; if out of bounds, flip using E
      if (x < 0.0f || x > ARENA_WIDTH) {
        float xFlip = ARENA_WIDTH - E;
        if (xFlip >= 0.0f && xFlip <= ARENA_WIDTH) x = xFlip;
      }
    }

    // Case 2: Hypothesis B (robot is rotated 90° relative to arena axes)
    else {
      // If swapped, then N/S sum matches WIDTH => N is along ±X.
      // E/W sum matches LENGTH => E is along ±Y.

      // Two possibilities to face +Y:
      // - If E points +Y already, then y = W (since W is along -? actually W is robot-left; but in swapped case,
      //   E/W are the long axis pair, so we can set y = W or y = LENGTH - E depending on direction.)
      //
      // We'll compute y candidates from the long-axis pair (E/W):
      float y_plusY_fromE = W;                 // if E points +Y, robot's "west" sensor points -Y, so W is distance to south wall
      float y_minusY_fromE = ARENA_LENGTH - E; // if E points -Y, E measures distance to south wall, so y = E; equivalently y = LENGTH - W
      // also compute the alternative form
      float y_plusY_alt = ARENA_LENGTH - E;
      float y_minusY_alt = W;

      // We’ll pick using the half assumption on y.
      bool usePlusY = true;
      pickY(y_plusY_fromE, y_plusY_alt, usePlusY);
      // Note: both expressions for "plusY" may swap depending on wiring/handedness.
      // For robustness, we evaluate both and keep the one that is valid & in south half.

      float yCand1 = y_plusY_fromE; // interpret as "E points +Y"
      float yCand2 = ARENA_LENGTH - E; // interpret as "E points -Y" but then to face +Y you'd rotate 180

      bool interpretEasPlusY = true;
      pickY(yCand1, yCand2, interpretEasPlusY);

      if (interpretEasPlusY) {
        y = yCand1;
        // To make robot face +Y, rotate so that its N sensor (currently ±X) becomes +Y:
        // If we're in swapped hypothesis, arena +Y corresponds to robot +E direction.
        // So target = best.headingDeg + 90 (rotate CCW 90 makes N become W; rotate CW 90 makes N become E).
        // Here we assume at best.headingDeg, robot's +E is +Y; therefore robot yaw is -90 from +Y, so rotate +90.
        targetHeading = best.headingDeg + 90.0f;
      } else {
        y = yCand2;
        // If +E was -Y, then rotate -90 to make +E become +Y (or +270)
        targetHeading = best.headingDeg - 90.0f;
      }

      // x from the width pair (N/S) in swapped case:
      // If N points +X, then x = S; if N points -X, x = WIDTH - N
      float x1 = S;
      float x2 = ARENA_WIDTH - N;
      // Choose whichever is in bounds; prefer in-bounds
      if (x1 >= 0.0f && x1 <= ARENA_WIDTH) x = x1;
      else x = x2;
    }

    // Normalize targetHeading to be near currentHeading
    targetHeading = currentHeadingDeg + wrapDeg180(targetHeading - currentHeadingDeg);

    Serial.println(F("\n--- ROBUST LOCALIZATION + ORIENT TO +Y ---"));
    Serial.print(F("Best scan cost: ")); Serial.println(best.cost, 2);
    Serial.print(F("Best heading (deg): ")); Serial.println(best.headingDeg, 2);
    Serial.print(F("Hypothesis: ")); Serial.println(best.hypothesisA ? F("Axis-aligned") : F("Swapped (90 deg)"));
    Serial.print(F("Estimated X (in): ")); Serial.println(x / 25.4f, 2);
    Serial.print(F("Estimated Y (in): ")); Serial.println(y / 25.4f, 2);
    Serial.print(F("Turn target heading (deg): ")); Serial.println(targetHeading, 2);
    Serial.println(F("-----------------------------------------\n"));

    // Store target in a static for the next state
    static float gTarget = 0.0f;
    gTarget = targetHeading;

    state = TURN_TO_PLUS_Y;
  }

  else if (state == TURN_TO_PLUS_Y) {
    // Retrieve target from the static inside ANALYZE
    extern float __attribute__((weak)) gTarget; // not actually used; avoid linker tricks
    // Instead: recompute from best (simple) as a fallback:
    // (In practice on Arduino, keep gTarget as a global; here we just do it directly.)
    // We'll re-derive quickly:
    float targetHeading = best.headingDeg;
    if (best.hypothesisA) {
      // Axis-aligned: decide +Y vs -Y using y half assumption
      float y_plus = best.d[0];             // y = S if facing +Y
      float y_minus = ARENA_LENGTH - best.d[2]; // y = L - N if facing -Y
      bool usePlus = (y_plus >= 0 && y_plus <= HALF_SPLIT_Y) || !(y_minus >= 0 && y_minus <= HALF_SPLIT_Y);
      targetHeading = best.headingDeg + (usePlus ? 0.0f : 180.0f);
    } else {
      // Swapped: rotate ±90 to make forward +Y (see ANALYZE for reasoning)
      // Use y consistency:
      float y1 = best.d[1];           // W
      float y2 = ARENA_LENGTH - best.d[3]; // L - E
      bool usePlus = (y1 >= 0 && y1 <= HALF_SPLIT_Y) || !(y2 >= 0 && y2 <= HALF_SPLIT_Y);
      targetHeading = best.headingDeg + (usePlus ? 90.0f : -90.0f);
    }
    targetHeading = currentHeadingDeg + wrapDeg180(targetHeading - currentHeadingDeg);

    turnToHeading(targetHeading);
    state = FINISHED;
  }

  else { // FINISHED
    stopMotorsHard();
    delay(500);
  }
}