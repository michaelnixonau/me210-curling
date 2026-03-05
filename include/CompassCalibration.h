/*
 * CompassCalibration.h
 *
 * Shared calibration constants for the QMC5883P magnetometer.
 * 
 * To generate these values:
 * 1. Run the compass_calibration app
 * 2. Rotate the robot through multiple complete 360-degree circles
 * 3. Wait for the min/max values to stabilize
 * 4. Copy the final X_OFFSET and Y_OFFSET values here
 * 
 * These offsets compensate for hard iron distortion (constant magnetic fields
 * from nearby ferrous materials or permanent magnets on the robot).
 */

#ifndef COMPASS_CALIBRATION_H
#define COMPASS_CALIBRATION_H

// Calibration offsets (update these after running compass_calibration)
// Default values are 0 (no calibration)
#define COMPASS_X_OFFSET -22
#define COMPASS_Y_OFFSET 122

// Helper function to apply calibration to raw magnetometer readings
inline void applyCompassCalibration(int16_t &x, int16_t &y) {
  x -= COMPASS_X_OFFSET;
  y -= COMPASS_Y_OFFSET;
}

#endif // COMPASS_CALIBRATION_H
