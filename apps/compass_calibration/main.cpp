/*
 * QMC5883P Calibration Sketch
 *
 * This program helps calibrate the QMC5883P 3-axis magnetometer for hard iron distortion.
 * 
 * Instructions:
 * 1. Upload this program to your robot
 * 2. Open the Serial Monitor
 * 3. Slowly rotate the robot in a complete 360-degree circle (or multiple circles)
 *    - Try to rotate smoothly and cover all angles
 * 4. Watch the min/max values update
 * 5. After completing several rotations, note the final X_OFFSET and Y_OFFSET values
 * 6. Copy these values to include/CompassCalibration.h for use in other programs
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_QMC5883P.h>

#define PCA9548A_ADDRESS 0x70
#define COMPASS_MUX_PORT 7

Adafruit_QMC5883P qmc;

// Calibration tracking variables
int16_t x_min = 32767;
int16_t x_max = -32768;
int16_t y_min = 32767;
int16_t y_max = -32768;

unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 500; // Print every 500ms

// Function to select a channel on the PCA9548A multiplexer
void selectMuxChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(PCA9548A_ADDRESS);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("========================================");
  Serial.println("QMC5883P Compass Calibration");
  Serial.println("========================================");
  Serial.println();
  Serial.println("INSTRUCTIONS:");
  Serial.println("1. Slowly rotate the robot 360 degrees");
  Serial.println("2. Complete multiple full rotations");
  Serial.println("3. Watch min/max values stabilize");
  Serial.println("4. Copy final offset values when done");
  Serial.println();
  Serial.println("========================================");
  Serial.println();

  // Initialize I2C
  Wire.begin();
  
  // Select compass channel on multiplexer
  selectMuxChannel(COMPASS_MUX_PORT);
  delay(10); // Allow time for channel to settle

  if (!qmc.begin()) {
    Serial.println("Failed to find QMC5883P chip");
    while (1)
      delay(10);
  }

  Serial.println("QMC5883P Found!");

  // Configure compass for optimal calibration
  qmc.setMode(QMC5883P_MODE_NORMAL);
  qmc.setODR(QMC5883P_ODR_50HZ);        // 50Hz sampling rate
  qmc.setOSR(QMC5883P_OSR_4);            // Over sample ratio 4
  qmc.setDSR(QMC5883P_DSR_2);            // Downsample ratio 2
  qmc.setRange(QMC5883P_RANGE_8G);       // ±8G range
  qmc.setSetResetMode(QMC5883P_SETRESET_ON);

  Serial.println("Compass configured. Starting calibration...");
  Serial.println();
  delay(1000);
}

void loop() {
  // Ensure compass channel is selected on multiplexer
  selectMuxChannel(COMPASS_MUX_PORT);

  if (qmc.isDataReady()) {
    int16_t x, y, z;

    if (qmc.getRawMagnetic(&x, &y, &z)) {
      // Update min/max values
      if (x < x_min) x_min = x;
      if (x > x_max) x_max = x;
      if (y < y_min) y_min = y;
      if (y > y_max) y_max = y;

      // Print status periodically
      if (millis() - lastPrintTime >= PRINT_INTERVAL) {
        // Calculate offsets
        int16_t x_offset = (x_max + x_min) / 2;
        int16_t y_offset = (y_max + y_min) / 2;

        Serial.println("----------------------------------------");
        Serial.print("Current Reading - X: ");
        Serial.print(x);
        Serial.print("  Y: ");
        Serial.print(y);
        Serial.print("  Z: ");
        Serial.println(z);
        
        Serial.println();
        Serial.print("X Range: [");
        Serial.print(x_min);
        Serial.print(" to ");
        Serial.print(x_max);
        Serial.print("]  Span: ");
        Serial.println(x_max - x_min);
        
        Serial.print("Y Range: [");
        Serial.print(y_min);
        Serial.print(" to ");
        Serial.print(y_max);
        Serial.print("]  Span: ");
        Serial.println(y_max - y_min);
        
        Serial.println();
        Serial.println(">>> CALIBRATION OFFSETS <<<");
        Serial.print(">>> X_OFFSET = ");
        Serial.println(x_offset);
        Serial.print(">>> Y_OFFSET = ");
        Serial.println(y_offset);
        Serial.println();

        if (qmc.isOverflow()) {
          Serial.println("WARNING: Data overflow detected!");
          Serial.println("Consider using a larger range setting.");
          Serial.println();
        }

        lastPrintTime = millis();
      }
    }
  }

  delay(20); // Small delay for stability
}
