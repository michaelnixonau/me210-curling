/*
 * IR Beacon Detector - 909 Hz
 * Target: Arduino Uno/Nano/Mega
 * Pin: IR Receiver (Analog or Digital) connected to A0
 */
#include <Arduino.h>
#include <math.h>

const int SENSOR_PIN = A0;
const float TARGET_FREQ = 909.0;
const int SAMPLES = 100; // Number of samples per check
const float SAMPLING_FREQ = 5000.0; // Hz (Must be > 2x Target Freq)

// Thresholds - adjust based on your specific sensor and distance
const float LOCK_THRESHOLD = 500.0; 

void setup() {
  Serial.begin(115200);
  pinMode(SENSOR_PIN, INPUT);
}

float detectFrequency(int pin, float targetFreq, int numSamples) {
  float coeff = 2.0 * cos(2.0 * PI * targetFreq / SAMPLING_FREQ);
  float q0 = 0, q1 = 0, q2 = 0;

  unsigned long microStep = 1000000 / SAMPLING_FREQ;

  for (int i = 0; i < numSamples; i++) {
    unsigned long startTime = micros();
    
    // Read sensor and center the value around zero
    float sample = analogRead(pin) - 512; 
    
    q0 = coeff * q1 - q2 + sample;
    q2 = q1;
    q1 = q0;

    // Maintain strict timing for accurate frequency detection
    while (micros() - startTime < microStep); 
  }

  // Calculate magnitude
  float magnitude = sqrt(q1 * q1 + q2 * q2 - q1 * q2 * coeff);
  return magnitude;
}

void loop() {
  float magnitude = detectFrequency(SENSOR_PIN, TARGET_FREQ, SAMPLES);

  // Serial.print("Signal Strength: ");
  Serial.println(magnitude);

  // if (magnitude > LOCK_THRESHOLD) {
  //   Serial.println(" -> [ LOCKED ON ]");
  //   // Add code here to light a "Lock" LED
  // } else if (magnitude > (LOCK_THRESHOLD * 0.2)) {
  //   Serial.println(" -> Signal Detected (Searching...)");
  // } else {
  //   Serial.println(" -> No Signal");
  // }

  delay(50); // Small stability delay
}