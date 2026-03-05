#include <Arduino.h>
const int IR_PIN = 12;

const int analogPins[6] = {A0, A1, A2, A3, A4, A5};

const int BAR_WIDTH = 32;   // width of ASCII bar

void drawBar(int value) {
  int filled = map(value, 0, 1023, 0, BAR_WIDTH);

  Serial.print("|");
  for (int i = 0; i < BAR_WIDTH; i++) {
    if (i < filled) Serial.print("#");
    else Serial.print(" ");
  }
  Serial.print("| ");
}

void setup() {
  Serial.begin(115200);

  pinMode(IR_PIN, OUTPUT);
  digitalWrite(IR_PIN, HIGH);   // turn on IR emitters

  Serial.println("Line sensor analog monitor");
  Serial.println("---------------------------------------------");
}

void loop() {

  for (int i = 0; i < 6; i++) {
    int value = analogRead(analogPins[i]);

    Serial.print("A");
    Serial.print(i + 1);
    Serial.print(" ");

    drawBar(value);

    Serial.println(value);
  }

  Serial.println();
  delay(150);
}