#include <Arduino.h>

const int LED_PIN = 5;

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  Serial.println("Hello World! from practice-1");

  // put your main code here, to run repeatedly:
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
}