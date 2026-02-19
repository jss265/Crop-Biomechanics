#include <Arduino.h>
#define LED A7

void setup() {
  Serial.begin(921600);
  pinMode(LED, OUTPUT);
}

void loop() {
  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);
  delay(500);
  Serial.println("still looping");
}