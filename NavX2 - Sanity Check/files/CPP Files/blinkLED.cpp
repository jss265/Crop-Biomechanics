#include <Arduino.h>
#define LED A7

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  delay(2000);
  Serial.println("Done");
}

void loop() {
  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);
  delay(500);
  Serial.println("still looping");
}