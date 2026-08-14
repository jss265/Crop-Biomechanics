#include <Arduino.h>

#define LED_PIN RX


void setup(){
    pinMode(LED_PIN, OUTPUT);
}

void loop(){
    delay(1000);
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
}
