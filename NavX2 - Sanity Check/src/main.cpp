#include <Arduino.h>
#include <Wire.h>
#define LED A7

#define NAVX_ADDR 0x41  // default I2C address

void blinkLED();

void setup() {
    Wire.begin(A5, A4);            // SDA=21, SCL=22 by default
    Wire.setClock(400000);   // optional: faster I2C (400 kHz)
    Serial.begin(921600);    // match your monitor_speed
    pinMode(LED, OUTPUT);
}

void loop() {
  Wire.beginTransmission(NAVX_ADDR);
  Wire.write(0x00);        // register/command you want to read
  Wire.endTransmission();

  Wire.requestFrom(NAVX_ADDR, 12); // example: 12 bytes from NavX2
  while (Wire.available()) {
      byte b = Wire.read();
      Serial.print(b, HEX);
      Serial.print(" ");
  }
  Serial.println();

  blinkLED();

  delay(5); // ~200 Hz read rate
}

void blinkLED() {
  static bool LEDstate = LOW;
  static unsigned long prevMillis = millis();
  if (millis() - prevMillis > 500) {
    digitalWrite(LED, LEDstate);    
    prevMillis = millis();
    LEDstate = ! LEDstate;
  }
}
