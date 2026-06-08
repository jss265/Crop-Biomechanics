#include <Arduino.h>

constexpr uint8_t IMU_INT = A2;
constexpr uint8_t MAG_INT = A3;

void setup()
{
    Serial.begin(115200);

    pinMode(IMU_INT, INPUT);
    pinMode(MAG_INT, INPUT);
}

void loop()
{
    static uint32_t lastPrint = 0;

    if (millis() - lastPrint >= 500)
    {
        lastPrint = millis();

        Serial.print("INT1=");
        Serial.print(digitalRead(IMU_INT));

        Serial.print("  DRDY=");
        Serial.println(digitalRead(MAG_INT));
    }
}