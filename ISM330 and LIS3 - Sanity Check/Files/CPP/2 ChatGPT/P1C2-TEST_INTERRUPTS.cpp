#include <Arduino.h>

constexpr uint8_t IMU_INT = A2;

volatile uint32_t count = 0;

void IRAM_ATTR imuISR()
{
    count++;
}

void setup()
{
    Serial.begin(115200);

    pinMode(IMU_INT, INPUT);

    attachInterrupt(
        digitalPinToInterrupt(IMU_INT),
        imuISR,
        CHANGE
    );
}

void loop()
{
    static uint32_t last = 0;

    if (millis() - last > 1000)
    {
        last = millis();

        Serial.println(count);
    }
}