#include <Arduino.h>
#include <SPI.h>

constexpr uint8_t IMU_CS = A0;

SPISettings sensorSPI(
    1000000,
    MSBFIRST,
    SPI_MODE3
);

uint8_t readRegister(uint8_t reg)
{
    SPI.beginTransaction(sensorSPI);

    digitalWrite(IMU_CS, LOW);

    SPI.transfer(reg | 0x80);
    uint8_t value = SPI.transfer(0);

    digitalWrite(IMU_CS, HIGH);

    SPI.endTransaction();

    return value;
}

void writeRegister(uint8_t reg, uint8_t value)
{
    SPI.beginTransaction(sensorSPI);

    digitalWrite(IMU_CS, LOW);

    SPI.transfer(reg & 0x7F);
    SPI.transfer(value);

    digitalWrite(IMU_CS, HIGH);

    SPI.endTransaction();
}

void setup()
{
    Serial.begin(115200);

    pinMode(IMU_CS, OUTPUT);
    digitalWrite(IMU_CS, HIGH);

    SPI.begin();

    delay(50);

    Serial.println("Configuring IMU");

    // CTRL1_XL
    writeRegister(0x10, 0x4A);

    // CTRL2_G
    writeRegister(0x11, 0x40);
}

void loop()
{
    static uint32_t last = 0;

    if (millis() - last > 500)
    {
        last = millis();

        uint8_t status = readRegister(0x1E);

        Serial.print("STATUS_REG = 0x");
        Serial.println(status, HEX);
    }
}