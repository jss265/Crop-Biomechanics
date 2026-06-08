#include <Arduino.h>
#include <SPI.h>

// -------------------------------
// Pin Definitions
// -------------------------------

constexpr uint8_t IMU_CS = A0;
constexpr uint8_t MAG_CS = A1;

constexpr uint8_t IMU_INT = A2;
constexpr uint8_t MAG_INT = A3;

// -------------------------------
// Registers
// -------------------------------

constexpr uint8_t WHO_AM_I_REG = 0x0F;

// Expected IDs
constexpr uint8_t ISM330DHCX_ID = 0x6B;
constexpr uint8_t LIS3MDL_ID    = 0x3D;

// -------------------------------
// SPI Settings
// -------------------------------

SPISettings sensorSPI(
    1000000,      // 1 MHz (safe startup speed)
    MSBFIRST,
    SPI_MODE3
);

// -------------------------------
// Timing
// -------------------------------

uint32_t lastPrint = 0;

// -------------------------------
// SPI Read Helper
// -------------------------------

uint8_t readRegister(uint8_t csPin, uint8_t reg)
{
    digitalWrite(csPin, LOW);

    SPI.beginTransaction(sensorSPI);

    // SPI read bit = bit7 set
    SPI.transfer(reg | 0x80);

    uint8_t value = SPI.transfer(0x00);

    SPI.endTransaction();

    digitalWrite(csPin, HIGH);

    return value;
}

// -------------------------------
// Setup
// -------------------------------

void setup()
{
    Serial.begin(115200);

    while (!Serial)
    {
        delay(10);
    }

    pinMode(IMU_CS, OUTPUT);
    pinMode(MAG_CS, OUTPUT);

    digitalWrite(IMU_CS, HIGH);
    digitalWrite(MAG_CS, HIGH);

    SPI.begin();

    Serial.println();
    Serial.println("=== SPI Sensor Detection ===");
}

// -------------------------------
// Loop
// -------------------------------

void loop()
{
    if (millis() - lastPrint >= 2500)
    {
        lastPrint = millis();

        uint8_t imuID = readRegister(IMU_CS, WHO_AM_I_REG);
        uint8_t magID = readRegister(MAG_CS, WHO_AM_I_REG);

        Serial.print("ISM330DHCX WHO_AM_I = 0x");
        Serial.print(imuID, HEX);

        if (imuID == ISM330DHCX_ID)
        {
            Serial.print("  [PASS]");
        }
        else
        {
            Serial.print("  [FAIL]");
        }

        Serial.print("    |    ");

        Serial.print("LIS3MDL WHO_AM_I = 0x");
        Serial.print(magID, HEX);

        if (magID == LIS3MDL_ID)
        {
            Serial.println("  [PASS]");
        }
        else
        {
            Serial.println("  [FAIL]");
        }
    }
}