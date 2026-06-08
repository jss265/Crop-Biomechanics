#include <Arduino.h>
#include <SPI.h>

// =====================================================
// Pins
// =====================================================

constexpr uint8_t IMU_CS  = A0;
constexpr uint8_t MAG_CS  = A1;

constexpr uint8_t IMU_INT = A2;
constexpr uint8_t MAG_INT = A3;

// =====================================================
// SPI
// =====================================================

SPISettings sensorSPI(
    1000000,
    MSBFIRST,
    SPI_MODE3
);

// =====================================================
// SPI Helpers
// =====================================================

uint8_t readRegister(uint8_t reg)
{
    SPI.beginTransaction(sensorSPI);

    digitalWrite(IMU_CS, LOW);

    SPI.transfer(reg | 0x80);
    uint8_t value = SPI.transfer(0x00);

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

// =====================================================
// Read IMU Burst
// =====================================================

int16_t gxRaw, gyRaw, gzRaw;
int16_t axRaw, ayRaw, azRaw;

void readIMU()
{
    uint8_t buf[12];

    SPI.beginTransaction(sensorSPI);

    digitalWrite(IMU_CS, LOW);

    SPI.transfer(0x22 | 0x80 | 0x40);

    for (int i = 0; i < 12; i++)
    {
        buf[i] = SPI.transfer(0);
    }

    digitalWrite(IMU_CS, HIGH);

    SPI.endTransaction();

    gxRaw = (int16_t)((buf[1] << 8) | buf[0]);
    gyRaw = (int16_t)((buf[3] << 8) | buf[2]);
    gzRaw = (int16_t)((buf[5] << 8) | buf[4]);

    axRaw = (int16_t)((buf[7] << 8) | buf[6]);
    ayRaw = (int16_t)((buf[9] << 8) | buf[8]);
    azRaw = (int16_t)((buf[11] << 8) | buf[10]);
}

// =====================================================
// Setup
// =====================================================

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

    pinMode(IMU_INT, INPUT);

    SPI.begin();

    delay(50);

    Serial.println();
    Serial.println("=== ISM330DHCX DIAGNOSTIC ===");

    uint8_t whoami = readRegister(0x0F);

    Serial.print("WHO_AM_I   = 0x");
    Serial.println(whoami, HEX);

    // CTRL1_XL
    writeRegister(0x10, 0x4A);

    // CTRL2_G
    writeRegister(0x11, 0x40);

    // INT1_CTRL
    writeRegister(0x0D, 0x01);

    delay(10);

    Serial.print("CTRL1_XL   = 0x");
    Serial.println(readRegister(0x10), HEX);

    Serial.print("CTRL2_G    = 0x");
    Serial.println(readRegister(0x11), HEX);

    Serial.print("INT1_CTRL  = 0x");
    Serial.println(readRegister(0x0D), HEX);

    Serial.println();
}

// =====================================================
// Loop
// =====================================================

void loop()
{
    static uint32_t lastPrint = 0;

    // Continuously read data
    readIMU();

    if (millis() - lastPrint >= 500)
    {
        lastPrint = millis();

        uint8_t status = readRegister(0x1E);

        Serial.print("INT1=");
        Serial.print(digitalRead(IMU_INT));

        Serial.print(" STATUS=0x");
        Serial.print(status, HEX);

        Serial.print(" AX=");
        Serial.print(axRaw);

        Serial.print(" AY=");
        Serial.print(ayRaw);

        Serial.print(" AZ=");
        Serial.print(azRaw);

        Serial.print(" GX=");
        Serial.print(gxRaw);

        Serial.print(" GY=");
        Serial.print(gyRaw);

        Serial.print(" GZ=");
        Serial.println(gzRaw);
    }
}