#include <Arduino.h>
#include <SPI.h>

// =====================================================
// Pin Definitions
// =====================================================

constexpr uint8_t IMU_CS  = A0;
constexpr uint8_t MAG_CS  = A1;

constexpr uint8_t IMU_INT = A2;
constexpr uint8_t MAG_INT = A3;

// =====================================================
// SPI Settings
// =====================================================

SPISettings sensorSPI(
    1000000,      // 1 MHz
    MSBFIRST,
    SPI_MODE3
);

// =====================================================
// SPI Helpers
// =====================================================

uint8_t readRegister(uint8_t csPin, uint8_t reg)
{
    SPI.beginTransaction(sensorSPI);

    digitalWrite(csPin, LOW);

    SPI.transfer(reg | 0x80); // read

    uint8_t value = SPI.transfer(0x00);

    digitalWrite(csPin, HIGH);

    SPI.endTransaction();

    return value;
}

void writeRegister(uint8_t csPin, uint8_t reg, uint8_t value)
{
    SPI.beginTransaction(sensorSPI);

    digitalWrite(csPin, LOW);

    SPI.transfer(reg & 0x7F); // write

    SPI.transfer(value);

    digitalWrite(csPin, HIGH);

    SPI.endTransaction();
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
    pinMode(MAG_INT, INPUT);

    SPI.begin();

    delay(50);

    Serial.println();
    Serial.println("=== ISM330DHCX INT1 TEST ===");

    // Verify communication

    uint8_t whoami = readRegister(IMU_CS, 0x0F);

    Serial.print("WHO_AM_I = 0x");
    Serial.println(whoami, HEX);

    // -------------------------------------------------
    // Configure Accelerometer
    //
    // CTRL1_XL (0x10)
    //
    // ODR = 104 Hz
    // FS  = ±4 g
    // -------------------------------------------------

    writeRegister(IMU_CS, 0x10, 0x4A);

    // -------------------------------------------------
    // Configure Gyroscope
    //
    // CTRL2_G (0x11)
    //
    // ODR = 104 Hz
    // FS  = 250 dps
    // -------------------------------------------------

    writeRegister(IMU_CS, 0x11, 0x40);

    // -------------------------------------------------
    // Route Accelerometer Data Ready to INT1
    //
    // INT1_CTRL (0x0D)
    //
    // INT1_DRDY_XL = 1
    // -------------------------------------------------

    writeRegister(IMU_CS, 0x0D, 0x01);

    Serial.println("IMU configured.");
    Serial.println();
}

// =====================================================
// Loop
// =====================================================

void loop()
{
    static uint32_t lastPrint = 0;

    if (millis() - lastPrint >= 250)
    {
        lastPrint = millis();

        Serial.print("INT1=");
        Serial.print(digitalRead(IMU_INT));

        Serial.print("   DRDY=");
        Serial.println(digitalRead(MAG_INT));
    }
}