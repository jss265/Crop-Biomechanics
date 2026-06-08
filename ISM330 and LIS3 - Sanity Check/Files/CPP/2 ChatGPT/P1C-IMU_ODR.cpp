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
    10000000,     // 10 MHz
    MSBFIRST,
    SPI_MODE3
);

// =====================================================
// Globals
// =====================================================

volatile bool imuReady = false;

volatile uint32_t imuInterruptCount = 0;

float ax, ay, az;
float gx, gy, gz;

uint32_t lastPrint = 0;
uint32_t lastCount = 0;

// =====================================================
// ISR
// =====================================================

void IRAM_ATTR imuISR()
{
    imuReady = true;
    imuInterruptCount++;
}

// =====================================================
// SPI Helpers
// =====================================================

uint8_t readRegister(uint8_t csPin, uint8_t reg)
{
    SPI.beginTransaction(sensorSPI);

    digitalWrite(csPin, LOW);

    SPI.transfer(reg | 0x80);

    uint8_t value = SPI.transfer(0x00);

    digitalWrite(csPin, HIGH);

    SPI.endTransaction();

    return value;
}

void writeRegister(uint8_t csPin, uint8_t reg, uint8_t value)
{
    SPI.beginTransaction(sensorSPI);

    digitalWrite(csPin, LOW);

    SPI.transfer(reg & 0x7F);

    SPI.transfer(value);

    digitalWrite(csPin, HIGH);

    SPI.endTransaction();
}

// =====================================================
// Read IMU
// =====================================================

void readIMU()
{
    uint8_t buf[12];

    SPI.beginTransaction(sensorSPI);

    digitalWrite(IMU_CS, LOW);

    // OUTX_L_G = 0x22
    // bit7 = read
    // bit6 = auto increment

    SPI.transfer(0x22 | 0x80 | 0x40);

    for (int i = 0; i < 12; i++)
    {
        buf[i] = SPI.transfer(0x00);
    }

    digitalWrite(IMU_CS, HIGH);

    SPI.endTransaction();

    int16_t gxRaw = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t gyRaw = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t gzRaw = (int16_t)((buf[5] << 8) | buf[4]);

    int16_t axRaw = (int16_t)((buf[7] << 8) | buf[6]);
    int16_t ayRaw = (int16_t)((buf[9] << 8) | buf[8]);
    int16_t azRaw = (int16_t)((buf[11] << 8) | buf[10]);

    // ±250 dps
    gx = gxRaw * 0.00875f;
    gy = gyRaw * 0.00875f;
    gz = gzRaw * 0.00875f;

    // ±4 g
    ax = axRaw * 0.122f / 1000.0f;
    ay = ayRaw * 0.122f / 1000.0f;
    az = azRaw * 0.122f / 1000.0f;
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
    Serial.println("=== IMU ODR TEST ===");

    uint8_t whoami = readRegister(IMU_CS, 0x0F);

    Serial.print("WHO_AM_I = 0x");
    Serial.println(whoami, HEX);

    // CTRL1_XL
    // ODR = 104 Hz
    // FS = ±4 g

    writeRegister(IMU_CS, 0x10, 0x4A);

    // CTRL2_G
    // ODR = 104 Hz
    // FS = ±250 dps

    writeRegister(IMU_CS, 0x11, 0x40);

    // INT1_CTRL
    // accel DRDY on INT1

    writeRegister(IMU_CS, 0x0D, 0x01);

    attachInterrupt(
        digitalPinToInterrupt(IMU_INT),
        imuISR,
        RISING
    );

    Serial.println("Interrupt attached.");
}

// =====================================================
// Loop
// =====================================================

void loop()
{
    if (imuReady)
    {
        imuReady = false;

        readIMU();
    }

    if (millis() - lastPrint >= 2500)
    {
        uint32_t currentCount = imuInterruptCount;

        float odr =
            (currentCount - lastCount) / 2.5f;

        Serial.print("IMU ODR: ");
        Serial.print(odr, 1);
        Serial.print(" Hz");

        Serial.print(" | AX=");
        Serial.print(ax, 3);

        Serial.print(" AY=");
        Serial.print(ay, 3);

        Serial.print(" AZ=");
        Serial.print(az, 3);

        Serial.println();

        lastCount = currentCount;
        lastPrint = millis();
    }
}