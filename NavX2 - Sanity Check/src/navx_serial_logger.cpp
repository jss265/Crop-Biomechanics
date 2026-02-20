#include <Arduino.h>
#include <Wire.h>
#include "IMURegisters.h"

// ----- navX I2C Config -----
#define NAVX_I2C_ADDR  0x32

// ----- Unit Conversion Constants -----
const float DEG2RAD   = 3.14159265358979f / 180.0f;
const float G_TO_MPS2 = 9.80665f;
const float DEV_UNITS_MAX = 32768.0f;

// ----- Sensor Scale Factors (read from navX at startup) -----
uint8_t  accelFsrG   = 2;     // default: ±2 G
uint16_t gyroFsrDps  = 2000;  // default: ±2000 deg/s

// ─── I2C Helpers ──────────────────────────────────────────────────────────────

// Read a block of registers from the navX over I2C.
// Returns the number of bytes actually read (0 on failure).
uint8_t readNavXRegisters(uint8_t reg, uint8_t* buf, uint8_t count) {
    // Tell the navX which register and how many bytes
    Wire.beginTransmission(NAVX_I2C_ADDR);
    Wire.write(reg);
    Wire.write(count);
    Wire.endTransmission();

    // Request the data
    Wire.requestFrom(NAVX_I2C_ADDR, (int)count);
    delay(1);

    uint8_t i = 0;
    while (Wire.available() && i < count) {
        buf[i++] = Wire.read();
    }
    return i;
}

// ─── setup() ──────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(921600);
    Wire.begin();
    delay(2000);
    Serial.println("beginning...");

    // --- Read board config registers once ---
    // ACCEL_FSR_G (1 byte at 0x05), GYRO_FSR_DPS (2 bytes at 0x06-0x07)
    uint8_t cfg[3];
    if (readNavXRegisters(NAVX_REG_ACCEL_FSR_G, cfg, 3) == 3) {
        accelFsrG  = cfg[0];
        gyroFsrDps = IMURegisters::decodeProtocolUint16((char*)&cfg[1]);
        Serial.printf("navX config — accelFSR: %d G, gyroFSR: %d DPS\n", accelFsrG, gyroFsrDps);
    } else {
        Serial.println("WARNING: could not read navX config, using defaults");
    }

    // --- Wait for navX to be operational ---
    uint8_t opStatus = 0;
    unsigned long start = millis();
    while (opStatus != NAVX_OP_STATUS_NORMAL && (millis() - start) < 10000) {
        readNavXRegisters(NAVX_REG_OP_STATUS, &opStatus, 1);
        delay(100);
    }
    if (opStatus == NAVX_OP_STATUS_NORMAL) {
        Serial.println("navX operational.");
    } else {
        Serial.printf("WARNING: navX op_status=%d after timeout\n", opStatus);
    }
}

// ─── loop() ───────────────────────────────────────────────────────────────────

// We read a contiguous block from TIMESTAMP (0x12) through ACC_Z (0x3F).
// That's 0x3F - 0x12 + 1 = 46 bytes, giving us all the data we need.
#define READ_START   NAVX_REG_TIMESTAMP_L_L  // 0x12
#define READ_END     NAVX_REG_ACC_Z_H        // 0x3F
#define READ_LEN     (READ_END - READ_START + 1)  // 46 bytes

// Offsets within the data buffer (relative to READ_START = 0x12)
#define OFF_TIMESTAMP (NAVX_REG_TIMESTAMP_L_L - READ_START)  //  0
#define OFF_YAW       (NAVX_REG_YAW_L         - READ_START)  //  4
#define OFF_ROLL      (NAVX_REG_ROLL_L         - READ_START)  //  6
#define OFF_PITCH     (NAVX_REG_PITCH_L        - READ_START)  //  8
#define OFF_GYRO_X    (NAVX_REG_GYRO_X_L       - READ_START)  // 34
#define OFF_GYRO_Y    (NAVX_REG_GYRO_Y_L       - READ_START)  // 36
#define OFF_GYRO_Z    (NAVX_REG_GYRO_Z_L       - READ_START)  // 38
#define OFF_ACC_X     (NAVX_REG_ACC_X_L        - READ_START)  // 40
#define OFF_ACC_Y     (NAVX_REG_ACC_Y_L        - READ_START)  // 42
#define OFF_ACC_Z     (NAVX_REG_ACC_Z_L        - READ_START)  // 44

void loop() {
    // --- Read 46-byte register block in one I2C transaction ---
    uint8_t data[READ_LEN];
    if (readNavXRegisters(READ_START, data, READ_LEN) != READ_LEN) {
        return;  // I2C read failed; try again next loop
    }

    // --- Dedup: skip if timestamp hasn't changed ---
    static uint32_t lastTimestamp = 0;
    uint32_t rawTimestamp = IMURegisters::decodeProtocolUint32((char*)&data[OFF_TIMESTAMP]);
    if (rawTimestamp == lastTimestamp) {
        return;  // no new sample yet
    }
    lastTimestamp = rawTimestamp;

    // --- Decode ---
    float timestamp = rawTimestamp / 1000.0f;  // ms → seconds

    // Yaw, Roll, Pitch: signed hundredths → degrees
    float yaw   = IMURegisters::decodeProtocolSignedHundredthsFloat((char*)&data[OFF_YAW]);
    float pitch = IMURegisters::decodeProtocolSignedHundredthsFloat((char*)&data[OFF_PITCH]);
    float roll  = IMURegisters::decodeProtocolSignedHundredthsFloat((char*)&data[OFF_ROLL]);

    // Raw Gyro: device units → deg/s → rad/s
    float gyroScale = (float)gyroFsrDps / DEV_UNITS_MAX;  // device units → deg/s
    float gyroX = IMURegisters::decodeProtocolInt16((char*)&data[OFF_GYRO_X]) * gyroScale * DEG2RAD;
    float gyroY = IMURegisters::decodeProtocolInt16((char*)&data[OFF_GYRO_Y]) * gyroScale * DEG2RAD;
    float gyroZ = IMURegisters::decodeProtocolInt16((char*)&data[OFF_GYRO_Z]) * gyroScale * DEG2RAD;

    // Raw Accel: device units → G → m/s²
    float accelScale = (float)accelFsrG / DEV_UNITS_MAX;  // device units → G
    float accelX = IMURegisters::decodeProtocolInt16((char*)&data[OFF_ACC_X]) * accelScale * G_TO_MPS2;
    float accelY = IMURegisters::decodeProtocolInt16((char*)&data[OFF_ACC_Y]) * accelScale * G_TO_MPS2;
    float accelZ = IMURegisters::decodeProtocolInt16((char*)&data[OFF_ACC_Z]) * accelScale * G_TO_MPS2;

    // ----- Fixed Width Output -----
    // Comment out any row to disable that field.
    // Python ACTIVE_FIELDS must mirror whatever is active here.
    struct LogField { const char* fmt; float val; };
    LogField fields[] = {
        {"%10.2f ", timestamp},  // timestamp (s)
        {"%8.2f ",  yaw},        // yaw (deg)
        {"%8.2f ",  pitch},      // pitch (deg)
        {"%8.2f ",  roll},       // roll (deg)
        {"%10.2f ", gyroX},      // gyroX (rad/s)
        {"%10.2f ", gyroY},      // gyroY (rad/s)
        {"%10.2f ", gyroZ},      // gyroZ (rad/s)
        {"%10.2f ", accelX},     // accelX (m/s^2)
        {"%10.2f ", accelY},     // accelY (m/s^2)
        {"%10.2f ", accelZ},     // accelZ (m/s^2)
    };

    // '$' header byte lets Python detect packet boundaries
    Serial.print("$");
    for (const auto& f : fields) {
        Serial.printf(f.fmt, f.val);
    }
    Serial.println();
}
