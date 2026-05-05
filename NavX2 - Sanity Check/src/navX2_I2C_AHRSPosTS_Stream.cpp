#include <Arduino.h>
#include <Wire.h>

#include "AHRSProtocol.h"
#include "IMUProtocol.h"
#include "IMURegisters.h"

#define NAVX_I2C_ADDR             0x32
#define NAVX_UPDATE_RATE_HZ       50
#define NAVX_READ_DELAY_MS        20

// AHRSPosTS packet begins at yaw register and ends at timestamp registers.
// This reads the full contiguous block required for AHRSPosTS-equivalent data.
#define FIRST_REGISTER            NAVX_REG_YAW_L
#define LAST_REGISTER             NAVX_REG_TIMESTAMP_H_H
#define NUM_BYTES_TO_READ         ((LAST_REGISTER - FIRST_REGISTER) + 1)

uint8_t rawData[NUM_BYTES_TO_READ];

bool readNavXRegisters(uint8_t first_reg, uint8_t* buffer, uint8_t len)
{
    Wire.beginTransmission(NAVX_I2C_ADDR);
    Wire.write(first_reg);
    Wire.write(len);

    if (Wire.endTransmission() != 0)
    {
        return false;
    }

    int received = Wire.requestFrom(NAVX_I2C_ADDR, len);

    if (received != len)
    {
        return false;
    }

    int i = 0;
    while (Wire.available() && i < len)
    {
        buffer[i++] = Wire.read();
    }

    return (i == len);
}

void printFloat(const char* label, float value, int precision = 2)
{
    Serial.print(label);
    Serial.print(": ");
    Serial.print(value, precision);
    Serial.print("  ");
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Wire.begin();
    Wire.setClock(400000);

    memset(rawData, 0, sizeof(rawData));

    Serial.println();
    Serial.println("========================================");
    Serial.println("navX AHRSPosTS Reader Started");
    Serial.println("========================================");
}

void loop()
{
    if (!readNavXRegisters(FIRST_REGISTER, rawData, NUM_BYTES_TO_READ))
    {
        Serial.println("ERROR: Failed to read navX registers");
        delay(NAVX_READ_DELAY_MS);
        return;
    }

    // ------------------------------------------------------------
    // Orientation
    // ------------------------------------------------------------

    float yaw = IMURegisters::decodeProtocolSignedHundredthsFloat(
        (char*)&rawData[NAVX_REG_YAW_L - FIRST_REGISTER]);

    float pitch = IMURegisters::decodeProtocolSignedHundredthsFloat(
        (char*)&rawData[NAVX_REG_PITCH_L - FIRST_REGISTER]);

    float roll = IMURegisters::decodeProtocolSignedHundredthsFloat(
        (char*)&rawData[NAVX_REG_ROLL_L - FIRST_REGISTER]);

    float compass_heading = IMURegisters::decodeProtocolUnsignedHundredthsFloat(
        (char*)&rawData[NAVX_REG_HEADING_L - FIRST_REGISTER]);

    float fused_heading = IMURegisters::decodeProtocolUnsignedHundredthsFloat(
        (char*)&rawData[NAVX_REG_FUSED_HEADING_L - FIRST_REGISTER]);

    // ------------------------------------------------------------
    // Linear Acceleration (World Frame, gravity removed)
    // ------------------------------------------------------------

    float linear_accel_x = IMURegisters::decodeProtocolSignedThousandthsFloat(
        (char*)&rawData[NAVX_REG_LINEAR_ACC_X_L - FIRST_REGISTER]);

    float linear_accel_y = IMURegisters::decodeProtocolSignedThousandthsFloat(
        (char*)&rawData[NAVX_REG_LINEAR_ACC_Y_L - FIRST_REGISTER]);

    float linear_accel_z = IMURegisters::decodeProtocolSignedThousandthsFloat(
        (char*)&rawData[NAVX_REG_LINEAR_ACC_Z_L - FIRST_REGISTER]);

    // ------------------------------------------------------------
    // Altitude / Pressure
    // ------------------------------------------------------------

    float altitude = IMURegisters::decodeProtocol1616Float(
        (char*)&rawData[NAVX_REG_ALTITUDE_D_L - FIRST_REGISTER]);

    float pressure = IMURegisters::decodeProtocol1616Float(
        (char*)&rawData[NAVX_REG_PRESSURE_DL - FIRST_REGISTER]);

    // ------------------------------------------------------------
    // Quaternion
    // ------------------------------------------------------------

    float quat_w = IMURegisters::decodeProtocolSignedThousandthsFloat(
        (char*)&rawData[NAVX_REG_QUAT_W_L - FIRST_REGISTER]);

    float quat_x = IMURegisters::decodeProtocolSignedThousandthsFloat(
        (char*)&rawData[NAVX_REG_QUAT_X_L - FIRST_REGISTER]);

    float quat_y = IMURegisters::decodeProtocolSignedThousandthsFloat(
        (char*)&rawData[NAVX_REG_QUAT_Y_L - FIRST_REGISTER]);

    float quat_z = IMURegisters::decodeProtocolSignedThousandthsFloat(
        (char*)&rawData[NAVX_REG_QUAT_Z_L - FIRST_REGISTER]);

    // ------------------------------------------------------------
    // Velocity (m/s)
    // ------------------------------------------------------------

    float vel_x = IMURegisters::decodeProtocol1616Float(
        (char*)&rawData[NAVX_REG_VEL_X_I_L - FIRST_REGISTER]);

    float vel_y = IMURegisters::decodeProtocol1616Float(
        (char*)&rawData[NAVX_REG_VEL_Y_I_L - FIRST_REGISTER]);

    float vel_z = IMURegisters::decodeProtocol1616Float(
        (char*)&rawData[NAVX_REG_VEL_Z_I_L - FIRST_REGISTER]);

    // ------------------------------------------------------------
    // Displacement (m)
    // ------------------------------------------------------------

    float disp_x = IMURegisters::decodeProtocol1616Float(
        (char*)&rawData[NAVX_REG_DISP_X_I_L - FIRST_REGISTER]);

    float disp_y = IMURegisters::decodeProtocol1616Float(
        (char*)&rawData[NAVX_REG_DISP_Y_I_L - FIRST_REGISTER]);

    float disp_z = IMURegisters::decodeProtocol1616Float(
        (char*)&rawData[NAVX_REG_DISP_Z_I_L - FIRST_REGISTER]);

    // ------------------------------------------------------------
    // Timestamp
    // ------------------------------------------------------------

    uint32_t sensor_timestamp = IMURegisters::decodeProtocolUint32(
        (char*)&rawData[NAVX_REG_TIMESTAMP_L_L - FIRST_REGISTER]);

    // ------------------------------------------------------------
    // Status
    // ------------------------------------------------------------

    uint8_t op_status = rawData[NAVX_REG_OP_STATUS - FIRST_REGISTER];
    uint8_t sensor_status = rawData[NAVX_REG_SENSOR_STATUS_L - FIRST_REGISTER];
    uint8_t cal_status = rawData[NAVX_REG_CAL_STATUS - FIRST_REGISTER];
    uint8_t selftest_status = rawData[NAVX_REG_SELFTEST_STATUS - FIRST_REGISTER];

    // ------------------------------------------------------------
    // Print Everything
    // ------------------------------------------------------------

    Serial.println();
    Serial.println("================ AHRSPosTS ================");

    Serial.println("Orientation:");
    printFloat("Yaw", yaw);
    printFloat("Pitch", pitch);
    printFloat("Roll", roll);
    Serial.println();

    printFloat("Compass Heading", compass_heading);
    printFloat("Fused Heading", fused_heading);
    Serial.println();

    Serial.println();
    Serial.println("Linear Acceleration (G):");
    printFloat("Accel X", linear_accel_x, 3);
    printFloat("Accel Y", linear_accel_y, 3);
    printFloat("Accel Z", linear_accel_z, 3);
    Serial.println();

    Serial.println();
    Serial.println("Velocity (m/s):");
    printFloat("Vel X", vel_x, 4);
    printFloat("Vel Y", vel_y, 4);
    printFloat("Vel Z", vel_z, 4);
    Serial.println();

    Serial.println();
    Serial.println("Displacement (m):");
    printFloat("Disp X", disp_x, 4);
    printFloat("Disp Y", disp_y, 4);
    printFloat("Disp Z", disp_z, 4);
    Serial.println();

    Serial.println();
    Serial.println("Quaternion:");
    printFloat("W", quat_w, 4);
    printFloat("X", quat_x, 4);
    printFloat("Y", quat_y, 4);
    printFloat("Z", quat_z, 4);
    Serial.println();

    Serial.println();
    Serial.println("Environmental:");
    printFloat("Altitude (m)", altitude, 2);
    printFloat("Pressure (mb)", pressure, 2);
    Serial.println();

    Serial.println();
    Serial.println("System:");
    Serial.print("Timestamp: ");
    Serial.println(sensor_timestamp);

    Serial.print("Op Status: ");
    Serial.println(op_status);

    Serial.print("Sensor Status: ");
    Serial.println(sensor_status, BIN);

    Serial.print("Calibration Status: ");
    Serial.println(cal_status, BIN);

    Serial.print("Self Test Status: ");
    Serial.println(selftest_status, BIN);

    Serial.println("===========================================");

    delay(NAVX_READ_DELAY_MS);
}
