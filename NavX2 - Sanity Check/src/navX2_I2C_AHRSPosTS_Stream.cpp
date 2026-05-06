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
#define FIRST_REGISTER            NAVX_REG_OP_STATUS
#define LAST_REGISTER             NAVX_REG_LAST
#define NUM_BYTES_TO_READ         ((LAST_REGISTER - FIRST_REGISTER) + 1)

uint8_t rawData[NUM_BYTES_TO_READ];

struct FlagBit { uint16_t mask; uint16_t value; const char* name; };
const FlagBit OP_STATUS_FLAGS[] = {{0xFF, 0, "Initializing"}, {0xFF, 1, "Self-Test"}, {0xFF, 2, "Error"}, {0xFF, 3, "Autocal"}, {0xFF, 4, "Normal"}};
const FlagBit SENSOR_STATUS_FLAGS[] = {{1, 1, "Moving"}, {2, 2, "Yaw Stable"}, {4, 4, "Mag Disturbance"}, {8, 8, "Altitude Valid"}, {16, 16, "Sea Level Press Set"}, {32, 32, "Fused Heading Valid"}};
const FlagBit CAL_STATUS_FLAGS[] = {{3, 0, "IMU In-Progress"}, {3, 1, "IMU Accumulating"}, {3, 2, "IMU Complete"}, {4, 4, "Mag Complete"}, {8, 8, "Baro Complete"}};
const FlagBit TEST_STATUS_FLAGS[] = {{0x80, 0x80, "Tests Complete"}, {1, 1, "Gyro Passed"}, {2, 2, "Accel Passed"}, {4, 4, "Mag Passed"}, {8, 8, "Baro Passed"}};
const FlagBit CAPABILITY_FLAGS[] = {{0x0004, 0x0004, "Omnimount"}, {0x0040, 0x0040, "Vel/Disp"}, {0x0080, 0x0080, "Yaw Reset"}, {0x0100, 0x0100, "AHRSPosTS"}};

bool readNavXRegisters(uint8_t first_reg, uint8_t* buffer, uint8_t len)
{
    Wire.beginTransmission(NAVX_I2C_ADDR);
    Wire.write(first_reg);
    Wire.write(len);

    if (Wire.endTransmission() != 0)
    {
        return false;
    }

    int received = Wire.requestFrom(NAVX_I2C_ADDR, (uint8_t)len);

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

void printInt(const char* label, uint32_t value)
{
    Serial.print(label);
    Serial.print(": ");
    Serial.print(value);
    Serial.print("  ");
}

/*
template ,size_t N> tells the compiler: 
"I am about to write a function that works for arrays of any size, and I want you
to automatically substitute N with the actual size of the array whenever you compile
this function."
*/
template <size_t N> 
void printFlags(const char* label, uint16_t state, const FlagBit (&flags)[N])
{
    Serial.print(label);
    Serial.print(": ");
    bool printed = false;
    for (size_t i = 0; i < N; i++)
    {
        if ((state & flags[i].mask) == flags[i].value)
        {
            if (printed) Serial.print(", ");
            Serial.print(flags[i].name);
            printed = true;
        }
    }
    if (!printed) Serial.print("None");
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
    delay(2000);
    Serial.println("navX AHRSPosTS Stream Started");
    delay(2000);
}

void loop()
{
    if (!readNavXRegisters(FIRST_REGISTER, rawData, NUM_BYTES_TO_READ))
    {
        Serial.println("ERROR: Failed to read navX registers");
        delay(NAVX_READ_DELAY_MS);
        return;
    }

    // ----------------
    // Update Everything
    // ----------------

    // Orientation (deg) (YPR {-180,180}, Heading {0,360})
    float yaw = IMURegisters::decodeProtocolSignedHundredthsFloat((char*)&rawData[NAVX_REG_YAW_L - FIRST_REGISTER]);
    float pitch = IMURegisters::decodeProtocolSignedHundredthsFloat((char*)&rawData[NAVX_REG_PITCH_L - FIRST_REGISTER]);
    float roll = IMURegisters::decodeProtocolSignedHundredthsFloat((char*)&rawData[NAVX_REG_ROLL_L - FIRST_REGISTER]);
    float compass_heading = IMURegisters::decodeProtocolUnsignedHundredthsFloat((char*)&rawData[NAVX_REG_HEADING_L - FIRST_REGISTER]);
    float fused_heading = IMURegisters::decodeProtocolUnsignedHundredthsFloat((char*)&rawData[NAVX_REG_FUSED_HEADING_L - FIRST_REGISTER]);

    // Linear Acceleration (G) (World Frame, gravity removed)
    float linear_accel_x = IMURegisters::decodeProtocolSignedThousandthsFloat((char*)&rawData[NAVX_REG_LINEAR_ACC_X_L - FIRST_REGISTER]);
    float linear_accel_y = IMURegisters::decodeProtocolSignedThousandthsFloat((char*)&rawData[NAVX_REG_LINEAR_ACC_Y_L - FIRST_REGISTER]);
    float linear_accel_z = IMURegisters::decodeProtocolSignedThousandthsFloat((char*)&rawData[NAVX_REG_LINEAR_ACC_Z_L - FIRST_REGISTER]);

    // Velocity (m/s)
    float vel_x = IMURegisters::decodeProtocol1616Float((char*)&rawData[NAVX_REG_VEL_X_I_L - FIRST_REGISTER]);
    float vel_y = IMURegisters::decodeProtocol1616Float((char*)&rawData[NAVX_REG_VEL_Y_I_L - FIRST_REGISTER]);
    float vel_z = IMURegisters::decodeProtocol1616Float((char*)&rawData[NAVX_REG_VEL_Z_I_L - FIRST_REGISTER]);

    // Displacement (m)
    float disp_x = IMURegisters::decodeProtocol1616Float((char*)&rawData[NAVX_REG_DISP_X_I_L - FIRST_REGISTER]);
    float disp_y = IMURegisters::decodeProtocol1616Float((char*)&rawData[NAVX_REG_DISP_Y_I_L - FIRST_REGISTER]);
    float disp_z = IMURegisters::decodeProtocol1616Float((char*)&rawData[NAVX_REG_DISP_Z_I_L - FIRST_REGISTER]);

    // Quaternion
    float quat_w = IMURegisters::decodeProtocolSignedThousandthsFloat((char*)&rawData[NAVX_REG_QUAT_W_L - FIRST_REGISTER]);
    float quat_x = IMURegisters::decodeProtocolSignedThousandthsFloat((char*)&rawData[NAVX_REG_QUAT_X_L - FIRST_REGISTER]);
    float quat_y = IMURegisters::decodeProtocolSignedThousandthsFloat((char*)&rawData[NAVX_REG_QUAT_Y_L - FIRST_REGISTER]);
    float quat_z = IMURegisters::decodeProtocolSignedThousandthsFloat((char*)&rawData[NAVX_REG_QUAT_Z_L - FIRST_REGISTER]);

    // Enviroment (m, mbar) 
    float altitude = IMURegisters::decodeProtocol1616Float((char*)&rawData[NAVX_REG_ALTITUDE_D_L - FIRST_REGISTER]);
    float pressure = IMURegisters::decodeProtocol1616Float((char*)&rawData[NAVX_REG_PRESSURE_DL - FIRST_REGISTER]);

    // Timestamp (ms)
    uint32_t sensor_timestamp = IMURegisters::decodeProtocolUint32((char*)&rawData[NAVX_REG_TIMESTAMP_L_L - FIRST_REGISTER]);

    // Status
    uint8_t op_status = rawData[NAVX_REG_OP_STATUS - FIRST_REGISTER];
    uint8_t sensor_status = rawData[NAVX_REG_SENSOR_STATUS_L - FIRST_REGISTER];
    uint8_t cal_status = rawData[NAVX_REG_CAL_STATUS - FIRST_REGISTER];
    uint8_t selftest_status = rawData[NAVX_REG_SELFTEST_STATUS - FIRST_REGISTER];
    uint16_t capability_flags = rawData[NAVX_REG_CAPABILITY_FLAGS_L - FIRST_REGISTER] | (rawData[NAVX_REG_CAPABILITY_FLAGS_H - FIRST_REGISTER] << 8);

    // ----------------
    // Print Everything
    // ----------------

    // Orientation (deg) (YPR {-180,180}, Heading {0,360})
    printFloat("Yaw", yaw);
    printFloat("Pitch", pitch);
    printFloat("Roll", roll);
    printFloat("Compass Heading", compass_heading);
    printFloat("Fused Heading", fused_heading);

    // Linear Acceleration (G) (World Frame, gravity removed)
    printFloat("Accel X", linear_accel_x, 3);
    printFloat("Accel Y", linear_accel_y, 3);
    printFloat("Accel Z", linear_accel_z, 3);

    // Velocity (m/s)
    printFloat("Vel X", vel_x, 4);
    printFloat("Vel Y", vel_y, 4);
    printFloat("Vel Z", vel_z, 4);

    // Displacement (m)
    printFloat("Disp X", disp_x, 4);
    printFloat("Disp Y", disp_y, 4);
    printFloat("Disp Z", disp_z, 4);

    // Quaternions
    printFloat("W", quat_w, 4);
    printFloat("X", quat_x, 4);
    printFloat("Y", quat_y, 4);
    printFloat("Z", quat_z, 4);

    // Enviroment (m, mbar)
    printFloat("Altitude (m)", altitude, 2);
    printFloat("Pressure (mb)", pressure, 2);

    // Timestamp (ms)
    printInt("Timestamp", sensor_timestamp);
    
    // Status
    printFlags("Op Status", op_status, OP_STATUS_FLAGS);
    printFlags("Sensor Status", sensor_status, SENSOR_STATUS_FLAGS);
    printFlags("Cal Status", cal_status, CAL_STATUS_FLAGS);
    printFlags("Test Status", selftest_status, TEST_STATUS_FLAGS);
    printFlags("Capabilities", capability_flags, CAPABILITY_FLAGS);

    Serial.println();

    delay(NAVX_READ_DELAY_MS);
}
