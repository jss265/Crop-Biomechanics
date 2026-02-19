#include <Wire.h>
#include "AHRS.h"

// ----- Constants -----
const float DEG2RAD = 3.14159265358979f / 180.0f;
const float G_TO_MPS2 = 9.80665f;

// ----- navX Object (I2C) -----
AHRS navx(SPI, 200);  // Will change below for I2C

void setup() {
    Serial.begin(921600);
    Wire.begin();

    // Proper I2C constructor:
    navx = AHRS(I2C, 200);  // 200 Hz update rate

    delay(1000);
}

void loop() {

    if (!navx.isConnected() || navx.isCalibrating()) {
        return;
    }

    // ----- Timestamp -----
    float timestamp = navx.getLastSensorTimestamp() / 1000.0f; // ms → seconds

    // ----- Orientation (degrees → keep in degrees or convert?) -----
    float yaw   = navx.getYaw();
    float pitch = navx.getPitch();
    float roll  = navx.getRoll();

    // ----- Gyro (deg/s → rad/s) -----
    float gyroX = navx.getRawGyroX() * DEG2RAD;
    float gyroY = navx.getRawGyroY() * DEG2RAD;
    float gyroZ = navx.getRawGyroZ() * DEG2RAD;

    // ----- Accel (G → m/s^2) -----
    float accelX = navx.getRawAccelX() * G_TO_MPS2;
    float accelY = navx.getRawAccelY() * G_TO_MPS2;
    float accelZ = navx.getRawAccelZ() * G_TO_MPS2;

    // ----- Fixed Width Output -----
    Serial.printf(
        "%10.2f "  // timestamp (s)
        "%8.2f %8.2f %8.2f "  // yaw pitch roll (deg)
        "%10.2f %10.2f %10.2f "  // gyro (rad/s)
        "%10.2f %10.2f %10.2f\n",  // accel (m/s^2)

        timestamp,
        yaw, pitch, roll,
        gyroX, gyroY, gyroZ,
        accelX, accelY, accelZ
    );

}
