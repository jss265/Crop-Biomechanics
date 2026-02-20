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
    delay(2000);
    Serial.println("beginning...");

    // Proper I2C constructor:
    navx = AHRS(I2C, 200); // 200 Hz update rate

    delay(1000);
}

void loop() {

    if (!navx.isConnected() || navx.isCalibrating()) {
        return;
    }

    // ----- Dedup: skip if navX hasn't produced a new sample yet -----
    static long lastTimestamp = -1;
    long rawTimestamp = navx.getLastSensorTimestamp(); // ms, integer comparison avoids float equality issues
    if (rawTimestamp == lastTimestamp) {
        return; // no new packet; loop() will be called again immediately
    }
    lastTimestamp = rawTimestamp;

    // ----- Timestamp -----
    float timestamp = rawTimestamp / 1000.0f; // ms → seconds

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
