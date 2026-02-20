// Minimal AHRS stub for Arduino to satisfy navx_serial_logger compilation
#ifndef NAVX_ARDUINO_AHRS_H
#define NAVX_ARDUINO_AHRS_H

#include <Arduino.h>

// Provide simple numeric fallbacks for `SPI` and `I2C` when not defined
#ifndef SPI
#define SPI 1
#endif
#ifndef I2C
#define I2C 2
#endif

class AHRS {
public:
    // Generic templated constructor to accept SPI/I2C objects or macros
    template<typename T>
    AHRS(T /*port*/, uint8_t /*update_rate_hz*/ = 200) { }

    AHRS() { }

    // Assignment operator to allow `navx = AHRS(I2C, 200);`
    AHRS& operator=(const AHRS& other) { (void)other; return *this; }

    // Status
    bool isConnected() const { return true; }
    bool isCalibrating() const { return false; }

    // Timing
    long getLastSensorTimestamp() const { return (long)millis(); }

    // Orientation (degrees)
    float getYaw() const { return 0.0f; }
    float getPitch() const { return 0.0f; }
    float getRoll() const { return 0.0f; }

    // Raw sensors
    float getRawGyroX() const { return 0.0f; }
    float getRawGyroY() const { return 0.0f; }
    float getRawGyroZ() const { return 0.0f; }
    float getRawAccelX() const { return 0.0f; }
    float getRawAccelY() const { return 0.0f; }
    float getRawAccelZ() const { return 0.0f; }
};

#endif // NAVX_ARDUINO_AHRS_H
