#ifndef IMU_MAG_SPI_H
#define IMU_MAG_SPI_H

// ============================================================================
//  ImuMagSpi  —  ISM330DHCX (IMU) + LIS3MDL (Mag) over a shared SPI bus.
//
//  June 2026 rewrite: FIFO-polling model (NO FreeRTOS tasks, NO data-ready
//  interrupts). The ISM330DHCX streams accel+gyro into its on-chip FIFO with
//  hardware timestamps; the host sketch drains the FIFO from pollSensors() at
//  loop() cadence. The LIS3MDL is polled by its STATUS register.
//
//  WHY THIS SHAPE:
//   - The old DRDY-per-sample task model existed only to beat the ~150 us
//     inter-sample deadline at 6.66 kHz. The FIFO buffers hundreds of samples
//     in hardware, removing that deadline entirely -> no task, no per-sample
//     ISR, no null-handle panics. Acquisition is now deadline-proof and lives
//     in one linear place.
//   - Timestamps: the FIFO's hardware timestamp is used ONLY to compute the
//     relative spacing of samples inside a single drain. The newest sample in
//     each drain is anchored to esp_timer_get_time() (the ESP32 microsecond
//     clock) and older samples are back-filled by subtracting their spacing.
//     The value reported to the caller is therefore always in esp_timer us,
//     which is exactly what collect_data.py expects (it subtracts one shared
//     ts_origin across ADS/IMU/Mag).
//
//  REGISTER VALUES were verified against STMicroelectronics' ism330dhcx_reg.h
//  and lsm6dso_reg.h, the Zephyr hal_st driver, and a working ESP32 FIFO
//  implementation (esp-gyrologger). See ImuMagSpi.cpp for citations per write.
//
//  Bus sharing: every transfer is wrapped in SPI.beginTransaction(settings)
//  so the ADS reads in the .ino cooperate on the same bus. The caller is
//  responsible for SPI.begin().
// ============================================================================

#include <Arduino.h>
#include <SPI.h>

namespace ImuMagSpi {

// ---- Public sample types (NOT wire structs; the .ino repacks these) --------
// ts_us is esp_timer microseconds (see header note above), already anchored.
struct ImuSample {
  int64_t ts_us;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};
struct MagSample {
  int64_t ts_us;
  int16_t mx, my, mz;
};

// ---- Configuration ---------------------------------------------------------
struct Config {
  SPIClass*   spi         = &SPI;
  SPISettings spiSettings = SPISettings(8000000, MSBFIRST, SPI_MODE3); // ST max 10MHz
  int imuCsPin  = -1;   // ISM330DHCX CS
  int magCsPin  = -1;   // LIS3MDL    CS
};

// ---- Lifecycle -------------------------------------------------------------
// begin(): stores config + drives both CS lines HIGH (deselected). Safe to
//          call before the sensors are known-present.
void begin(const Config& cfg);

// initImu()/initMag(): probe WHO_AM_I, configure, and (for the IMU) start the
// FIFO in Continuous mode with timestamps. Return false if WHO_AM_I mismatches
// (e.g. the FeatherWing isn't installed) — the .ino uses this to skip the
// sensor entirely so a missing board can never wedge anything.
bool initImu();
bool initMag();

// resetFifo(): Bypass->Continuous toggle. Empties the FIFO and re-arms it, and
// re-anchors the internal clock mapping so the next drain starts clean. Call on
// every (re)connect so a session never begins with stale, pre-connection data.
// No-op (returns false) if the IMU isn't present.
bool resetFifo();

// ---- Polling (called from pollSensors() in CONNECTED state) ----------------
// drainImu(): pull all currently-buffered IMU samples out of the FIFO into out[]
//             (up to maxOut). Returns the count written. Non-blocking; returns 0
//             if the FIFO is empty. Decodes accel/gyro/timestamp FIFO tags and
//             assigns esp_timer-anchored ts_us to each emitted sample.
int drainImu(ImuSample* out, int maxOut);

// readMag(): if the LIS3MDL has a new sample ready (STATUS.ZYXDA), read it into
//            *out (ts_us = esp_timer now) and return true; else false.
bool readMag(MagSample* out);

// ---- Presence flags (cheap accessors; the .ino also caches these) ----------
bool imuPresent();
bool magPresent();

} // namespace ImuMagSpi

#endif // IMU_MAG_SPI_H