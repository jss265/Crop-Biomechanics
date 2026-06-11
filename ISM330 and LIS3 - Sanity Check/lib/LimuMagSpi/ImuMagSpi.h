/****************************************************************************
 * ImuMagSpi.h
 *
 * Wrapper for the Adafruit ISM330DHCX + LIS3MDL FeatherWing (P/N 4569)
 * over SPI, designed to drop into a larger Arduino-ESP32 sketch that
 * also services ADS1220 chips and a WiFi telemetry path.
 *
 * Contract with the sketch:
 *   - The sketch owns SPI.begin(...) and is free to swap MOSI/MISO.
 *   - The sketch owns the DRDY ISRs and calls onImuDrdyFromIsr() /
 *     onMagDrdyFromIsr() from inside them.
 *   - The sketch retrieves samples via the FreeRTOS queues returned by
 *     getImuQueue() / getMagQueue() and packetizes them however it wants.
 ****************************************************************************/
#pragma once

#include <Arduino.h>
#include <SPI.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

namespace ImuMagSpi {

// ---- Sample types -------------------------------------------------------
// Field order/sizes match your ImuPacket / MagPacket exactly so you can
// memcpy a sample straight into the payload after the type byte.
struct ImuSample {
  uint64_t ts_us;
  int16_t  ax, ay, az;
  int16_t  gx, gy, gz;
};
struct MagSample {
  uint64_t ts_us;
  int16_t  mx, my, mz;
};

// ---- Unit conversions (optional, for human-readable debugging) ----------
constexpr float ACC_LSB_TO_G     = 0.000122f;       // FS = +/-4 g
constexpr float GYR_LSB_TO_DPS   = 0.070f;          // FS = +/-2000 dps
constexpr float MAG_LSB_TO_GAUSS = 1.0f / 6842.0f;  // FS = +/-4 gauss

// ---- Configuration ------------------------------------------------------
// Populate once in setup() and pass to begin().  SPI.begin() must already
// have been called by the sketch with the correct (possibly swapped) pins.
struct Config {
  SPIClass*    spi             = &SPI;
  SPISettings  spiSettings     = SPISettings(8000000, MSBFIRST, SPI_MODE3);
  int          imuCsPin        = -1;   // ISM330DHCX CS  (required)
  int          magCsPin        = -1;   // LIS3MDL    CS  (required)
  int          magIntPin       = -1;   // LIS3MDL DRDY input - for priming
  BaseType_t   taskCore        = 1;    // recommend core 1 when WiFi is on core 0
  UBaseType_t  imuTaskPriority = configMAX_PRIORITIES - 2;
  UBaseType_t  magTaskPriority = configMAX_PRIORITIES - 3;
  uint32_t     imuTaskStack    = 4096;
  uint32_t     magTaskStack    = 4096;
  size_t       imuQueueDepth   = 512;  // ~77 ms headroom at 6.66 kHz
  size_t       magQueueDepth   = 16;   // ~15 ms cushion at 1 kHz
};

// ---- Public API ---------------------------------------------------------
// Stage 1: tell the wrapper about your SPI bus and pins. Call once.
void begin(const Config& cfg);

// Stage 2: bring up each chip (WHO_AM_I + register writes).
// Returns false if the chip didn't ACK with the right WHO_AM_I.
bool initImu();
bool initMag();

// Stage 3: create the FreeRTOS tasks and queues. Call AFTER you have
// attached the DRDY interrupts in the sketch. Also primes LIS3MDL DRDY.
bool startTasks();

// ---- ISR helpers (call from your sketch's IRAM_ATTR ISRs) ---------------
void IRAM_ATTR onImuDrdyFromIsr(int64_t ts_us);
void IRAM_ATTR onMagDrdyFromIsr(int64_t ts_us);

// ---- Data access for your batching / WiFi layer -------------------------
QueueHandle_t getImuQueue();   // payload = ImuSample
QueueHandle_t getMagQueue();   // payload = MagSample

// ---- One-shot recovery helpers ------------------------------------------
// Call kickMag() from normal task context after attachInterrupt() if the
// MAG DRDY pin might already be HIGH (missed RISING edge).  Uses the normal
// xTaskNotifyGive (not FromISR), safe to call outside an ISR.
void kickMag();

// ---- Diagnostics (fold into your telemetry) -----------------------------
uint32_t getImuSampleCount();
uint32_t getMagSampleCount();
uint32_t getImuDroppedCount();   // sample arrived but queue was full
uint32_t getMagDroppedCount();

} // namespace ImuMagSpi