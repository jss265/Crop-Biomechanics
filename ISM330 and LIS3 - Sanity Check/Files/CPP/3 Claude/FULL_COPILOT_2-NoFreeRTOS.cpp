/****************************************************************************
 * Arduino Nano ESP32  <->  Adafruit ISM330DHCX + LIS3MDL FeatherWing (4569)
 * High-rate, interrupt-driven, lossless sample acquisition.
 * BARE-METAL variant: no FreeRTOS tasks; everything is serviced in loop().
 *
 * Architecture
 *   - ISM330DHCX: its INT1 pin is configured as a FIFO *watermark* (WTM)
 *     interrupt.  The chip batches accel+gyro into its hardware FIFO and
 *     only raises INT1 once WTM unread words have accumulated.  A minimal
 *     IRAM ISR captures esp_timer_get_time() (1 us resolution) and sets a
 *     "pending" flag.  loop() polls that flag and, when set, bursts the
 *     whole FIFO out over SPI, demultiplexes the per-word tag byte into
 *     accel/gyro pairs, publishes a "latest sample" snapshot and bumps a
 *     counter for every pair drained.
 *   - LIS3MDL: the part has NO hardware FIFO, so it stays DRDY-driven.  Its
 *     DRDY pin is wired to a GPIO; the ISR timestamps + counts edges and
 *     sets a pending flag.  loop() does the 6-byte burst read.
 *   - Because there is no RTOS, there is no core pinning and no task
 *     preemption: loop() services the IMU first (it has the hard FIFO
 *     overrun deadline), then the MAG, then the periodic Serial output.
 *     The print cadence is gated so it never starves the FIFO drain.
 *   - A "missed" counter tracks losslessness.  For the IMU it counts FIFO
 *     overruns (data lost because the FIFO filled before being drained);
 *     for the MAG it counts DRDY edges that arrived while one was already
 *     pending.  Under normal load both stay at 0.
 *
 * Concurrency
 *   - The only data shared between ISR and loop() are the int64 timestamps,
 *     the "pending" flags, and the MAG edge counter.  Those are guarded with
 *     short noInterrupts()/interrupts() critical sections.  Everything else
 *     lives entirely in loop() context, so no other locking is required.
 *
 * Timing
 *   - All wall-clock timing uses esp_timer_get_time() (int64 microseconds).
 *     Unlike millis() (uint32 ms, wraps at ~49.7 days) this matches the
 *     sample timestamps and will not wrap for any practical run length, so
 *     the ODR window math stays correct well past 72 minutes.
 *
 * Configured ODRs   :  IMU 6.66 kHz (accel+gyro)   MAG 1.00 kHz
 * Full-scale ranges :  +/-4 g, +/-2000 dps, +/-4 gauss
 ****************************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include "esp_timer.h"

// =========================== Pin map ====================================
static constexpr int IMU_CS_PIN  = A4;  // ISM330DHCX CS
static constexpr int MAG_CS_PIN  = A5;  // LIS3MDL    CS
static constexpr int IMU_INT_PIN = A6;  // ISM330DHCX INT1 (FIFO watermark)
static constexpr int MAG_INT_PIN = A7;  // LIS3MDL    DRDY

// =========================== SPI bus ====================================
// Both parts: SPI mode 3, MSB first, <=10 MHz.  8 MHz leaves headroom.
static const SPISettings kSpi(8000000, MSBFIRST, SPI_MODE3);

// =========================== ISM330DHCX registers =======================
static constexpr uint8_t ISM_WHO_AM_I         = 0x0F;  // expect 0x6B
static constexpr uint8_t ISM_FIFO_CTRL1       = 0x07;  // WTM[7:0]
static constexpr uint8_t ISM_FIFO_CTRL2       = 0x08;  // WTM[8], FIFO options
static constexpr uint8_t ISM_FIFO_CTRL3       = 0x09;  // BDR_XL / BDR_GY
static constexpr uint8_t ISM_FIFO_CTRL4       = 0x0A;  // FIFO_MODE
static constexpr uint8_t ISM_COUNTER_BDR_REG1 = 0x0B;
static constexpr uint8_t ISM_INT1_CTRL        = 0x0D;
static constexpr uint8_t ISM_CTRL1_XL         = 0x10;
static constexpr uint8_t ISM_CTRL2_G          = 0x11;
static constexpr uint8_t ISM_CTRL3_C          = 0x12;
static constexpr uint8_t ISM_FIFO_STATUS1     = 0x3A;  // DIFF_FIFO[7:0]
static constexpr uint8_t ISM_FIFO_STATUS2     = 0x3B;  // DIFF_FIFO[9:8] + flags
static constexpr uint8_t ISM_FIFO_DATA_OUT_TAG = 0x78; // 1 tag + 6 data bytes

// CTRL1_XL : ODR_XL=1010 (6.66 kHz), FS_XL=10 (+/-4 g), LPF2_XL=0
static constexpr uint8_t ISM_CTRL1_XL_VAL     = 0xA8;
// CTRL2_G  : ODR_G =1010 (6.66 kHz), FS_G =11 (+/-2000 dps), FS_125=0
static constexpr uint8_t ISM_CTRL2_G_VAL      = 0xAC;
// CTRL3_C  : BDU=1, IF_INC=1 (block-data-update + register auto-increment)
static constexpr uint8_t ISM_CTRL3_C_VAL      = 0x44;

// -------- FIFO watermark configuration --------
// 1 FIFO word = 1 tag byte + 6 data bytes = 7 bytes.  Accel & gyro are
// batched at the same ODR, so they arrive as alternating words: a full
// accel+gyro sample-pair therefore costs TWO words.  The 512-word FIFO
// thus holds ~256 sample-pairs (~38 ms @ 6.66 kHz).
//
// WTM = 64 words = 32 accel/gyro sample-pairs:
//   - IRQ rate  ~104 Hz  (64x fewer interrupts than per-sample DRDY)
//   - latency   ~4.8 ms  (fresh enough for the printVals() sanity display)
//   - burst     64*7 = 448 bytes  (~0.5 ms @ 8 MHz; short SPI bus hold)
//   - headroom  ~3/4 of the FIFO left as margin against scheduling jitter
// Change this single constant to re-tune; everything else scales off it.
static constexpr uint16_t ISM_FIFO_WTM         = 64;
static constexpr size_t   ISM_FIFO_MAX_WORDS   = 512;  // physical FIFO depth

// FIFO_CTRL1: WTM[7:0]
static constexpr uint8_t ISM_FIFO_CTRL1_VAL   = (uint8_t)(ISM_FIFO_WTM & 0xFF);
// FIFO_CTRL2: WTM[8] only (bit0); all other FIFO features off.
static constexpr uint8_t ISM_FIFO_CTRL2_VAL   = (uint8_t)((ISM_FIFO_WTM >> 8) & 0x01);
// FIFO_CTRL3: BDR_GY=1010 (6.66 kHz) | BDR_XL=1010 (6.66 kHz) -> 0xAA
static constexpr uint8_t ISM_FIFO_CTRL3_VAL   = 0xAA;
// FIFO_CTRL4: FIFO_MODE=110 (Continuous/Stream mode), no temp/ts batching.
static constexpr uint8_t ISM_FIFO_CTRL4_VAL   = 0x06;
// INT1_CTRL: INT1_FIFO_TH=1 (FIFO watermark drives INT1).  bit3 = 0x08.
static constexpr uint8_t ISM_INT1_CTRL_VAL    = 0x08;
// COUNTER_BDR_REG1: DATAREADY_PULSED=1.  Harmless here (INT1 is FIFO-driven)
// but keeps the data-ready pulse behavior consistent with the chip default.
static constexpr uint8_t ISM_COUNTER_BDR_VAL  = 0x80;

// FIFO tag values (FIFO_DATA_OUT_TAG bits 7:3 = tag_sensor).
static constexpr uint8_t ISM_TAG_GYRO_NC      = 0x01;  // gyro,  no compression
static constexpr uint8_t ISM_TAG_XL_NC        = 0x02;  // accel, no compression

// raw int16 -> physical units
static constexpr float ISM_ACC_LSB_TO_G   = 0.000122f;  // +/-4 g
static constexpr float ISM_GYR_LSB_TO_DPS = 0.070f;     // +/-2000 dps

// =========================== LIS3MDL registers ==========================
// NOTE: the LIS3MDL has no hardware FIFO, so it remains DRDY-driven.
static constexpr uint8_t MAG_WHO_AM_I   = 0x0F;  // expect 0x3D
static constexpr uint8_t MAG_CTRL_REG1  = 0x20;
static constexpr uint8_t MAG_CTRL_REG2  = 0x21;
static constexpr uint8_t MAG_CTRL_REG3  = 0x22;
static constexpr uint8_t MAG_CTRL_REG4  = 0x23;
static constexpr uint8_t MAG_CTRL_REG5  = 0x24;
static constexpr uint8_t MAG_OUT_X_L    = 0x28;  // 6 bytes XYZ

// CTRL_REG1: OM=00 (Low-Power XY), FAST_ODR=1  ->  1000 Hz output data rate
static constexpr uint8_t MAG_CTRL_REG1_VAL = 0x02;
// CTRL_REG2: FS=00 (+/-4 gauss)
static constexpr uint8_t MAG_CTRL_REG2_VAL = 0x00;
// CTRL_REG3: MD=00 continuous conversion
static constexpr uint8_t MAG_CTRL_REG3_VAL = 0x00;
// CTRL_REG4: OMZ=00 (Z in Low-Power, same rate as XY -> recommended)
static constexpr uint8_t MAG_CTRL_REG4_VAL = 0x00;
// CTRL_REG5: BDU=1
static constexpr uint8_t MAG_CTRL_REG5_VAL = 0x40;

static constexpr float MAG_LSB_TO_GAUSS = 1.0f / 6842.0f;  // +/-4 gauss

// =========================== Sample types ===============================
struct ImuSample {
  int64_t t_us;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};
struct MagSample {
  int64_t t_us;
  int16_t mx, my, mz;
};

// =========================== Shared state ===============================
// Single-threaded: these live in loop() context and are updated only by the
// service routines below.  No locking needed for the snapshots/counters.
static ImuSample latestImu      = {};
static MagSample latestMag      = {};
static uint32_t  imuSampleCount = 0;   // every accel/gyro pair drained from FIFO
static uint32_t  magSampleCount = 0;   // every successful MAG read
static uint32_t  imuMissedCount = 0;   // FIFO overruns (lost IMU data)
static uint32_t  magMissedCount = 0;   // DRDYs that arrived while one pending

// Written by ISR, read/cleared by loop().  Guarded with brief
// noInterrupts()/interrupts() sections where read-modify-write matters.
static volatile bool    imuPending      = false;
static volatile bool    magPending      = false;
static volatile int64_t imuIsrTimestamp = 0;
static volatile int64_t magIsrTimestamp = 0;
static volatile uint32_t magIsrEdges    = 0;  // total MAG DRDY edges seen

// =========================== SPI helpers ================================
// SPI.beginTransaction()/endTransaction() bracket every access so the bus
// settings are applied consistently; in this single-threaded build there is
// no cross-task contention to serialize.
static inline void spiWriteReg(int csPin, uint8_t reg, uint8_t val) {
  SPI.beginTransaction(kSpi);
  digitalWrite(csPin, LOW);
  SPI.transfer(reg & 0x7F);          // bit7 = 0 -> write
  SPI.transfer(val);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
}
static inline uint8_t spiReadReg(int csPin, uint8_t reg) {
  SPI.beginTransaction(kSpi);
  digitalWrite(csPin, LOW);
  SPI.transfer(reg | 0x80);          // bit7 = 1 -> read
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  return v;
}
// ISM330DHCX: multi-byte read needs only bit7=1; auto-increment is provided
// by IF_INC=1 in CTRL3_C.
static inline void imuReadBurst(uint8_t reg, uint8_t *buf, size_t n) {
  SPI.beginTransaction(kSpi);
  digitalWrite(IMU_CS_PIN, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transferBytes(nullptr, buf, n);
  digitalWrite(IMU_CS_PIN, HIGH);
  SPI.endTransaction();
}
// LIS3MDL: multi-byte read needs bit7=1 AND bit6=1 (MS bit).
static inline void magReadBurst(uint8_t reg, uint8_t *buf, size_t n) {
  SPI.beginTransaction(kSpi);
  digitalWrite(MAG_CS_PIN, LOW);
  SPI.transfer(reg | 0xC0);
  SPI.transferBytes(nullptr, buf, n);
  digitalWrite(MAG_CS_PIN, HIGH);
  SPI.endTransaction();
}

// =========================== ISRs (in IRAM) =============================
// ISRs do the minimum: timestamp the edge and raise a pending flag.  All SPI
// work happens later in loop().  Keeping them tiny preserves accurate edge
// capture even though there is no RTOS task to wake.
static void IRAM_ATTR imuIsr() {
  imuIsrTimestamp = esp_timer_get_time();
  imuPending      = true;
}
static void IRAM_ATTR magIsr() {
  magIsrTimestamp = esp_timer_get_time();
  magIsrEdges    += 1;       // count every edge; loop() detects "while busy"
  magPending      = true;
}

// =========================== IMU FIFO helpers ===========================
// Number of unread words currently in the FIFO (DIFF_FIFO, 10-bit).
static inline uint16_t imuFifoLevel() {
  uint8_t s1 = spiReadReg(IMU_CS_PIN, ISM_FIFO_STATUS1);
  uint8_t s2 = spiReadReg(IMU_CS_PIN, ISM_FIFO_STATUS2);
  return (uint16_t)(((uint16_t)(s2 & 0x03) << 8) | s1);
}
// FIFO overrun flag (FIFO_STATUS2.fifo_ovr_ia, bit6): set if the FIFO
// filled completely before being drained -> data was lost.
static inline bool imuFifoOverrun() {
  return (spiReadReg(IMU_CS_PIN, ISM_FIFO_STATUS2) & 0x40) != 0;
}

// =========================== Sensor service routines ====================
// Drain the IMU FIFO if a watermark interrupt is pending.  Called every
// loop() iteration BEFORE the MAG so the hard overrun deadline wins.
static void serviceImu() {
  // One FIFO word = 7 bytes (tag + 6 data).  Static so the (large) drain
  // buffer never lands on the stack.
  static uint8_t buf[ISM_FIFO_MAX_WORDS * 7];

  // Latch-and-clear the pending flag + timestamp atomically vs. the ISR.
  noInterrupts();
  if (!imuPending) { interrupts(); return; }
  imuPending  = false;
  int64_t ts  = imuIsrTimestamp;
  interrupts();

  // If the FIFO ever overran, data was lost -> count it, then drain.
  if (imuFifoOverrun()) imuMissedCount += 1;

  uint16_t words = imuFifoLevel();
  if (words == 0) return;
  if (words > ISM_FIFO_MAX_WORDS) words = ISM_FIFO_MAX_WORDS;

  // Burst the whole batch: tag+data are auto-incremented from 0x78.
  imuReadBurst(ISM_FIFO_DATA_OUT_TAG, buf, (size_t)words * 7);

  // Demultiplex by tag.  Accel & gyro arrive as separate words; pair them
  // into one ImuSample.  We hold the most recent of each and publish a
  // pair whenever both have been seen, counting every completed pair.
  ImuSample s = {};
  s.t_us = ts;
  bool haveAcc = false, haveGyr = false;
  uint32_t pairs = 0;

  for (uint16_t i = 0; i < words; ++i) {
    const uint8_t *w   = &buf[(size_t)i * 7];
    uint8_t        tag = w[0] >> 3;          // tag_sensor = bits 7:3
    int16_t        x   = (int16_t)(w[1] | (w[2] << 8));
    int16_t        y   = (int16_t)(w[3] | (w[4] << 8));
    int16_t        z   = (int16_t)(w[5] | (w[6] << 8));

    if (tag == ISM_TAG_XL_NC) {
      s.ax = x; s.ay = y; s.az = z; haveAcc = true;
    } else if (tag == ISM_TAG_GYRO_NC) {
      s.gx = x; s.gy = y; s.gz = z; haveGyr = true;
    } else {
      continue;                              // ignore temp/ts/other tags
    }

    if (haveAcc && haveGyr) {                // one full accel+gyro pair
      pairs += 1;
      haveAcc = haveGyr = false;
    }
  }

  if (pairs == 0) return;
  latestImu       = s;                        // newest fully-paired sample
  imuSampleCount += pairs;
}

// Read one MAG sample if a DRDY interrupt is pending.
static void serviceMag() {
  static uint32_t magEdgesHandled = 0;        // how many edges we've serviced

  // Latch-and-clear the pending flag + snapshot the edge count & timestamp.
  noInterrupts();
  if (!magPending) { interrupts(); return; }
  magPending      = false;
  int64_t  ts     = magIsrTimestamp;
  uint32_t edges  = magIsrEdges;
  interrupts();

  uint8_t buf[6];
  magReadBurst(MAG_OUT_X_L, buf, sizeof(buf));

  MagSample s;
  s.t_us = ts;
  s.mx   = (int16_t)(buf[0] | (buf[1] << 8));
  s.my   = (int16_t)(buf[2] | (buf[3] << 8));
  s.mz   = (int16_t)(buf[4] | (buf[5] << 8));

  latestMag       = s;
  magSampleCount += 1;

  // Any edges that arrived beyond the one we just serviced were lost
  // (single-sample part, no FIFO) -> count them as missed.
  uint32_t newlyMissed = (edges - magEdgesHandled) - 1;
  if (newlyMissed > 0) magMissedCount += newlyMissed;
  magEdgesHandled = edges;
}

// =========================== Periodic, non-blocking prints ==============
// All timing is in microseconds via esp_timer_get_time() (int64), so the
// interval gate and ODR window never wrap for any practical run length.
static constexpr int64_t PRINT_INTERVAL_US = 2500000;  // 2.5 s
static int64_t  lastPrintUs          = 0;
static uint32_t lastImuCountSnapshot = 0;
static uint32_t lastMagCountSnapshot = 0;

void printODR() {
  int64_t now = esp_timer_get_time();
  if (now - lastPrintUs < PRINT_INTERVAL_US) return;    // non-blocking gate

  uint32_t imuN    = imuSampleCount;
  uint32_t imuMiss = imuMissedCount;
  uint32_t magN    = magSampleCount;
  uint32_t magMiss = magMissedCount;

  float    dt   = (now - lastPrintUs) / 1000000.0f;
  uint32_t fImu = (uint32_t)lroundf((imuN - lastImuCountSnapshot) / dt);
  uint32_t fMag = (uint32_t)lroundf((magN - lastMagCountSnapshot) / dt);

  lastImuCountSnapshot = imuN;
  lastMagCountSnapshot = magN;
  lastPrintUs          = now;

  // Spec format, plus a missed-sample diagnostic so you can verify
  // losslessness in real time. For the IMU "missed" = FIFO overrun events;
  // for the MAG it = DRDY edges dropped while busy. Drop the bracketed
  // suffix if undesired.
  Serial.printf("IMU ODR: %u Hz | MAG ODR: %u Hz   [missed: IMU=%u MAG=%u]\n",
                (unsigned)fImu, (unsigned)fMag,
                (unsigned)imuMiss, (unsigned)magMiss);
}

void printVals() {
  int64_t now = esp_timer_get_time();
  if (now - lastPrintUs < PRINT_INTERVAL_US) return;    // non-blocking gate
  lastPrintUs = now;

  ImuSample i = latestImu;
  MagSample m = latestMag;

  Serial.printf(
    "Accel(g):  %+7.3f %+7.3f %+7.3f | "
    "Gyro(dps): %+8.2f %+8.2f %+8.2f | "
    "Mag(gauss):%+7.3f %+7.3f %+7.3f\n",
    i.ax * ISM_ACC_LSB_TO_G,  i.ay * ISM_ACC_LSB_TO_G,  i.az * ISM_ACC_LSB_TO_G,
    i.gx * ISM_GYR_LSB_TO_DPS,i.gy * ISM_GYR_LSB_TO_DPS,i.gz * ISM_GYR_LSB_TO_DPS,
    m.mx * MAG_LSB_TO_GAUSS,  m.my * MAG_LSB_TO_GAUSS,  m.mz * MAG_LSB_TO_GAUSS);
}

// =========================== Chip init ==================================
static bool waitForWhoAmI(int csPin, uint8_t expected, const char *name) {
  for (int attempt = 0; attempt < 10; ++attempt) {
    uint8_t who = spiReadReg(csPin, 0x0F);
    Serial.printf("%s WHO_AM_I attempt %d = 0x%02X (expected 0x%02X)\n",
                  name, attempt, who, expected);
    if (who == expected) return true;
    delay(5);
  }
  return false;
}

static bool initImu() {
  delay(50);  // Give IMU time to boot up
  if (!waitForWhoAmI(IMU_CS_PIN, 0x6B, "ISM330DHCX")) return false;

  // Core config: BDU + auto-increment, then accel/gyro ODR & full-scale.
  spiWriteReg(IMU_CS_PIN, ISM_CTRL3_C,          ISM_CTRL3_C_VAL);
  spiWriteReg(IMU_CS_PIN, ISM_CTRL1_XL,         ISM_CTRL1_XL_VAL);
  spiWriteReg(IMU_CS_PIN, ISM_CTRL2_G,          ISM_CTRL2_G_VAL);
  spiWriteReg(IMU_CS_PIN, ISM_COUNTER_BDR_REG1, ISM_COUNTER_BDR_VAL);

  // FIFO config: set watermark, batch accel+gyro at 6.66 kHz, route the
  // watermark IRQ to INT1, then put the FIFO in Continuous (Stream) mode.
  // FIFO_MODE is written LAST so batching starts from a known-empty state.
  spiWriteReg(IMU_CS_PIN, ISM_FIFO_CTRL1,       ISM_FIFO_CTRL1_VAL);
  spiWriteReg(IMU_CS_PIN, ISM_FIFO_CTRL2,       ISM_FIFO_CTRL2_VAL);
  spiWriteReg(IMU_CS_PIN, ISM_FIFO_CTRL3,       ISM_FIFO_CTRL3_VAL);
  spiWriteReg(IMU_CS_PIN, ISM_INT1_CTRL,        ISM_INT1_CTRL_VAL);
  spiWriteReg(IMU_CS_PIN, ISM_FIFO_CTRL4,       ISM_FIFO_CTRL4_VAL);
  return true;
}

static bool initMag() {
  delay(10);
  if (!waitForWhoAmI(MAG_CS_PIN, 0x3D, "LIS3MDL   ")) return false;

  spiWriteReg(MAG_CS_PIN, MAG_CTRL_REG2, MAG_CTRL_REG2_VAL);
  spiWriteReg(MAG_CS_PIN, MAG_CTRL_REG4, MAG_CTRL_REG4_VAL);
  spiWriteReg(MAG_CS_PIN, MAG_CTRL_REG5, MAG_CTRL_REG5_VAL);
  spiWriteReg(MAG_CS_PIN, MAG_CTRL_REG1, MAG_CTRL_REG1_VAL);
  spiWriteReg(MAG_CS_PIN, MAG_CTRL_REG3, MAG_CTRL_REG3_VAL); // last: go continuous
  return true;
}

// =========================== setup / loop ===============================
void setup() {
  Serial.begin(115200);
  while (!Serial && esp_timer_get_time() < 2000000) {}
  Serial.println("\nISM330DHCX (FIFO watermark) + LIS3MDL (DRDY)  [SPI, bare-metal]");

  pinMode(IMU_CS_PIN, OUTPUT);  digitalWrite(IMU_CS_PIN, HIGH);
  pinMode(MAG_CS_PIN, OUTPUT);  digitalWrite(MAG_CS_PIN, HIGH);

  SPI.begin(13, 11, 12);   // Nano ESP32 default: SCK=D13, MISO=D12, MOSI=D11

  if (!initImu()) { Serial.println("IMU init failed - halting"); while (1) delay(1000); }
  if (!initMag()) { Serial.println("MAG init failed - halting"); while (1) delay(1000); }

  pinMode(IMU_INT_PIN, INPUT);
  pinMode(MAG_INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), imuIsr, RISING);
  attachInterrupt(digitalPinToInterrupt(MAG_INT_PIN), magIsr, RISING);

  // -------- Prime the IMU FIFO watermark line --------
  // INT1 is level/pulse-driven off the watermark flag.  If the FIFO already
  // holds >= WTM words at boot, the LOW->HIGH edge may have happened before
  // attachInterrupt() was armed.  Force one service pass so loop() drains any
  // pre-existing batch; steady-state IRQs take over from there.
  imuPending = true;

  // -------- Prime the LIS3MDL DRDY line --------
  // The LIS3MDL's DRDY pin is level-triggered.  Reading the output registers
  // drops DRDY LOW; the next sample (1 ms later) drives the LOW->HIGH edge
  // we need.  Loop a few times to close the small race window where a new
  // sample lands during the read.
  {
    uint8_t dummy[6];
    for (int i = 0; i < 5; ++i) {
      magReadBurst(MAG_OUT_X_L, dummy, sizeof(dummy));
      delayMicroseconds(50);
      if (digitalRead(MAG_INT_PIN) == LOW) break;
    }
  }
  // ---------------------------------------------

  // Clear any priming-induced flags/edges so the first ODR window is clean.
  noInterrupts();
  imuPending   = false;
  magPending   = false;
  magIsrEdges  = 0;
  interrupts();

  // Baseline for the first ODR window so the first reading is accurate.
  lastImuCountSnapshot = imuSampleCount;
  lastMagCountSnapshot = magSampleCount;
  lastPrintUs          = esp_timer_get_time();
}

void loop() {
  // Service the sensors first, IMU before MAG (the FIFO has the hard
  // overrun deadline; a late MAG read just drops a sample, which the
  // missed-counter records).  Both are no-ops when nothing is pending.
  serviceImu();
  serviceMag();

  // Then the periodic Serial output. Comment out exactly ONE at a time:
  // printODR();
  printVals();
}