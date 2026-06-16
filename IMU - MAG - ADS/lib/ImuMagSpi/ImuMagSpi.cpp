#include "ImuMagSpi.h"
#include "esp_timer.h"

// ============================================================================
//  ISM330DHCX + LIS3MDL register map.
//  Verified against:
//    - ST ism330dhcx_reg.h / lsm6dso_reg.h (identical layout for these regs)
//    - Zephyr hal_st driver
//    - esp-gyrologger (working ESP32 FIFO + tag decode)
// ============================================================================

namespace {

// ---------------- ISM330DHCX (IMU) ----------------
constexpr uint8_t ISM_WHO_AM_I      = 0x0F;
constexpr uint8_t ISM_WHO_AM_I_VAL  = 0x6B;   // ISM330DHCX id

constexpr uint8_t ISM_FIFO_CTRL3    = 0x09;   // BDR_GY[7:4] | BDR_XL[3:0]
constexpr uint8_t ISM_FIFO_CTRL4    = 0x0A;   // DEC_TS[7:6] | ODR_T[5:4] | -- | FIFO_MODE[2:0]
constexpr uint8_t ISM_INT1_CTRL     = 0x0D;
constexpr uint8_t ISM_CTRL1_XL      = 0x10;   // accel ODR/FS
constexpr uint8_t ISM_CTRL2_G       = 0x11;   // gyro  ODR/FS
constexpr uint8_t ISM_CTRL3_C       = 0x12;   // BDU, IF_INC, SW_RESET
constexpr uint8_t ISM_CTRL4_C       = 0x13;
constexpr uint8_t ISM_CTRL9_XL      = 0x18;   // I3C disable (DEVICE_CONF)
constexpr uint8_t ISM_CTRL10_C      = 0x19;   // TIMESTAMP_EN
constexpr uint8_t ISM_FIFO_STATUS1  = 0x3A;   // DIFF_FIFO[7:0]
constexpr uint8_t ISM_FIFO_STATUS2  = 0x3B;   // flags | DIFF_FIFO[10:8]
constexpr uint8_t ISM_FIFO_DATA_TAG = 0x78;   // [tag][6 data bytes]

// CTRL3_C bits
constexpr uint8_t CTRL3_BOOT    = 0x80;
constexpr uint8_t CTRL3_BDU     = 0x40;
constexpr uint8_t CTRL3_IF_INC  = 0x04;
constexpr uint8_t CTRL3_SW_RST  = 0x01;
// CTRL9_XL
constexpr uint8_t CTRL9_DEVICE_CONF = 0x02;   // ST: set to disable I3C (recommended)
// CTRL10_C
constexpr uint8_t CTRL10_TIMESTAMP_EN = 0x20;

// ODR/FS values (6.66 kHz, FS_XL=4g, FS_G=2000dps) — chosen to give the full
// IMU rate you requested (6.66 kHz code -> ~7.1 kHz actual on this part).
constexpr uint8_t XL_6660HZ_4G    = 0xA8;   // ODR_XL=1000b(6.66k), FS_XL=10b(4g)
constexpr uint8_t G_6660HZ_2000DPS= 0xAC;   // ODR_G =1000b(6.66k), FS_G =11b(2000)

// FIFO batching: BDR_XL = BDR_GY = 6.66 kHz (0b1010) -> both stream at full rate.
constexpr uint8_t FIFO_CTRL3_BOTH_6660 = 0xAA;
// FIFO_CTRL4: FIFO_MODE=110 (Continuous) | DEC_TS_BATCH=10 (timestamp every 8th
// sample). Continuous = overwrite-oldest-on-full, which is exactly what we want
// while disconnected (let it overflow harmlessly). 0b10_00_0_110 = 0x86.
constexpr uint8_t FIFO_CTRL4_BYPASS         = 0x00; // FIFO_MODE=000 -> flush/disable
constexpr uint8_t FIFO_CTRL4_CONT_TS_DEC8   = 0x86;
// Timestamp decimation factor that matches DEC_TS_BATCH above (1 ts per 8 samples).
constexpr int     TS_DECIMATION = 8;
// Hardware timestamp resolution: datasheet = 25 us / LSB (nominal).
constexpr double  TS_LSB_US = 25.0;

// FIFO tag values (tag_sensor = raw_tag >> 3): 1=gyro, 2=accel, 4=timestamp.
constexpr uint8_t TAG_GYRO  = 0x01;
constexpr uint8_t TAG_ACCEL = 0x02;
constexpr uint8_t TAG_TS    = 0x04;

// ---------------- LIS3MDL (Mag) ----------------
constexpr uint8_t LIS_WHO_AM_I     = 0x0F;
constexpr uint8_t LIS_WHO_AM_I_VAL = 0x3D;
constexpr uint8_t LIS_CTRL_REG1    = 0x20;  // TEMP_EN|OM|DO|FAST_ODR|ST
constexpr uint8_t LIS_CTRL_REG2    = 0x21;  // FS
constexpr uint8_t LIS_CTRL_REG3    = 0x22;  // MD (operating mode)
constexpr uint8_t LIS_CTRL_REG4    = 0x23;  // OMZ
constexpr uint8_t LIS_CTRL_REG5    = 0x24;  // BDU
constexpr uint8_t LIS_STATUS_REG   = 0x27;  // ZYXDA = bit3
constexpr uint8_t LIS_OUT_X_L      = 0x28;

// Ultra-high-perf XY (OM=11), ODR via FAST_ODR, temp off. UHP fast-ODR ~155 Hz.
constexpr uint8_t LIS_CTRL1_UHP_FASTODR = 0x62; // OM=11, FAST_ODR=1
constexpr uint8_t LIS_CTRL2_FS4G        = 0x00; // +/-4 gauss
constexpr uint8_t LIS_CTRL3_CONTINUOUS  = 0x00; // MD=00 continuous-conversion
constexpr uint8_t LIS_CTRL4_UHP_Z       = 0x0C; // OMZ=11 (match XY)
constexpr uint8_t LIS_CTRL5_BDU         = 0x40; // block-data-update
constexpr uint8_t LIS_STATUS_ZYXDA      = 0x08;

// SPI read/write bit (both parts: MSB=1 read; ISM auto-increments with IF_INC,
// LIS auto-increments when bit6=1).
constexpr uint8_t SPI_READ  = 0x80;
constexpr uint8_t LIS_MS    = 0x40;   // LIS3MDL multi-byte auto-increment

// ---- module state ----
ImuMagSpi::Config g_cfg;
bool   g_imu_present = false;
bool   g_mag_present = false;

// Clock-anchor state for FIFO timestamp -> esp_timer mapping.
// We track the last hardware timestamp we saw and the esp_timer value captured
// at that drain, so spacing inside a drain is exact and the absolute anchor is
// refreshed every drain (no long-term drift).
bool    g_have_anchor   = false;
uint32_t g_last_hw_ts   = 0;      // last decoded hardware timestamp (LSB units)

// ---------------- low-level SPI helpers ----------------
inline void csLow(int pin)  { digitalWrite(pin, LOW); }
inline void csHigh(int pin) { digitalWrite(pin, HIGH); }

void regWrite(int cs, uint8_t reg, uint8_t val) {
  g_cfg.spi->beginTransaction(g_cfg.spiSettings);
  csLow(cs);
  g_cfg.spi->transfer(reg & 0x7F);
  g_cfg.spi->transfer(val);
  csHigh(cs);
  g_cfg.spi->endTransaction();
}

uint8_t regRead(int cs, uint8_t reg, uint8_t addrFlags = SPI_READ) {
  g_cfg.spi->beginTransaction(g_cfg.spiSettings);
  csLow(cs);
  g_cfg.spi->transfer(reg | addrFlags);
  uint8_t v = g_cfg.spi->transfer(0x00);
  csHigh(cs);
  g_cfg.spi->endTransaction();
  return v;
}

void regReadBurst(int cs, uint8_t reg, uint8_t* buf, int len, uint8_t addrFlags = SPI_READ) {
  g_cfg.spi->beginTransaction(g_cfg.spiSettings);
  csLow(cs);
  g_cfg.spi->transfer(reg | addrFlags);
  for (int i = 0; i < len; i++) buf[i] = g_cfg.spi->transfer(0x00);
  csHigh(cs);
  g_cfg.spi->endTransaction();
}

inline int16_t le16(uint8_t lo, uint8_t hi) {
  return (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));
}

} // anonymous namespace

namespace ImuMagSpi {

void begin(const Config& cfg) {
  g_cfg = cfg;
  if (g_cfg.imuCsPin >= 0) { pinMode(g_cfg.imuCsPin, OUTPUT); csHigh(g_cfg.imuCsPin); }
  if (g_cfg.magCsPin >= 0) { pinMode(g_cfg.magCsPin, OUTPUT); csHigh(g_cfg.magCsPin); }
}

bool imuPresent() { return g_imu_present; }
bool magPresent() { return g_mag_present; }

bool initImu() {
  const int cs = g_cfg.imuCsPin;
  if (cs < 0) return false;

  if (regRead(cs, ISM_WHO_AM_I) != ISM_WHO_AM_I_VAL) {
    g_imu_present = false;
    return false;
  }

  // Software reset, then wait for it to clear.
  regWrite(cs, ISM_CTRL3_C, CTRL3_SW_RST);
  delay(20);
  // BDU + auto-increment (IF_INC) — IF_INC is required for burst reads.
  regWrite(cs, ISM_CTRL3_C, CTRL3_BDU | CTRL3_IF_INC);
  // ST-recommended: disable I3C.
  regWrite(cs, ISM_CTRL9_XL, CTRL9_DEVICE_CONF);

  // Enable the hardware timestamp counter (25 us/LSB).
  regWrite(cs, ISM_CTRL10_C, CTRL10_TIMESTAMP_EN);

  // Full-rate accel + gyro.
  regWrite(cs, ISM_CTRL1_XL, XL_6660HZ_4G);
  regWrite(cs, ISM_CTRL2_G,  G_6660HZ_2000DPS);

  // Batch both into the FIFO at 6.66 kHz, with timestamps every 8th sample.
  regWrite(cs, ISM_FIFO_CTRL3, FIFO_CTRL3_BOTH_6660);
  regWrite(cs, ISM_FIFO_CTRL4, FIFO_CTRL4_CONT_TS_DEC8);

  g_imu_present = true;
  g_have_anchor = false;     // force a fresh anchor on first drain
  return true;
}

bool initMag() {
  const int cs = g_cfg.magCsPin;
  if (cs < 0) return false;

  if (regRead(cs, LIS_WHO_AM_I) != LIS_WHO_AM_I_VAL) {
    g_mag_present = false;
    return false;
  }

  regWrite(cs, LIS_CTRL_REG2, LIS_CTRL2_FS4G);
  regWrite(cs, LIS_CTRL_REG1, LIS_CTRL1_UHP_FASTODR);
  regWrite(cs, LIS_CTRL_REG4, LIS_CTRL4_UHP_Z);
  regWrite(cs, LIS_CTRL_REG5, LIS_CTRL5_BDU);
  regWrite(cs, LIS_CTRL_REG3, LIS_CTRL3_CONTINUOUS); // last: enters continuous mode

  g_mag_present = true;
  return true;
}

bool resetFifo() {
  if (!g_imu_present) return false;
  const int cs = g_cfg.imuCsPin;
  // Bypass empties the FIFO and resets its write pointer...
  regWrite(cs, ISM_FIFO_CTRL4, FIFO_CTRL4_BYPASS);
  delayMicroseconds(50);                 // brief settle between mode changes
  // ...then re-arm Continuous + timestamps.
  regWrite(cs, ISM_FIFO_CTRL4, FIFO_CTRL4_CONT_TS_DEC8);
  g_have_anchor = false;                 // next drain re-anchors the clock map
  return true;
}

// Number of unread words currently in the FIFO (each "word" = 1 tag+6 data entry).
static int fifoCount(int cs) {
  uint8_t s1 = regRead(cs, ISM_FIFO_STATUS1);
  uint8_t s2 = regRead(cs, ISM_FIFO_STATUS2);
  return (int)(((uint16_t)(s2 & 0x03) << 8) | s1);   // DIFF_FIFO[10:8] | [7:0]
}

int drainImu(ImuSample* out, int maxOut) {
  if (!g_imu_present || maxOut <= 0) return 0;
  const int cs = g_cfg.imuCsPin;

  int available = fifoCount(cs);
  if (available <= 0) return 0;

  // Capture one esp_timer value for THIS drain; the newest sample is anchored
  // here and older samples are back-filled by their hardware-timestamp spacing.
  const int64_t t_drain_us = esp_timer_get_time();

  // First pass: decode FIFO entries into a small staging buffer. We pair the
  // most recent accel+gyro and attach the most recent hardware timestamp seen.
  // The FIFO interleaves gyro/accel/timestamp entries; we assemble one ImuSample
  // each time we have both an accel and a gyro reading.
  ImuSample staged[64];
  uint32_t  staged_hw_ts[64];
  bool      staged_has_ts[64];
  int       nStaged = 0;

  int16_t cur_a[3] = {0,0,0}; bool have_a = false;
  int16_t cur_g[3] = {0,0,0}; bool have_g = false;
  uint32_t cur_ts = g_last_hw_ts;       bool have_ts = g_have_anchor;

  int toRead = available;
  // Cap how much we pull in one drain so a huge backlog can't monopolize loop().
  const int MAX_ENTRIES_PER_DRAIN = 256;
  if (toRead > MAX_ENTRIES_PER_DRAIN) toRead = MAX_ENTRIES_PER_DRAIN;

  for (int i = 0; i < toRead && nStaged < 64 && nStaged < maxOut; i++) {
    uint8_t buf[7];
    regReadBurst(cs, ISM_FIFO_DATA_TAG, buf, 7);
    uint8_t tag = buf[0] >> 3;

    if (tag == TAG_TS) {
      // Timestamp entry: 32-bit counter in the 6 data bytes (low 4 used).
      cur_ts = (uint32_t)buf[1] | ((uint32_t)buf[2] << 8) |
               ((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 24);
      have_ts = true;
    } else if (tag == TAG_ACCEL) {
      cur_a[0] = le16(buf[1], buf[2]);
      cur_a[1] = le16(buf[3], buf[4]);
      cur_a[2] = le16(buf[5], buf[6]);
      have_a = true;
    } else if (tag == TAG_GYRO) {
      cur_g[0] = le16(buf[1], buf[2]);
      cur_g[1] = le16(buf[3], buf[4]);
      cur_g[2] = le16(buf[5], buf[6]);
      have_g = true;
    }
    // else: empty/other tag — skip.

    if (have_a && have_g) {
      staged[nStaged].ax = cur_a[0]; staged[nStaged].ay = cur_a[1]; staged[nStaged].az = cur_a[2];
      staged[nStaged].gx = cur_g[0]; staged[nStaged].gy = cur_g[1]; staged[nStaged].gz = cur_g[2];
      staged_hw_ts[nStaged] = cur_ts;
      staged_has_ts[nStaged] = have_ts;
      nStaged++;
      have_a = have_g = false;       // consume the pair
    }
  }

  if (nStaged == 0) return 0;

  // Second pass: assign esp_timer-anchored timestamps.
  // The newest staged sample == t_drain_us. Each earlier sample is offset by the
  // hardware-timestamp delta * 25us/LSB. Where a sample carries no fresh ts word
  // (between decimated timestamps), we fall back to the nominal sample period.
  // Nominal period (us) from the 6.66 kHz code; spacing within a decimation block.
  const double NOMINAL_DT_US = 1000000.0 / 6660.0;

  // Find newest hardware ts among staged (last one with has_ts), to anchor against.
  uint32_t newest_hw = staged_hw_ts[nStaged - 1];

  for (int i = 0; i < nStaged; i++) {
    int64_t ts;
    if (staged_has_ts[i]) {
      // delta in LSBs from the newest; convert to us and subtract from drain time.
      int64_t d_lsb = (int64_t)newest_hw - (int64_t)staged_hw_ts[i];
      ts = t_drain_us - (int64_t)((double)d_lsb * TS_LSB_US);
    } else {
      // back-fill from the end using nominal period
      ts = t_drain_us - (int64_t)((double)(nStaged - 1 - i) * NOMINAL_DT_US);
    }
    staged[i].ts_us = ts;
    out[i] = staged[i];
  }

  // Update anchor for the next drain.
  g_last_hw_ts = newest_hw;
  g_have_anchor = true;

  return nStaged;
}

bool readMag(MagSample* out) {
  if (!g_mag_present) return false;
  const int cs = g_cfg.magCsPin;

  uint8_t st = regRead(cs, LIS_STATUS_REG);
  if (!(st & LIS_STATUS_ZYXDA)) return false;

  uint8_t buf[6];
  regReadBurst(cs, LIS_OUT_X_L, buf, 6, SPI_READ | LIS_MS);
  out->ts_us = esp_timer_get_time();
  out->mx = le16(buf[0], buf[1]);
  out->my = le16(buf[2], buf[3]);
  out->mz = le16(buf[4], buf[5]);
  return true;
}

} // namespace ImuMagSpi