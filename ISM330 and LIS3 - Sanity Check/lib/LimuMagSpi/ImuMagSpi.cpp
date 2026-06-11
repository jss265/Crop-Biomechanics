#include "ImuMagSpi.h"
#include "esp_timer.h"

namespace ImuMagSpi {
namespace {  // ====================================================== internals

// ---- ISM330DHCX register map ----
constexpr uint8_t ISM_WHO_AM_I         = 0x0F;  // expect 0x6B
constexpr uint8_t ISM_COUNTER_BDR_REG1 = 0x0B;
constexpr uint8_t ISM_INT1_CTRL        = 0x0D;
constexpr uint8_t ISM_CTRL1_XL         = 0x10;
constexpr uint8_t ISM_CTRL2_G          = 0x11;
constexpr uint8_t ISM_CTRL3_C          = 0x12;
constexpr uint8_t ISM_OUTX_L_G         = 0x22;  // 12-byte burst: gyro XYZ, accel XYZ
constexpr uint8_t ISM_CTRL1_XL_VAL     = 0xA8;  // 6.66 kHz, +/-4 g
constexpr uint8_t ISM_CTRL2_G_VAL      = 0xAC;  // 6.66 kHz, +/-2000 dps
constexpr uint8_t ISM_CTRL3_C_VAL      = 0x44;  // BDU + IF_INC
constexpr uint8_t ISM_INT1_CTRL_VAL    = 0x01;  // INT1_DRDY_XL
constexpr uint8_t ISM_COUNTER_BDR_VAL  = 0x80;  // DATAREADY_PULSED

// ---- LIS3MDL register map ----
constexpr uint8_t MAG_WHO_AM_I    = 0x0F;  // expect 0x3D
constexpr uint8_t MAG_CTRL_REG1   = 0x20;
constexpr uint8_t MAG_CTRL_REG2   = 0x21;
constexpr uint8_t MAG_CTRL_REG3   = 0x22;
constexpr uint8_t MAG_CTRL_REG4   = 0x23;
constexpr uint8_t MAG_CTRL_REG5   = 0x24;
constexpr uint8_t MAG_OUT_X_L     = 0x28;  // 6-byte burst: mag XYZ
constexpr uint8_t MAG_CTRL_REG1_VAL = 0x02;  // LP XY, FAST_ODR -> 1000 Hz
constexpr uint8_t MAG_CTRL_REG2_VAL = 0x00;  // +/-4 gauss
constexpr uint8_t MAG_CTRL_REG3_VAL = 0x00;  // continuous-conversion
constexpr uint8_t MAG_CTRL_REG4_VAL = 0x00;  // LP Z
constexpr uint8_t MAG_CTRL_REG5_VAL = 0x40;  // BDU

// ---- Module state ----
Config            g_cfg;
bool              g_configured   = false;
bool              g_imuInited    = false;
bool              g_magInited    = false;

TaskHandle_t      g_imuTask      = nullptr;
TaskHandle_t      g_magTask      = nullptr;
QueueHandle_t     g_imuQueue     = nullptr;
QueueHandle_t     g_magQueue     = nullptr;

portMUX_TYPE      g_mux          = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t g_imuCount     = 0;
volatile uint32_t g_magCount     = 0;
volatile uint32_t g_imuDropped   = 0;
volatile uint32_t g_magDropped   = 0;

volatile int64_t  g_imuIsrTs     = 0;
volatile int64_t  g_magIsrTs     = 0;

// ---- SPI helpers (use g_cfg.spi + g_cfg.spiSettings) ----
inline void wr(int cs, uint8_t reg, uint8_t val) {
  g_cfg.spi->beginTransaction(g_cfg.spiSettings);
  digitalWrite(cs, LOW);
  g_cfg.spi->transfer(reg & 0x7F);
  g_cfg.spi->transfer(val);
  digitalWrite(cs, HIGH);
  g_cfg.spi->endTransaction();
}
inline uint8_t rd(int cs, uint8_t reg) {
  g_cfg.spi->beginTransaction(g_cfg.spiSettings);
  digitalWrite(cs, LOW);
  g_cfg.spi->transfer(reg | 0x80);
  uint8_t v = g_cfg.spi->transfer(0x00);
  digitalWrite(cs, HIGH);
  g_cfg.spi->endTransaction();
  return v;
}
inline void imuBurst(uint8_t reg, uint8_t* buf, size_t n) {
  g_cfg.spi->beginTransaction(g_cfg.spiSettings);
  digitalWrite(g_cfg.imuCsPin, LOW);
  g_cfg.spi->transfer(reg | 0x80);              // ISM: only bit7=1 for read
  g_cfg.spi->transferBytes(nullptr, buf, n);
  digitalWrite(g_cfg.imuCsPin, HIGH);
  g_cfg.spi->endTransaction();
}
inline void magBurst(uint8_t reg, uint8_t* buf, size_t n) {
  g_cfg.spi->beginTransaction(g_cfg.spiSettings);
  digitalWrite(g_cfg.magCsPin, LOW);
  g_cfg.spi->transfer(reg | 0xC0);              // LIS3MDL: bit7=1 AND bit6=MS=1
  g_cfg.spi->transferBytes(nullptr, buf, n);
  digitalWrite(g_cfg.magCsPin, HIGH);
  g_cfg.spi->endTransaction();
}

bool waitWho(int cs, uint8_t expected) {
  for (int i = 0; i < 10; ++i) {
    if (rd(cs, 0x0F) == expected) return true;  // both chips: WHO_AM_I @ 0x0F
    delay(5);
  }
  return false;
}

// ---- Sensor tasks ----
void imuTask(void*) {
  uint8_t buf[12];
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ImuSample s;
    s.ts_us = (uint64_t)g_imuIsrTs;
    imuBurst(ISM_OUTX_L_G, buf, sizeof(buf));
    s.gx = (int16_t)(buf[0]  | (buf[1]  << 8));
    s.gy = (int16_t)(buf[2]  | (buf[3]  << 8));
    s.gz = (int16_t)(buf[4]  | (buf[5]  << 8));
    s.ax = (int16_t)(buf[6]  | (buf[7]  << 8));
    s.ay = (int16_t)(buf[8]  | (buf[9]  << 8));
    s.az = (int16_t)(buf[10] | (buf[11] << 8));

    if (xQueueSend(g_imuQueue, &s, 0) == pdTRUE) {
      portENTER_CRITICAL(&g_mux); g_imuCount   += 1; portEXIT_CRITICAL(&g_mux);
    } else {
      portENTER_CRITICAL(&g_mux); g_imuDropped += 1; portEXIT_CRITICAL(&g_mux);
    }
  }
}
void magTask(void*) {
  uint8_t buf[6];
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    MagSample s;
    s.ts_us = (uint64_t)g_magIsrTs;
    magBurst(MAG_OUT_X_L, buf, sizeof(buf));
    s.mx = (int16_t)(buf[0] | (buf[1] << 8));
    s.my = (int16_t)(buf[2] | (buf[3] << 8));
    s.mz = (int16_t)(buf[4] | (buf[5] << 8));

    if (xQueueSend(g_magQueue, &s, 0) == pdTRUE) {
      portENTER_CRITICAL(&g_mux); g_magCount   += 1; portEXIT_CRITICAL(&g_mux);
    } else {
      portENTER_CRITICAL(&g_mux); g_magDropped += 1; portEXIT_CRITICAL(&g_mux);
    }
  }
}

} // anonymous namespace
// ================================================================= public API

void begin(const Config& cfg) {
  g_cfg        = cfg;
  g_configured = true;
}

bool initImu() {
  if (!g_configured || g_cfg.imuCsPin < 0) return false;
  pinMode(g_cfg.imuCsPin, OUTPUT);
  digitalWrite(g_cfg.imuCsPin, HIGH);   // deselect BEFORE any SPI activity
  delay(50);                            // ISM330DHCX power-on settling
  if (!waitWho(g_cfg.imuCsPin, 0x6B)) return false;

  wr(g_cfg.imuCsPin, ISM_CTRL3_C,          ISM_CTRL3_C_VAL);
  wr(g_cfg.imuCsPin, ISM_CTRL1_XL,         ISM_CTRL1_XL_VAL);
  wr(g_cfg.imuCsPin, ISM_CTRL2_G,          ISM_CTRL2_G_VAL);
  wr(g_cfg.imuCsPin, ISM_COUNTER_BDR_REG1, ISM_COUNTER_BDR_VAL);
  wr(g_cfg.imuCsPin, ISM_INT1_CTRL,        ISM_INT1_CTRL_VAL);
  g_imuInited = true;
  return true;
}

bool initMag() {
  if (!g_configured || g_cfg.magCsPin < 0) return false;
  pinMode(g_cfg.magCsPin, OUTPUT);
  digitalWrite(g_cfg.magCsPin, HIGH);
  delay(10);
  if (!waitWho(g_cfg.magCsPin, 0x3D)) return false;

  wr(g_cfg.magCsPin, MAG_CTRL_REG2, MAG_CTRL_REG2_VAL);
  wr(g_cfg.magCsPin, MAG_CTRL_REG4, MAG_CTRL_REG4_VAL);
  wr(g_cfg.magCsPin, MAG_CTRL_REG5, MAG_CTRL_REG5_VAL);
  wr(g_cfg.magCsPin, MAG_CTRL_REG1, MAG_CTRL_REG1_VAL);
  wr(g_cfg.magCsPin, MAG_CTRL_REG3, MAG_CTRL_REG3_VAL);  // last: continuous mode
  g_magInited = true;
  return true;
}

bool startTasks() {
  if (g_imuInited && !g_imuTask) {
    g_imuQueue = xQueueCreate(g_cfg.imuQueueDepth, sizeof(ImuSample));
    if (!g_imuQueue) return false;
    if (xTaskCreatePinnedToCore(imuTask, "imu_spi",
            g_cfg.imuTaskStack, nullptr,
            g_cfg.imuTaskPriority, &g_imuTask, g_cfg.taskCore) != pdPASS)
      return false;
  }
  if (g_magInited && !g_magTask) {
    g_magQueue = xQueueCreate(g_cfg.magQueueDepth, sizeof(MagSample));
    if (!g_magQueue) return false;
    if (xTaskCreatePinnedToCore(magTask, "mag_spi",
            g_cfg.magTaskStack, nullptr,
            g_cfg.magTaskPriority, &g_magTask, g_cfg.taskCore) != pdPASS)
      return false;

    // Prime the LIS3MDL DRDY line: it is level-triggered (no pulsed mode),
    // so it has been stuck HIGH since continuous mode started. Reading the
    // output registers drops it LOW; the next sample provides the RISING
    // edge our ISR is waiting for.
    if (g_cfg.magIntPin >= 0) {
      uint8_t dummy[6];
      for (int i = 0; i < 5; ++i) {
        magBurst(MAG_OUT_X_L, dummy, sizeof(dummy));
        delayMicroseconds(50);
        if (digitalRead(g_cfg.magIntPin) == LOW) break;
      }
    }
  }
  return true;
}

void IRAM_ATTR onImuDrdyFromIsr(int64_t ts_us) {
  g_imuIsrTs = ts_us;
  BaseType_t hpw = pdFALSE;
  vTaskNotifyGiveFromISR(g_imuTask, &hpw);
  if (hpw) portYIELD_FROM_ISR();
}
void IRAM_ATTR onMagDrdyFromIsr(int64_t ts_us) {
  g_magIsrTs = ts_us;
  BaseType_t hpw = pdFALSE;
  vTaskNotifyGiveFromISR(g_magTask, &hpw);
  if (hpw) portYIELD_FROM_ISR();
}

QueueHandle_t getImuQueue() { return g_imuQueue; }
QueueHandle_t getMagQueue() { return g_magQueue; }

void kickMag() {
  // Safe to call from normal task context (uses xTaskNotifyGive, not FromISR).
  // Wakes the mag SPI task once so it does a burst read and lowers the DRDY
  // line; subsequent samples then generate clean RISING edges for the ISR.
  if (g_magTask) xTaskNotifyGive(g_magTask);
}

uint32_t getImuSampleCount()  { return g_imuCount;   }
uint32_t getMagSampleCount()  { return g_magCount;   }
uint32_t getImuDroppedCount() { return g_imuDropped; }
uint32_t getMagDroppedCount() { return g_magDropped; }

} // namespace ImuMagSpi