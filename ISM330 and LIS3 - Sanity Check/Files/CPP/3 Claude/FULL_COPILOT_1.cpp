/****************************************************************************
 * Arduino Nano ESP32  <->  Adafruit ISM330DHCX + LIS3MDL FeatherWing (4569)
 * High-rate, interrupt-driven, lossless SPI sample acquisition.
 *
 * Architecture
 *   - DRDY pin of each chip is wired to a Nano-ESP32 GPIO.
 *   - A minimal IRAM ISR captures esp_timer_get_time() (1 us resolution)
 *     and gives a FreeRTOS notification to a per-chip sensor task that is
 *     pinned to core 0.
 *   - The sensor task performs the SPI burst read of the output registers,
 *     parses the int16 axes, then publishes a "latest sample" snapshot and
 *     bumps a counter (both protected by a portMUX critical section).
 *   - loop() runs on core 1 and only does periodic Serial output, so
 *     printf never blocks the sensor path.
 *   - A "missed" counter tracks any DRDY edge that arrived while the
 *     previous one was still queued.  Under normal load it stays at 0,
 *     which is your empirical proof that no samples are being lost.
 *
 * Configured ODRs   :  IMU 6.66 kHz (accel+gyro)   MAG 1.00 kHz
 * Full-scale ranges :  +/-4 g, +/-2000 dps, +/-4 gauss
 ****************************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

// =========================== Pin map ====================================
static constexpr int IMU_CS_PIN  = A0;  // ISM330DHCX CS
static constexpr int MAG_CS_PIN  = A1;  // LIS3MDL    CS
static constexpr int IMU_INT_PIN = A2;  // ISM330DHCX INT1 (DRDY)
static constexpr int MAG_INT_PIN = A3;  // LIS3MDL    DRDY

// =========================== SPI bus ====================================
// Both parts: SPI mode 3, MSB first, <=10 MHz.  8 MHz leaves headroom.
static const SPISettings kSpi(8000000, MSBFIRST, SPI_MODE3);

// =========================== ISM330DHCX registers =======================
static constexpr uint8_t ISM_WHO_AM_I         = 0x0F;  // expect 0x6B
static constexpr uint8_t ISM_COUNTER_BDR_REG1 = 0x0B;
static constexpr uint8_t ISM_INT1_CTRL        = 0x0D;
static constexpr uint8_t ISM_CTRL1_XL         = 0x10;
static constexpr uint8_t ISM_CTRL2_G          = 0x11;
static constexpr uint8_t ISM_CTRL3_C          = 0x12;
static constexpr uint8_t ISM_OUTX_L_G         = 0x22;  // 12 bytes: gyro XYZ, accel XYZ

// CTRL1_XL : ODR_XL=1010 (6.66 kHz), FS_XL=10 (+/-4 g), LPF2_XL=0
static constexpr uint8_t ISM_CTRL1_XL_VAL     = 0xA8;
// CTRL2_G  : ODR_G =1010 (6.66 kHz), FS_G =11 (+/-2000 dps), FS_125=0
static constexpr uint8_t ISM_CTRL2_G_VAL      = 0xAC;
// CTRL3_C  : BDU=1, IF_INC=1 (block-data-update + register auto-increment)
static constexpr uint8_t ISM_CTRL3_C_VAL      = 0x44;
// INT1_CTRL: INT1_DRDY_XL=1 (accel DRDY drives INT1; gyro DRDY is
// synchronous because both ODRs are identical).
static constexpr uint8_t ISM_INT1_CTRL_VAL    = 0x01;
// COUNTER_BDR_REG1: DATAREADY_PULSED=1 -> DRDY is a 75 us pulse per sample
// (prevents the pin from getting "stuck high" if a sample is ever missed).
static constexpr uint8_t ISM_COUNTER_BDR_VAL  = 0x80;

// raw int16 -> physical units
static constexpr float ISM_ACC_LSB_TO_G   = 0.000122f;  // +/-4 g
static constexpr float ISM_GYR_LSB_TO_DPS = 0.070f;     // +/-2000 dps

// =========================== LIS3MDL registers ==========================
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
// Multi-core safe via portMUX critical sections.
static portMUX_TYPE imuMux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE magMux = portMUX_INITIALIZER_UNLOCKED;

static ImuSample latestImu      = {};
static MagSample latestMag      = {};
static uint32_t  imuSampleCount = 0;   // every successful IMU read
static uint32_t  magSampleCount = 0;   // every successful MAG read
static uint32_t  imuMissedCount = 0;   // DRDYs that arrived while busy
static uint32_t  magMissedCount = 0;

// Written by ISR, read by task.  Memory ordering is provided by the
// FreeRTOS task notification that follows the write.
static volatile int64_t imuIsrTimestamp = 0;
static volatile int64_t magIsrTimestamp = 0;

static TaskHandle_t imuTaskHandle = nullptr;
static TaskHandle_t magTaskHandle = nullptr;

// =========================== SPI helpers ================================
// NOTE: SPI.beginTransaction()/endTransaction() on the Arduino-ESP32 core
// already serialize bus access across tasks, so no extra mutex is needed
// even though the two sensor tasks share the bus.
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
static void IRAM_ATTR imuIsr() {
  imuIsrTimestamp = esp_timer_get_time();
  BaseType_t hpw = pdFALSE;
  vTaskNotifyGiveFromISR(imuTaskHandle, &hpw);
  if (hpw) portYIELD_FROM_ISR();
}
static void IRAM_ATTR magIsr() {
  magIsrTimestamp = esp_timer_get_time();
  BaseType_t hpw = pdFALSE;
  vTaskNotifyGiveFromISR(magTaskHandle, &hpw);
  if (hpw) portYIELD_FROM_ISR();
}

// =========================== Sensor tasks ===============================
static void imuTask(void *) {
  uint8_t buf[12];
  for (;;) {
    uint32_t pending = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    int64_t  ts      = imuIsrTimestamp;
    imuReadBurst(ISM_OUTX_L_G, buf, sizeof(buf));     // gyro XYZ + accel XYZ

    ImuSample s;
    s.t_us = ts;
    s.gx   = (int16_t)(buf[0]  | (buf[1]  << 8));
    s.gy   = (int16_t)(buf[2]  | (buf[3]  << 8));
    s.gz   = (int16_t)(buf[4]  | (buf[5]  << 8));
    s.ax   = (int16_t)(buf[6]  | (buf[7]  << 8));
    s.ay   = (int16_t)(buf[8]  | (buf[9]  << 8));
    s.az   = (int16_t)(buf[10] | (buf[11] << 8));

    portENTER_CRITICAL(&imuMux);
    latestImu = s;
    imuSampleCount += 1;
    if (pending > 1) imuMissedCount += (pending - 1);
    portEXIT_CRITICAL(&imuMux);
  }
}
static void magTask(void *) {
  uint8_t buf[6];
  for (;;) {
    uint32_t pending = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    int64_t  ts      = magIsrTimestamp;
    magReadBurst(MAG_OUT_X_L, buf, sizeof(buf));

    MagSample s;
    s.t_us = ts;
    s.mx   = (int16_t)(buf[0] | (buf[1] << 8));
    s.my   = (int16_t)(buf[2] | (buf[3] << 8));
    s.mz   = (int16_t)(buf[4] | (buf[5] << 8));

    portENTER_CRITICAL(&magMux);
    latestMag = s;
    magSampleCount += 1;
    if (pending > 1) magMissedCount += (pending - 1);
    portEXIT_CRITICAL(&magMux);
  }
}

// =========================== Periodic, non-blocking prints ==============
static constexpr uint32_t PRINT_INTERVAL_MS = 2500;
static uint32_t lastPrintMs          = 0;
static uint32_t lastImuCountSnapshot = 0;
static uint32_t lastMagCountSnapshot = 0;

void printODR() {
  uint32_t now = millis();
  if (now - lastPrintMs < PRINT_INTERVAL_MS) return;     // non-blocking gate

  uint32_t imuN, magN, imuMiss, magMiss;
  portENTER_CRITICAL(&imuMux);
  imuN    = imuSampleCount;
  imuMiss = imuMissedCount;
  portEXIT_CRITICAL(&imuMux);
  portENTER_CRITICAL(&magMux);
  magN    = magSampleCount;
  magMiss = magMissedCount;
  portEXIT_CRITICAL(&magMux);

  float    dt   = (now - lastPrintMs) / 1000.0f;
  uint32_t fImu = (uint32_t)lroundf((imuN - lastImuCountSnapshot) / dt);
  uint32_t fMag = (uint32_t)lroundf((magN - lastMagCountSnapshot) / dt);

  lastImuCountSnapshot = imuN;
  lastMagCountSnapshot = magN;
  lastPrintMs          = now;

  // Spec format, plus a missed-sample diagnostic so you can verify
  // losslessness in real time. Drop the bracketed suffix if undesired.
  Serial.printf("IMU ODR: %u Hz | MAG ODR: %u Hz   [missed: IMU=%u MAG=%u]\n",
                (unsigned)fImu, (unsigned)fMag,
                (unsigned)imuMiss, (unsigned)magMiss);
}

void printVals() {
  uint32_t now = millis();
  if (now - lastPrintMs < PRINT_INTERVAL_MS) return;     // non-blocking gate
  lastPrintMs = now;

  ImuSample i;
  MagSample m;
  portENTER_CRITICAL(&imuMux); i = latestImu; portEXIT_CRITICAL(&imuMux);
  portENTER_CRITICAL(&magMux); m = latestMag; portEXIT_CRITICAL(&magMux);

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

  spiWriteReg(IMU_CS_PIN, ISM_CTRL3_C,          ISM_CTRL3_C_VAL);
  spiWriteReg(IMU_CS_PIN, ISM_CTRL1_XL,         ISM_CTRL1_XL_VAL);
  spiWriteReg(IMU_CS_PIN, ISM_CTRL2_G,          ISM_CTRL2_G_VAL);
  spiWriteReg(IMU_CS_PIN, ISM_COUNTER_BDR_REG1, ISM_COUNTER_BDR_VAL);
  spiWriteReg(IMU_CS_PIN, ISM_INT1_CTRL,        ISM_INT1_CTRL_VAL);
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
  while (!Serial && millis() < 2000) {}
  Serial.println("\nISM330DHCX + LIS3MDL  (SPI, interrupt-driven)");

  pinMode(IMU_CS_PIN, OUTPUT);  digitalWrite(IMU_CS_PIN, HIGH);
  pinMode(MAG_CS_PIN, OUTPUT);  digitalWrite(MAG_CS_PIN, HIGH);

  SPI.begin();   // Nano ESP32 default: SCK=D13, MISO=D12, MOSI=D11

  if (!initImu()) { Serial.println("IMU init failed - halting"); while (1) delay(1000); }
  if (!initMag()) { Serial.println("MAG init failed - halting"); while (1) delay(1000); }

  // Sensor tasks on core 0 (PRO_CPU); loop()/Serial stays on core 1 (APP_CPU).
  xTaskCreatePinnedToCore(imuTask, "imu", 4096, nullptr,
                          configMAX_PRIORITIES - 1, &imuTaskHandle, 0);
  xTaskCreatePinnedToCore(magTask, "mag", 4096, nullptr,
                          configMAX_PRIORITIES - 2, &magTaskHandle, 0);

  pinMode(IMU_INT_PIN, INPUT);
  pinMode(MAG_INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), imuIsr, RISING);
  attachInterrupt(digitalPinToInterrupt(MAG_INT_PIN), magIsr, RISING);

  // -------- Prime the LIS3MDL DRDY line --------
  // Unlike the ISM330DHCX (which is in PULSED DRDY mode), the LIS3MDL's
  // DRDY pin is level-triggered.
  // Reading the output registers drops DRDY LOW; the next sample (1 ms
  // later) drives the LOW->HIGH edge we need. We loop a few times to
  // close the small race window where a new sample lands during the read.
  {
    uint8_t dummy[6];
    for (int i = 0; i < 5; ++i) {
      magReadBurst(MAG_OUT_X_L, dummy, sizeof(dummy));
      delayMicroseconds(50);
      if (digitalRead(MAG_INT_PIN) == LOW) break;
    }
  }
  // ---------------------------------------------

  // Baseline for the first ODR window so the first reading is accurate.
  portENTER_CRITICAL(&imuMux); lastImuCountSnapshot = imuSampleCount; portEXIT_CRITICAL(&imuMux);
  portENTER_CRITICAL(&magMux); lastMagCountSnapshot = magSampleCount; portEXIT_CRITICAL(&magMux);
  lastPrintMs = millis();
}

void loop() {
  // Comment out exactly ONE of the two lines below at a time:
  // printODR();
  printVals();
}