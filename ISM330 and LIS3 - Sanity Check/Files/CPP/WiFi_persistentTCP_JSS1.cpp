//////////////////////////////////////////////////////////////////////////////////////////
//
//    Arduino code for reading from up to 5 ADS1220 chips PLUS an ISM330DHCX IMU and a
//    LIS3MDL magnetometer on an Arduino Nano ESP32. All sensors share the same SPI bus
//    (CS/DRDY are independent). Each ADS1220 reads 2 differential channels (AIN0-AIN1
//    and AIN2-AIN3), labeled as sensors A-E with channels 1-2. The IMU and magnetometer
//    are serviced by the ImuMagSpi wrapper (Adafruit ISM330DHCX + LIS3MDL FeatherWing).
//
//    Architecture (June 2026 rewrite):
//    - Per-record typed packets: every DRDY produces ONE binary record (PKT_ADS, PKT_IMU
//      or PKT_MAG). The host's collect_data.py demultiplexes by record-type and adc_id,
//      so we no longer have to wait for "all sensors ready" before emitting anything.
//    - Records are concatenated into the payload of one outer frame
//      [len:2][seq:2][crc:2][payload] every BATCH_SEND_INTERVAL_MS (10 ms / 100 Hz),
//      preserving the existing CRC/seq framing the Python WiFiDataServer expects.
//    - ISR-Lite: every DRDY ISR does TWO things and exits (capture esp_timer_get_time()
//      microsecond timestamp + vTaskNotifyGiveFromISR a dedicated task). All SPI reads
//      happen in task context so they cooperate with the IMU/Mag tasks through
//      SPI.beginTransaction()'s bus mutex.
//
//    Key Decisions and Justifications (Updated for 2026 ISR-Lite + IMU/Mag Integration):
//    - Modular Design:
//      - Same NUM_SENSORS + all_configs[MAX_SENSORS] mechanism as before. To use fewer
//        ADS chips (e.g., A-C), set NUM_SENSORS=3. The first N entries of all_configs
//        are used; the array stays in canonical A->E order so no pin/ID lookup changes.
//      - NOTE: Whatever sensors you enable here MUST match the "sensors=..." string
//        passed to run_collection() on the host. Otherwise the host's lead-sensor sync
//        will silently wait forever for a sensor that never arrives.
//    - Pin Assignments:
//      - SPI pins (Nano ESP32): D13 (SCK), D12 (MOSI/COPI), D11 (MISO/CIPO).
//        IMPORTANT: MOSI/MISO are intentionally SWAPPED (D11<->D12) here because
//        the Hi-STIFFS rev-2 PCB flips them on the physical board. The two constants
//        below feed SPI.begin() so the library follows the trace, not the silkscreen.
//      - ADS1220 CS pins: D10, D9, D8, D7, D6 (outputs).
//      - ADS1220 DRDY pins: D5, D4, D3, D2, A6 (inputs, all interrupt-capable on ESP32).
//      - IMU CS: A0. MAG CS: A1. IMU INT1 (data-ready): A2. MAG DRDY: A3. A0-A3 sit on
//        the analog header on the Hi-STIFFS PCB, are valid as digital I/O on the Nano
//        ESP32, and do not collide with any ADS pin or the SPI bus itself.
//      - Justification: zero conflicts with the SPI bus or USB Serial; every interrupt
//        line is a true ESP32 GPIO so attachInterrupt() with FALLING/RISING is safe.
//    - Operating Mode and Data Rate (ADS1220):
//      - Turbo mode (MODE_TURBO), DR_330SPS -> ~660 SPS aggregate per chip -> ~330 SPS
//        per channel after the ch1/ch2 MUX ping-pong. Unchanged from prior firmware.
//    - Ratiometric Measurement:
//      - VREF tied to AVDD (VREF_ANALOG) and PGA=128. Raw sign-extended 24-bit counts
//        are shipped; any mV/N/mm conversion happens on the host. Unchanged.
//    - Synchronization Within Each Sensor:
//      - Continuous mode + alternating MUX (AIN0-1 then AIN2-3) on each DRDY. The pair
//        is timestamped exactly once, when ch2 is read, then emitted as one PKT_ADS.
//        Unchanged in spirit; the timestamping moved from micros() (32-bit) inside an
//        ISR to esp_timer_get_time() (64-bit) captured in the ISR and consumed in the
//        per-chip reader task.
//    - Synchronization Between Sensors:
//      - Still none on-chip. We rely on each record's per-record ts_us so the host can
//        align streams in post.
//    - Output Format (Wireless):
//      - PKT_ADS / PKT_IMU / PKT_MAG records (packed, little-endian, see PacketType
//        block below). Records are concatenated into one outer frame per batch.
//      - Frame: [len:2][seq:2][crc:2][payload]. CRC-16/CCITT (poly 0xA001).
//      - One TCP frame every BATCH_SEND_INTERVAL_MS (10 ms / 100 Hz).
//      - Persistent TCP to host_ip:host_port. Serial output for ADS retained as a
//        debug aid; IMU/Mag are intentionally NOT echoed to serial (would flood at
//        6.66 kHz and would gate the loop on UART throughput).
//    - Wireless Features:
//      - Station mode + persistent TCP + optional OTA (only when disconnected).
//    - ISR-Lite Pattern (NEW, applies to ALL sensors now):
//      - Each DRDY ISR does TWO things and exits: (1) capture a 64-bit microsecond
//        timestamp with esp_timer_get_time(), (2) vTaskNotifyGiveFromISR() to wake a
//        pinned-priority FreeRTOS task that owns that chip's SPI traffic.
//      - Justification: keeps the ISR < 1 us, avoids taking the SPI bus mutex from
//        interrupt context (which would race the IMU/Mag tasks that now share the bus),
//        and lets the FreeRTOS scheduler preempt WiFi housekeeping to service the read.
//        This mirrors the proven pattern already used by ImuMagSpi for the
//        ISM330DHCX/LIS3MDL.
//    - ImuMagSpi Integration:
//      - ImuMagSpi::begin/initImu/initMag are called from setup(); startTasks() is
//        called BEFORE attachInterrupt() for IMU/MAG so the task handle exists by the
//        time the first DRDY edge fires. We pull ImuSample/MagSample off the library's
//        queues at loop() cadence and copy them field-by-field into packed
//        Imu/MagPacket records before queuing for TCP send.
//      - Field-by-field (not memcpy(sizeof)) because the library's sample structs are
//        NOT #pragma-packed: sizeof(ImuSample)==24 and sizeof(MagSample)==16 due to
//        natural alignment around the leading uint64_t. The wire packet sizes are
//        20 and 14 respectively, which only the packed Imu/MagPacket guarantee.
//    - SPI Sharing / Error Robustness:
//      - All sensor reads happen in task context, so SPI.beginTransaction()'s mutex
//        properly serializes ADS vs IMU vs MAG access.
//      - The Protocentral select_mux_channels() call is now wrapped in our own
//        beginTransaction so it cooperates with the wrapper's imuBurst/magBurst.
//      - initializeADCs() temporarily detaches the IMU/Mag interrupts on any call
//        after the first, so the unprotected ADS library register-writes inside it
//        cannot race against an in-flight imuBurst/magBurst from the wrapper tasks.
//      - Queues are drained, never blocked-on, in loop(); a stalled WiFi send cannot
//        wedge sensor acquisition because all acquisition runs on its own tasks.
//
//    Dependencies: Protocentral_ADS1220, ImuMagSpi (project-local), ArduinoOTA.
//
/////////////////////////////////////////////////////////////////////////////////////////

#include "Protocentral_ADS1220.h"
#include "ImuMagSpi.h"            // Adafruit ISM330DHCX + LIS3MDL FeatherWing wrapper
#include "esp_timer.h"            // 64-bit microsecond clock (esp_timer_get_time)
#include <SPI.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <deque>
#include <vector>
#include <cstring>                // memcpy for packed-struct serialization

// ---- Typed record layout (must mirror collect_data.py exactly) ---------------------
// These structs are #pragma-packed so sizeof() matches the Python struct format byte
// for byte (no compiler-inserted padding after adc_id, etc.). The host expects:
//   PKT_ADS -> '<QBii'    (17 bytes)
//   PKT_IMU -> '<Qhhhhhh' (20 bytes)
//   PKT_MAG -> '<Qhhh'    (14 bytes)
// Justification: hand-rolling the byte order would work but is easy to get subtly
// wrong across edits; packed structs let us memcpy a whole record in one shot, and
// the static_asserts below will refuse to compile if a future edit breaks the layout.
enum PacketType : uint8_t {
  PKT_IMU = 1,
  PKT_MAG = 2,
  PKT_ADS = 3
};
#pragma pack(push, 1)
struct AdsPacket {
  uint64_t ts_us;
  uint8_t  adc_id;   // A=0, B=1, C=2, D=3, E=4 (letter - 'A')
  int32_t  ch1;
  int32_t  ch2;
};
struct MagPacket {
  uint64_t ts_us;
  int16_t  mx, my, mz;
};
struct ImuPacket {
  uint64_t ts_us;
  int16_t  ax, ay, az;
  int16_t  gx, gy, gz;
};
#pragma pack(pop)
static_assert(sizeof(AdsPacket) == 17, "AdsPacket must be 17 bytes (host expects <QBii)");
static_assert(sizeof(ImuPacket) == 20, "ImuPacket must be 20 bytes (host expects <Qhhhhhh)");
static_assert(sizeof(MagPacket) == 14, "MagPacket must be 14 bytes (host expects <Qhhh)");

// ---- ADS1220 pin map ---------------------------------------------------------------
#define FULL_SCALE (1LL << 23) // 2^23 for 24-bit signed scaling - retained for reference
#define A_DRDY_PIN 9
#define A_CS_PIN 10
#define B_DRDY_PIN 7
#define B_CS_PIN 8
#define C_DRDY_PIN 5
#define C_CS_PIN 6
#define D_DRDY_PIN 4
#define D_CS_PIN 3
#define E_DRDY_PIN 2
#define E_CS_PIN A6

// ---- IMU / MAG pin map (FeatherWing on the Hi-STIFFS PCB) --------------------------
// Justification: A0-A3 are unused by the ADS path and are exposed on the analog header
// where the FeatherWing physically lands. All four pins are GPIO-capable on Nano ESP32.
#define IMU_CS_PIN   A0   // ISM330DHCX chip select
#define MAG_CS_PIN   A1   // LIS3MDL    chip select
#define IMU_INT_PIN  A2   // ISM330DHCX INT1 (accel/gyro data-ready, RISING)
#define MAG_DRDY_PIN A3   // LIS3MDL DRDY (RISING)

// ---- SPI bus -----------------------------------------------------------------------
// MOSI/MISO are intentionally SWAPPED (D11<->D12) to match the Hi-STIFFS PCB wiring.
const int SPI_SCK  = 13;   // default D13
const int SPI_MISO = 11;   // default D12 (CIPO)  -- swapped on PCB
const int SPI_MOSI = 12;   // default D11 (COPI)  -- swapped on PCB

const int MAX_SENSORS = 5;     // Maximum possible sensors (A to E)
const int NUM_SENSORS = 3;     // Set to 1-5 to use the first N sensors from all_configs.
                               // NOTE: Must match the host-side sensors=... string.
const uint8_t dr_code = DR_330SPS;  // Turbo mode -> ~660 SPS pairs/sec per chip
// ADS1220 SPI: 4 MHz keeps us well inside the part's SCLK timing budget while leaving
// headroom for the higher-clocked IMU/Mag traffic on the same bus.
const SPISettings spi_settings_ads(4000000, MSBFIRST, SPI_MODE1);
// IMU/Mag SPI: library default (8 MHz, MODE3). Declared here so the ImuMagSpi Config
// uses the same settings the wrapper expects.
const SPISettings spi_settings_imu(8000000, MSBFIRST, SPI_MODE3);

// ---- WiFi / TCP --------------------------------------------------------------------
const char* ssid = "Hi-STIFFS_Host";
const char* password = "BYUCropBio";
const char* ota_password = "BYUCropBio";
const char* host_ip = "192.168.137.1";
const int host_port = 8080;
WiFiClient client;

// Unique Nano ID, retained for debug prints. The new wire format no longer carries
// nano_id in every frame; the host derives it from the TCP source address / probe
// registration in collect_data.py (see WiFiDataServer.register_probe).
const char* NANO_ID = "01";

// Batching: drain everything queued every 10 ms (100 Hz TCP frames, as before).
const unsigned long BATCH_SEND_INTERVAL_MS = 10;

struct SensorConfig {
  char id;                     // Sensor ID ('A', 'B', etc.)
  int  cs_pin;                 // Chip Select pin
  int  drdy_pin;               // Data Ready pin
};

// Full list of configurations for sensors A to E. The code uses only the first
// NUM_SENSORS. To change pins or IDs, edit this array. No commenting out needed.
SensorConfig all_configs[MAX_SENSORS] = {
  {'A', A_CS_PIN, A_DRDY_PIN},
  // {'B', B_CS_PIN, B_DRDY_PIN},
  {'C', C_CS_PIN, C_DRDY_PIN},
  // {'D', D_CS_PIN, D_DRDY_PIN},
  {'E', E_CS_PIN, E_DRDY_PIN}
};

Protocentral_ADS1220 adcs[MAX_SENSORS];
int32_t          raw_values[MAX_SENSORS][2];        // [sensor][channel]
volatile int64_t pair_ts_us[MAX_SENSORS] = {0};     // ts of the most recent ch2 read
uint8_t          current_channels[MAX_SENSORS] = {0};

// ---- ISR-Lite shared state ---------------------------------------------------------
// One TaskHandle + one volatile timestamp per ADS chip. The ISR writes ts_us and pings
// the task; the task does the SPI read at normal context. esp_timer_get_time() is the
// only ISR-safe way to grab microseconds with 64-bit headroom on ESP32.
volatile int64_t  isr_ts_us[MAX_SENSORS] = {0};
TaskHandle_t      adsTaskHandle[MAX_SENSORS] = {nullptr};

// Tracks whether the IMU/Mag interrupts are currently live, so initializeADCs() knows
// whether it needs to quiesce them before touching the bus. False at first boot;
// flipped to true once setup() finishes attaching them.
static bool g_imu_mag_attached = false;
static bool g_imu_ok = false;
static bool g_mag_ok = false;

// ---- Shared packet queue (filled by ADS tasks + loop, drained by loop) -------------
// std::deque of std::vector<uint8_t> mirrors the previous packet_queue, just now
// carrying typed records ([type:1][packet]) instead of nano_id-tagged blobs.
std::deque<std::vector<uint8_t>> packet_queue;
portMUX_TYPE   queue_mux = portMUX_INITIALIZER_UNLOCKED;
uint16_t       seq_num = 0;
unsigned long  last_batch_send = 0;

const unsigned long WiFi_CHECK_INTERVAL_MS = 1000;
const unsigned long WiFi_RETRY_DELAY_MS = 5000;

// ---- FSM (unchanged shape) ---------------------------------------------------------
enum State_Serial { NO_SERIAL, HAS_SERIAL };
State_Serial SerialState;
enum State_WiFi   { CONNECTING, CONNECTED };
State_WiFi WiFiState = CONNECTING;
bool hasSerial = false;

// ---- Forward declarations ----------------------------------------------------------
uint16_t compute_crc(const uint8_t* data, size_t len);
void     enqueueRecord(const uint8_t* rec, size_t len);
void     adsTask(void* arg);
void     buildAndQueueAdsRecord(int i);
void     drainImuMagQueues();

// ====================================================================================
//                                    ISRs (one per ADS)
// ====================================================================================
// Each ISR runs in IRAM, captures the arrival time with esp_timer_get_time(), and
// notifies the dedicated reader task. The SPI burst happens in the task, NOT here.
// Justification: keeps the ISR to ~tens of cycles, avoids taking the SPI bus mutex
// from interrupt context (which would race the IMU/Mag tasks that now share the bus),
// and lets the FreeRTOS scheduler preempt WiFi housekeeping to service the read.
void IRAM_ATTR handleDrdyA() {
  isr_ts_us[0] = esp_timer_get_time();
  BaseType_t hpw = pdFALSE;
  vTaskNotifyGiveFromISR(adsTaskHandle[0], &hpw);
  if (hpw) portYIELD_FROM_ISR();
}
void IRAM_ATTR handleDrdyB() {
  isr_ts_us[1] = esp_timer_get_time();
  BaseType_t hpw = pdFALSE;
  vTaskNotifyGiveFromISR(adsTaskHandle[1], &hpw);
  if (hpw) portYIELD_FROM_ISR();
}
void IRAM_ATTR handleDrdyC() {
  isr_ts_us[2] = esp_timer_get_time();
  BaseType_t hpw = pdFALSE;
  vTaskNotifyGiveFromISR(adsTaskHandle[2], &hpw);
  if (hpw) portYIELD_FROM_ISR();
}
void IRAM_ATTR handleDrdyD() {
  isr_ts_us[3] = esp_timer_get_time();
  BaseType_t hpw = pdFALSE;
  vTaskNotifyGiveFromISR(adsTaskHandle[3], &hpw);
  if (hpw) portYIELD_FROM_ISR();
}
void IRAM_ATTR handleDrdyE() {
  isr_ts_us[4] = esp_timer_get_time();
  BaseType_t hpw = pdFALSE;
  vTaskNotifyGiveFromISR(adsTaskHandle[4], &hpw);
  if (hpw) portYIELD_FROM_ISR();
}

// IMU/Mag ISRs simply forward to the ImuMagSpi wrapper, which owns its own tasks.
// We grab the timestamp inline so the wrapper's queued samples carry the true arrival
// time of the DRDY edge, not the (slightly later) time the wrapper task wakes up.
void IRAM_ATTR handleImuDrdy() { ImuMagSpi::onImuDrdyFromIsr(esp_timer_get_time()); }
void IRAM_ATTR handleMagDrdy() { ImuMagSpi::onMagDrdyFromIsr(esp_timer_get_time()); }

// ====================================================================================
//                              ADS reader tasks (one per chip)
// ====================================================================================
// Mirrors the ImuMagSpi imuTask/magTask pattern: wait on a task notification, do the
// SPI burst, hand the finished record off to the shared packet queue. Each task is
// pinned to core 1 (WiFi lives on core 0) at high priority so a busy WiFi stack
// cannot delay an SPI read past the next DRDY edge. Priority is set just below the
// IMU task inside ImuMagSpi (configMAX_PRIORITIES - 2) so the 6.66 kHz IMU stream
// always wins arbitration if both happen to be runnable at the same instant.
void adsTask(void* arg) {
  const int i = (int)(intptr_t)arg;
  const int cs_pin = all_configs[i].cs_pin;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // --- SPI burst (3 data bytes) under the shared-bus mutex ---
    SPI.beginTransaction(spi_settings_ads);
    digitalWrite(cs_pin, LOW);
    delayMicroseconds(1);                       // tCSSC settling per datasheet
    byte SPI_Buf[3];
    SPI_Buf[0] = SPI.transfer(0);
    SPI_Buf[1] = SPI.transfer(0);
    SPI_Buf[2] = SPI.transfer(0);
    delayMicroseconds(1);
    digitalWrite(cs_pin, HIGH);
    SPI.endTransaction();

    long bits24 = (long)SPI_Buf[0] << 16 | (long)SPI_Buf[1] << 8 | SPI_Buf[2];
    int32_t val = (bits24 << 8) >> 8;           // sign-extend 24 -> 32

    // --- MUX ping-pong: ch1 read -> arm ch2, ch2 read -> arm ch1 + emit pair ---
    // The Protocentral library's select_mux_channels() drives its own CS internally,
    // so we wrap it in our own beginTransaction to keep it bus-mutex-friendly with
    // the IMU/Mag tasks. (The original ISR version got away with no wrapping because
    // nothing else used SPI; that's no longer true.)
    if (current_channels[i] == 0) {
      raw_values[i][0] = val;
      SPI.beginTransaction(spi_settings_ads);
      adcs[i].select_mux_channels(MUX_AIN2_AIN3);
      SPI.endTransaction();
      current_channels[i] = 1;
    } else {
      raw_values[i][1] = val;
      SPI.beginTransaction(spi_settings_ads);
      adcs[i].select_mux_channels(MUX_AIN0_AIN1);
      SPI.endTransaction();
      current_channels[i] = 0;
      pair_ts_us[i] = isr_ts_us[i];           // ts captured by the ISR for THIS edge
      buildAndQueueAdsRecord(i);
    }
  }
}

// Build one PKT_ADS record for sensor index i and push it onto the shared packet queue.
// The record is [type:1][AdsPacket:17] = 18 bytes; the outer frame is assembled at
// drain time so multiple records can ride in one TCP write.
void buildAndQueueAdsRecord(int i) {
  AdsPacket pkt;
  pkt.ts_us  = (uint64_t)pair_ts_us[i];
  pkt.adc_id = (uint8_t)(all_configs[i].id - 'A');   // letter-based, matches host
  pkt.ch1    = raw_values[i][0];
  pkt.ch2    = raw_values[i][1];

  uint8_t rec[1 + sizeof(AdsPacket)];
  rec[0] = (uint8_t)PKT_ADS;
  memcpy(rec + 1, &pkt, sizeof(AdsPacket));
  enqueueRecord(rec, sizeof(rec));
}

// Thread-safe push of a single typed record onto packet_queue. We hold the spinlock
// only for the std::deque mutation itself; the vector copy happens unprotected on the
// caller's stack first. Justification: keeps the critical section microseconds-short
// even when WiFi is preempting us across cores.
void enqueueRecord(const uint8_t* rec, size_t len) {
  std::vector<uint8_t> v(rec, rec + len);
  portENTER_CRITICAL(&queue_mux);
  packet_queue.push_back(std::move(v));
  portEXIT_CRITICAL(&queue_mux);
}

// ====================================================================================
//                       Pulling IMU/Mag samples out of ImuMagSpi
// ====================================================================================
// The ImuMagSpi wrapper owns its own FreeRTOS tasks and queues. We just drain those
// queues here at loop() cadence and reshape each sample into a packed Imu/MagPacket.
// IMPORTANT: ImuSample / MagSample are NOT #pragma-packed inside the library, so we
// CANNOT memcpy(sizeof(ImuSample)) -- sizeof would be 24 and 16 instead of 20 and 14.
// Assigning field-by-field into the packed packet is self-documenting and immune to
// any future library layout change.
void drainImuMagQueues() {
  QueueHandle_t imuQ = ImuMagSpi::getImuQueue();
  QueueHandle_t magQ = ImuMagSpi::getMagQueue();

  if (imuQ) {
    ImuMagSpi::ImuSample s;
    // xQueueReceive with 0 timeout drains everything available without blocking loop().
    while (xQueueReceive(imuQ, &s, 0) == pdTRUE) {
      ImuPacket pkt;
      pkt.ts_us = s.ts_us;
      pkt.ax = s.ax; pkt.ay = s.ay; pkt.az = s.az;
      pkt.gx = s.gx; pkt.gy = s.gy; pkt.gz = s.gz;
      uint8_t rec[1 + sizeof(ImuPacket)];
      rec[0] = (uint8_t)PKT_IMU;
      memcpy(rec + 1, &pkt, sizeof(ImuPacket));
      enqueueRecord(rec, sizeof(rec));
    }
  }
  if (magQ) {
    ImuMagSpi::MagSample s;
    while (xQueueReceive(magQ, &s, 0) == pdTRUE) {
      MagPacket pkt;
      pkt.ts_us = s.ts_us;
      pkt.mx = s.mx; pkt.my = s.my; pkt.mz = s.mz;
      uint8_t rec[1 + sizeof(MagPacket)];
      rec[0] = (uint8_t)PKT_MAG;
      memcpy(rec + 1, &pkt, sizeof(MagPacket));
      enqueueRecord(rec, sizeof(rec));
    }
  }
}

// ====================================================================================
//                                ADS control helpers
// ====================================================================================

// Broadcast a command to all active sensors (used for RESET in initializeADCs).
// Wrapped in beginTransaction so it cooperates with the IMU/Mag tasks when present.
void broadcast_command(uint8_t cmd) {
  SPI.beginTransaction(spi_settings_ads);
  for (int i = 0; i < NUM_SENSORS; i++) digitalWrite(all_configs[i].cs_pin, LOW);
  SPI.transfer(cmd);
  for (int i = 0; i < NUM_SENSORS; i++) digitalWrite(all_configs[i].cs_pin, HIGH);
  SPI.endTransaction();
}

void disableADCInterrupts() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    detachInterrupt(digitalPinToInterrupt(all_configs[i].drdy_pin));
  }
}

// Reattach the five fixed-index ISRs. Same justification as the prior version: a
// captured-lambda ISR on ESP32 is fragile with IRAM_ATTR; an array of plain function
// pointers keeps things compiler-friendly.
void enableADCInterrupts() {
  void (*isrHandlers[5])() = {handleDrdyA, handleDrdyB, handleDrdyC, handleDrdyD, handleDrdyE};
  for (int i = 0; i < NUM_SENSORS; i++) {
    attachInterrupt(digitalPinToInterrupt(all_configs[i].drdy_pin), isrHandlers[i], FALLING);
  }
}

// Reset and reconfigure all ADCs.
// If the IMU/Mag interrupts are already live (any call after the first), temporarily
// detach them so the unprotected library register-writes below cannot race against an
// in-flight imuBurst/magBurst from the wrapper tasks. They're reattached and the MAG
// DRDY line is re-primed before we return.
void initializeADCs() {
  if (g_imu_mag_attached) {
    if (g_imu_ok) detachInterrupt(digitalPinToInterrupt(IMU_INT_PIN));
    if (g_mag_ok) detachInterrupt(digitalPinToInterrupt(MAG_DRDY_PIN));
    delay(2);
  }

  broadcast_command(RESET);
  delay(10);                   // datasheet: post-RESET settling
  disableADCInterrupts();

  for (int i = 0; i < NUM_SENSORS; i++) {
    adcs[i].begin(all_configs[i].cs_pin, all_configs[i].drdy_pin);
    adcs[i].set_OperationMode(MODE_TURBO);
    adcs[i].set_data_rate(dr_code);
    adcs[i].PGA_ON();
    adcs[i].set_pga_gain(PGA_GAIN_128);
    adcs[i].set_VREF(VREF_ANALOG);
    adcs[i].set_conv_mode_continuous();
    adcs[i].select_mux_channels(MUX_AIN0_AIN1);
    current_channels[i] = 0;

    if (hasSerial) {
      Serial.print("Setup complete for ADS sensor [index "); Serial.print(i);
      Serial.print(", id "); Serial.print(all_configs[i].id); Serial.println("]");
      Serial.print("  CS: "); Serial.print(all_configs[i].cs_pin);
      Serial.print("  DRDY: "); Serial.println(all_configs[i].drdy_pin);
    }
    delay(100);
  }
  if (hasSerial) Serial.println("Starting datastream from ADS sensors");
  delay(100);

  // Staggered START: evenly space within one conversion cycle to minimize SPI bus
  // contention between the five chips (and now with the IMU/Mag too).
  uint16_t turbo_sps = 0;
  switch (dr_code) {
    case DR_20SPS:   turbo_sps = 40;   break;
    case DR_45SPS:   turbo_sps = 90;   break;
    case DR_90SPS:   turbo_sps = 180;  break;
    case DR_175SPS:  turbo_sps = 350;  break;
    case DR_330SPS:  turbo_sps = 660;  break;
    case DR_600SPS:  turbo_sps = 1200; break;
    case DR_1000SPS: turbo_sps = 2000; break;
    default:
      if (hasSerial) Serial.println("Invalid DR code; using default 180 SPS");
      turbo_sps = 180;
  }
  unsigned long period_us  = 1000000UL / turbo_sps;
  unsigned long stagger_us = period_us / 5;

  for (int i = 0; i < NUM_SENSORS; i++) {
    adcs[i].Start_Conv();
    if (i < NUM_SENSORS - 1) delayMicroseconds(stagger_us);
  }

  enableADCInterrupts();
  delay(50);

  // Reattach IMU/Mag interrupts and re-prime the LIS3MDL DRDY line if they were live
  // when we entered. kickMag() forces a burst-read so the level-triggered LIS3MDL
  // DRDY goes LOW; the next sample then provides the RISING edge our ISR wants.
  if (g_imu_mag_attached) {
    if (g_imu_ok) attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN),  handleImuDrdy, RISING);
    if (g_mag_ok) attachInterrupt(digitalPinToInterrupt(MAG_DRDY_PIN), handleMagDrdy, RISING);
    if (g_mag_ok) ImuMagSpi::kickMag();
  }
}

// Create one FreeRTOS task per active ADS chip. Pinned to core 1, priority high but
// below the IMU task (which runs at configMAX_PRIORITIES-2 inside ImuMagSpi) so the
// 6.66 kHz IMU stream always wins arbitration. Stack is generous at 4 kB to absorb
// any WiFi-induced jitter on the bus mutex.
bool startAdsTasks() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (adsTaskHandle[i]) continue;
    char name[12];
    snprintf(name, sizeof(name), "ads_%c", all_configs[i].id);
    BaseType_t ok = xTaskCreatePinnedToCore(
        adsTask, name, 4096, (void*)(intptr_t)i,
        configMAX_PRIORITIES - 4, &adsTaskHandle[i], 1);
    if (ok != pdPASS) {
      if (hasSerial) { Serial.print("Failed to create task for "); Serial.println(name); }
      return false;
    }
  }
  return true;
}

// Handle OTA updates (called only in non-CONNECTED states; unchanged).
void handleOTA() { ArduinoOTA.handle(); }

// Drain packet_queue into ONE outer frame [len][seq][crc][payload] and ship it.
// Returns false on send failure so the caller can stop tearing down the connection.
// Justification: one frame per batch keeps wire overhead low; the host already expects
// concatenated typed records inside each frame (see WiFiDataServer._client_loop).
bool sendBatchedFrame() {
  if (!client.connected()) {
    if (hasSerial) Serial.println("TCP not connected; skipping send");
    return false;
  }

  // Move the queued records into a local buffer under the lock, then build outside it
  // so the critical section stays sub-microsecond per record.
  std::vector<std::vector<uint8_t>> local;
  portENTER_CRITICAL(&queue_mux);
  if (packet_queue.empty()) { portEXIT_CRITICAL(&queue_mux); return true; }
  local.reserve(packet_queue.size());
  while (!packet_queue.empty()) {
    local.push_back(std::move(packet_queue.front()));
    packet_queue.pop_front();
  }
  portEXIT_CRITICAL(&queue_mux);

  // Concatenate every record into one payload.
  size_t payload_len = 0;
  for (auto& r : local) payload_len += r.size();
  if (payload_len == 0) return true;
  if (payload_len > 0xFFFF) {
    // Extremely unlikely (>65 kB queued in 10 ms), but guard the u16 length field.
    if (hasSerial) Serial.println("Payload overflow; dropping batch");
    return true;
  }

  std::vector<uint8_t> payload;
  payload.reserve(payload_len);
  for (auto& r : local) payload.insert(payload.end(), r.begin(), r.end());

  uint16_t crc = compute_crc(payload.data(), payload.size());

  // Build the outer frame [len:2][seq:2][crc:2][payload], little-endian throughout.
  std::vector<uint8_t> frame;
  frame.reserve(6 + payload_len);
  uint16_t length = (uint16_t)payload_len;
  frame.push_back((uint8_t)(length & 0xFF));
  frame.push_back((uint8_t)((length >> 8) & 0xFF));
  frame.push_back((uint8_t)(seq_num & 0xFF));
  frame.push_back((uint8_t)((seq_num >> 8) & 0xFF));
  seq_num++;
  frame.push_back((uint8_t)(crc & 0xFF));
  frame.push_back((uint8_t)((crc >> 8) & 0xFF));
  frame.insert(frame.end(), payload.begin(), payload.end());

  if (client.write(frame.data(), frame.size()) != frame.size()) {
    if (hasSerial) Serial.println("Partial or failed frame send");
    return false;
  }
  client.flush();
  return true;
}

// Optional wired-serial echo of the most recent ADS pair, for debugging only. IMU/Mag
// are intentionally NOT echoed (would flood at 6.66 kHz). Timestamps are printed in
// seconds with microsecond precision; cast to double is fine for a debug print and
// drops no useful bits inside a single session.
void sendDataSerial() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (i > 0) Serial.print(",");
    Serial.print((double)pair_ts_us[i] / 1000000.0, 6);
    Serial.print(","); Serial.print(raw_values[i][0]);
    Serial.print(","); Serial.print(raw_values[i][1]);
  }
  Serial.println();
}

// Attempt to connect to host AP, then to the TCP server. On success, reset the ADS
// chips (which re-enables their interrupts) and clear any stale packets from before
// the previous disconnect.
void connectToHost() {
  static unsigned long lastRetry = 0;
  unsigned long now = millis();
  if (now - lastRetry >= WiFi_RETRY_DELAY_MS) {
    lastRetry = now;
    if (hasSerial) {
      Serial.println(" ");
      Serial.print("Connecting to "); Serial.println(ssid);
    }

    WiFi.begin(ssid, password);
    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 8000) {
      delay(500);
      if (hasSerial) { Serial.print("."); Serial.print(WiFi.status()); }
    }

    if (WiFi.status() == WL_CONNECTED) {
      if (hasSerial) {
        Serial.println("\nWiFi connected");
        Serial.print("IP address: "); Serial.println(WiFi.localIP());
        Serial.println("Attempting TCP server connection...");
      }
      if (client.connect(host_ip, host_port)) {
        if (hasSerial) Serial.println("Persistent TCP connected to host");
        WiFiState = CONNECTED;
        initializeADCs();        // re-arms ADS DRDY ints; transparently re-quiesces IMU/Mag
        portENTER_CRITICAL(&queue_mux);
        packet_queue.clear();
        portEXIT_CRITICAL(&queue_mux);
        ArduinoOTA.end();
        if (hasSerial) Serial.println("Entering CONNECTED state.");
      } else {
        if (hasSerial) Serial.println("TCP server connection failed");
      }
    } else {
      if (hasSerial) Serial.println("\nWiFi Connection failed, retrying...");
    }
  }
}

// Monitor connection to host (unchanged shape).
void monitorConnection() {
  static unsigned long lastCheck = 0;
  unsigned long now = millis();
  if (now - lastCheck >= WiFi_CHECK_INTERVAL_MS) {
    lastCheck = now;
    if (WiFi.status() != WL_CONNECTED || !client.connected()) {
      if (hasSerial) Serial.println("WiFi or TCP disconnected.");
      client.stop();
      WiFiState = CONNECTING;
      ArduinoOTA.begin();
      if (hasSerial) Serial.println("Returning to CONNECTING state with OTA re-enabled.");
    }
  }
}

// CRC-16/CCITT (poly 0xA001), init 0xFFFF. Matches collect_data.py's _crc16_ccitt.
uint16_t compute_crc(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
  }
  return crc;
}

// ====================================================================================
//                                       setup()
// ====================================================================================
void setup() {
  // Serial detection (non-blocking).
  Serial.begin(115200);
  delay(5000);
  Serial.print("Reset reason: "); Serial.println((int)esp_reset_reason());
  // 1=POWERON, 4=PANIC, 6=TASK_WDT, 15=BROWNOUT
  unsigned long startTime = millis();
  while (!Serial) { if (millis() - startTime > 5000) break; }
  hasSerial = Serial;
  if (hasSerial) {
    Serial.println(" ");
    Serial.println("Serial connected for debugging");
    Serial.print("Nano_ID: "); Serial.println(NANO_ID);
  }

  // Shared SPI bus. MOSI/MISO swap matches the Hi-STIFFS rev-2 PCB; do not "fix" the
  // pin numbers in either direction without checking the schematic first.
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  // ---- IMU / MAG: pin modes and library bring-up ----
  // CS pins are driven HIGH (deselected) BEFORE we configure anything, so a floating
  // line cannot accidentally enable a chip mid-SPI-transaction during boot.
  pinMode(IMU_CS_PIN, OUTPUT); digitalWrite(IMU_CS_PIN, HIGH);
  pinMode(MAG_CS_PIN, OUTPUT); digitalWrite(MAG_CS_PIN, HIGH);
  pinMode(IMU_INT_PIN, INPUT_PULLDOWN);
  pinMode(MAG_DRDY_PIN, INPUT_PULLDOWN);

  ImuMagSpi::Config imuCfg;
  imuCfg.spi         = &SPI;
  imuCfg.spiSettings = spi_settings_imu;     // 8 MHz, MODE3 (library default)
  imuCfg.imuCsPin    = IMU_CS_PIN;
  imuCfg.magCsPin    = MAG_CS_PIN;
  imuCfg.magIntPin   = MAG_DRDY_PIN;         // used by the library to prime LIS3MDL DRDY
  imuCfg.taskCore    = 1;                    // WiFi runs on core 0
  ImuMagSpi::begin(imuCfg);

  g_imu_ok = ImuMagSpi::initImu();
  g_mag_ok = ImuMagSpi::initMag();
  if (hasSerial) {
    Serial.print("ISM330DHCX init: "); Serial.println(g_imu_ok ? "OK" : "FAIL (WHO_AM_I mismatch)");
    Serial.print("LIS3MDL    init: "); Serial.println(g_mag_ok ? "OK" : "FAIL (WHO_AM_I mismatch)");
  }

  // ---- WiFi station mode (scan is intentionally informational only) ----
  if (hasSerial) {
    Serial.println("Starting station mode. Arduino is WiFi client looking for following network...");
    Serial.print("SSID: "); Serial.print(ssid);
    Serial.print(". Password: "); Serial.println(password);
    Serial.println("Scanning for available networks...");
  }
  WiFi.mode(WIFI_STA);
  int n = WiFi.scanNetworks();
  if (hasSerial) {
    Serial.println("WiFi network scan done. Listing networks...");
    if (n == 0) Serial.println("No networks found");
    else for (int i = 0; i < n; ++i) {
      Serial.print(WiFi.SSID(i));
      Serial.print(" ("); Serial.print(WiFi.RSSI(i)); Serial.print(" dBm)");
      Serial.print(" ("); Serial.print(WiFi.encryptionType(i), HEX); Serial.print(")");
      Serial.println();
    }
  }

  // OTA setup (kept commented to match the prior firmware behavior; uncomment when you
  // actually want OTA from CONNECTING state).
  // ArduinoOTA.setHostname("Hi-STIFFS_Nano");
  // ArduinoOTA.setPassword(ota_password);
  // ArduinoOTA.begin();

  // Spin up the per-ADS reader tasks FIRST. initializeADCs() transiently arms the
  // ADS DRDY interrupts at its end (followed by a 50 ms settle), and the very first
  // edge will vTaskNotifyGiveFromISR(adsTaskHandle[i]). If those handles are still
  // nullptr we panic (reset_reason=4). Creating the tasks here makes the handles
  // valid before any ISR can fire. The tasks block on notifyTake forever until
  // their ISR pings them, so creating them early is harmless.
  if (!startAdsTasks() && hasSerial) Serial.println("ADS task creation failed");

  initializeADCs();
  disableADCInterrupts();

  // Start the IMU/Mag wrapper tasks BEFORE attaching their interrupts, so the task
  // handles inside the wrapper are valid by the time the first DRDY edge fires. This
  // is the reverse of the order in the library's docstring; the wrapper's ISR helpers
  // dereference g_imuTask / g_magTask, so attaching the interrupts first would crash
  // on the very first edge.
  if (!ImuMagSpi::startTasks() && hasSerial) Serial.println("ImuMagSpi::startTasks failed");
    if (g_imu_ok) attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN),  handleImuDrdy, RISING);
  if (g_mag_ok) attachInterrupt(digitalPinToInterrupt(MAG_DRDY_PIN), handleMagDrdy, RISING);
  if (g_mag_ok) ImuMagSpi::kickMag();
  g_imu_mag_attached = (g_imu_ok || g_mag_ok);

  SerialState = hasSerial ? HAS_SERIAL : NO_SERIAL;
  if (hasSerial) {
    Serial.print("Initial state: ");
    Serial.println(SerialState == HAS_SERIAL ? "HAS_SERIAL" : "NO_SERIAL");
  }
  last_batch_send = millis();
}

// ====================================================================================
//                                       loop()
// ====================================================================================
// The loop now only does three things in CONNECTED state: drain the IMU/Mag library
// queues into the packet queue, drain the packet queue into one TCP frame every
// BATCH_SEND_INTERVAL_MS, and watch the connection. All ADS SPI work has migrated
// into the per-chip tasks. This keeps loop() short and predictable, which is exactly
// what we need to hold the 100 Hz TCP cadence under WiFi jitter.
void loop() {
  switch (SerialState) {
    case NO_SERIAL: {
      switch (WiFiState) {
        case CONNECTED: {
          drainImuMagQueues();
          unsigned long now = millis();
          if (now - last_batch_send >= BATCH_SEND_INTERVAL_MS) {
            sendBatchedFrame();
            last_batch_send = now;
          }
          monitorConnection();
        } break;
        case CONNECTING: {
          disableADCInterrupts();
          handleOTA();
          connectToHost();
        } break;
      }
    } break;

    case HAS_SERIAL: {
      switch (WiFiState) {
        case CONNECTED: {
          drainImuMagQueues();
          unsigned long now = millis();
          if (now - last_batch_send >= BATCH_SEND_INTERVAL_MS) {
            sendBatchedFrame();
            sendDataSerial();          // optional debug echo of the latest ADS pair
            last_batch_send = now;
          }
          monitorConnection();
        } break;
        case CONNECTING: {
          disableADCInterrupts();
          handleOTA();
          connectToHost();
        } break;
      }
    } break;
  }
}