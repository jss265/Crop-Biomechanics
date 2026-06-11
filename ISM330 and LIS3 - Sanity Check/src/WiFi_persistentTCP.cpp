//////////////////////////////////////////////////////////////////////////////////////////
//
//    Arduino code for reading from up to 5 ADS1220 chips on an Arduino Nano ESP32.
//    Each ADS1220 reads 2 differential channels (AIN0-AIN1 and AIN2-AIN3),
//    labeled as sensors A-E with channels 1-2 (e.g., A1, A2, B1, B2, etc.).
//    Total possible channels: 10.
//
//    This code is based on the provided Protocentral_ADS1220 library and the example
//    new_DAQ_2.ino. It uses interrupts on DRDY pins for efficient data reading.
//
//    Key Decisions and Justifications (Updated for 2026 Wireless Integration):
//    - Modular Design: 
//      - Use a configurable number of sensors (NUM_SENSORS) and an array
//        of configurations. To use fewer sensors (e.g., only A-C), set NUM_SENSORS=3. 
//      - The all_configs array lists all possible configurations in order (A to E). 
//      - The code will only use the first NUM_SENSORS entries from this array. This allows 
//        easy enabling/disabling by just changing NUM_SENSORS without commenting out lines 
//        in the array initializer. If you need to change pin assignments or IDs, edit the 
//        all_configs array directly.
//    - Pin Assignments: 
//      - SPI pins: Fixed on Nano ESP32 as D13 (SCK), D11 (MOSI/COPI), D12 (MISO/CIPO).
//      - CS pins: D10, D9, D8, D7, D6 (outputs, chosen to avoid SPI pins and common serial pins).
//      - DRDY pins: D5, D4, D3, D2, A6 (inputs with interrupt support). A6 is used as digital input
//        (valid on ESP32). Avoided D0/D1 (potential UART) and analog-only pins without interrupt needs.
//        These pins are plentiful on Nano ESP32 (14 digital + 8 analog-as-digital).
//      - Justification: Ensures no conflicts with SPI bus or USB Serial. All DRDY pins support
//        attachInterrupt on ESP32.
//    - Operating Mode and Data Rate:
//      - Set to Turbo mode (MODE_TURBO) as requested.
//      - Data rate set to DR_90SPS, which becomes ~180 SPS in turbo mode for ~90 SPS per channel
//        (per ADS1220 datasheet, Table 11). Adjusted from DR_175SPS for stability with WiFi.
//    - Ratiometric Measurement: 
//      - Configured for ratiometric bridge readings as recommended
//        in the ADS1220 datasheet (section 9.2.3). The bridge excitation voltage (5.1V) is
//        used as the external reference by setting VREF to AVDD (ANALOG). This cancels excitation voltage
//        variations. 
//      - Note: Since we now send raw ADC integers instead of mV, the voltage 
//        calculation (which previously used VREF=5.1V) is no longer performed here. Any 
//        conversion to physical units should be done on the receiving side if needed.
//      - Justification: Datasheet recommends using excitation as reference for ratiometric
//        operation to improve accuracy in bridge sensors like load cells.
//    - Synchronization Within Each Sensor:
//      - Each ADS1220 has one ADC, so channels are multiplexed and converted sequentially.
//      - Use continuous mode with mux switching on DRDY interrupt (as in example). After reading channel 1,
//        switch to channel 2 (restarts conversion). Next DRDY interupt reads channel 2.
//      - The two channels per chip are ~1/data_rate apart (~11ms at 90Hz), which is "reasonably synchronized".
//      - Single timestamp (micros()) taken after reading the second channel, applied to both. This approximates
//        the pair as synchronized.
//      - Justification: Simultaneous conversion impossible with single ADC. This method minimizes delay.
//    - Synchronization Between Sensors: 
//      - Not implemented. Sensors run independently (internal oscillators may drift slightly over time, ~2% per datasheet). 
//      - If needed, add periodic START/SYNC commands to all chips (requires careful handling to avoid MISO conflicts).
//    - Output Format (Wireless Update): 
//      - Send data only when all sensors are ready (a full cycle), in a
//        single efficient CSV line: tsA,A1_raw,A2_raw,tsB,B1_raw,B2_raw,... Each sensor has its
//        own timestamp (taken after reading its second channel). Timestamps are always per-sensor to account for any minor delays.
//      - The _raw values are the 24-bit signed integers directly from the ADC (sign-extended to 32-bit for printing).
//        Previously, these were converted to mV using the formula: (float(val) * VREF / PGA_GAIN / (1LL<<23)) * 1000.0f,
//        but now we send the raw integers for flexibility (conversion can be done later if needed).
//      - Now outputs over WiFi using persistent TCP connection to a host server (port 80) for streaming.
//        The Nano connects as a WiFi client to the host's AP.
//      - Serial output retained for debugging (with timeout to avoid blocking if no USB connected).
//      - Justification: Enables untethered field operation; persistent TCP provides efficient, reliable streaming compatible with Python servers.
//        High baud rate serial fallback for wired debugging.
//    - Wireless Features:
//      - Station mode: Nano connects to host's WiFi network (configurable SSID/password).
//      - OTA: Supports over-the-air sketch uploads via WiFi for wireless updates.
//      - Power Impact: Station mode adds ~80-150mA draw; monitor for battery-powered setups (see Nano ESP32 manual, Section 11).
//    - Other Settings: 
//      - PGA=128 (Programmable Gain Amplifier), internal VREF=2.048V (from datasheet). Continuous mode for steady sampling.
//    - Interrupt Handling: 
//      - Use lambdas to pass sensor index to handler (C++11, supported in Arduino).
//      - Note: In the code, we use a fixed array of 5 ISR handlers (handleDrdy0 to handleDrdy4) because 
//        attaching interrupts with lambdas that capture variables can be tricky on ESP32 (especially with IRAM_ATTR).
//        We define all 5 handlers even if NUM_SENSORS < 5, but only attach the first NUM_SENSORS. This is simple 
//        and avoids issues with captured lambdas in interrupts.
//    - Error Handling: 
//      - Basic; assumes DRDY triggers reliably. Added serial timeout for non-blocking boot.
//
//    Dependencies: Protocentral_ADS1220 library (include .h and .cpp as provided).
//
//    For information on ADS1220, see datasheet. For Nano ESP32 pins and WiFi, see provided PDF.
//
//    This software is licensed under the MIT License (as in original library).
//
//    Updated Architecture (2026):
//    - Finite State Machine (FSM): Manages connection states to attempt connection when idle,
//      send data when connected, and handle Serial presence determined at setup. States: CONNECTING,
//      CONNECTED, SERIAL_ONLY, NO_SERIAL. This ensures efficient resource use on Nano ESP32.
//    - Modular Functions: Broke out key operations (e.g., OTA handling, DRDY processing, data sending)
//      for tight, efficient code. Prepares for higher data rates by minimizing loop overhead.
//    - OTA Restriction: Disabled during active connections to prevent disruptions.
//    - ADC Reset: On new connection, reset ADCs and timestamps for session synchronization.
//    - Error Gracefulness: Added checks for missed DRDY (e.g., skip if flag not set), connection errors
//      (retry), and suppressed Serial if absent to avoid blocking.
//    - Efficiency for Nano ESP32: Used IRAM_ATTR for ISRs, non-blocking loops, conditional debug
//      prints. Aligned with PDF specs (e.g., low power notes for future optimizations, pin capabilities).
//    - Future-Proofing: Code structure allows easy data rate increases (e.g., to DR_1000SPS in turbo,
//      ~2000 SPS total) once tested; focus on tight execution to handle higher interrupt rates.
//    - Multi-Nano Compatibility: Each Nano has a unique ID (NANO_ID) sent in data packets.
//      Host handles aggregation; no changes here for multi-Nano beyond ID inclusion.
//    - Persistent TCP: Replaced per-packet HTTP POST with a single persistent TCP connection
//      established once in CONNECTED state. Data is streamed as length-prefixed binary frames
//      for reliable, low-overhead transmission. This avoids repeated handshakes and headers.
//    - Queued Batching: Added a queue (std::deque<std::vector<uint8_t>>) to store complete
//      binary packets when data is ready. Every BATCH_SEND_INTERVAL_MS (configurable, default 100ms),
//      the queue is drained and all packets are sent over the persistent connection. This batches
//      sends to amortize network overhead while maintaining low latency.
//
/////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include "Protocentral_ADS1220.h"
#include <SPI.h>
#include <WiFi.h>       // For WiFi Station mode
#include <ArduinoOTA.h> // For Over-The-Air updates
#include <vector>       // For storing binary packet data
#include "esp_timer.h"  // For esp_timer_get_time() 64-bit microsecond timestamps
#include "ImuMagSpi.h"  // ISM330DHCX + LIS3MDL FeatherWing driver (replaces NavX)

#define FULL_SCALE (1LL << 23) // 2^23 for 24-bit signed scaling - retained for reference, though not used in raw output
#define A_DRDY_PIN 9
#define A_CS_PIN 10
#define B_DRDY_PIN 7
#define B_CS_PIN 8
#define C_DRDY_PIN 5
#define C_CS_PIN 6
#define D_DRDY_PIN 3
#define D_CS_PIN 4
#define E_DRDY_PIN A6
#define E_CS_PIN 2

const int SPI_SCK  = 13;   // default D13
const int SPI_MISO = 11;   // default D12 (CIPO)
const int SPI_MOSI = 12;   // default D11 (COPI)

// ISM330DHCX + LIS3MDL FeatherWing pins (shares the SPI bus with the ADS1220 chips)
#define IMU_CS_PIN   A0   // ISM330DHCX chip select
#define MAG_CS_PIN   A1   // LIS3MDL chip select
#define IMU_INT_PIN  A2   // ISM330DHCX INT1 (accel/gyro data-ready)
#define MAG_DRDY_PIN A3   // LIS3MDL DRDY

const int MAX_SENSORS = 5;     // Maximum possible sensors (A to E)
const int NUM_SENSORS = 3;     // Set to 1-5 to use the first N sensors from all_configs below.
const uint8_t dr_code = DR_330SPS;  // Data Rate value. In turbo, value is for pairs/sec. In normal, value is for samples/sec
const SPISettings spi_settings(4000000, MSBFIRST, SPI_MODE1);  // ADS1220 bus speed (4 MHz; IMU/Mag use 8 MHz via ImuMagSpi)

// ---- Wire packet definitions (sent as [type][packed struct]) ----------------
// Records are produced by the ADS read tasks and the ImuMagSpi tasks, then
// concatenated into a single batch and wrapped in one [len][seq][crc] frame.
enum PacketType : uint8_t {
  PKT_IMU = 1,
  PKT_MAG = 2,
  PKT_ADS = 3
};

#pragma pack(push, 1)
struct AdsPacket {
  uint64_t ts_us;
  uint8_t  adc_id;   // A=0, B=1, C=2, D=3, E=4
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

// WiFi access point to connect to (host's AP)
const char* ssid = "Hi-STIFFS_Host";       // Host WiFi network name (SSID) - choose something unique
const char* password = "BYUCropBio";       // Host WiFi password (minimum 8 characters)
const char* ota_password = "BYUCropBio";   // Optional OTA password for security (change this)
// Host server details
// const char* host_ip = "192.168.137.1";       // IPv4 of the host server (e.g., Raspberry Pi or laptop), from device settings, not Python code
const char* host_ip = "192.168.137.1";    // For Pi_001
const int host_port = 8080;                  // Port on host for data streaming
WiFiClient client;                         // Global client for sending data to host

// Unique Nano ID (2-digit, e.g., "01" to "99")
const char* NANO_ID = "01";                // Assign per Nano; for now, fixed to "01"

// Batching configuration: Send queued packets every this interval (ms)
const unsigned long BATCH_SEND_INTERVAL_MS = 10;  // Default 0.1s; adjust for desired batch frequency

struct SensorConfig {
  char id;                     // Sensor ID ('A', 'B', etc.)
  int cs_pin;                  // Chip Select pin
  int drdy_pin;                // Data Ready pin
};

// Full list of configurations for sensors A to E. The code uses only the first NUM_SENSORS.
// To change pins or IDs, edit this array. No commenting out needed - just set NUM_SENSORS.
SensorConfig all_configs[MAX_SENSORS] = {
  {'A', A_CS_PIN, A_DRDY_PIN},
  {'B', B_CS_PIN, B_DRDY_PIN},
  {'C', C_CS_PIN, C_DRDY_PIN},
  {'D', D_CS_PIN, D_DRDY_PIN},
  {'E', E_CS_PIN, E_DRDY_PIN}
};

Protocentral_ADS1220 adcs[MAX_SENSORS];             // ADC objects from library
int32_t raw_values[MAX_SENSORS][2];                 // [sensor][channel]: scratch for ch1 between DRDY events (read by ADS tasks)
uint8_t current_channels[MAX_SENSORS] = {0};        // Indicates which MUX channel to read from for an ADS1220 module
const unsigned long WiFi_CHECK_INTERVAL_MS = 1000;  // 1 Hz
const unsigned long WiFi_RETRY_DELAY_MS = 5000;     // Retry connection every 5s if failed

// ADS1220 deferred-read plumbing: short ISRs timestamp + notify; tasks do the SPI read.
volatile int64_t g_adsIsrTs[MAX_SENSORS] = {0};        // Per-sensor DRDY timestamp (us) captured in ISR
TaskHandle_t     g_adsTask[MAX_SENSORS]  = {nullptr};  // Per-sensor read task handle
QueueHandle_t    g_adsQueue              = nullptr;    // Queue of completed AdsPacket records
const size_t     ADS_QUEUE_DEPTH         = 128;        // AdsPacket records buffered before drops

uint16_t seq_num = 0;                               // Sequence number for batch frames, increments per frame
unsigned long last_batch_send = 0;                  // Timestamp of last batch send

// Forward declaration (used by the batch sender before its definition below)
uint16_t compute_crc(const uint8_t* data, size_t len);

// FSM States
enum State_Serial {
  NO_SERIAL,    // No Serial connection
  HAS_SERIAL,   // Serial connection active
};
State_Serial SerialState;

enum State_WiFi {
  CONNECTING,   // Attempting to connect to host AP
  CONNECTED,    // Connected to host, sending data
};
State_WiFi WiFiState = CONNECTING;

// Global flag for Serial presence, set in setup
bool hasSerial = false;

// ---- ADS1220 deferred-read tasks --------------------------------------------
// The DRDY ISRs are intentionally tiny: they only timestamp the event and notify
// the matching read task. The task performs the SPI read off interrupt context,
// switches the MUX, and (after the second channel) publishes an AdsPacket.
void adsReadTask(void* arg) {
  const int i = (int)(intptr_t)arg;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    int64_t ts = g_adsIsrTs[i];

    // Read the 24-bit conversion result over SPI
    SPI.beginTransaction(spi_settings);
    digitalWrite(all_configs[i].cs_pin, LOW);
    delayMicroseconds(1);
    byte SPI_Buf[3];
    SPI_Buf[0] = SPI.transfer(0);
    SPI_Buf[1] = SPI.transfer(0);
    SPI_Buf[2] = SPI.transfer(0);
    delayMicroseconds(1);
    digitalWrite(all_configs[i].cs_pin, HIGH);
    SPI.endTransaction();

    long bits24 = (long)SPI_Buf[0] << 16 | (long)SPI_Buf[1] << 8 | SPI_Buf[2];
    int32_t val = (bits24 << 8) >> 8;  // Sign-extend 24-bit to 32-bit

    if (current_channels[i] == 0) {
      raw_values[i][0] = val;                      // Stash channel 1
      adcs[i].select_mux_channels(MUX_AIN2_AIN3);
      current_channels[i] = 1;
    } else {
      adcs[i].select_mux_channels(MUX_AIN0_AIN1);
      current_channels[i] = 0;

      // Pair complete: publish one AdsPacket (timestamp taken at channel-2 DRDY)
      AdsPacket pk;
      pk.ts_us  = (uint64_t)ts;
      pk.adc_id = (uint8_t)(all_configs[i].id - 'A');  // A=0, C=2, E=4 — matches Python adcid_to_idx;
      pk.ch1    = raw_values[i][0];
      pk.ch2    = val;
      xQueueSend(g_adsQueue, &pk, 0);
    }
  }
}

// Tiny DRDY ISRs: timestamp + notify the matching read task. No SPI in interrupt context.
void IRAM_ATTR handleDrdyA() {
  g_adsIsrTs[0] = esp_timer_get_time();
  BaseType_t hpw = pdFALSE;
  vTaskNotifyGiveFromISR(g_adsTask[0], &hpw);
  if (hpw) portYIELD_FROM_ISR();
}
void IRAM_ATTR handleDrdyB() {
  g_adsIsrTs[1] = esp_timer_get_time();
  BaseType_t hpw = pdFALSE;
  vTaskNotifyGiveFromISR(g_adsTask[1], &hpw);
  if (hpw) portYIELD_FROM_ISR();
}
void IRAM_ATTR handleDrdyC() {
  g_adsIsrTs[2] = esp_timer_get_time();
  BaseType_t hpw = pdFALSE;
  vTaskNotifyGiveFromISR(g_adsTask[2], &hpw);
  if (hpw) portYIELD_FROM_ISR();
}
void IRAM_ATTR handleDrdyD() {
  g_adsIsrTs[3] = esp_timer_get_time();
  BaseType_t hpw = pdFALSE;
  vTaskNotifyGiveFromISR(g_adsTask[3], &hpw);
  if (hpw) portYIELD_FROM_ISR();
}
void IRAM_ATTR handleDrdyE() {
  g_adsIsrTs[4] = esp_timer_get_time();
  BaseType_t hpw = pdFALSE;
  vTaskNotifyGiveFromISR(g_adsTask[4], &hpw);
  if (hpw) portYIELD_FROM_ISR();
}

// ImuMagSpi DRDY ISRs: timestamp + hand off to the library's tasks.
void IRAM_ATTR imuDrdyISR() { ImuMagSpi::onImuDrdyFromIsr(esp_timer_get_time()); }
void IRAM_ATTR magDrdyISR() { ImuMagSpi::onMagDrdyFromIsr(esp_timer_get_time()); }

// Broadcast a command to all active sensors
void broadcast_command(uint8_t cmd) {
  // Lower all CS pins for the active sensors to broadcast the command to all chips.
  // We loop only over NUM_SENSORS to avoid affecting unused pins.
  for (int i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(all_configs[i].cs_pin, LOW);
  }
  SPI.transfer(cmd);  // Send the command to all sensors at once
  // Raise all CS pins for active sensors
  for (int i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(all_configs[i].cs_pin, HIGH);
  }
}

// Disable interrupts for sensor DRDY pins
void disableADCInterrupts() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    detachInterrupt(digitalPinToInterrupt(all_configs[i].drdy_pin));
  }
}

// Enable interrupts for sensor DRDY pins
void enableADCInterrupts() {
  void (*isrHandlers[5])() = {handleDrdyA, handleDrdyB, handleDrdyC, handleDrdyD, handleDrdyE};
  for (int i = 0; i < NUM_SENSORS; i++) {
    attachInterrupt(digitalPinToInterrupt(all_configs[i].drdy_pin), isrHandlers[i], FALLING);
  }
}

// Reset and reconfigure all ADCs
void initializeADCs() {
  // Broadcast RESET to all active sensors
  broadcast_command(RESET);
  delay(10);  // Short delay for reset to take effect (per datasheet)

  // Detach interrupts before re-attaching
  disableADCInterrupts();

  // ADS1220 initialization
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
      Serial.print("Setup complete for Sensor [enumerated as: ");
      Serial.print(i);
      Serial.println("]");
    }
    delay(100);
  }
  if (hasSerial) Serial.println("Starting datastream from sensors");

  // Allow settling
  delay(100);               

  // Staggered START: Evenly space within one conversion cycle, assuming 5 sensors max
  uint16_t turbo_sps = 0;
  switch (dr_code) {
    case DR_20SPS: turbo_sps = 40; break;
    case DR_45SPS: turbo_sps = 90; break;
    case DR_90SPS: turbo_sps = 180; break;
    case DR_175SPS: turbo_sps = 350; break;
    case DR_330SPS: turbo_sps = 660; break;
    case DR_600SPS: turbo_sps = 1200; break;
    case DR_1000SPS: turbo_sps = 2000; break;
    default: 
      if (hasSerial) Serial.println("Invalid DR code; using default 180 SPS");
      turbo_sps = 180;
  }
  unsigned long period_us = 1000000UL / turbo_sps;  // Conversion period in µs
  unsigned long stagger_us = period_us / 5;  // Step for 5 sensors

  for (int i = 0; i < NUM_SENSORS; i++) {
    adcs[i].Start_Conv();
    if (i < NUM_SENSORS - 1) {
      delayMicroseconds(stagger_us);
    }
  }

  // Enable DRDY interrupts after letting ADS1220 modules stabilize
  enableADCInterrupts();
  delay(50);
}

// Handle OTA updates (called only in non-CONNECTED states)
void handleOTA() {
  ArduinoOTA.handle();
}

// Append raw bytes to the batch buffer
static inline void appendBytes(std::vector<uint8_t>& buf, const void* data, size_t n) {
  const uint8_t* p = reinterpret_cast<const uint8_t*>(data);
  buf.insert(buf.end(), p, p + n);
}

// Drain all sensor queues, build one batched frame, and send it over TCP.
// Frame layout: [len:2][seq:2][crc:2][ (type:1 + packed struct) * N ]  (little-endian)
bool buildAndSendBatch() {
  if (!client.connected()) {
    if (hasSerial) Serial.println("TCP not connected; skipping send");
    return false;
  }

  std::vector<uint8_t> payload;

  // ADS records (packed struct copied straight to the wire)
  AdsPacket ads;
  while (g_adsQueue && xQueueReceive(g_adsQueue, &ads, 0) == pdTRUE) {
    payload.push_back(PKT_ADS);
    appendBytes(payload, &ads, sizeof(ads));
  }

  // IMU records (copy lib sample into the packed wire struct)
  QueueHandle_t imuQ = ImuMagSpi::getImuQueue();
  ImuMagSpi::ImuSample is;
  while (imuQ && xQueueReceive(imuQ, &is, 0) == pdTRUE) {
    ImuPacket pk;
    pk.ts_us = is.ts_us;
    pk.ax = is.ax; pk.ay = is.ay; pk.az = is.az;
    pk.gx = is.gx; pk.gy = is.gy; pk.gz = is.gz;
    payload.push_back(PKT_IMU);
    appendBytes(payload, &pk, sizeof(pk));
  }

  // Mag records
  QueueHandle_t magQ = ImuMagSpi::getMagQueue();
  ImuMagSpi::MagSample ms;
  while (magQ && xQueueReceive(magQ, &ms, 0) == pdTRUE) {
    MagPacket pk;
    pk.ts_us = ms.ts_us;
    pk.mx = ms.mx; pk.my = ms.my; pk.mz = ms.mz;
    payload.push_back(PKT_MAG);
    appendBytes(payload, &pk, sizeof(pk));
  }

  if (payload.empty()) return true;  // Nothing to send this cycle

  uint16_t crc = compute_crc(payload.data(), payload.size());

  std::vector<uint8_t> frame;
  frame.reserve(6 + payload.size());

  // Length prefix (uint16_t, little-endian)
  uint16_t length = static_cast<uint16_t>(payload.size());
  frame.push_back(static_cast<uint8_t>(length & 0xFF));
  frame.push_back(static_cast<uint8_t>((length >> 8) & 0xFF));

  // Sequence number (uint16_t, little-endian)
  frame.push_back(static_cast<uint8_t>(seq_num & 0xFF));
  frame.push_back(static_cast<uint8_t>((seq_num >> 8) & 0xFF));
  seq_num++;

  // CRC (uint16_t, little-endian)
  frame.push_back(static_cast<uint8_t>(crc & 0xFF));
  frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));

  // Payload
  frame.insert(frame.end(), payload.begin(), payload.end());

  if (client.write(frame.data(), frame.size()) != frame.size()) {
    if (hasSerial) Serial.println("Partial or failed frame send");
    return false;
  }
  client.flush();
  return true;
}

// Print a lightweight diagnostic line over serial (rate-limited to ~1 Hz)
void printDiagnostics() {
  static unsigned long lastPrint = 0;
  unsigned long now = millis();
  if (now - lastPrint < 1000) return;
  lastPrint = now;
  Serial.print("IMU samples="); Serial.print(ImuMagSpi::getImuSampleCount());
  Serial.print(" dropped="); Serial.print(ImuMagSpi::getImuDroppedCount());
  Serial.print(" | MAG samples="); Serial.print(ImuMagSpi::getMagSampleCount());
  Serial.print(" dropped="); Serial.print(ImuMagSpi::getMagDroppedCount());
  Serial.print(" | ADS queued=");
  Serial.println(g_adsQueue ? (unsigned)uxQueueMessagesWaiting(g_adsQueue) : 0);
}

// Attempt to connect to host AP
void connectToHost() {
  static unsigned long lastRetry = 0;
  unsigned long now = millis();
  if (now - lastRetry >= WiFi_RETRY_DELAY_MS) {
    lastRetry = now;
    if (hasSerial) { Serial.print("Connecting to "); Serial.println(ssid); }

    // Disconnect from any previous AP before retrying (keep radio on).
    WiFi.disconnect(false);
    delay(100);
    WiFi.begin(ssid, password);
    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 8000) {
      delay(500);
      if (hasSerial) { Serial.print("."); Serial.print(WiFi.status()); }
    }

    if (WiFi.status() == WL_CONNECTED) {
      if (hasSerial) {
        Serial.println("\nWiFi connected"); 
        Serial.print("IP address: "); 
        Serial.println(WiFi.localIP());
        Serial.println("Attemping TCP server connection...");
      }

      // Establish persistent TCP connection
      if (client.connect(host_ip, host_port)) {
        if (hasSerial) Serial.println("Persistent TCP connected to host");
        WiFiState = CONNECTED;
        initializeADCs();  // Reset ADCs for new session
        // Drop any stale samples buffered while disconnected
        if (g_adsQueue) xQueueReset(g_adsQueue);
        if (ImuMagSpi::getImuQueue()) xQueueReset(ImuMagSpi::getImuQueue());
        if (ImuMagSpi::getMagQueue()) xQueueReset(ImuMagSpi::getMagQueue());
        seq_num = 0;
        ArduinoOTA.end();
        if (hasSerial) Serial.println("Entering CONNECTED state.");
      } 
      else {
        if (hasSerial) Serial.println("TCP server connection failed");
      }
    } 
    else {
      if (hasSerial) Serial.println("\nWiFi Connection failed, retrying...");
    }
  }
}

// Monitor connection to host
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

// Compute WiFi cyclic redundancy check code for packet integrity check
uint16_t compute_crc(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;  // CRC-CCITT initial value
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
  }
  return crc;
}

void setup() {
  // Serial detection and init
  Serial.begin(1000000);
  unsigned long startTime = millis();
  while (!Serial) {
    if (millis() - startTime > 5000) {
      // Continuing without serial
      break;
    }
  }
  hasSerial = Serial;  // Set flag based on connection
  if (hasSerial) {
    Serial.println(" "); 
    Serial.println("Serial connected for debugging");
    Serial.print("Nano_ID: ");
    Serial.println(NANO_ID);
  }

  // SPI init
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  // ---- ADS1220 deferred-read queue + per-sensor tasks (pinned to core 1) ----
  g_adsQueue = xQueueCreate(ADS_QUEUE_DEPTH, sizeof(AdsPacket));
  if (!g_adsQueue && hasSerial) Serial.println("ERROR: failed to create ADS queue");
  for (int i = 0; i < NUM_SENSORS; i++) {
    xTaskCreatePinnedToCore(adsReadTask, "ads_rd", 3072,
                            (void*)(intptr_t)i, configMAX_PRIORITIES - 2,
                            &g_adsTask[i], 1);
  }

  // ---- ISM330DHCX + LIS3MDL bring-up (shares the SPI bus at 8 MHz) ----
  {
    ImuMagSpi::Config imuCfg;
    imuCfg.spi         = &SPI;
    imuCfg.spiSettings = SPISettings(8000000, MSBFIRST, SPI_MODE3);
    imuCfg.imuCsPin    = IMU_CS_PIN;
    imuCfg.magCsPin    = MAG_CS_PIN;
    imuCfg.magIntPin      = MAG_DRDY_PIN;
    imuCfg.taskCore       = 1;
    imuCfg.imuQueueDepth  = 512;  // ~77 ms headroom at 6.66 kHz
    ImuMagSpi::begin(imuCfg);

    bool imuOk = ImuMagSpi::initImu();
    bool magOk = ImuMagSpi::initMag();
    if (hasSerial) {
      Serial.print("ISM330DHCX init: "); Serial.println(imuOk ? "OK" : "FAILED");
      Serial.print("LIS3MDL init: ");    Serial.println(magOk ? "OK" : "FAILED");
    }

    // Create the library's tasks and prime the LIS3MDL DRDY line, THEN attach
    // ISRs (the library task handles must exist before any DRDY interrupt fires).
    // The LIS3MDL DRDY is level-triggered, so these stay attached for the whole
    // session; stale samples are dropped via xQueueReset on each (re)connect.
    ImuMagSpi::startTasks();
    pinMode(IMU_INT_PIN, INPUT);
    pinMode(MAG_DRDY_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), imuDrdyISR, RISING);
    attachInterrupt(digitalPinToInterrupt(MAG_DRDY_PIN), magDrdyISR, RISING);

    // If DRDY was already HIGH when we attached RISING, the edge was missed and
    // the magTask will never wake (LIS3MDL holds DRDY high until data is read).
    // Give the pin a moment to settle, then kick the task if still HIGH.
    delayMicroseconds(200);
    if (hasSerial) {
      Serial.print("MAG DRDY pin (A3) state after ISR attach: ");
      Serial.println(digitalRead(MAG_DRDY_PIN) ? "HIGH (kicking mag task)" : "LOW (ok)");
    }
    if (digitalRead(MAG_DRDY_PIN) == HIGH) {
      ImuMagSpi::kickMag();
    }
  }
  if (hasSerial) {
    Serial.println("Starting station mode. Arduino is WiFi client looking for following network...");
    Serial.print("SSID: "); Serial.print(ssid);
    Serial.print(". Password: "); Serial.println(password);
    Serial.println("Scanning for available networks...");
  }

  WiFi.mode(WIFI_STA);
  int n = WiFi.scanNetworks();
  Serial.println("WiFi network scan done. Listing networks...");
  if (n == 0) {
    Serial.println("No networks found");
  } else {
    for (int i = 0; i < n; ++i) {
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(" dBm)");
      Serial.print(" (");
      Serial.print(WiFi.encryptionType(i), HEX);
      Serial.print(")");
      Serial.println();
    }
  }

  // OTA setup
  // ArduinoOTA.setHostname("Hi-STIFFS_Nano");
  // ArduinoOTA.setPassword(ota_password);
  // ArduinoOTA.begin();
  // if (hasSerial) Serial.println("OTA ready. Connect to WiFi and use OTA for updates.");

  // ADS1220 initialization (but interrupts disabled until connected)
  initializeADCs();
  disableADCInterrupts();

  // Set initial state based on Serial
  SerialState = hasSerial ? HAS_SERIAL : NO_SERIAL;
  if (hasSerial) { 
    Serial.print("Initial state: ");
    Serial.println(SerialState == HAS_SERIAL ? "HAS_SERIAL" : "NO_SERIAL");
  }

  last_batch_send = millis();
}

void loop() {

  // State-specific actions
  switch (SerialState) {
    case NO_SERIAL: { // Do nothing with the serial port
      
      switch (WiFiState) {
        case CONNECTED: {
          unsigned long now = millis();
          if (now - last_batch_send >= BATCH_SEND_INTERVAL_MS) {
            buildAndSendBatch();
            last_batch_send = now;
          }
          monitorConnection();
        } break;
        case CONNECTING: {
          disableADCInterrupts();
          connectToHost();
          delay(10);  // yield to idle task — prevents TWDT reset during retry gap
        } break;

      }
    } break;
    
    case HAS_SERIAL: {  // Send data over serial port at all times

      switch (WiFiState) {
        case CONNECTED: { // Connected, do not allow OTA updates and send datastream to host 
          unsigned long now = millis();
          if (now - last_batch_send >= BATCH_SEND_INTERVAL_MS) {
            buildAndSendBatch();
            last_batch_send = now;
          }
          printDiagnostics();  // ~1 Hz serial diagnostics
          monitorConnection();
        } break;        
        case CONNECTING: { // Not connected, but attempt connection and allow OTA updates
          disableADCInterrupts();
          connectToHost();
          delay(10);  // yield to idle task — prevents TWDT reset during retry gap
        } break;
      }
    } break;
  }

}