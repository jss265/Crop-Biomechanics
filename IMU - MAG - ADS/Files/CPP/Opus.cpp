//////////////////////////////////////////////////////////////////////////////////////////
//
//  Hi-STIFFS acquisition firmware — Arduino Nano ESP32.
//
//  Reads up to 5 ADS1220 strain ADCs (sensors A..E, 2 channels each) PLUS an
//  ISM330DHCX IMU and a LIS3MDL magnetometer, all on one shared SPI bus, and
//  streams typed binary records to collect_data.py over a persistent TCP link.
//
//  ============================  ARCHITECTURE  ===============================
//  - pollSensors() (non-blocking): the single place that touches more than one
//    sensor family. It (a) reads any ADS chip whose DRDY ISR has flagged it and
//    (b) drains the IMU FIFO + polls the Mag, packing every reading into a typed
//    record on the outbound queue. NOTHING blocks here.
//  - sendTCP() (non-blocking): assembles all queued records into ONE framed
//    payload [len:2][seq:2][crc:2][payload] and ships it every 10 ms, OR early
//    if the queue approaches the frame cap. Matches WiFiDataServer._client_loop.
//  - ADS uses lightweight DRDY interrupts (timestamp + flag only); the SPI read
//    happens in pollSensors(). IMU/Mag use NO interrupts at all — the FIFO makes
//    data-ready edges unnecessary, which is why this firmware no longer panics
//    on floating IMU/Mag pins.
//
//  Per-record wire format (little-endian; mirrors collect_data.py exactly):
//    PKT_ADS=3  '<QBii'    (17B)  ts_us, adc_id(A=0..E=4), ch1, ch2
//    PKT_IMU=1  '<Qhhhhhh' (20B)  ts_us, ax, ay, az, gx, gy, gz
//    PKT_MAG=2  '<Qhhh'    (14B)  ts_us, mx, my, mz
//  ALL ts_us are esp_timer microseconds; the host subtracts one shared origin.
//
//  Sections below are strictly partitioned: WiFi-only, IMU/Mag-only, ADS-only,
//  and a minimal Helpers section for the cross-cutting pieces (pollSensors,
//  sendTCP, the queue, CRC, and the FSM).
//////////////////////////////////////////////////////////////////////////////////////////

#include "Protocentral_ADS1220.h"
#include "ImuMagSpi.h"
#include "esp_timer.h"
#include <SPI.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <deque>
#include <vector>
#include <cstring>

// ====================================================================================
// ============================  SHARED WIRE / RECORD TYPES  ==========================
// ====================================================================================
enum PacketType : uint8_t { PKT_IMU = 1, PKT_MAG = 2, PKT_ADS = 3 };

#pragma pack(push, 1)
struct AdsPacket { uint64_t ts_us; uint8_t adc_id; int32_t ch1; int32_t ch2; };
struct MagPacket { uint64_t ts_us; int16_t mx, my, mz; };
struct ImuPacket { uint64_t ts_us; int16_t ax, ay, az; int16_t gx, gy, gz; };
#pragma pack(pop)
static_assert(sizeof(AdsPacket) == 17, "AdsPacket must be 17B (<QBii)");
static_assert(sizeof(ImuPacket) == 20, "ImuPacket must be 20B (<Qhhhhhh)");
static_assert(sizeof(MagPacket) == 14, "MagPacket must be 14B (<Qhhh)");

// ====================================================================================
// ================================  WiFi / TCP SECTION  ==============================
// ====================================================================================
// Everything here is WiFi/TCP-only: credentials, the client, the framed-send
// queue plumbing, connection bring-up and monitoring, and the FSM state.

const char* ssid         = "Hi-STIFFS_Host";
const char* password     = "BYUCropBio";
const char* ota_password = "BYUCropBio";
const char* host_ip      = "192.168.137.1";   // matches Config.HOST_IP
const int   host_port    = 8080;              // matches Config.HOST_PORT
const char* NANO_ID      = "01";              // debug only; host derives ID per-connection
WiFiClient client;

const unsigned long BATCH_SEND_INTERVAL_MS = 10;     // 100 Hz frames
const unsigned long WIFI_CHECK_INTERVAL_MS = 1000;
const unsigned long WIFI_RETRY_DELAY_MS    = 5000;

// Outbound record queue (filled by pollSensors, drained by sendTCP). Each entry is
// one [type:1][packet] record. A frame's payload length field is u16, so we send
// early if we approach that cap (the "queue full" safety valve).
std::deque<std::vector<uint8_t>> packet_queue;
portMUX_TYPE  queue_mux = portMUX_INITIALIZER_UNLOCKED;
uint16_t      seq_num = 0;
unsigned long last_batch_send = 0;
// Send early when the accumulated payload would approach the 0xFFFF frame cap.
// Largest record is 21 bytes; 2800*21 ~= 58 KB leaves margin under 65535.
const size_t  QUEUE_FLUSH_THRESHOLD = 2800;

enum State_WiFi { CONNECTING, CONNECTED };
State_WiFi WiFiState = CONNECTING;
bool hasSerial = false;

uint16_t computeCrc(const uint8_t* data, size_t len) {  // CRC-16/CCITT, poly 0xA001
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}

void handleOTA() { ArduinoOTA.handle(); }

// Forward decls for cross-section helpers (defined in Helpers section).
void initializeADCs();
void disableADCInterrupts();

void connectToHost() {
  static unsigned long lastRetry = 0;
  unsigned long now = millis();
  if (now - lastRetry < WIFI_RETRY_DELAY_MS) return;
  lastRetry = now;

  if (hasSerial) { Serial.println(); Serial.print("Connecting to "); Serial.println(ssid); }
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) {
    delay(500);
    if (hasSerial) { Serial.print("."); Serial.print(WiFi.status()); }
  }

  if (WiFi.status() != WL_CONNECTED) {
    if (hasSerial) Serial.println("\nWiFi connect failed, retrying...");
    return;
  }
  if (hasSerial) { Serial.println("\nWiFi connected"); Serial.print("IP: "); Serial.println(WiFi.localIP()); }

  if (client.connect(host_ip, host_port)) {
    if (hasSerial) Serial.println("TCP connected to host");
    // ---- Transition CONNECTING -> CONNECTED (all "fresh start on connect" work) ----
    WiFiState = CONNECTED;
    initializeADCs();                       // re-arm ADS DRDY interrupts + restart conv
    if (ImuMagSpi::imuPresent())
      ImuMagSpi::resetFifo();               // flush stale IMU samples; re-anchor clock
    portENTER_CRITICAL(&queue_mux);
    packet_queue.clear();                   // drop anything queued before this session
    portEXIT_CRITICAL(&queue_mux);
    ArduinoOTA.end();
    last_batch_send = millis();
  } else {
    if (hasSerial) Serial.println("TCP connect failed");
  }
}

void monitorConnection() {
  static unsigned long lastCheck = 0;
  unsigned long now = millis();
  if (now - lastCheck < WIFI_CHECK_INTERVAL_MS) return;
  lastCheck = now;
  if (WiFi.status() != WL_CONNECTED || !client.connected()) {
    if (hasSerial) Serial.println("WiFi/TCP dropped -> CONNECTING");
    client.stop();
    WiFiState = CONNECTING;                 // transition back; loop() will quiesce + retry
    ArduinoOTA.begin();
  }
}

void initWiFiClient() {
  WiFi.mode(WIFI_STA);
  if (hasSerial) {
    Serial.print("Station mode. Seeking SSID: "); Serial.println(ssid);
    int n = WiFi.scanNetworks();
    Serial.println("Scan done.");
    for (int i = 0; i < n; ++i) {
      Serial.print("  "); Serial.print(WiFi.SSID(i));
      Serial.print(" ("); Serial.print(WiFi.RSSI(i)); Serial.println(" dBm)");
    }
  }
  // OTA available only while disconnected (kept disabled until CONNECTING uses it).
  ArduinoOTA.setHostname("Hi-STIFFS_Nano");
  ArduinoOTA.setPassword(ota_password);
}

// ====================================================================================
// ===============================  IMU / MAG SECTION  ================================
// ====================================================================================
// IMU/Mag-only config + bring-up. NOTE: no interrupts and no tasks — acquisition
// is by FIFO/STATUS polling inside pollSensors(). The library owns all register
// detail; this section only wires pins and presence flags.

#define IMU_CS_PIN   A0   // ISM330DHCX CS
#define MAG_CS_PIN   A1   // LIS3MDL    CS
#define IMU_INT_PIN  A2   // (unused in FIFO model; left as INPUT for board safety)
#define MAG_DRDY_PIN A3   // (unused in FIFO model; left as INPUT for board safety)

const SPISettings spi_settings_imu(8000000, MSBFIRST, SPI_MODE3); // library default

bool g_imu_ok = false;
bool g_mag_ok = false;

void initImuMag() {
  // Park the (now unused) data-ready pins as plain inputs. With FIFO polling we
  // never attach interrupts to them, so a floating/absent FeatherWing is harmless
  // — the exact failure mode that used to panic the old ISR firmware.
  pinMode(IMU_INT_PIN,  INPUT);
  pinMode(MAG_DRDY_PIN, INPUT);

  ImuMagSpi::Config cfg;
  cfg.spi         = &SPI;
  cfg.spiSettings = spi_settings_imu;
  cfg.imuCsPin    = IMU_CS_PIN;
  cfg.magCsPin    = MAG_CS_PIN;
  ImuMagSpi::begin(cfg);

  g_imu_ok = ImuMagSpi::initImu();
  g_mag_ok = ImuMagSpi::initMag();
  if (hasSerial) {
    Serial.print("ISM330DHCX init: "); Serial.println(g_imu_ok ? "OK" : "FAIL (absent/who_am_i)");
    Serial.print("LIS3MDL    init: "); Serial.println(g_mag_ok ? "OK" : "FAIL (absent/who_am_i)");
  }
}

// ====================================================================================
// ==================================  ADS SECTION  ===================================
// ====================================================================================
// ADS-only. CONFIGS AND RATES ARE PRESERVED VERBATIM from the pre-IMU firmware
// (Turbo, DR_330SPS, PGA=128, VREF_ANALOG, continuous, MUX ping-pong, staggered
// start, 4 MHz/MODE1). The ONLY change vs. the original is that the DRDY ISR now
// just timestamps + flags; the SPI read moved into pollSensors().

#define A_DRDY_PIN 9
#define A_CS_PIN   10
#define B_DRDY_PIN 7
#define B_CS_PIN   8
#define C_DRDY_PIN 5
#define C_CS_PIN   6
#define D_DRDY_PIN 4
#define D_CS_PIN   3
#define E_DRDY_PIN 2
#define E_CS_PIN   A6

// SPI pins — MOSI/MISO intentionally SWAPPED to match the Hi-STIFFS rev-2 PCB.
const int SPI_SCK  = 13;
const int SPI_MISO = 11;   // D12 on silk (CIPO) — swapped
const int SPI_MOSI = 12;   // D11 on silk (COPI) — swapped

const int     MAX_SENSORS = 5;
const int     NUM_SENSORS = 5;     // first N of all_configs. MUST match host sensors=...
const uint8_t dr_code     = DR_330SPS;
const SPISettings spi_settings_ads(4000000, MSBFIRST, SPI_MODE1);

struct SensorConfig { char id; int cs_pin; int drdy_pin; };
SensorConfig all_configs[MAX_SENSORS] = {
  {'A', A_CS_PIN, A_DRDY_PIN},
  {'C', C_CS_PIN, C_DRDY_PIN},
  {'B', B_CS_PIN, B_DRDY_PIN},
  {'D', D_CS_PIN, D_DRDY_PIN},
  {'E', E_CS_PIN, E_DRDY_PIN}
};

Protocentral_ADS1220 adcs[MAX_SENSORS];
int32_t          raw_values[MAX_SENSORS][2];
volatile int64_t isr_ts_us[MAX_SENSORS]      = {0};   // esp_timer stamp captured in ISR
volatile bool    ads_ready[MAX_SENSORS]      = {false};
uint8_t          current_channels[MAX_SENSORS] = {0};

// --- ADS DRDY ISRs: timestamp + flag ONLY (no SPI in interrupt context) ---
void IRAM_ATTR drdyA() { isr_ts_us[0] = esp_timer_get_time(); ads_ready[0] = true; }
void IRAM_ATTR drdyB() { isr_ts_us[1] = esp_timer_get_time(); ads_ready[1] = true; }
void IRAM_ATTR drdyC() { isr_ts_us[2] = esp_timer_get_time(); ads_ready[2] = true; }
void IRAM_ATTR drdyD() { isr_ts_us[3] = esp_timer_get_time(); ads_ready[3] = true; }
void IRAM_ATTR drdyE() { isr_ts_us[4] = esp_timer_get_time(); ads_ready[4] = true; }

void enableADCInterrupts() {
  void (*isr[5])() = { drdyA, drdyB, drdyC, drdyD, drdyE };
  for (int i = 0; i < NUM_SENSORS; i++)
    attachInterrupt(digitalPinToInterrupt(all_configs[i].drdy_pin), isr[i], FALLING);
}
void disableADCInterrupts() {
  for (int i = 0; i < NUM_SENSORS; i++)
    detachInterrupt(digitalPinToInterrupt(all_configs[i].drdy_pin));
}

void broadcastCommand(uint8_t cmd) {
  SPI.beginTransaction(spi_settings_ads);
  for (int i = 0; i < NUM_SENSORS; i++) digitalWrite(all_configs[i].cs_pin, LOW);
  SPI.transfer(cmd);
  for (int i = 0; i < NUM_SENSORS; i++) digitalWrite(all_configs[i].cs_pin, HIGH);
  SPI.endTransaction();
}

void initializeADCs() {
  broadcastCommand(RESET);
  delay(10);
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
    ads_ready[i] = false;
    if (hasSerial) {
      Serial.print("ADS "); Serial.print(all_configs[i].id);
      Serial.print(" CS="); Serial.print(all_configs[i].cs_pin);
      Serial.print(" DRDY="); Serial.println(all_configs[i].drdy_pin);
    }
    delay(100);
  }
  if (hasSerial) Serial.println("Starting ADS datastream");
  delay(100);

  // Staggered START within one conversion period (preserved from original).
  uint16_t turbo_sps = 660;          // DR_330SPS in Turbo
  unsigned long stagger_us = (1000000UL / turbo_sps) / 5;
  for (int i = 0; i < NUM_SENSORS; i++) {
    adcs[i].Start_Conv();
    if (i < NUM_SENSORS - 1) delayMicroseconds(stagger_us);
  }
  enableADCInterrupts();
  delay(50);
}

// Read one ADS chip that the ISR flagged, run the MUX ping-pong, and (on the ch2
// read) emit a PKT_ADS record. Logic preserved from the original ISR body; only
// the location (task/ISR -> pollSensors) and the bus-mutex wrapping changed.
void serviceAdsChip(int i, std::vector<std::vector<uint8_t>>& outRecs) {
  const int cs = all_configs[i].cs_pin;

  SPI.beginTransaction(spi_settings_ads);
  digitalWrite(cs, LOW);
  delayMicroseconds(1);
  uint8_t b0 = SPI.transfer(0), b1 = SPI.transfer(0), b2 = SPI.transfer(0);
  delayMicroseconds(1);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();

  long bits24 = (long)b0 << 16 | (long)b1 << 8 | b2;
  int32_t val = (bits24 << 8) >> 8;          // sign-extend 24->32

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

    AdsPacket pkt;
    pkt.ts_us  = (uint64_t)isr_ts_us[i];
    pkt.adc_id = (uint8_t)(all_configs[i].id - 'A');   // A=0..E=4 (host: ord-'A')
    pkt.ch1    = raw_values[i][0];
    pkt.ch2    = raw_values[i][1];
    std::vector<uint8_t> rec(1 + sizeof(AdsPacket));
    rec[0] = PKT_ADS;
    memcpy(rec.data() + 1, &pkt, sizeof(AdsPacket));
    outRecs.push_back(std::move(rec));
  }
}

// Throttled, non-blocking debug echo of the latest ADS pair per sensor
// (format: t,ch1,ch2,t,ch1,ch2,...). IMU/Mag are intentionally NOT echoed
// (they'd flood at ~7 kHz). Self-throttles via a static timer so it can be
// called every loop() iteration but only prints at ~100 Hz, and never blocks.
void printDataSerial() {
  if (!hasSerial) return;
  static unsigned long last_serial = 0;
  unsigned long now = millis();
  if (now - last_serial < BATCH_SEND_INTERVAL_MS) return;   // ~100 Hz cap
  last_serial = now;

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (i > 0) Serial.print(",");
    Serial.print((double)isr_ts_us[i] / 1000000.0, 6);
    Serial.print(","); Serial.print(raw_values[i][0]);
    Serial.print(","); Serial.print(raw_values[i][1]);
  }
  Serial.println();
}

// ====================================================================================
// ==================================  HELPERS SECTION  ===============================
// ====================================================================================
// Only the genuinely cross-cutting pieces live here: the queue push, pollSensors()
// (spans ADS + IMU + Mag), sendTCP(), and the disableInterrupts() used on the
// CONNECTING transition.

void enqueueRecord(std::vector<uint8_t>&& rec) {
  portENTER_CRITICAL(&queue_mux);
  packet_queue.push_back(std::move(rec));
  portEXIT_CRITICAL(&queue_mux);
}

// Non-blocking. Services any flagged ADS chips, drains the IMU FIFO, polls the Mag,
// and pushes every reading as a typed record. Builds records on the stack first,
// then enqueues, so the queue critical-section stays tiny.
void pollSensors() {
  std::vector<std::vector<uint8_t>> recs;
  
  // Debug
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (ads_ready[i]) {
      if (hasSerial) { Serial.print("[poll] svc chip "); Serial.println(i); }
      ads_ready[i] = false;
      serviceAdsChip(i, recs);
      if (hasSerial) { Serial.print("[poll] done chip "); Serial.println(i); }
    }
  }
  if (hasSerial) Serial.println("[poll] ads loop done");
  // Debug

  // --- ADS: read whichever chips the DRDY ISRs flagged ---
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (ads_ready[i]) {
      ads_ready[i] = false;
      serviceAdsChip(i, recs);
    }
  }

  // --- IMU: drain the on-chip FIFO (esp_timer-anchored timestamps) ---
  if (g_imu_ok) {
    ImuMagSpi::ImuSample s[64];
    int n = ImuMagSpi::drainImu(s, 64);
    for (int k = 0; k < n; k++) {
      ImuPacket pkt;
      pkt.ts_us = (uint64_t)s[k].ts_us;
      pkt.ax = s[k].ax; pkt.ay = s[k].ay; pkt.az = s[k].az;
      pkt.gx = s[k].gx; pkt.gy = s[k].gy; pkt.gz = s[k].gz;
      std::vector<uint8_t> rec(1 + sizeof(ImuPacket));
      rec[0] = PKT_IMU;
      memcpy(rec.data() + 1, &pkt, sizeof(ImuPacket));
      recs.push_back(std::move(rec));
    }
  }

  // --- Mag: one sample if STATUS says ready ---
  if (g_mag_ok) {
    ImuMagSpi::MagSample m;
    if (ImuMagSpi::readMag(&m)) {
      MagPacket pkt;
      pkt.ts_us = (uint64_t)m.ts_us;
      pkt.mx = m.mx; pkt.my = m.my; pkt.mz = m.mz;
      std::vector<uint8_t> rec(1 + sizeof(MagPacket));
      rec[0] = PKT_MAG;
      memcpy(rec.data() + 1, &pkt, sizeof(MagPacket));
      recs.push_back(std::move(rec));
    }
  }

  if (!recs.empty()) {
    portENTER_CRITICAL(&queue_mux);
    for (auto& r : recs) packet_queue.push_back(std::move(r));
    portEXIT_CRITICAL(&queue_mux);
  }
}

// Non-blocking. Emits ONE framed payload every 10 ms OR early when the queue grows
// near the frame cap. Frame = [len:2][seq:2][crc:2][payload]; payload = concatenated
// [type:1][packet] records. Returns without sending if nothing is queued.
void sendTCP() {
  unsigned long now = millis();

  size_t qsize;
  portENTER_CRITICAL(&queue_mux);
  qsize = packet_queue.size();
  portEXIT_CRITICAL(&queue_mux);

  bool timer_due = (now - last_batch_send >= BATCH_SEND_INTERVAL_MS);
  bool queue_full = (qsize >= QUEUE_FLUSH_THRESHOLD);
  if (!timer_due && !queue_full) return;
  last_batch_send = now;

  if (!client.connected()) return;

  // Move queued records out under the lock; build the frame outside it.
  std::vector<std::vector<uint8_t>> local;
  portENTER_CRITICAL(&queue_mux);
  if (packet_queue.empty()) { portEXIT_CRITICAL(&queue_mux); return; }
  local.reserve(packet_queue.size());
  while (!packet_queue.empty()) { local.push_back(std::move(packet_queue.front())); packet_queue.pop_front(); }
  portEXIT_CRITICAL(&queue_mux);

  size_t payload_len = 0;
  for (auto& r : local) payload_len += r.size();
  if (payload_len == 0) return;
  if (payload_len > 0xFFFF) {                 // u16 length guard (shouldn't happen)
    if (hasSerial) Serial.println("payload>64KB, dropping batch");
    return;
  }

  std::vector<uint8_t> payload;
  payload.reserve(payload_len);
  for (auto& r : local) payload.insert(payload.end(), r.begin(), r.end());

  uint16_t crc = computeCrc(payload.data(), payload.size());
  std::vector<uint8_t> frame;
  frame.reserve(6 + payload_len);
  uint16_t len = (uint16_t)payload_len;
  frame.push_back(len & 0xFF);        frame.push_back((len >> 8) & 0xFF);
  frame.push_back(seq_num & 0xFF);    frame.push_back((seq_num >> 8) & 0xFF);
  seq_num++;
  frame.push_back(crc & 0xFF);        frame.push_back((crc >> 8) & 0xFF);
  frame.insert(frame.end(), payload.begin(), payload.end());

  if (client.write(frame.data(), frame.size()) != frame.size()) {
    if (hasSerial) Serial.println("partial/failed send");
    return;
  }
  client.flush();
}

// Quiesce all acquisition interrupts for the CONNECTING state. ADS interrupts are
// detached here (mirroring the original firmware). IMU/Mag have NO interrupts in
// the FIFO model — instead we simply stop draining while disconnected and let the
// FIFO overflow harmlessly; resetFifo() on reconnect throws the stale data away.
// (This is the IMU/Mag analog of disabling the ADS interrupts, per your request.)
void disableInterrupts() {
  disableADCInterrupts();
  // IMU/Mag: nothing to detach. Draining stops because pollSensors() only runs in
  // CONNECTED. The FIFO is reset + re-anchored in connectToHost() on reconnect.
}

// ====================================================================================
// ====================================  setup / loop  ================================
// ====================================================================================
void setup() {
  Serial.begin(1000000);
  unsigned long t0 = millis();
  while (!Serial) { if (millis() - t0 > 5000) break; }
  hasSerial = Serial;
  if (hasSerial) {
    Serial.print("Reset reason: "); Serial.println((int)esp_reset_reason());
    Serial.print("Nano_ID: "); Serial.println(NANO_ID);
  }

  // Shared SPI bus (MOSI/MISO swap matches the PCB — do not "fix" without schematic).
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  initializeADCs();          // ADS up first (configs frozen); interrupts armed then off below
  disableADCInterrupts();    // stay quiet until we actually connect
  initImuMag();              // IMU FIFO + Mag continuous; no interrupts, no tasks
  initWiFiClient();          // station mode + scan + OTA config (begin() deferred)

  last_batch_send = millis();
}

void loop() {
  switch (WiFiState) {
    case CONNECTED: {
      // debug
      if (hasSerial) {
        static unsigned long hb = 0;
        if (millis() - hb >= 500) { Serial.println("[loop] CONNECTED tick"); hb = millis(); }
      }
      // debug
      pollSensors();
      sendTCP();
      monitorConnection();   // may transition back to CONNECTING
      printDataSerial();     // throttled + non-blocking; self-gates internally
    } break;

    case CONNECTING: {
      disableInterrupts();   // ADS off; IMU/Mag FIFO left to overflow harmlessly
      handleOTA();
      connectToHost();       // transitions to CONNECTED on success (resets FIFO, re-arms ADS)
    } break;
  }
}