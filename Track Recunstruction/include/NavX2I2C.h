#ifndef NAVX2_I2C_H
#define NAVX2_I2C_H

/*
NavX2I2C.h

Purpose
- Lightweight navX2 I2C wrapper that reads the navX register block in one burst and
  exposes a compact, polling-based API for higher-level code.

Required files and where to get them
- IMURegisters.h and its protocol helpers IMUProtocol.h 
  are part of the Kauai Labs navX-MXP Arduino source.
  Online source: https://github.com/kauailabs/navxmxp/tree/master/arduino/navX_I2C 
  (libraries/navxmxp/arduino/navX_I2C)
  This header includes IMURegisters.h directly; the others are optional unless you
  want additional decode helpers or protocol context.

Optional references for understanding the protocol (not required at build time)
- AHRSProtocol.h and IMUProtocol.h from the same repo, for protocol definitions.
- navX I2C Arduino example (navX_I2C.ino) in the navX-MXP examples folder.
  These are cited only to aid future engineers; this header does not depend on them.

Organization and flow
- Public API
  - begin(): init I2C, set bus speed, and kick off the first update.
  - update(): poll and read the navX register block into raw_data_, then decode
    fields into Sample.
  - sample()/available()/fresh(): data access and validity flags.
  - printCsv()/appendBinaryPayload(): reporting helpers.
- Private helpers
  - readRegisters(): performs the I2C write(start,len) + requestFrom burst read,
    mirroring the working sanity-check sketch.
  - readU16/readU32/readI16/readI32: little-endian decode utilities.

Implementation notes
- The register map is aligned to IMURegisters.h constants so the read window
  matches NAVX_REG_OP_STATUS through NAVX_REG_LAST.
- All multi-byte values are little-endian; signed values use two's complement.
- Timing is controlled by poll_interval_us_ (defaults to 5 ms).
*/

#include <Arduino.h>
#include <Wire.h>
#include <vector>
#include "IMURegisters.h"

class NavX2I2C {
public:
  static const uint8_t DEFAULT_ADDRESS = 0x32;
  static const uint8_t DEFAULT_UPDATE_RATE_HZ = 200;
  static const uint16_t DEFAULT_POLL_INTERVAL_US = 5000;

  struct Sample {
    uint32_t timestamp_us = 0;
    uint32_t navx_timestamp_ms = 0;
    int16_t yaw_cd = 0;       // centidegrees
    int16_t pitch_cd = 0;     // centidegrees
    int16_t roll_cd = 0;      // centidegrees
    uint16_t heading_cd = 0;  // centidegrees, 0..35999
    uint16_t fused_heading_cd = 0;
    int16_t accel_x_mg = 0;   // milli-g, gravity removed/world-frame linear accel
    int16_t accel_y_mg = 0;
    int16_t accel_z_mg = 0;
    int16_t quat_w = 0;       // signed value scaled by 16384.0
    int16_t quat_x = 0;
    int16_t quat_y = 0;
    int16_t quat_z = 0;
    int32_t disp_x_1616 = 0;  // 16:16 fixed-point meters
    int32_t disp_y_1616 = 0;
    int32_t disp_z_1616 = 0;
    uint8_t op_status = 0;
    uint8_t cal_status = 0;
    uint8_t selftest_status = 0;
    uint16_t sensor_status = 0;
    uint16_t capability_flags = 0;
    bool valid = false;
    bool fresh = false;
  };

  explicit NavX2I2C(
      TwoWire& wire = Wire,
      uint8_t address = DEFAULT_ADDRESS,
      uint8_t update_rate_hz = DEFAULT_UPDATE_RATE_HZ,
      uint16_t poll_interval_us = DEFAULT_POLL_INTERVAL_US)
      : wire_(wire),
        address_(address),
        update_rate_hz_(update_rate_hz),
        poll_interval_us_(poll_interval_us) {}

  bool begin(uint32_t i2c_clock_hz = 400000) {
    wire_.begin();
    wire_.setClock(i2c_clock_hz);
    last_poll_us_ = micros() - poll_interval_us_;
    return update();
  }

  void setTimeOrigin(uint32_t time_origin_us) {
    time_origin_us_ = time_origin_us;
  }

  bool update() {
    const uint32_t now_us = micros();
    if ((uint32_t)(now_us - last_poll_us_) < poll_interval_us_) {
      sample_.fresh = false;
      return false;
    }
    last_poll_us_ = now_us;

    if (!readRegisters(FIRST_DATA_REGISTER, raw_data_, NAVX_BYTES_TO_READ)) {
      sample_.valid = false;
      sample_.fresh = false;
      return false;
    }

    const uint32_t navx_ts = readU32(regPtr(REG_TIMESTAMP));
    sample_.fresh = sample_.valid && (navx_ts != last_navx_timestamp_ms_);
    last_navx_timestamp_ms_ = navx_ts;

    sample_.timestamp_us = now_us - time_origin_us_;
    sample_.navx_timestamp_ms = navx_ts;
    sample_.op_status = *regPtr(REG_OP_STATUS);
    sample_.cal_status = *regPtr(REG_CAL_STATUS);
    sample_.selftest_status = *regPtr(REG_SELFTEST_STATUS);
    sample_.sensor_status = readU16(regPtr(REG_SENSOR_STATUS));
    sample_.capability_flags = readU16(regPtr(REG_CAPABILITY_FLAGS));
    sample_.yaw_cd = readI16(regPtr(REG_YAW));
    sample_.pitch_cd = readI16(regPtr(REG_PITCH));
    sample_.roll_cd = readI16(regPtr(REG_ROLL));
    sample_.heading_cd = readU16(regPtr(REG_HEADING));
    sample_.fused_heading_cd = readU16(regPtr(REG_FUSED_HEADING));
    sample_.accel_x_mg = readI16(regPtr(REG_LINEAR_ACCEL_X));
    sample_.accel_y_mg = readI16(regPtr(REG_LINEAR_ACCEL_Y));
    sample_.accel_z_mg = readI16(regPtr(REG_LINEAR_ACCEL_Z));
    sample_.quat_w = readI16(regPtr(REG_QUAT_W));
    sample_.quat_x = readI16(regPtr(REG_QUAT_X));
    sample_.quat_y = readI16(regPtr(REG_QUAT_Y));
    sample_.quat_z = readI16(regPtr(REG_QUAT_Z));
    sample_.disp_x_1616 = readI32(regPtr(REG_DISP_X));
    sample_.disp_y_1616 = readI32(regPtr(REG_DISP_Y));
    sample_.disp_z_1616 = readI32(regPtr(REG_DISP_Z));
    sample_.valid = true;
    return true;
  }

  const Sample& sample() const {
    return sample_;
  }

  bool available() const {
    return sample_.valid;
  }

  bool fresh() const {
    return sample_.fresh;
  }

  float yawDeg() const {
    return sample_.yaw_cd * 0.01f;
  }

  float pitchDeg() const {
    return sample_.pitch_cd * 0.01f;
  }

  float rollDeg() const {
    return sample_.roll_cd * 0.01f;
  }

  float quatW() const {
    return sample_.quat_w / 16384.0f;
  }

  float quatX() const {
    return sample_.quat_x / 16384.0f;
  }

  float quatY() const {
    return sample_.quat_y / 16384.0f;
  }

  float quatZ() const {
    return sample_.quat_z / 16384.0f;
  }

  float dispXMeters() const {
    return sample_.disp_x_1616 / 65536.0f;
  }

  float dispYMeters() const {
    return sample_.disp_y_1616 / 65536.0f;
  }

  float dispZMeters() const {
    return sample_.disp_z_1616 / 65536.0f;
  }

  bool isCalibrating() const {
    return (sample_.cal_status & NAVX_CAL_STATUS_IMU_CAL_COMPLETE) == 0;
  }

  bool resetYaw() {
    return writeRegister(REG_INTEGRATION_CTL, NAVX_INTEGRATION_CTL_RESET_YAW);
  }

  size_t binaryPayloadBytes() const {
    return sizeof(uint8_t) + (sizeof(uint32_t) * 2) + (sizeof(int16_t) * 12) +
           (sizeof(int32_t) * 3) + (sizeof(uint16_t) * 2) + (sizeof(uint8_t) * 2);
  }

  void appendBinaryPayload(std::vector<uint8_t>& payload) const {
    const uint8_t flags = sample_.valid ? (sample_.fresh ? 0x03 : 0x01) : 0x00;
    appendPod(payload, flags);
    appendPod(payload, sample_.timestamp_us);
    appendPod(payload, sample_.navx_timestamp_ms);
    appendPod(payload, sample_.yaw_cd);
    appendPod(payload, sample_.pitch_cd);
    appendPod(payload, sample_.roll_cd);
    appendPod(payload, sample_.heading_cd);
    appendPod(payload, sample_.fused_heading_cd);
    appendPod(payload, sample_.accel_x_mg);
    appendPod(payload, sample_.accel_y_mg);
    appendPod(payload, sample_.accel_z_mg);
    appendPod(payload, sample_.quat_w);
    appendPod(payload, sample_.quat_x);
    appendPod(payload, sample_.quat_y);
    appendPod(payload, sample_.quat_z);
    appendPod(payload, sample_.disp_x_1616);
    appendPod(payload, sample_.disp_y_1616);
    appendPod(payload, sample_.disp_z_1616);
    appendPod(payload, sample_.sensor_status);
    appendPod(payload, sample_.capability_flags);
    appendPod(payload, sample_.cal_status);
    appendPod(payload, sample_.selftest_status);
  }

  void printCsv(Stream& out) const {
    out.print(sample_.valid ? 1 : 0);
    out.print(',');
    out.print(sample_.timestamp_us / 1000000.0f, 6);
    out.print(',');
    out.print(yawDeg(), 2);
    out.print(',');
    out.print(pitchDeg(), 2);
    out.print(',');
    out.print(rollDeg(), 2);
    out.print(',');
    out.print(sample_.heading_cd * 0.01f, 2);
    out.print(',');
    out.print(sample_.accel_x_mg / 1000.0f, 3);
    out.print(',');
    out.print(sample_.accel_y_mg / 1000.0f, 3);
    out.print(',');
    out.print(sample_.accel_z_mg / 1000.0f, 3);
    out.print(',');
    out.print(dispXMeters(), 4);
    out.print(',');
    out.print(dispYMeters(), 4);
    out.print(',');
    out.print(dispZMeters(), 4);
    out.print(',');
    out.print(quatW(), 4);
    out.print(',');
    out.print(quatX(), 4);
    out.print(',');
    out.print(quatY(), 4);
    out.print(',');
    out.print(quatZ(), 4);
  }

private:
  static const uint8_t REG_UPDATE_RATE_HZ = NAVX_REG_UPDATE_RATE_HZ;
  static const uint8_t REG_OP_STATUS = NAVX_REG_OP_STATUS;
  static const uint8_t REG_CAL_STATUS = NAVX_REG_CAL_STATUS;
  static const uint8_t REG_SELFTEST_STATUS = NAVX_REG_SELFTEST_STATUS;
  static const uint8_t REG_SENSOR_STATUS = NAVX_REG_SENSOR_STATUS_L;
  static const uint8_t REG_TIMESTAMP = NAVX_REG_TIMESTAMP_L_L;
  static const uint8_t REG_YAW = NAVX_REG_YAW_L;
  static const uint8_t REG_PITCH = NAVX_REG_PITCH_L;
  static const uint8_t REG_ROLL = NAVX_REG_ROLL_L;
  static const uint8_t REG_HEADING = NAVX_REG_HEADING_L;
  static const uint8_t REG_FUSED_HEADING = NAVX_REG_FUSED_HEADING_L;
  static const uint8_t REG_LINEAR_ACCEL_X = NAVX_REG_LINEAR_ACC_X_L;
  static const uint8_t REG_LINEAR_ACCEL_Y = NAVX_REG_LINEAR_ACC_Y_L;
  static const uint8_t REG_LINEAR_ACCEL_Z = NAVX_REG_LINEAR_ACC_Z_L;
  static const uint8_t REG_QUAT_W = NAVX_REG_QUAT_W_L;
  static const uint8_t REG_QUAT_X = NAVX_REG_QUAT_X_L;
  static const uint8_t REG_QUAT_Y = NAVX_REG_QUAT_Y_L;
  static const uint8_t REG_QUAT_Z = NAVX_REG_QUAT_Z_L;
  static const uint8_t REG_CAPABILITY_FLAGS = NAVX_REG_CAPABILITY_FLAGS_L;
  static const uint8_t REG_DISP_X = NAVX_REG_DISP_X_I_L;
  static const uint8_t REG_DISP_Y = NAVX_REG_DISP_Y_I_L;
  static const uint8_t REG_DISP_Z = NAVX_REG_DISP_Z_I_L;
  static const uint8_t REG_INTEGRATION_CTL = NAVX_REG_INTEGRATION_CTL;
  static const uint8_t FIRST_DATA_REGISTER = NAVX_REG_OP_STATUS;
  static const uint8_t LAST_DATA_REGISTER = NAVX_REG_LAST;
  static const uint8_t NAVX_BYTES_TO_READ = (LAST_DATA_REGISTER - FIRST_DATA_REGISTER) + 1;

  TwoWire& wire_;
  const uint8_t address_;
  const uint8_t update_rate_hz_;
  const uint16_t poll_interval_us_;
  uint32_t time_origin_us_ = 0;
  uint32_t last_poll_us_ = 0;
  uint32_t last_navx_timestamp_ms_ = 0;
  uint8_t raw_data_[NAVX_BYTES_TO_READ] = {0};
  Sample sample_;

  bool writeRegister(uint8_t reg, uint8_t value) {
    wire_.beginTransmission(address_);
    wire_.write(reg);
    wire_.write(value);
    return wire_.endTransmission() == 0;
  }

  bool readRegisters(uint8_t start_reg, uint8_t* dst, size_t len) {
    if (len > 255) {
      return false;
    }

    wire_.beginTransmission(address_);
    wire_.write(start_reg);
    wire_.write((uint8_t)len);
    if (wire_.endTransmission() != 0) {
      return false;
    }

    int received = wire_.requestFrom(address_, (uint8_t)len);
    if (received != (int)len) {
      return false;
    }

    size_t i = 0;
    while (wire_.available() && i < len) {
      dst[i++] = (uint8_t)wire_.read();
    }
    return i == len;
  }

  const uint8_t* regPtr(uint8_t reg) const {
    return &raw_data_[reg - FIRST_DATA_REGISTER];
  }

  static uint16_t readU16(const uint8_t* p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
  }

  static int16_t readI16(const uint8_t* p) {
    return (int16_t)readU16(p);
  }

  static uint32_t readU32(const uint8_t* p) {
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
  }

  static int32_t readI32(const uint8_t* p) {
    return (int32_t)readU32(p);
  }

  template <typename T>
  static void appendPod(std::vector<uint8_t>& payload, const T& value) {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&value);
    payload.insert(payload.end(), p, p + sizeof(T));
  }
};

// NOTE: If you prefer clearer decoding, you can swap the manual readU16/readI32
// usage for IMURegisters::decodeProtocol* helpers. This is optional and mainly
// improves readability, not performance.

#endif
