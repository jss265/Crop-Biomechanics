#include <Arduino.h>
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_LIS3MDL.h>
#include <SPI.h>

// ==================== USER CONFIG ====================
#define IMU_CS   A0     // Change to your actual CS pin for ISM330DHCX
#define MAG_CS   A1     // Change to your actual CS pin for LIS3MDL

// Optional interrupt pins (connect if you want to use them)
#define IMU_INT1  A2
#define MAG_DRDY  A3
// =====================================================

Adafruit_ISM330DHCX imu;
Adafruit_LIS3MDL    mag;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Initializing ISM330DHCX + LIS3MDL over SPI...");

  // Initialize SPI bus
  SPI.begin();
  delay(1000);

  // Initialize sensors using SPI + CS pin
  imu.begin_SPI(IMU_CS);
  //   if (!imu.begin_SPI(IMU_CS)) {
  //     Serial.println("Failed to find ISM330DHCX chip");
  //     while (1) delay(10);
  //   }
  mag.begin_SPI(MAG_CS);
  //   if (!mag.begin_SPI(MAG_CS)) {
  //     Serial.println("Failed to find LIS3MDL chip");
  //     while (1) delay(10);
  //   }

  //   Serial.println("Both sensors found!");

  // === IMU Configuration (using adafruit_lsm6ds) ===
  imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
  imu.setGyroDataRate(LSM6DS_RATE_104_HZ);
  imu.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  imu.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);

  mag.setDataRate(LIS3MDL_DATARATE_300_HZ); // or 560/1000 Hz later
  mag.setRange(LIS3MDL_RANGE_4_GAUSS);

  // Optional: enable interrupts later for high-rate / no-drop performance
  // imu.configInt1(....);   // FIFO watermark or Data Ready
  // attachInterrupt(digitalPinToInterrupt(IMU_INT1), imuISR, FALLING);

  Serial.println("Setup complete. Reading data...\n");
}

void loop() {
  // === Read IMU ===
  sensors_event_t accel, gyro, temp;
  imu.getEvent(&accel, &gyro, &temp);

  Serial.print("Accel X:"); Serial.print(accel.acceleration.x); Serial.print(" ");
  Serial.print("Y:"); Serial.print(accel.acceleration.y); Serial.print(" ");
  Serial.print("Z:"); Serial.print(accel.acceleration.z); Serial.print("  |  ");

  Serial.print("Gyro X:"); Serial.print(gyro.gyro.x); Serial.print(" ");
  Serial.print("Y:"); Serial.print(gyro.gyro.y); Serial.print(" ");
  Serial.print("Z:"); Serial.println(gyro.gyro.z);

  // === Read Magnetometer ===
  mag.read();
  Serial.print("Mag X:"); Serial.print(mag.x); Serial.print(" ");
  Serial.print("Y:"); Serial.print(mag.y); Serial.print(" ");
  Serial.print("Z:"); Serial.println(mag.z);

  Serial.println();
  delay(100);   // Slow for initial testing. Remove or reduce later.
}