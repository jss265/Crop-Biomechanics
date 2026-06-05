// This code works with Pinout connections on Task Jounral

#include <Arduino.h>
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_LIS3MDL.h>
#include <SPI.h>

volatile bool imuDataReady = false;
volatile bool magDataReady = false;
volatile uint32_t imuTimeUs = 0;
volatile uint32_t magTimeUs = 0;
Adafruit_ISM330DHCX imu;
Adafruit_LIS3MDL    mag;

void IRAM_ATTR handleImuInt1() {
  imuDataReady = true;
  imuTimeUs = micros();
}

void IRAM_ATTR handleMagDrdy() {
  magDataReady = true;
  magTimeUs = micros();
}

void checkImuDrdy() {
  if (imuDataReady) {
    imuDataReady = false;

    float ax, ay, az, gx, gy, gz;
    imu.readAcceleration(ax, ay, az);
    imu.readGyroscope(gx, gy, gz);

    Serial.printf(
        "IMU,%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
        imuTimeUs,
        ax, ay, az,
        gx, gy, gz
    );
  }
}

void checkMagDrdy() {
  if (magDataReady) {
    magDataReady = false;

    float mx, my, mz;
    mag.readMagneticField(mx, my, mz);

    Serial.printf(
        "MAG,%lu,%.6f,%.6f,%.6f\n",
        magTimeUs,
        mx, my, mz
    );
  }
}

void initializeIMU9DOF(int imu_cs, int mag_cs, int imuInt1, int magDrdy) {
  // Initialize sensors using SPI + CS pin
  if (!imu.begin_SPI(imu_cs)) {
    Serial.println("Failed to find ISM330DHCX chip");
    while (1) delay(10);
  }
  if (!mag.begin_SPI(mag_cs)) {
    Serial.println("Failed to find LIS3MDL chip");
    while (1) delay(10);
  }

  Serial.println("Both ISM330DHCX + LIS3MDL sensors found!");

  // === IMU Configuration (using adafruit_lsm6ds for ISM330DHCX) ===
  imu.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
  imu.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  imu.setGyroDataRate(LSM6DS_RATE_104_HZ);

  // === Magnetometer Configurations ===
  mag.setRange(LIS3MDL_RANGE_4_GAUSS);
  mag.setDataRate(LIS3MDL_DATARATE_300_HZ); // or 560/1000 Hz later

  // === Setup IMU Interrupt ===
  pinMode(imuInt1, INPUT);
  imu.configInt1(false, true, true);   // Accel + Gyro DRDY on Int1
  attachInterrupt(digitalPinToInterrupt(imuInt1), handleImuInt1, FALLING);
  
  // === Setup MAG Interrup ===
  pinMode(magDrdy, INPUT);
  attachInterrupt(digitalPinToInterrupt(magDrdy), handleMagDrdy, FALLING);

  Serial.println("IMU + MAG interrupts attahed.\n");
}

void setup() {
  Serial.begin(921600);
  while (!Serial) delay(10);

  Serial.println("Initializing ISM330DHCX + LIS3MDL over SPI...");

  // Initialize SPI bus
  SPI.begin();

  initializeIMU9DOF(A0, A1, A2, A3);
}

void loop() {
  checkImuDrdy();
  checkMagDrdy();
}