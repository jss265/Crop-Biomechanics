// This code works with Pinout connections on Task Journal

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
  Serial.println("Initializing ISM330DHCX + LIS3MDL over SPI...");
  
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

  // === Recovery: ensure clean sensor state ===
  imu.reset();
  mag.reset();
  delay(10);

  // === IMU Configuration ===
  imu.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  imu.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
  imu.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  imu.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);

  // Force known interrupt pin characteristics (active-high, push-pull)
  imu.configIntOutputs(false, false);

  // Route Accel + Gyro DRDY to INT1
  imu.configInt1(false, true, true);

  // === Magnetometer Configuration ===
  mag.setRange(LIS3MDL_RANGE_4_GAUSS);
  mag.setDataRate(LIS3MDL_DATARATE_300_HZ);

  // === IMU Interrupt Pin ===
  pinMode(imuInt1, INPUT);
  attachInterrupt(digitalPinToInterrupt(imuInt1), handleImuInt1, FALLING);

  // === MAG Interrupt Pin ===
  pinMode(magDrdy, INPUT);
  attachInterrupt(digitalPinToInterrupt(magDrdy), handleMagDrdy, FALLING);

  Serial.println("IMU + MAG interrupts attached.\n");
}

void setup() {
  Serial.begin(921600);
  while (!Serial) delay(10);

  SPI.begin();

  initializeIMU9DOF(A0, A1, A2, A3);
}

void loop() {
  checkImuDrdy();
  checkMagDrdy();
}