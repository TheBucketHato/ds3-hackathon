/*
  imu_v2 — MPU-6050 IMU Reader
  Adafruit MPU6050 library
  Arduino Uno Q

  Wiring:
    MPU-6050  -->  Uno Q
    VCC       -->  3.3V
    GND       -->  GND
    SCL       -->  SCL
    SDA       -->  SDA
    AD0       -->  GND
*/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10);

  Serial.println("Initializing MPU-6050...");

  if (!mpu.begin()) {
    Serial.println("ERROR: MPU-6050 not found. Check wiring.");
    while (1) delay(10);
  }

  Serial.println("MPU-6050 connected!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  delay(100);
  Serial.println("\n  ax(m/s²)  ay(m/s²)  az(m/s²) | gx(°/s)  gy(°/s)  gz(°/s) | temp(°C)");
  Serial.println("--------------------------------------------------------------------------------");
}

void loop() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  Serial.print("  "); Serial.print(accel.acceleration.x, 3);
  Serial.print("    ");  Serial.print(accel.acceleration.y, 3);
  Serial.print("    ");  Serial.print(accel.acceleration.z, 3);
  Serial.print("  |  "); Serial.print(gyro.gyro.x, 2);
  Serial.print("    ");  Serial.print(gyro.gyro.y, 2);
  Serial.print("    ");  Serial.print(gyro.gyro.z, 2);
  Serial.print("  |  "); Serial.println(temp.temperature, 1);

  delay(100);
}
