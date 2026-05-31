/*
 * imu.h — 6-DOF yaw-rate source over raw I2C, no library.
 *
 * Auto-detects an LSM6DS3 (preferred) or MPU6050 on any of the UNO Q's three
 * I2C buses (Wire = SDA/SCL header, Wire1 = QWIIC, Wire2 = A4/A5). We only
 * consume gyro Z (yaw rate); heading fusion to the SLAM frame is done in the
 * sketch using vision. Mount the board flat so Z is vertical.
 */
#pragma once
#include <Arduino.h>

bool  imuBegin();                 // returns false if WHO_AM_I doesn't match
void  imuCalibrateBias(uint16_t ms);   // robot MUST be still during this
float imuYawRate();               // rad/s, bias-corrected, sign-adjusted
