#include "imu.h"
#include <Wire.h>

// Auto-detects an LSM6DS3 (preferred) or MPU6050 on any of the UNO Q's three
// I2C buses: Wire = SDA/SCL header, Wire1 = QWIIC (i2c4), Wire2 = A4/A5 (i2c3).
// We only consume gyro Z (yaw rate). Mount the board flat so Z is vertical.

// ---- LSM6DS3 registers -----------------------------------------------------
#define LSM_WHO_AM_I 0x0F     // -> 0x69 (DS3) or 0x6A (DS3TR-C)
#define LSM_CTRL2_G  0x11     // gyro ODR + full-scale
#define LSM_CTRL3_C  0x12     // BDU / IF_INC
#define LSM_OUTZ_L_G 0x26     // little-endian (L then H)
#define LSM_CTRL2_CFG 0x50    // ODR 208 Hz, FS 250 dps -> 0.00875 dps/LSB

// ---- MPU6050 registers -----------------------------------------------------
#define MPU_PWR1     0x6B     // power mgmt (boots in SLEEP -> must wake)
#define MPU_CONFIG   0x1A     // DLPF
#define MPU_GYRO_CFG 0x1B     // gyro full-scale
#define MPU_ZOUT_H   0x47     // big-endian (H then L)

#define CHIP_NONE 0
#define CHIP_LSM  1
#define CHIP_MPU  2

// Flip if the board is mounted so a left (CCW) turn reads negative.
// Calibrated 2026-05-31: a left/CCW turn read NEGATIVE, so invert to make CCW +.
static const float IMU_YAW_SIGN = -1.0f;
static const float DPS_TO_RAD   = (float)PI / 180.0f;

static TwoWire* s_bus  = nullptr;
static uint8_t  s_addr = 0;
static uint8_t  s_chip = CHIP_NONE;
static float    s_dpsPerLsb = 0.00875f;
static float    s_biasZ = 0.0f;           // raw-LSB bias (counts)

static void wr(uint8_t reg, uint8_t val) {
  s_bus->beginTransmission(s_addr);
  s_bus->write(reg); s_bus->write(val);
  s_bus->endTransmission();
}

static uint8_t rd(TwoWire* bus, uint8_t addr, uint8_t reg) {
  bus->beginTransmission(addr);
  bus->write(reg);
  bus->endTransmission(false);
  bus->requestFrom((int)addr, 1);
  return bus->available() ? bus->read() : 0;
}

static int16_t rdGyroZ() {
  uint8_t reg = (s_chip == CHIP_LSM) ? LSM_OUTZ_L_G : MPU_ZOUT_H;
  s_bus->beginTransmission(s_addr);
  s_bus->write(reg);
  s_bus->endTransmission(false);
  s_bus->requestFrom((int)s_addr, 2);
  uint8_t b0 = s_bus->available() ? s_bus->read() : 0;   // LSM: low  | MPU: high
  uint8_t b1 = s_bus->available() ? s_bus->read() : 0;   // LSM: high | MPU: low
  return (s_chip == CHIP_LSM) ? (int16_t)((b1 << 8) | b0)
                              : (int16_t)((b0 << 8) | b1);
}

bool imuBegin() {
  TwoWire* buses[] = { &Wire, &Wire1, &Wire2 };
  for (int b = 0; b < 3; b++) buses[b]->begin();

  for (int b = 0; b < 3; b++) {              // prefer LSM6DS3
    for (uint8_t a = 0x6A; a <= 0x6B; a++) {
      uint8_t who = rd(buses[b], a, LSM_WHO_AM_I);
      if (who == 0x69 || who == 0x6A) {
        s_bus = buses[b]; s_addr = a; s_chip = CHIP_LSM; s_dpsPerLsb = 0.00875f;
        wr(LSM_CTRL3_C, 0x44);               // BDU + IF_INC
        wr(LSM_CTRL2_G, LSM_CTRL2_CFG);
        delay(20);
        return true;
      }
    }
  }
  for (int b = 0; b < 3; b++) {              // fall back to MPU6050
    for (uint8_t a = 0x68; a <= 0x69; a++) {
      buses[b]->beginTransmission(a);
      if (buses[b]->endTransmission() == 0) { // clones lie about WHO_AM_I; trust the ACK
        s_bus = buses[b]; s_addr = a; s_chip = CHIP_MPU; s_dpsPerLsb = 1.0f / 131.0f;
        wr(MPU_PWR1, 0x01);                  // wake (boots asleep), PLL X-gyro clock
        wr(MPU_CONFIG, 0x03);                // DLPF ~41 Hz
        wr(MPU_GYRO_CFG, 0x00);              // FS +/-250 dps
        delay(20);
        return true;
      }
    }
  }
  s_chip = CHIP_NONE; s_addr = 0;
  return false;
}

// Average a stack of samples while stationary to capture the constant bias.
void imuCalibrateBias(uint16_t ms) {
  if (s_chip == CHIP_NONE) return;
  long   n = 0;
  double acc = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    acc += rdGyroZ();
    n++;
    delay(5);
  }
  s_biasZ = (n > 0) ? (float)(acc / n) : 0.0f;
}

float imuYawRate() {
  if (s_chip == CHIP_NONE) return 0.0f;
  float counts = (float)rdGyroZ() - s_biasZ;
  return IMU_YAW_SIGN * counts * s_dpsPerLsb * DPS_TO_RAD;   // rad/s
}
