/*
 * sketch.ino — Interactive calibration helper for the lora-2 robot.
 *
 * Flash once, then drive it by typing single keys in the serial monitor
 * (`arduino-app-cli monitor`). Each routine prints what to do and the number
 * to copy into the real firmware. Self-contained: inlines the TB6612 + LSM6DS3
 * code so it doesn't depend on the main app's motor.cpp / imu.cpp.
 *
 * `Serial` here is the RouterBridge Monitor (RPC to Linux), not a UART.
 *
 * Pins MUST match the real firmware:
 *   TB6612: PWMA=D5 AIN1=D4 AIN2=D3  PWMB=D9 BIN1=D6 BIN2=D7  STBY=A1
 *   IMU LSM6DS3: I2C SDA=A4 SCL=A5
 *
 * Menu:
 *   i  IMU: bias + heading sign + scale (rotate the robot by hand)
 *   m  Motor wiring/direction check (each wheel forward briefly)
 *   v  Top speed run -> V_MAX_MPS  (drives forward 3 s, you measure distance)
 *   s  Stiction ramp -> MIN_MOVE_PWM (lift wheels; note when they start)
 *   w  Spin test -> effective WHEELBASE (needs your V_MAX from 'v')
 *   h  reprint this menu
 */

#include <Arduino_RouterBridge.h>
#include <Wire.h>

// ---- TB6612 pins (Elegoo Smart Robot Car V4.0, from its stock firmware) -----
// One direction pin per channel (board ties the 2nd input internally).
// Group A = RIGHT motors, Group B = LEFT motors. DIR HIGH = forward.
#define PIN_PWMA   5    // right speed
#define PIN_AIN1   7    // right direction
#define PIN_PWMB   6    // left speed
#define PIN_BIN1   8    // left direction
#define PIN_STBY   3    // enable (HIGH = run)

// ---- IMU: auto-detect LSM6DS3 (0x6A/0x6B) or MPU6050 (0x68/0x69) ------------
// LSM6DS3 regs
#define LSM_WHO_AM_I 0x0F
#define LSM_CTRL2_G  0x11
#define LSM_CTRL3_C  0x12
#define LSM_OUTZ_L_G 0x26                 // little-endian (L then H)
#define LSM_CTRL2_CFG 0x50                // ODR 208 Hz, FS 250 dps -> 0.00875 dps/LSB
// MPU6050 regs
#define MPU_WHO_AM_I 0x75
#define MPU_PWR1     0x6B
#define MPU_CONFIG   0x1A
#define MPU_GYRO_CFG 0x1B
#define MPU_ZOUT_H   0x47                 // big-endian (H then L)

#define CHIP_NONE 0
#define CHIP_LSM  1
#define CHIP_MPU  2

static uint8_t imuChip = CHIP_NONE;
static uint8_t imuAddr = 0;
static float   biasZ   = 0.0f;            // raw counts
static float   gyroDpsPerLsb = 0.00875f;  // set per chip

// ============================ TB6612 ========================================
static void motorInit() {
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_PWMA, OUTPUT); pinMode(PIN_PWMB, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, LOW);
}

// fwd=true spins that wheel in the robot-forward direction.
static void wheel(int pwmPin, int dirPin, int pwm, bool fwd) {
  digitalWrite(dirPin, fwd ? HIGH : LOW);
  analogWrite(pwmPin, pwm);
}

static void drive(int lpwm, bool lfwd, int rpwm, bool rfwd) {
  digitalWrite(PIN_STBY, HIGH);
  wheel(PIN_PWMB, PIN_BIN1, lpwm, lfwd);   // left  = group B
  wheel(PIN_PWMA, PIN_AIN1, rpwm, rfwd);   // right = group A
}

static void motorStop() {
  analogWrite(PIN_PWMA, 0); analogWrite(PIN_PWMB, 0);
  digitalWrite(PIN_STBY, LOW);
}

// ===================== IMU (multi-bus, LSM6DS3 or MPU6050) ==================
// The UNO Q exposes three I2C buses: Wire = SDA/SCL header, Wire1 = QWIIC
// connector (i2c4), Wire2 = A4/A5 (i2c3). We scan all three.
static TwoWire* const BUSES[] = { &Wire, &Wire1, &Wire2 };
static const char*  const BUS_NAMES[] = { "Wire (SDA/SCL header)",
                                          "Wire1 (QWIIC)",
                                          "Wire2 (A4/A5)" };
static const int NBUS = 3;
static TwoWire* imuBus = &Wire;           // set by imuInit() to the live bus

static void iwr(uint8_t reg, uint8_t val) {
  imuBus->beginTransmission(imuAddr); imuBus->write(reg); imuBus->write(val);
  imuBus->endTransmission();
}
static uint8_t ird(TwoWire* bus, uint8_t addr, uint8_t reg) {
  bus->beginTransmission(addr); bus->write(reg); bus->endTransmission(false);
  bus->requestFrom((int)addr, 1);
  return bus->available() ? bus->read() : 0;
}
static int16_t gyroZraw() {
  uint8_t reg = (imuChip == CHIP_LSM) ? LSM_OUTZ_L_G : MPU_ZOUT_H;
  imuBus->beginTransmission(imuAddr); imuBus->write(reg); imuBus->endTransmission(false);
  imuBus->requestFrom((int)imuAddr, 2);
  uint8_t b0 = imuBus->available() ? imuBus->read() : 0;   // LSM: low  | MPU: high
  uint8_t b1 = imuBus->available() ? imuBus->read() : 0;   // LSM: high | MPU: low
  return (imuChip == CHIP_LSM) ? (int16_t)((b1 << 8) | b0)
                               : (int16_t)((b0 << 8) | b1);
}
static float gyroZdps() { return ((float)gyroZraw() - biasZ) * gyroDpsPerLsb; }

// Scan every address on all three buses and print what's there.
static void scanI2C() {
  int total = 0;
  for (int b = 0; b < NBUS; b++) {
    BUSES[b]->begin();
    Serial.print("Scanning "); Serial.print(BUS_NAMES[b]); Serial.println("...");
    for (uint8_t a = 1; a < 127; a++) {
      BUSES[b]->beginTransmission(a);
      if (BUSES[b]->endTransmission() == 0) {
        Serial.print("  device at 0x"); Serial.println(a, HEX);
        total++;
      }
    }
  }
  if (total == 0)
    Serial.println("Nothing on any bus -> check 3V3 power, GND, SDA/SCL not swapped.");
  else
    Serial.println("LSM6DS3 @ 0x6A/0x6B, MPU6050 @ 0x68/0x69.");
}

// Prefer the LSM6DS3 (our chosen chip) on any bus; fall back to MPU6050.
static bool imuInit() {
  for (int b = 0; b < NBUS; b++) BUSES[b]->begin();

  for (int b = 0; b < NBUS; b++) {            // pass 1: LSM6DS3
    for (uint8_t a = 0x6A; a <= 0x6B; a++) {
      uint8_t who = ird(BUSES[b], a, LSM_WHO_AM_I);
      if (who == 0x69 || who == 0x6A) {
        imuBus = BUSES[b]; imuAddr = a; imuChip = CHIP_LSM;
        gyroDpsPerLsb = 0.00875f;
        iwr(LSM_CTRL3_C, 0x44);               // BDU + IF_INC
        iwr(LSM_CTRL2_G, LSM_CTRL2_CFG);
        delay(20);
        Serial.print("LSM6DS3 on "); Serial.print(BUS_NAMES[b]);
        Serial.print(" @ 0x"); Serial.println(a, HEX);
        return true;
      }
    }
  }
  for (int b = 0; b < NBUS; b++) {            // pass 2: MPU6050
    for (uint8_t a = 0x68; a <= 0x69; a++) {
      BUSES[b]->beginTransmission(a);
      if (BUSES[b]->endTransmission() == 0) { // clones lie about WHO_AM_I; trust the ACK
        imuBus = BUSES[b]; imuAddr = a; imuChip = CHIP_MPU;
        gyroDpsPerLsb = 1.0f / 131.0f;        // FS +/-250 dps
        iwr(MPU_PWR1, 0x01);                  // wake (boots asleep!), PLL X-gyro clock
        iwr(MPU_CONFIG, 0x03);                // DLPF ~41 Hz
        iwr(MPU_GYRO_CFG, 0x00);              // FS +/-250 dps
        delay(20);
        Serial.print("MPU6050 on "); Serial.print(BUS_NAMES[b]);
        Serial.print(" @ 0x"); Serial.println(a, HEX);
        return true;
      }
    }
  }
  imuChip = CHIP_NONE; imuAddr = 0;
  Serial.println("No IMU found on any bus. Scanning to show what's present:");
  scanI2C();
  return false;
}

static void calibBias(uint16_t ms) {
  long n = 0; double acc = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < ms) { acc += gyroZraw(); n++; delay(5); }
  biasZ = n ? (float)(acc / n) : 0.0f;
}

// Discard any buffered input (e.g. the newline after the menu key).
static void drainSerial() { while (Serial.available()) Serial.read(); }

// abort helper: true only if a real key was pressed (newlines/spaces ignored,
// so the Enter that submitted the menu command doesn't instantly abort).
static bool keyPressed() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\r' && c != '\n' && c != ' ') return true;
  }
  return false;
}

// ============================ routines ======================================
static void runImu() {
  Serial.println();
  Serial.println("== IMU calibration ==");
  if (!imuAddr && !imuInit()) return;
  Serial.println("Hold the robot STILL. Measuring gyro bias (2 s)...");
  calibBias(2000);
  Serial.print("Gyro Z bias = "); Serial.print(biasZ, 1);
  Serial.print(" counts ("); Serial.print(biasZ * gyroDpsPerLsb, 3);
  Serial.println(" deg/s). Lower is better.");
  Serial.println("Now streaming 15 s. (1) leave still -> heading should barely move.");
  Serial.println("(2) Rotate the robot ~90 deg to the LEFT (counter-clockwise, viewed");
  Serial.println("    from above). Heading should INCREASE. (3) optionally spin a full");
  Serial.println("    360 deg to check scale. Press any key to stop early.");
  float heading = 0.0f;
  uint32_t last = micros(), t0 = millis(), lastPrint = 0;
  drainSerial();
  while (millis() - t0 < 15000) {
    uint32_t now = micros();
    float dt = (now - last) * 1e-6f; last = now;
    float dps = gyroZdps();
    heading += dps * dt;
    if (millis() - lastPrint >= 250) {
      lastPrint = millis();
      Serial.print("  rate "); Serial.print(dps, 1);
      Serial.print(" deg/s   heading "); Serial.print(heading, 1); Serial.println(" deg");
    }
    if (keyPressed()) break;
    delay(5);
  }
  Serial.println("Result:");
  Serial.println("  * If a LEFT/CCW turn made heading go POSITIVE -> keep IMU_YAW_SIGN=+1.");
  Serial.println("    If it went NEGATIVE -> set IMU_YAW_SIGN=-1 in imu.cpp.");
  Serial.println("  * If a physical 360 deg read far from +/-360 -> note the scale error.");
}

static void runMotorDir() {
  Serial.println();
  Serial.println("== Motor wiring / direction ==");
  Serial.println("Lift the wheels off the ground. Watching for FORWARD spin on each side.");
  Serial.println("LEFT wheel forward (2 s)...");
  drive(200, true, 0, true); delay(2000); motorStop(); delay(800);
  Serial.println("RIGHT wheel forward (2 s)...");
  drive(0, true, 200, true); delay(2000); motorStop();
  Serial.println("Each wheel should have spun so the robot would roll FORWARD.");
  Serial.println("If a wheel spun BACKWARD: swap that motor's two wires on the TB6612");
  Serial.println("(or swap its INx pins). Re-run with 'm' until both go forward.");
}

static void runSpeed() {
  Serial.println();
  Serial.println("== Top speed -> V_MAX_MPS ==");
  Serial.println("Put the robot on the floor with >2 m clear ahead. Mark the start point.");
  Serial.println("Full-power straight run for 3.0 s in 3...");
  delay(1000); Serial.println("2..."); delay(1000); Serial.println("1..."); delay(1000);
  Serial.println("GO");
  drive(255, true, 255, true); delay(3000); motorStop();
  Serial.println("STOP. Measure the distance traveled (meters).");
  Serial.println("  V_MAX_MPS = distance / 3.0   -> put in motor.cpp (and MAX_V in sketch.ino).");
}

static void runStiction() {
  Serial.println();
  Serial.println("== Stiction -> MIN_MOVE_PWM ==");
  Serial.println("Lift the wheels off the ground. Ramping PWM 0 -> 120.");
  Serial.println("Note the PWM where the wheels JUST start turning. Press any key to stop.");
  drainSerial();
  for (int pwm = 0; pwm <= 120; pwm += 2) {
    drive(pwm, true, pwm, true);
    Serial.print("  PWM = "); Serial.println(pwm);
    uint32_t t = millis();
    while (millis() - t < 350) { if (keyPressed()) { pwm = 999; break; } delay(5); }
  }
  motorStop();
  Serial.println("MIN_MOVE_PWM = the PWM where motion began (round up a little). -> motor.cpp");
}

static void runSpin() {
  Serial.println();
  Serial.println("== Spin test -> effective WHEELBASE ==");
  if (!imuAddr && !imuInit()) return;
  Serial.println("Put the robot on the floor with room to spin in place.");
  Serial.println("Hold still 1 s (bias)..."); calibBias(1000);
  Serial.println("Spinning at full power for 3 s (left fwd, right reverse = CCW)...");
  float deg = 0.0f; uint32_t last = micros(), t0 = millis();
  drive(255, true, 255, false);
  while (millis() - t0 < 3000) {
    uint32_t now = micros();
    float dt = (now - last) * 1e-6f; last = now;
    deg += gyroZdps() * dt;
    delay(3);
  }
  motorStop();
  float secs = 3.0f;
  float wdps = deg / secs;
  float wrad = wdps * (float)PI / 180.0f;
  Serial.print("Turned "); Serial.print(deg, 1);
  Serial.print(" deg in 3 s -> yaw rate "); Serial.print(wdps, 1);
  Serial.print(" deg/s ("); Serial.print(wrad, 3); Serial.println(" rad/s).");
  Serial.println("Effective WHEELBASE = 2 * V_MAX_MPS / yaw_rate_rad_per_s");
  Serial.println("  (use the V_MAX you measured with 'v'). -> WHEELBASE in sketch.ino.");
}

static void printMenu() {
  Serial.println();
  Serial.println("==== lora-calib menu (type a key) ====");
  Serial.println("  i  IMU: bias, heading sign, scale");
  Serial.println("  m  Motor wiring/direction check");
  Serial.println("  v  Top speed run -> V_MAX_MPS");
  Serial.println("  s  Stiction ramp -> MIN_MOVE_PWM");
  Serial.println("  w  Spin test -> effective WHEELBASE");
  Serial.println("  a  Scan I2C bus (IMU troubleshooting)");
  Serial.println("  h  reprint this menu");
}

void setup() {
  motorInit();
  Serial.begin(115200);
  Bridge.begin();
  Monitor.begin(115200);
  uint32_t t0 = millis();
  while (millis() - t0 < 6000) { delay(100); }   // let the monitor attach
  Serial.println();
  Serial.println("lora-calib ready.");
  imuInit();
  printMenu();
}

void loop() {
  if (!Serial.available()) { delay(10); return; }
  char c = (char)Serial.read();
  switch (c) {
    case 'i': runImu();       break;
    case 'm': runMotorDir();  break;
    case 'v': runSpeed();     break;
    case 's': runStiction();  break;
    case 'w': runSpin();      break;
    case 'a': scanI2C();      break;
    case 'h': printMenu();    break;
    case '\r': case '\n': case ' ': break;
    default:
      Serial.print("? unknown key '"); Serial.print(c); Serial.println("'");
      printMenu();
  }
}