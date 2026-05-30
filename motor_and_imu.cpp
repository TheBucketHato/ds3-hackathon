/*
  robot_v1 — Motor Control + IMU
  ELEGOO Smart Car Shield v1.1 + MPU-6050
  Arduino Uno Q

  Motors:
    Pin 3  → STBY
    Pin 5  → PWMA (left speed)
    Pin 6  → PWMB (right speed)
    Pin 7  → AIN1 (left direction)
    Pin 9  → BIN1 (right direction)

  IMU (MPU-6050):
    SDA → dedicated SDA pin
    SCL → dedicated SCL pin
    AD0 → GND
*/

#include "MPU6050.h"
#include "Wire.h"

// ── Motor pins ────────────────────────────────────────────────────────────────
#define PWMA 5
#define PWMB 6
#define AIN1 7
#define BIN1 9
#define STBY 3

#define SPEED 200

// ── IMU ───────────────────────────────────────────────────────────────────────
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

unsigned long lastIMUPrint = 0;
#define IMU_PRINT_INTERVAL 500  // print IMU every 500ms

void setup() {
  Serial.begin(9600);
  delay(2000);

  // Motor setup
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  stopMotors();
  Serial.println("Motors ready.");

  // IMU setup
  Wire.begin();
  accelgyro.initialize();
  if (accelgyro.testConnection()) {
    Serial.println("MPU-6050 connected!");
  } else {
    Serial.println("ERROR: MPU-6050 not found. Check wiring.");
  }

  delay(1000);
  Serial.println("Starting...\n");
}

void loop() {
  // ── Drive sequence ──────────────────────────────────────────────────────────
  Serial.println(">> Forward");
  forward();
  runWithIMU(2000);

  Serial.println(">> Stop");
  stopMotors();
  runWithIMU(1000);

  Serial.println(">> Backward");
  backward();
  runWithIMU(2000);

  Serial.println(">> Stop");
  stopMotors();
  runWithIMU(1000);

  Serial.println(">> Turn Left");
  turnLeft();
  runWithIMU(1000);

  Serial.println(">> Stop");
  stopMotors();
  runWithIMU(1000);

  Serial.println(">> Turn Right");
  turnRight();
  runWithIMU(1000);

  Serial.println(">> Stop");
  stopMotors();
  runWithIMU(2000);
}

// ── Runs for `duration` ms while printing IMU data ───────────────────────────
void runWithIMU(unsigned long duration) {
  unsigned long start = millis();
  while (millis() - start < duration) {
    printIMU();
  }
}

void printIMU() {
  if (millis() - lastIMUPrint < IMU_PRINT_INTERVAL) return;
  lastIMUPrint = millis();

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  Serial.print("  ax=");
  Serial.print(ax / 16384.0, 2);
  Serial.print("g  ");
  Serial.print("ay=");
  Serial.print(ay / 16384.0, 2);
  Serial.print("g  ");
  Serial.print("az=");
  Serial.print(az / 16384.0, 2);
  Serial.print("g  | ");
  Serial.print("gx=");
  Serial.print(gx / 131.0, 1);
  Serial.print("°/s  ");
  Serial.print("gy=");
  Serial.print(gy / 131.0, 1);
  Serial.print("°/s  ");
  Serial.print("gz=");
  Serial.print(gz / 131.0, 1);
  Serial.println("°/s");
}

// ── Motor commands ────────────────────────────────────────────────────────────
void forward() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, HIGH);
  analogWrite(PWMA, SPEED);
  analogWrite(PWMB, SPEED);
}

void backward() {
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, LOW);
  analogWrite(PWMA, SPEED);
  analogWrite(PWMB, SPEED);
}

void turnLeft() {
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, HIGH);
  analogWrite(PWMA, SPEED);
  analogWrite(PWMB, SPEED);
}

void turnRight() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  analogWrite(PWMA, SPEED);
  analogWrite(PWMB, SPEED);
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, LOW);
}
