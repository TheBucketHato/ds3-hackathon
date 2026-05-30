/*
  Motor Control — ELEGOO Smart Car Shield v1.1
  Arduino Uno Q (Zephyr MCU side)
  TB6612FNG motor driver

  KEY FIX: STBY pin must be HIGH or motors won't move at all.

  Pin mapping:
    Pin 5  → PWMA (left motors speed)
    Pin 6  → PWMB (right motors speed)
    Pin 7  → AIN1 (left direction)
    Pin 9  → BIN1 (right direction)
    Pin 3  → STBY (standby — must be HIGH to enable motors)
*/

#define PWMA  5
#define PWMB  6
#define AIN1  7
#define BIN1  9
#define STBY  3   // standby pin — HIGH = motors enabled

#define SPEED 80

void setup() {
  Serial.begin(9600);

  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Enable motor driver — this is the critical line
  digitalWrite(STBY, HIGH);

  stopMotors();
  Serial.println("Motors enabled. Starting test...");
  delay(2000);
}

void loop() {
  Serial.println("Forward");
  forward();
  delay(2000);

  Serial.println("Stop");
  stopMotors();
  delay(1000);

  Serial.println("Backward");
  backward();
  delay(2000);

  Serial.println("Stop");
  stopMotors();
  delay(1000);

  Serial.println("Turn Left");
  turnLeft();
  delay(1000);

  Serial.println("Stop");
  stopMotors();
  delay(1000);

  Serial.println("Turn Right");
  turnRight();
  delay(1000);

  Serial.println("Stop");
  stopMotors();
  delay(2000);
}

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
