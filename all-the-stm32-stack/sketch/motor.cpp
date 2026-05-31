#include "motor.h"

// ---- TB6612FNG pins — Elegoo Smart Robot Car V4.0 (fixed by the shield) -----
// Taken from the kit's stock firmware (DeviceDriverSet_xxx0). ONE direction pin
// per channel (the board ties the 2nd TB6612 input internally). DIR HIGH = fwd.
// Group A = RIGHT motors, Group B = LEFT motors.
//   NOTE: D8 (left dir) collides with the LoRa DIO0/INT default — the radio's
//   INT has been moved to A0 in sketch.ino, and the ultrasonic/servo head must
//   be unplugged to free D10-D13 for LoRa SPI.
#define PIN_PWMA   5    // right speed
#define PIN_AIN1   7    // right direction
#define PIN_PWMB   6    // left  speed
#define PIN_BIN1   8    // left  direction
#define PIN_STBY   3    // enable (HIGH = run)

// ---- empirical calibration (MEASURE THESE on the bench) --------------------
// V_MAX_MPS: wheel speed at full PWM with a fresh pack (calib 'v' run).
// MIN_MOVE_PWM_*: smallest PWM that breaks stiction, per side = stiffest wheel
// on that side (calib 's' ramp). Left wheels FL30/BL18 -> 30; right FR28/BR28 -> 28.
static const float V_MAX_MPS      = 0.98f;   // 116 in / 3 s, calib 'v' 2026-05-31
static const int   MIN_MOVE_PWM_L = 30;      // 0..255
static const int   MIN_MOVE_PWM_R = 28;
static const int   MAX_PWM        = 255;

static void setChannel(int pwmPin, int dirPin, float v, int minPwm) {
  bool fwd = (v >= 0.0f);
  float mag = fabs(v);
  int pwm = 0;
  if (mag > 1e-3f) {
    float frac = mag / V_MAX_MPS;
    if (frac > 1.0f) frac = 1.0f;
    pwm = minPwm + (int)((MAX_PWM - minPwm) * frac + 0.5f);
  }
  digitalWrite(dirPin, fwd ? HIGH : LOW);
  analogWrite(pwmPin, pwm);
}

void motorBegin() {
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_PWMA, OUTPUT); pinMode(PIN_PWMB, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);
  motorSetWheels(0, 0);
}

void motorSetWheels(float vLeftMps, float vRightMps) {
  digitalWrite(PIN_STBY, HIGH);
  setChannel(PIN_PWMB, PIN_BIN1, vLeftMps,  MIN_MOVE_PWM_L);   // left  = group B
  setChannel(PIN_PWMA, PIN_AIN1, vRightMps, MIN_MOVE_PWM_R);   // right = group A
}

void motorStop() {
  analogWrite(PIN_PWMA, 0);
  analogWrite(PIN_PWMB, 0);
  digitalWrite(PIN_STBY, LOW);                 // hardware-disable as a safety stop
}
