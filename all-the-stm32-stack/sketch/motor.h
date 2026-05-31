/*
 * motor.h — TB6612FNG dual H-bridge driver for the Elegoo skid-steer car.
 *
 * Wheel-level interface only: you hand it signed wheel speeds in m/s and it
 * deals with direction pins, PWM, stiction deadband and the STBY enable. The
 * unicycle (v, w) -> left/right inverse kinematics lives up in the sketch.
 */
#pragma once
#include <Arduino.h>

void motorBegin();
// Signed wheel speeds in m/s. Positive = forward. Magnitude above V_MAX is
// clamped; near-zero coasts that wheel.
void motorSetWheels(float vLeftMps, float vRightMps);
// Hard stop: PWM 0 + brake, and drop STBY so the bridge is disabled.
void motorStop();
