/*
 * thermal.h — MLX90640 flame detection for the robot.
 *
 * Reads the 32x24 IR array on Wire (dedicated SDA/SCL header, addr 0x33) and
 * reports whether a hot source (flame) is in the forward field of view. Uses
 * the repeated-start-patched Adafruit_MLX90640 (Zephyr i2c_write_read on bus 0).
 * NOTE: thermalFlameDetected() reads a frame and BLOCKS ~150 ms — call it at a
 * low rate, not every control tick.
 */
#pragma once
#include <Arduino.h>

bool thermalBegin();           // false if the MLX90640 isn't found
bool thermalFlameDetected();   // true if a hot source (>=FLAME_C) is in view
