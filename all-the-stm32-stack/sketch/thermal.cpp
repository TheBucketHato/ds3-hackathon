#include "thermal.h"
#include <Wire.h>
#include <Adafruit_MLX90640.h>

// Matches the standalone thermal app: anything >=50C is a hot source; a flame
// reads far higher. One pixel over threshold is enough to call it.
static const float FLAME_C   = 50.0f;
static const int   FLAME_MIN = 1;

static Adafruit_MLX90640 mlx;
static float frame[32 * 24];
static bool  ready = false;

bool thermalBegin() {
  Wire.begin();                 // dedicated SDA/SCL header (also used by IMU probe)
  Wire.setClock(400000);
  if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    ready = false;
    return false;
  }
  mlx.setMode(MLX90640_CHESS);
  mlx.setResolution(MLX90640_ADC_18BIT);
  mlx.setRefreshRate(MLX90640_8_HZ);   // fast frames -> shorter getFrame() stall
  ready = true;
  return true;
}

bool thermalFlameDetected() {
  if (!ready) return false;
  if (mlx.getFrame(frame) != 0) return false;   // read failed -> no detection this cycle
  int hot = 0;
  for (int i = 0; i < 32 * 24; i++)
    if (frame[i] >= FLAME_C) hot++;
  return hot >= FLAME_MIN;
}
