# lora-2 robot firmware — integration guide

Robot-side firmware for an **Arduino UNO Q** (STM32U585, Zephyr core; FQBN
`arduino:zephyr:unoq`) driving an **Elegoo Smart Robot Car V4.0** (TB6612
skid-steer, 4 motors / 2 channels, no encoders). It follows collision-free
waypoint paths sent over LoRa, localizing by fusing onboard gyro heading with a
vision pose streamed from the board's Debian/Linux side.

`Serial`/`Monitor` is the RouterBridge RPC link to the Linux side, **not a UART**.

## Read these files, in this order

1. **`sketch/sketch.ino`** — the brain. `setup()`/`loop()`, the 5 Hz control
   tick, pose fusion, and `followStep()` (pure-pursuit waypoint steering). Start
   here; everything else is a driver it calls.
2. **`sketch/motor.cpp` / `.h`** — TB6612 driver. Pins are **fixed by the Elegoo
   V4.0 shield** and the per-side calibration constants live at the top.
3. **`sketch/imu.cpp` / `.h`** — gyro yaw-rate source. Auto-detects LSM6DS3
   (preferred) or MPU6050 on any of the 3 I2C buses (`Wire`=SDA/SCL header,
   `Wire1`=QWIIC, `Wire2`=A4/A5). Raw I2C, no library.
4. **`python/main.py`** — the Debian-side app. **Currently a stub** — this is
   where the vision→MCU bridge must be implemented (see Integration seams).

## Integration seams (the actual interface)

- **Vision pose IN** (the main one): the firmware registers
  `Bridge.provide("vision_pose", onVisionRPC)` and pumps `Bridge.update()` in
  `loop()`. The Debian side must call **`vision_pose(x, y, theta)` at ~5 Hz**,
  coordinates in **SLAM-frame meters / radians** (same frame as the waypoints).
  Implement that call in `python/main.py`. Without it the robot safety-stops.
- **Waypoints IN** (LoRa, from the station): `"P x y x y ..."` sets the path,
  `"S"` stops/clears. Parsed in `handleCommand()`.
- **Pose OUT** (LoRa, to the station UI): `"R x y theta"` every tick, the fused
  estimate. `sendPose()`.

## Localization model (no encoders)

- Heading: gyro integrated every loop; locked to the SLAM frame from vision
  direction-of-travel (`onVisionPose` updates `yawOffset`).
- Position: vision-primary (complementary blend `K_POS`); dead-reckoned forward
  from commanded speed only to coast between vision frames.
- Safety: no vision for `STALE_MS` (0.8 s) → stop. Heading bootstrap drives a
  slow straight line until vision motion locks the yaw offset, then follows.

## Config flags

- **`REQUIRE_VISION`** (sketch.ino): `1` = require vision (real use). `0` =
  bench mode, follow paths on dead-reckoning alone (drifts; for motor/IMU
  bring-up without the Debian side).

## NEW BOARD / NEW ROBOT — what must be redone

PORTABLE (no change if same Elegoo V4.0 kit):
- Motor pin map (`motor.cpp`): PWMA=5 AIN1=7 PWMB=6 BIN1=8 STBY=3, one dir pin
  per channel, A=right/B=left, DIR HIGH=forward.
- IMU driver: auto-detects chip+bus, nothing to set.

MUST RE-VERIFY / RE-CALIBRATE per physical robot — use the **`lora-calib`** app
(separate sibling app, interactive menu over `arduino-app-cli monitor`):
- `V_MAX_MPS` (motor.cpp) + `MAX_V` (sketch.ino) — calib `v`.
- `MIN_MOVE_PWM_L` / `_R` (motor.cpp) stiction — calib `s`.
- `WHEELBASE` (sketch.ino) effective skid-steer track — calib `w` (= 2*V_MAX/spin_rate).
- `IMU_YAW_SIGN` (imu.cpp) — calib `i` (must read + for a left/CCW turn).
- Current values are for ONE specific robot (2026-05-31): V_MAX 0.98, stiction
  L30/R28, wheelbase 0.40, yaw sign -1.

## HARDWARE GOTCHA — LoRa vs Elegoo pin conflict (must resolve)

The Elegoo shield and the RFM9x LoRa default pins collide. Motors are fixed, so
the LoRa side moves:
- LoRa SPI (SCK13/MISO12/MOSI11) + CS10 collide with the Elegoo **ultrasonic +
  camera servos** → physically **unplug the ultrasonic/servo head** (vision
  replaces it) to free D10–D13.
- LoRa INT was moved **D8 → A0** (D8 is the left-motor dir pin); rewire the
  RFM9x DIO0 wire to A0.
- RST=D2 vs the Elegoo Key button = harmless.
- IMU: LSM6DS3 on QWIIC works as-is.

## Build / flash

```
arduino-app-cli app start /home/arduino/ArduinoApps/lora-2      # build + flash MCU + run python
arduino-app-cli monitor                                          # serial console
arduino-cli compile -b arduino:zephyr:unoq sketch/               # compile-only check
```
