
ra-2 robot firmware - integration guide

Robot-side firmware for an **Arduino UNO Q** (STM32U585, Zephyr core; FQBN
`arduino:zephyr:unoq`) driving an **Elegoo Smart Robot Car V4.0** (TB6612
skid-steer, 4 motors / 2 channels, no encoders). It follows collision-free
waypoint paths sent over LoRa, localizing by fusing onboard gyro heading with a
vision pose streamed from the board's Debian/Linux side.

`Serial`/`Monitor` is the RouterBridge RPC link to the Linux side, **not a UART**.

## Read these files, in this order

1. **`sketch/sketch.ino`** - the brain. `setup()`/`loop()`, the 5 Hz control
   tick, pose fusion, and `followStep()` (pure-pursuit waypoint steering). Start
   here; everything else is a driver it calls.
2. **`sketch/motor.cpp` / `.h`** - TB6612 driver. Pins are **fixed by the Elegoo
   V4.0 shield** and the per-side calibration constants live at the top.
3. **`sketch/imu.cpp` / `.h`** - gyro yaw-rate source. Auto-detects LSM6DS3
   (preferred) or MPU6050 on any of the 3 I2C buses (`Wire`=SDA/SCL header,
   `Wire1`=QWIIC, `Wire2`=A4/A5). Raw I2C, no library.
4. **`python/main.py`** - the Debian-side app. **Currently a stub** - this is
   where the visionMCU bridge must be implemented (see Integration seams).

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
- Safety: no vision for `STALE_MS` (0.8 s)  stop. Heading bootstrap drives a
  slow straight line until vision motion locks the yaw offset, then follows.

## Config flags

- **`REQUIRE_VISION`** (sketch.ino): `1` = require vision (real use). `0` =
  bench mode, follow paths on dead-reckoning alone (drifts; for motor/IMU
  bring-up without the Debian side).

## Build / flash

```
arduino-app-cli app start /home/arduino/ArduinoApps/lora-2      # build + flash MCU + run python
arduino-app-cli monitor                                          # serial console
arduino-cli compile -b arduino:zephyr:unoq sketch/               # compile-only check
```


## Build dependencies / libraries (to run lora-2 on another board)

Target board: **Arduino UNO Q**, platform **`arduino:zephyr` 0.55.2**, FQBN
`arduino:zephyr:unoq`. Install the platform, then:

- **Declared in `sketch/sketch.yaml` (the only one you add):**
  - `RadioLib` **7.7.0** — SX1276/RFM9x LoRa driver.
- **Bundled with the UNO Q core — installed automatically, do NOT add manually:**
  - `Arduino_RouterBridge` 0.4.2 — Monitor + RPC bridge to the Linux side
    (`Serial`/`Monitor`, `Bridge.provide`/`update`). Pulls in:
    `Arduino_RPClite` 0.3.0, `MsgPack` 0.4.2, `DebugLog` 0.8.4,
    `ArxTypeTraits` 0.3.2, `ArxContainer` 0.7.0.
  - `Wire` (I2C, for the IMU) and `SPI` (for RadioLib) — core libraries.
- **Vendored in `sketch/src/` — no install needed:**
  - AES-256-GCM (rweather `Crypto`, AES+GCM files only; its `RNG.cpp` is omitted
    because it clashes with the STM32 `RNG` macro). This is why `Crypto` is NOT
    a `sketch.yaml` dependency — copy the `sketch/src/` folder with the sketch.

So a fresh checkout needs just: the `arduino:zephyr` core + `RadioLib 7.7.0`
(the RouterBridge stack rides along with the core). Build/flash with
`arduino-app-cli app start <dir>`.

NOTE: this all assumes another **UNO Q**. RouterBridge/Monitor and the vision
RPC are UNO-Q-specific (the MCU↔Linux bridge); a non-UNO-Q board would need
those replaced with a real UART/transport.

Python side (Jetson station + Debian app): `pip install cryptography` (AES-GCM).

## New changes (LoRa encryption, box relay, chunked paths)

### Debian-side responsibilities (`python/main.py`) — call these MCU RPCs
The Debian/webcam side talks to the MCU over the RouterBridge by calling MCU
functions registered with `Bridge.provide(...)` (serviced by `Bridge.update()`
in the firmware loop). Call them the same way any provided method is called via
`arduino.app_utils`:

- **`vision_pose(x, y, theta)`** — fused-localization input. SLAM-frame
  meters/radians, same frame as the waypoints. Send at ~5 Hz. Without it the
  robot safety-stops after `STALE_MS` (0.8 s).
- **`lora_tx(msg)`** — hand the MCU a station-bound string; it encrypts + sends
  it over LoRa. Use for **bounding boxes** `"B x y w x y w ..."` and **info
  points** `"I type x y"`. **Chunk `B` to ≤8 boxes per call** (each packet's
  plaintext must fit ~223 bytes). Do NOT send `R` — the MCU transmits its own
  fused pose every tick.

So boxes flow: Debian webcam detects → `lora_tx("B ...")` → MCU encrypts → LoRa
→ Jetson. (The earlier simulated `WORLD[]`/`sendBoxes()` was removed; this RPC
relay replaces it.)

### LoRa link is now AES-256-GCM encrypted
Every LoRa packet's payload (everything after the 4-byte RadioHead header) is
AES-256-GCM. Wire format: `[RadioHead 4B][nonce 12B][ciphertext N][GCM tag 16B]`,
i.e. after the header: `nonce(12) || ciphertext || tag(16)` (matches Python
`cryptography` AESGCM which returns ciphertext||tag). 32-byte shared key, no AAD.
- Firmware: `sketch/lora_crypto.{h,cpp}` + vendored AES-GCM in `sketch/src/`
  (rweather Crypto, minus its `RNG.cpp` which clashes with the STM32 `RNG`
  macro). Key is in `lora_crypto.cpp` — **must match the Jetson; change before
  any real deployment** (anyone who dumps the flash gets it).
- Jetson + Debian: `pip install cryptography`; same key (hex
  `3161ce431b045a53804fe80ca3d7a9061bc182ded107ebd22152cc293e03030e`).
- Max plaintext per packet ≈ 223 B (255 − 4 − 12 − 16).

### Chunked waypoint paths (P / A / S)
A path longer than one packet (~12 waypoints) is sent as multiple packets:
- `"P x y x y ..."` — start a NEW path (clears existing), ≤12 waypoints.
- `"A x y x y ..."` — APPEND more waypoints (one or more of these).
- `"S"` — stop / clear. Robot caps total at `MAX_WP` = 48.

