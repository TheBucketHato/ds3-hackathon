/*
 * sketch.ino — Robot-side firmware for the UNO Q (real hardware).
 *
 * Drives a TB6612FNG skid-steer car toward a collision-free waypoint path that
 * the Jetson station sends over LoRa ("P x y x y ..."). Localization is FUSED:
 *   - position : vision pose streamed from the UNO Q's Debian side (webcam SLAM)
 *                over the RouterBridge, ~5 Hz, already in SLAM frame.
 *   - heading  : LSM6DS3 gyro (smooth, relative), locked to the SLAM frame using
 *                vision direction-of-motion (no encoders, no magnetometer).
 * Between vision fixes we dead-reckon position forward from the commanded speed;
 * if vision goes stale we STOP (no encoder fallback to trust). The robot also
 * transmits its fused pose back ("R x y theta") for the operator UI.
 *
 * Stack: RadioLib 7.7.0 (SX1276) + Arduino_RouterBridge Monitor + Wire (IMU).
 *   `Serial`/`Monitor` here is the RouterBridge RPC to the Linux side, not UART.
 *
 * LoRa wiring (RFM9x -> UNO Q):  CS=D10 INT/G0=D8 RST=D2  SCK=D13 MISO=D12 MOSI=D11
 * Motors (TB6612):  PWMA=D5 AIN1=D4 AIN2=D3  PWMB=D9 BIN1=D6 BIN2=D7  STBY=A1
 * IMU (LSM6DS3):    I2C SDA=A4 SCL=A5
 *   Attach the LoRa antenna before power. RFM9x VIN is 3.3V ONLY.
 *
 * Protocol (ASCII, one msg/packet, no trailing NUL):
 *   TX robot->station:  "R x y theta"   fused pose, every tick
 *   RX station->robot:  "P x y x y ..." waypoint path   |   "S" stop/clear
 *   Vision Debian->MCU: RPC call vision_pose(x, y, theta) over the RouterBridge
 *                       (registered with Bridge.provide; serviced by Bridge.update).
 */

#include <Arduino_RouterBridge.h>
#include <RadioLib.h>
#include "motor.h"
#include "imu.h"
#include "thermal.h"       // MLX90640 flame detection
#include "lora_crypto.h"   // AES-256-GCM on the LoRa payload

// ---- LoRa pin map / config -------------------------------------------------
// D8 is the Elegoo LEFT-motor direction pin, so LoRa DIO0/INT is moved to A0.
// SPI (D11/12/13) + CS (D10) require the Elegoo ultrasonic/servo head unplugged.
#define RFM95_CS   10
#define RFM95_INT  A0    // moved off D8 (motor BIN_1); rewire DIO0 to A0
#define RFM95_RST   2
#define RF95_FREQ  915.0
#define TX_POWER   20
#define HDR_TO     0xFF
#define HDR_FROM   0xFF
#define HDR_ID     0x00
#define HDR_FLAGS  0x00
#define HDR_LEN    4

SX1276 radio = new Module(RFM95_CS, RFM95_INT, RFM95_RST, RADIOLIB_NC);

// ---- vehicle / control params ----------------------------------------------
static const float TICK_S     = 0.2f;    // control + R-telemetry period
static const float MAX_V      = 0.98f;   // m/s  (= motor.cpp V_MAX_MPS, calib 'v')
static const float MAX_W      = 2.0f;    // rad/s
static const float WHEELBASE  = 0.40f;   // m, EFFECTIVE track (slip baked in):
                                         // 2*V_MAX/spin_rate, calib 'w' 4.886 rad/s

// ---- localization-fusion params --------------------------------------------
static const uint32_t STALE_MS = 800;    // no vision this long -> safety stop
static const float K_POS       = 0.6f;   // vision->position correction gain
static const float K_OFF       = 0.3f;   // motion-course->yaw-offset gain
static const float MIN_DISP    = 0.08f;  // m of travel needed to trust a course
static const float BOOTSTRAP_V = 0.5f * MAX_V;   // slow straight probe to lock heading

// Bench mode: skip the vision gate so the car follows paths off dead-reckoning
// alone (open loop, drifts). Set REQUIRE_VISION 0 to drive LoRa waypoints
// without the Debian vision side (start pose = origin, facing +x at boot).
// NOTE: set to 0 now for LoRa-waypoint bench testing; return to 1 for vision.
#define REQUIRE_VISION 0

// ---- fused robot state (SLAM frame) ----------------------------------------
static float rx = 0.0f, ry = 0.0f, rtheta = 0.0f;
static float imuYaw = 0.0f;        // integrated gyro heading (relative frame)
static float yawOffset = 0.0f;     // rtheta = imuYaw + yawOffset
static bool  headingLocked = false;
static float vLastCmd = 0.0f;      // last commanded linear speed (dead-reckon)

static bool     haveVision = false;
static uint32_t lastVisionMs = 0;
static float    visPrevX = 0, visPrevY = 0;

static uint32_t lastImuUs = 0;
static uint32_t stepCount = 0;

#define MAX_WP 48          // long paths arrive chunked across P + A packets
static float wpx[MAX_WP], wpy[MAX_WP];
static uint8_t wpCount = 0, wpIndex = 0;

// Flee-from-flame state: wpDir +1 follows the path forward, -1 walks it back
// toward the start. Latched once a flame is seen; cleared by a new P/S command.
static int   wpDir = 1;
static bool  fleeing = false;
static float flameX = 0, flameY = 0;   // captured flame location (for reporting)

// ---- RX interrupt flag ------------------------------------------------------
volatile bool rxFlag = false;
void setRxFlag() { rxFlag = true; }

// ---- small helpers ----------------------------------------------------------
static void blink(int times, int on_ms, int off_ms) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(on_ms);
    digitalWrite(LED_BUILTIN, LOW);  delay(off_ms);
  }
}

static void die(const char *what, int code) {
  while (1) {
    Serial.print("FATAL: "); Serial.print(what);
    Serial.print(" (code "); Serial.print(code);
    Serial.println(") — check wiring/antenna, will keep reporting");
    motorStop();
    blink(3, 120, 120);
    delay(1500);
  }
}

static float angWrap(float a) {
  while (a > PI)  a -= 2.0f * PI;
  while (a < -PI) a += 2.0f * PI;
  return a;
}

// Integer-only float formatting (no %f / dtostrf on this core). Appends " <v>".
static char *appendFloat(char *dst, float v, int prec) {
  *dst++ = ' ';
  if (v < 0) { *dst++ = '-'; v = -v; }
  long scale = 1;
  for (int i = 0; i < prec; i++) scale *= 10;
  long scaled = (long)(v * (float)scale + 0.5f);
  dst += sprintf(dst, "%ld", scaled / scale);
  if (prec > 0) {
    char fmt[8];
    sprintf(fmt, ".%%0%dld", prec);
    dst += sprintf(dst, fmt, scaled % scale);
  }
  return dst;
}

// ---- motion: unicycle (v,w) -> TB6612 wheel speeds -------------------------
static void motorCommand(float v, float w) {
  if (v >  MAX_V) v =  MAX_V;  if (v < -MAX_V) v = -MAX_V;
  if (w >  MAX_W) w =  MAX_W;  if (w < -MAX_W) w = -MAX_W;
  float vL = v - w * WHEELBASE * 0.5f;
  float vR = v + w * WHEELBASE * 0.5f;
  motorSetWheels(vL, vR);
  vLastCmd = v;
}

// Pure-pursuit toward the current waypoint. wpDir picks forward (+1) or fleeing
// (-1, walk the list back to the start). Outputs (v,w); false = path done.
static bool followStep(float *vOut, float *wOut) {
  if (wpCount == 0) return false;
  if (wpDir > 0 && wpIndex >= wpCount) return false;   // finished forward
  float dx = wpx[wpIndex] - rx, dy = wpy[wpIndex] - ry;
  float dist = sqrt(dx * dx + dy * dy);
  if (dist < 0.25f) {                                  // reached this waypoint
    if (wpDir < 0) { if (wpIndex == 0) return false; wpIndex--; }  // back at start
    else wpIndex++;
    *vOut = 0; *wOut = 0; return true;
  }
  float err = angWrap(atan2(dy, dx) - rtheta);
  float wCmd = 3.0f * err;
  float vCmd;
  if (fabs(err) > 1.2f) {
    vCmd = 0.0f;                                  // rotate in place first
  } else {
    vCmd = MAX_V * (1.0f - fabs(err) / 1.2f);
    if (vCmd < 0.0f) vCmd = 0.0f;
    float vMax = dist / TICK_S;                   // don't overshoot the point
    if (vCmd > vMax) vCmd = vMax;
  }
  *vOut = vCmd; *wOut = wCmd;
  return true;
}

// ---- localization -----------------------------------------------------------
// Fold one vision pose (SLAM frame) into the estimate.
static void onVisionPose(float x, float y, float /*th*/) {
  if (!haveVision) {                  // first fix: adopt it outright
    rx = x; ry = y;
  } else {                            // complementary position correction
    rx += K_POS * (x - rx);
    ry += K_POS * (y - ry);
    // Lock gyro heading to the SLAM frame from direction of travel (forward only).
    if (vLastCmd > 0.05f) {
      float dx = x - visPrevX, dy = y - visPrevY;
      if (dx * dx + dy * dy > MIN_DISP * MIN_DISP) {
        float course = atan2(dy, dx);
        yawOffset = angWrap(yawOffset + K_OFF * angWrap(course - imuYaw - yawOffset));
        headingLocked = true;
      }
    }
  }
  // Vision blob-heading 'th' is intentionally ignored — orientation of a small
  // target is noisy; the gyro+course lock above is the heading source.
  visPrevX = x; visPrevY = y;
  lastVisionMs = millis();
  haveVision = true;
}

// RouterBridge RPC endpoint the Debian side calls at ~5 Hz with the fused
// webcam pose (SLAM frame). Registered in setup() via Bridge.provide and
// dispatched by Bridge.update() in loop(). Python side: call vision_pose(x,y,theta).
static void onVisionRPC(float x, float y, float theta) {
  onVisionPose(x, y, theta);
}

// Integrate gyro into heading as fast as we loop; refresh rtheta in SLAM frame.
static void updateHeading() {
  uint32_t nowUs = micros();
  float dt = (nowUs - lastImuUs) * 1e-6f;
  lastImuUs = nowUs;
  if (dt <= 0 || dt > 0.5f) return;         // skip first call / long stalls
  imuYaw = angWrap(imuYaw + imuYawRate() * dt);
  rtheta = angWrap(imuYaw + yawOffset);
}

// ---- radio TX/RX ------------------------------------------------------------
static void sendLine(const char *line) {
  size_t mlen = strlen(line);
  if (mlen > 200) mlen = 200;                 // leave room for header+nonce+tag
  uint8_t out[HDR_LEN + 200 + LORA_OVERHEAD];
  out[0] = HDR_TO; out[1] = HDR_FROM; out[2] = HDR_ID; out[3] = HDR_FLAGS;
  size_t blob = loraEncrypt((const uint8_t *)line, mlen, out + HDR_LEN);
  int state = radio.transmit(out, HDR_LEN + blob);
  radio.setCRC(true);
  radio.startReceive();
  Serial.print("[TX] "); Serial.println(line);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("  (tx err "); Serial.print(state); Serial.println(")");
  }
}

// RouterBridge RPC the Debian/webcam side calls to relay station-bound telemetry
// it produces — bounding boxes "B x y w ..." and info points "I type x y". The
// MCU just encrypts + transmits it over LoRa. Chunk B to <=8 boxes per call so
// each stays within one packet (~223 B plaintext). The MCU sends R pose itself.
static void onLoraTx(String msg) {
  sendLine(msg.c_str());
}

static void sendPose() {
  char line[48]; char *p = line;
  *p++ = 'R';
  p = appendFloat(p, rx, 3);
  p = appendFloat(p, ry, 3);
  p = appendFloat(p, rtheta, 4);
  *p = '\0';
  sendLine(line);
}

// 'P' starts a NEW path (clears existing); 'A' appends to it. Long paths that
// don't fit one LoRa packet (~12 waypoints) are sent as P then one or more A.
// Flame info point for the GUI: "I 4 x y", type 4 = flame, at the given point.
static void sendFlame(float fx, float fy) {
  char line[40]; char *p = line;
  p += sprintf(p, "I 4");
  p = appendFloat(p, fx, 3);
  p = appendFloat(p, fy, 3);
  *p = '\0';
  sendLine(line);
}

// Throttled flame check (getFrame blocks ~150 ms, so ~1 Hz). On a new flame:
// capture its location 0.5 m ahead, flip to fleeing (reverse down the path),
// and report it; re-report a couple times while still visible for reliability.
static void pollFlame() {
  static uint32_t lastCheck = 0, lastReport = 0;
  uint32_t now = millis();
  if (now - lastCheck < 1000) return;
  lastCheck = now;
  if (!thermalFlameDetected()) return;
  if (!fleeing) {                          // rising edge: scared -> retreat
    fleeing = true;
    wpDir = -1;
    if (wpIndex > 0) wpIndex--;            // aim at the waypoint behind us
    flameX = rx + 0.5f * cos(rtheta);      // flame ~0.5 m ahead of the robot
    flameY = ry + 0.5f * sin(rtheta);
    sendFlame(flameX, flameY);
    lastReport = now;
    Serial.println("[FLAME] retreating down path");
  } else if (now - lastReport > 2000) {    // re-send captured location
    sendFlame(flameX, flameY);
    lastReport = now;
  }
}

static void handleCommand(char *line) {
  Serial.print("[RX] "); Serial.println(line);
  if (line[0] == 'S') {
    wpCount = wpIndex = 0; wpDir = 1; fleeing = false; motorStop(); vLastCmd = 0;
    return;
  }
  if (line[0] == 'P' || line[0] == 'A') {
    if (line[0] == 'P') {                 // new path: clear + cancel any flee
      wpCount = wpIndex = 0; wpDir = 1; fleeing = false;
    }
    char *tok = strtok(line + 1, " ");
    while (tok != NULL) {
      float x = atof(tok);
      tok = strtok(NULL, " ");
      if (tok == NULL) break;
      float y = atof(tok);
      tok = strtok(NULL, " ");
      if (wpCount < MAX_WP) { wpx[wpCount] = x; wpy[wpCount] = y; wpCount++; }
    }
  }
}

static void pollRadio() {
  if (!rxFlag) return;
  rxFlag = false;
  uint8_t buf[256];
  int state = radio.readData(buf, sizeof(buf));
  int n = radio.getPacketLength();
  if (state == RADIOLIB_ERR_NONE && n > (int)(HDR_LEN + LORA_OVERHEAD)) {
    uint8_t plain[251];
    int plen = loraDecrypt(buf + HDR_LEN, n - HDR_LEN, plain);
    if (plen > 0 && plen < 250) {
      char line[251];
      memcpy(line, plain, plen);
      line[plen] = '\0';
      handleCommand(line);
    } else {
      Serial.println("[RX] dropped (auth/decrypt fail)");
    }
  }
  radio.startReceive();
}

// ---- setup / loop -----------------------------------------------------------
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  motorBegin();
  motorStop();                      // stay put until we know where we are

  Serial.begin(115200);
  Bridge.begin();
  Monitor.begin(115200);
  Bridge.provide("vision_pose", onVisionRPC);   // Debian webcam pose feed
  Bridge.provide("lora_tx", onLoraTx);          // Debian relays boxes/info -> LoRa
  loraCryptoBegin();                            // AES-256-GCM link encryption
  unsigned long t0 = millis();
  while (millis() - t0 < 8000) { blink(1, 100, 100); }   // let a monitor attach

  Serial.println();
  Serial.println("sketch: real-hardware robot (TB6612 + LSM6DS3 + LoRa)");

  if (!imuBegin())
    Serial.println("WARN: LSM6DS3 not found on I2C (0x6A/0x6B) — heading dead");
  else {
    Serial.println("IMU ok — calibrating gyro bias, hold still...");
    imuCalibrateBias(1500);         // robot is stationary here
  }
  lastImuUs = micros();

  if (!thermalBegin())
    Serial.println("WARN: MLX90640 not found — flame detection disabled");
  else
    Serial.println("Thermal (MLX90640) ok — flame -> flee + report");

  int state = radio.begin(RF95_FREQ, 125.0, 7, 5, 0x12, TX_POWER, 8);
  if (state != RADIOLIB_ERR_NONE) die("radio.begin() failed", state);
  radio.setCRC(true);
  radio.setPacketReceivedAction(setRxFlag);
  radio.startReceive();

  Serial.print("Radio init OK at "); Serial.print(RF95_FREQ);
  Serial.println(" MHz — following P paths, TX R pose, listening for V vision");
}

void loop() {
  static uint32_t lastTick = 0;

  pollRadio();        // inbound P/S
  Bridge.update();    // service inbound vision_pose RPC
  updateHeading();    // integrate gyro as fast as possible
  pollFlame();        // ~1 Hz flame check -> flee + report (blocks ~150 ms)

  uint32_t now = millis();
  if (now - lastTick < (uint32_t)(TICK_S * 1000)) return;
  lastTick = now;
  stepCount++;

  // Dead-reckon position forward from the last commanded speed (coasts gaps).
  rx += cos(rtheta) * vLastCmd * TICK_S;
  ry += sin(rtheta) * vLastCmd * TICK_S;

  bool visionFresh = haveVision && (now - lastVisionMs) <= STALE_MS;

#if !REQUIRE_VISION
  visionFresh = true; headingLocked = true;   // bench mode: trust dead-reckoning
#endif

  if (!visionFresh) {
    motorStop(); vLastCmd = 0;                 // pose untrustworthy -> stop
  } else if (!headingLocked) {
    motorCommand(BOOTSTRAP_V, 0.0f);           // nudge straight to lock heading
  } else {
    float v, w;
    if (followStep(&v, &w)) motorCommand(v, w);
    else { motorStop(); vLastCmd = 0; }        // path complete
  }

  sendPose();
}
