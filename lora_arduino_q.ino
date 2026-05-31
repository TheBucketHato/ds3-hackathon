/*
 * RFM9x LoRa link test for Arduino UNO Q  — RadioLib port
 *
 * Serial note (UNO Q): `Serial` is the Arduino_RouterBridge "Monitor" — an RPC
 * link to the Linux side, NOT a UART. Output is dropped unless the bridge is up
 * and a monitor client is attached, so we start the bridge explicitly and, on
 * any radio failure, print the error FOREVER (not once) so a monitor opened
 * late still sees it. The built-in LED also blinks as a hardware heartbeat.
 *
 * Watch with:  arduino-app-cli monitor   (or the App Lab serial monitor).
 *
 * Wiring (RFM9x breakout -> UNO Q):
 *   VIN -> 3.3V   SCK  -> D13 (white)   CS   -> D10 (blue)
 *   GND -> GND    MISO -> D12 (yellow)  G0   -> D8  (green)  DIO0 / IRQ
 *                 MOSI -> D11 (orange)  RST  -> D2  (gray)
 *   SCK/MISO/MOSI are fixed hardware-SPI pins; CS/DIO0/RST are software-set.
 * Attach the antenna before powering up. VIN is 3.3V ONLY (not 5V).
 *
 * Library: RadioLib 7.7.0 by Jan Gromes.
 */

#include <Arduino_RouterBridge.h>
#include <RadioLib.h>

// ---- choose this board's role ----
#define ROLE_SENDER    0
#define ROLE_RECEIVER  1
#define NODE_ROLE      ROLE_SENDER   // this Arduino transmits; the Jetson receives

// ---- pin map ----
#define RFM95_CS   10    // NSS  (blue,  D10)
#define RFM95_INT   8    // DIO0 / G0 (green, D8)
#define RFM95_RST   2    // RST  (gray,  D2)

// ---- radio config ----
#define RF95_FREQ  915.0
#define TX_POWER   20    // dBm, PA_BOOST path (RadioLib max 20 on SX1276)

// RadioHead 4-byte header defaults (broadcast)
#define HDR_TO     0xFF
#define HDR_FROM   0xFF
#define HDR_ID     0x00
#define HDR_FLAGS  0x00

SX1276 radio = new Module(RFM95_CS, RFM95_INT, RFM95_RST, RADIOLIB_NC);

uint16_t packetCount = 0;

void blink(int times, int on_ms, int off_ms) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(on_ms);
    digitalWrite(LED_BUILTIN, LOW);  delay(off_ms);
  }
}

// Spin forever, re-reporting a fatal error so a late-attached monitor sees it.
void die(const char *what, int code) {
  while (1) {
    Serial.print("FATAL: ");
    Serial.print(what);
    Serial.print(" (code ");
    Serial.print(code);
    Serial.println(") — check wiring/antenna, will keep reporting");
    blink(3, 120, 120);     // 3 quick blinks = error heartbeat
    delay(1500);
  }
}

// Prepend the RadioHead header and transmit.
int sendMessage(const char *msg) {
  size_t mlen = strlen(msg) + 1;            // include trailing NUL, like the original
  uint8_t out[4 + 250];
  out[0] = HDR_TO;  out[1] = HDR_FROM;  out[2] = HDR_ID;  out[3] = HDR_FLAGS;
  memcpy(out + 4, msg, mlen);
  int state = radio.transmit(out, 4 + mlen);
  radio.setCRC(true);                       // restore RX modem config after TX
  return state;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  Bridge.begin();
  Monitor.begin(115200);
  // Give a monitor up to ~8 s to attach so the init banner isn't missed.
  unsigned long t0 = millis();
  while (!Monitor && millis() - t0 < 8000) { blink(1, 100, 100); }

  Serial.println();
  Serial.print("RFM9x link test (RadioLib), role: ");
  Serial.println(NODE_ROLE == ROLE_SENDER ? "SENDER" : "RECEIVER");

  // begin(freq, bw, sf, cr, syncWord, power_dBm, preambleLength)
  int state = radio.begin(RF95_FREQ, 125.0, 7, 5, 0x12, TX_POWER, 8);
  if (state != RADIOLIB_ERR_NONE) {
    die("radio.begin() failed", state);     // prints forever instead of dying silently
  }
  radio.setCRC(true);
  Serial.print("Radio init OK at ");
  Serial.print(RF95_FREQ);
  Serial.println(" MHz");
}

// Send our own "hello" on a timer so listening isn't blocked.
unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL = 5000;   // ms between outgoing messages

void loop() {
  // 1) Always be listening (receive() returns after a short timeout, then we loop)
  uint8_t buf[256];
  int state = radio.receive(buf, sizeof(buf));
  if (state == RADIOLIB_ERR_NONE) {
    int n = radio.getPacketLength();
    if (n >= 4) {
      Serial.print("Got: ");
      for (int i = 4; i < n; i++) Serial.write(buf[i]);
      Serial.print("   RSSI: ");
      Serial.println(radio.getRSSI(), DEC);
    }
  }

  // 2) Periodically send our own message
  if (millis() - lastSend >= SEND_INTERVAL) {
    char msg[24];
    snprintf(msg, sizeof(msg), "hello %u", packetCount++);
    Serial.print("Sending: ");
    Serial.println(msg);
    sendMessage(msg);
    digitalWrite(LED_BUILTIN, HIGH); delay(20); digitalWrite(LED_BUILTIN, LOW);  // TX heartbeat
    lastSend = millis();
  }
}
