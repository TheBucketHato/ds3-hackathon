/*
 * RFM9x LoRa link test for Arduino UNO Q  — RadioLib port
 *
 * Why RadioLib instead of RadioHead: RadioHead's RH_ASK.cpp writes AVR-only
 * timer registers (TCCR1A/TIMSK/ISR) that don't exist on the Arduino Zephyr
 * core, so the library won't compile here. RadioLib targets this SPI API.
 *
 * Flash one board as the SENDER and one as the RECEIVER by toggling
 * NODE_ROLE below. Open Serial Monitor at 115200 on each.
 *
 * On-air format is RadioHead / adafruit_rfm9x compatible:
 *   modem: 915 MHz, BW 125 kHz, SF7, CR 4/5, preamble 8, sync 0x12, CRC on
 *   each packet carries a 4-byte header [to, from, id, flags]; we use the
 *   0xFF broadcast defaults, matching RH_RF95's plain send()/recv().
 *
 * Wiring (RFM9x breakout -> UNO Q):
 *   VIN -> 3.3V   SCK  -> D13   CS  -> D10   RST -> D9
 *   GND -> GND    MISO -> D12   MOSI-> D11   G0  -> D2  (DIO0 / IRQ)
 * Attach the antenna before powering up.
 *
 * Library: install "RadioLib 7.7.0" by Jan Gromes via Library Manager.
 */ 

#include <RadioLib.h>

// ---- choose this board's role ----
#define ROLE_SENDER    0
#define ROLE_RECEIVER  1
#define NODE_ROLE      ROLE_SENDER   // this Arduino transmits; the Jetson receives

// ---- pin map ----
#define RFM95_CS   10
#define RFM95_INT   2    // DIO0 / G0
#define RFM95_RST   9

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
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }

  Serial.println();
  Serial.print("RFM9x link test (RadioLib), role: ");
  Serial.println(NODE_ROLE == ROLE_SENDER ? "SENDER" : "RECEIVER");

  // begin(freq, bw, sf, cr, syncWord, power_dBm, preambleLength)
  int state = radio.begin(RF95_FREQ, 125.0, 7, 5, 0x12, TX_POWER, 8);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("ERROR: radio.begin() failed, code ");
    Serial.println(state);
    while (1) { delay(1000); }
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
    lastSend = millis();
  }
}
