#!/usr/bin/env python3
"""
RFM9x (SX1276/77/78/79) LoRa test for Jetson Nano.

Wiring (Jetson Nano 40-pin header, SPI1 / spidev0):
    RFM9x      Jetson Nano phys pin   Blinka name
    -------    --------------------   -----------
    VIN/3V3 -> Pin 1  or 17 (3.3V)    *** NOT 5V ***
    GND     -> Pin 6   (GND)
    SCK     -> Pin 23                 board.SCK
    MISO    -> Pin 21                 board.MISO
    MOSI    -> Pin 19                 board.MOSI
    CS/NSS  -> Pin 24                 board.CE0
    RST     -> Pin 15                 board.D22
    G0/DIO0 -> Pin 22                 board.D25   (optional, only for IRQ/RX callbacks)

Usage:
    python3 lora_test.py            # send + listen
    python3 lora_test.py tx         # transmit only
    python3 lora_test.py rx         # receive only
"""

import sys
import time

import board
import busio
import digitalio
import adafruit_rfm9x

# --- Radio config -----------------------------------------------------------
# Must match on both ends of the link. 915.0 for US, 868.0 for EU, 433.0 for Asia.
RADIO_FREQ_MHZ = 915.0

# Control pins. board.D25 / board.D24 map to BCM-style numbers via Blinka.
CS_PIN  = digitalio.DigitalInOut(board.CE0)   # NSS  -> header pin 24
RESET_PIN = digitalio.DigitalInOut(board.D22) # RST  -> header pin 15


def init_radio():
    spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
    rfm9x = adafruit_rfm9x.RFM9x(spi, CS_PIN, RESET_PIN, RADIO_FREQ_MHZ)
    rfm9x.tx_power = 23          # max for RFM95/96/97/98 (5..23 dBm)
    print("RFM9x initialized OK")
    print(f"  frequency : {RADIO_FREQ_MHZ} MHz")
    print(f"  tx_power  : {rfm9x.tx_power} dBm")
    print(f"  version   : 0x{rfm9x._read_u8(0x42):02x} (0x12 expected)")
    return rfm9x


def transmit(rfm9x, n=5):
    for i in range(n):
        msg = f"hello from jetson #{i}".encode("utf-8")
        rfm9x.send(msg)
        print(f"TX -> {msg!r}")
        time.sleep(1)


def receive(rfm9x, timeout=5.0):
    print(f"Listening for {timeout}s windows... (Ctrl-C to stop)")
    while True:
        packet = rfm9x.receive(timeout=timeout)
        if packet is None:
            print("  ... no packet")
        else:
            try:
                text = packet.decode("utf-8")
            except UnicodeError:
                text = repr(packet)
            print(f"RX <- {text}  (RSSI {rfm9x.last_rssi} dBm)")


def pong(rfm9x):
    """Listen for packets and reply — pairs with the Arduino ping sketch."""
    print("Pong mode: waiting for pings (Ctrl-C to stop)...")
    while True:
        packet = rfm9x.receive(timeout=5.0)
        if packet is None:
            continue
        try:
            text = packet.decode("utf-8")
        except UnicodeError:
            text = repr(packet)
        print(f"RX <- {text}  (RSSI {rfm9x.last_rssi} dBm)")
        rfm9x.send(b"pong from jetson")
        print("TX -> b'pong from jetson'")


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "both"
    radio = init_radio()
    try:
        if mode == "pong":
            pong(radio)
        if mode in ("tx", "both"):
            transmit(radio)
        if mode in ("rx", "both"):
            receive(radio)
    except KeyboardInterrupt:
        print("\nstopped")
