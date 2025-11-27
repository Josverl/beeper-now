# PIEP
import asyncio
import time
from math import pi

import espnow

import wifi
from config import COLORS, NP_PIN, np, set_color, signal_led
from device import DEVICE, get_device_name
from net_id import MAC_ADDRESS

try:
    from esp32_cosine import CosineDAC
except (ImportError, NotImplementedError):
    CosineDAC = None
    check_hardware_sine_support = None

speaker_pin=25

hw_cosine = None
if CosineDAC: 
    print("Creating hardware sine generator...")
    try: 
        hw_cosine = CosineDAC(pin_num=speaker_pin, freq=1_200)
    except RuntimeError as e:
        print(f"Error initializing hardware sine generator: {e}")
        # Nur ein reboot macht alles gut
        import machine
        print("Rebooting...")
        machine.reset()

if not np:
    print("Neopixel not available")

# Initialize Wi-Fi
sta, ap = wifi.reset(sta=False, ap=False, channel=1)  # STA off, AP off, channel=1
espn: espnow.ESPNow | None = None


def init():
    global espn
    set_color("YELLOW")

    # Print the MAC address
    print(
        f"Receiver {DEVICE}: MAC address :"
        + "".join(["\\x%02x" % b for b in MAC_ADDRESS])
    )

    sta.active(True)
    sta.disconnect()
    # Initialize ESP-NOW
    espn = espnow.ESPNow()
    espn.active(True)


def listen():
    assert espn is not None, "ESPNow not initialized. Call init() first."
    # Listen for incoming messages
    while True:
        if signal_led:
            signal_led(0)
        host, msg = espn.recv()
        if msg:
            if signal_led:
                signal_led(1)
            message = msg.decode().strip()
            if host:
                print(f"Received from {get_device_name(host)}[{host.hex()}]: {message}")

            # Use string color names directly
            if message in COLORS:
                set_color(message)
                if hw_cosine: 
                    if message == "OFF":
                        hw_cosine.stop()
                    if message == "GREEN":
                        # Start a 1_200Hz tone
                        hw_cosine.set_frequency(1_200)
                        hw_cosine.start()

