import espnow
import network
from machine import Pin

from config import COLORS, NP_PIN, np, set_color, sig
from device import DEVICE, get_device_name
from net_id import MAC_ADDRESS

if not np:
    print("Neopixel not available")

# Initialize Wi-Fi in station mode
sta = network.WLAN(network.STA_IF)
espn : espnow.ESPNow|None = None


def init():
    global espn
    set_color("YELLOW")

    # Print the MAC address
    print(f"Receiver {DEVICE}: MAC address :" + ''.join(['\\x%02x' % b for b in MAC_ADDRESS]))

    sta.active(True)
    sta.disconnect()
    # Initialize ESP-NOW
    espn = espnow.ESPNow()
    espn.active(True)

def listen():
    # Listen for incoming messages
    while True:
        if sig:
            sig(0)
        host, msg = espn.recv()
        if msg:
            if sig:
                sig(1)
            message = msg.decode().strip()
            if host:
                print(f"Received from {get_device_name(host)}[{host.hex()}]: {message}")
            
            # Use string color names directly
            if message in COLORS:
                set_color(message)
            else:
                # Handle partial matches for backward compatibility
                for color_name in COLORS:
                    if message.startswith(color_name):
                        set_color(color_name)
                        break
