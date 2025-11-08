import neopixel
from machine import Pin, unique_id

from device import ALL_DEVICES, DEVICE

# Color dictionary mapping color names to RGB tuples
COLORS = {
    "OFF": (0,0,0),
    "GREEN": (0, 255, 0),
    "RED": (255, 0, 0),
    "BLUE":  (0, 0, 255),
    "YELLOW": (255, 255, 0),
    "PURPLE": (128, 0, 128),
    "ORANGE" : (255,165,0),
}

MAC_BROADCAST = b'\xff\xff\xff\xff\xff\xff'

RECIEVERS = [
    ALL_DEVICES["M5STACK-FIRE"],
    MAC_BROADCAST,
]

NP_PIN = None
NP_LEN = 1
SIG_PIN = 3

if DEVICE == "M5STACK-FIRE":
     NP_PIN = 15
     NP_LEN = 10
     SIG_PIN = 26 # Grove B , pin3 

elif DEVICE == "C3-MINI":
     NP_PIN = 7

elif DEVICE in {"AI-C3","C3-SUPER-MINI","ESP8266"}:
     NP_PIN = 8

elif DEVICE == "M5STACK-GREY":
    NP_PIN = None

if SIG_PIN:
    signal_led = Pin(SIG_PIN, Pin.OUT)
else:
    signal_led = None

if NP_PIN:
     np = neopixel.NeoPixel(Pin(NP_PIN), NP_LEN)
else:
    np = None

# BOOT button is usually on GPIO9 on ESP32
boot_btn = Pin(9, Pin.IN, Pin.PULL_UP)

def set_color(color:tuple|str):
    if np:
        # If color is a string, look it up in the COLORS dictionary
        if isinstance(color, str):
            if color in COLORS:
                color = COLORS[color]
            else:
                print(f"Warning: Unknown color name '{color}', using OFF")
                color = COLORS["OFF"]
        np.fill(color)
        np.write()


print(f"{DEVICE} Pin {NP_PIN}")


