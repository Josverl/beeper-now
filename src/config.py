import neopixel
from machine import Pin, unique_id

from device import DEVICE

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
    # MAC_C3_MINI,
    # MAC_M5STACK_FIRE,
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

elif DEVICE in {"AI-C3","C3_SUPER_MINI","ESP8266"}:
     NP_PIN = 8

elif DEVICE == "M5STACK-GREY":
    NP_PIN = None


if SIG_PIN:
    sig = Pin(SIG_PIN, Pin.OUT)
else:
    sig = None

if NP_PIN:
     np = neopixel.NeoPixel(Pin(NP_PIN), NP_LEN)
else:
    np = None

def set_color(color:tuple|str):
    if np:
        # If color is a string, look it up in the COLORS dictionary
        if isinstance(color, str):
            if color in COLORS:
                color = COLORS[color]
            else:
                print(f"Warning: Unknown color name '{color}', using OFF")
                color = COLORS["OFF"]
        np[0] = color
        np.write()


print(f"{DEVICE} Pin {NP_PIN}")


