import time

import espnow

import wifi
from config import COLORS, NP_PIN, RECIEVERS, np, set_color, sig
from device import DEVICE

sta, ap = wifi.reset(sta=True, ap=False, channel=1)  # STA on, AP off, channel=1
espn : espnow.ESPNow|None = None

def init():
    global espn

    # Print the MAC address
    mac = sta.config('mac')
    print("Transmit : MAC address : " + ''.join(['\\x%02x' % b for b in mac]))
    print(f"Receivers : {RECIEVERS}")

    # Initialize ESP-NOW
    espn = espnow.ESPNow()
    espn.active(True)

def transmit(receivers:list[bytes] = RECIEVERS,delay:int=5):
    if not espn:
        raise ValueError("please init espnow before using")
    set_color("YELLOW")

    for mac in receivers:
        print("Adding peer: " + ''.join(['\\x%02x' % b for b in mac]))
        espn.add_peer(mac)

    # initial burst 
    send_burst()
    # Send messages in a loop
    while True:
        for color in ["ORANGE","GREEN", "BLUE", "PURPLE"]:
            send_color_message(color, receivers)
            time.sleep(delay/2)
            set_color("OFF")
            time.sleep(delay/2)

def send_burst(receivers:list[bytes] = RECIEVERS):
    if not espn:
        raise ValueError("please init espnow before using")        
    for mac in receivers:
        for message in ["ORANGE","GREEN", "BLUE", "PURPLE"]:
            try:
                espn.send(mac, message, False) # No Wait
            except Exception as e:
                print(f"Error sending burst message to {mac}: {e}")



def send_color_message(message:str, receivers:list[bytes] = RECIEVERS):
    if not espn:
        raise ValueError("please init espnow before using")    
    for mac in receivers:
        if sig: 
            sig(1)
        try:
            if espn.send(mac, message, True): # Wait for acknowledgment
                if sig: 
                    sig(0)
                print(f"Sent: {message}")
                set_color(message)
            else:
                if sig:
                    sig(0)
                print("Failed to send message to " + ''.join(['\\x%02x' % b for b in mac]))
                set_color("RED")
        except Exception as e:
            print(f"Error sending color message to {mac}: {e}")

