import asyncio
import time

import espnow

import wifi
from config import COLORS, NP_PIN, RECIEVERS, WIFI_CHANNEL, np, set_color, signal_led
from device import DEVICE, get_device_name

sta, ap = wifi.reset(
    sta=True, ap=False, channel=WIFI_CHANNEL
)  # STA on, AP off, channel=1
espn: espnow.ESPNow | None = None


def init():
    global espn

    # Print the MAC address
    mac = sta.config("mac")
    print("Transmit : MAC address : " + "".join(["\\x%02x" % b for b in mac]))
    print(f"Receivers : {RECIEVERS}")

    # Initialize ESP-NOW
    espn = espnow.ESPNow()
    espn.active(True)


def init_espnow():
    """Initialize ESPNow for wireless communication"""
    global sta, espn

    # Initialize WiFi and ESPNow
    sta, _ = wifi.reset(
        sta=True, ap=False, channel=WIFI_CHANNEL
    )  # STA on, AP off, channel=1

    # Print the MAC address
    mac = sta.config("mac")
    print("Transmit : MAC address : " + "".join([f"\\x{b:02x}" for b in mac]))
    print(f"Receivers : {RECIEVERS}")

    # Initialize ESP-NOW
    espn = espnow.ESPNow()
    espn.active(True)

    # Add all receivers as peers
    for mac in RECIEVERS:
        try:
            print("Adding peer: " + "".join([f"\\x{b:02x}" for b in mac]))
            espn.add_peer(mac)
        except Exception as e:
            print(f"Error adding peer {mac}: {e}")


def deactivate_espnow():
    global espn, sta
    if espn:
        espn.active(False)
        espn = None
        print("ESPNow deinitialized")
    if sta:
        sta.active(False)


def transmit(receivers: list[bytes] = RECIEVERS, delay: int = 5):
    if not espn:
        raise ValueError("please init espnow before using")
    set_color("YELLOW")

    for mac in receivers:
        print("Adding peer: " + "".join(["\\x%02x" % b for b in mac]))
        espn.add_peer(mac)

    # initial burst
    send_burst()
    # Send messages in a loop
    while True:
        for color in ["ORANGE", "GREEN", "BLUE", "PURPLE"]:
            send_color_message(color, receivers)
            time.sleep(delay / 2)
            set_color("OFF")
            time.sleep(delay / 2)


def send_burst(receivers: list[bytes] = RECIEVERS):
    if not espn:
        raise ValueError("please init espnow before using")
    for mac in receivers:
        for message in ["ORANGE", "GREEN", "BLUE", "PURPLE"]:
            try:
                espn.send(mac, message, False)  # No Wait
            except Exception as e:
                print(f"Error sending burst message to {mac}: {e}")


async def send_burst_async(receivers: list[bytes] = RECIEVERS):
    """Async version of send_burst for initial setup"""
    if not espn:
        print("ESPNow not initialized, cannot send burst")
        return False

    messages = ["ORANGE", "GREEN", "BLUE", "PURPLE"]
    for mac in receivers:
        for message in messages:
            try:
                espn.send(mac, message, False)  # No Wait for burst
                await asyncio.sleep_ms(10)  # Small delay between messages
            except Exception as e:
                print(f"Error sending burst message {message} to {mac}: {e}")

    print("Initial burst sent to all receivers")
    return True


def send_color_message(message: str, receivers: list[bytes] = RECIEVERS):
    if not espn:
        raise ValueError("please init espnow before using")
    for mac in receivers:
        if signal_led:
            signal_led(1)
        try:
            if espn.send(mac, message, True):  # Wait for acknowledgment
                if signal_led:
                    signal_led(0)
                print(f"Sent: {message}")
                set_color(message)
            else:
                if signal_led:
                    signal_led(0)
                print(
                    "Failed to send message to " + "".join(["\\x%02x" % b for b in mac])
                )
                set_color("RED")
        except Exception as e:
            print(f"Error sending color message to {mac}: {e}")


async def send_color_message_async(message: str, receivers: list[bytes] = RECIEVERS):
    """Async version of send_color_message"""
    if not espn:
        print("ESPNow not initialized, cannot send message")
        return False

    success = True
    for mac in receivers:
        dest = get_device_name(mac) or ":".join([f"{b:02x}" for b in mac])
        try:
            # Use a small delay to make it async-friendly
            await asyncio.sleep_ms(1)

            if espn.send(mac, message, True):  # Wait for acknowledgment
                print(f"Sent: {message} to {dest}")
            else:
                print(f"Failed to send message to {dest}")
                success = False
        except Exception as e:
            print(f"Error sending color message to {dest}: {e}")
            success = False

    return success
