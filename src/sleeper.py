import asyncio
from typing import NoReturn

import esp32
import machine
from machine import Pin

from config import BUTTON_GPIO, set_color
from transmit import deactivate_espnow


async def prepare_for_sleep():
    # Show visual indication we're going to sleep
    set_color("PURPLE")
    await asyncio.sleep_ms(500)
    set_color("OFF")
    go_to_sleep()
    print("This line should never be reached after prepare_for_sleep()")


def go_to_sleep() -> NoReturn:
    """Prepare the system for deep sleep and enter deep sleep mode"""
    print("Preparing for deep sleep...")

    esp32.wake_on_gpio((Pin(BUTTON_GPIO),), esp32.WAKEUP_ANY_HIGH)
    print("GPIO wake-up configured")

    print("Entering deep sleep...")
    print("Wake-up: Press reset or power cycle to wake")

    # Turn off all peripherals before sleep
    deactivate_espnow()

    # Enter deep sleep
    machine.deepsleep()