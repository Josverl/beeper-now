
import asyncio

import aiorepl
import machine
from machine import Pin
from primitives.switch import Switch

from config import BUTTON_GPIO, RECIEVERS, np, set_color
from inactivity import InactivityTimer
from sleeper import go_to_sleep
from transmit import (
    deactivate_espnow,
    init_espnow,
    send_burst_async,
    send_color_message_async,
)

button_pin = machine.Pin(BUTTON_GPIO, Pin.IN, pull=Pin.PULL_DOWN)

# set_color = print

async def evt_button(event: asyncio.Event, kleur: str):
    """Handle button events with ESPNow transmission"""
    e = 0
    while True:
        event.clear()
        await event.wait()
        e += 1
        # Reset activity timer whenever an event is triggered
        # inactivity_timer.reset()
        print(f"Event {event} triggered {e} times, pulsing {kleur}")

        # Set local color
        set_color(kleur)



sw = Switch(button_pin)


async def a_ds():
    await asyncio.sleep(1)
    await asyncio.sleep(2)
    await asyncio.sleep(5)
    for cd in range (5, 0, -1):
        print(f"sleeping in {cd}")
        await asyncio.sleep(1)
    go_to_sleep()
    # # esp32.wake_on_gpio((machine.Pin(5),), esp32.WAKEUP_ANY_HIGH) # OK 
    # esp32.wake_on_gpio((button_pin,), esp32.WAKEUP_ANY_HIGH) # OK 
    # set_color("OFF")
    # machine.deepsleep()



async def tryme():
    print("trying")
    # Register events to be set on open and close
    sw.open_func(None)
    sw.close_func(None)

    tasks = []
    # Add butten event tasks: 
    # - GREEN for button press/(closed), Off for button released (open)
    tasks.append(asyncio.create_task(evt_button(sw.open, "GREEN")))  # Button press
    tasks.append(asyncio.create_task(evt_button(sw.close, "OFF")))  # Button release
    # Start other program tasks.
    repl = asyncio.create_task(aiorepl.task())
    ds_t = asyncio.create_task(a_ds())
    try:
        print("Running main loop...")
        await asyncio.gather(*tasks, repl, ds_t)
    finally:
        # Cancel all tasks when done
        for task in tasks:
            task.cancel()
    print("Main loop exited.")


asyncio.run(tryme())