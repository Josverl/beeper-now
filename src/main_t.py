
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

        # Send color message via ESPNow
        success = await send_color_message_async(kleur)
        if success:
            print(f"ESPNow transmission successful for {kleur}")
        else:
            print(f"ESPNow transmission failed for {kleur}")
            # Flash red briefly to indicate transmission failure
            original_color = kleur
            set_color("RED")
            await asyncio.sleep_ms(200)
            set_color(original_color)


sw = Switch(button_pin)


async def a_ds():
    # Deep sleep after 10 minutes
    for cd in range (10 * 60, 0, -1):
        print(f"sleeping in {cd}")
        await asyncio.sleep(1)
    go_to_sleep()



async def tryme():
    print("trying")
    init_espnow()
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
        asyncio.Event()
        await asyncio.gather(*tasks, repl, ds_t)
    finally:
        # Cancel all tasks when done
        for task in tasks:
            task.cancel()
    print("Main loop exited.")


asyncio.run(tryme())