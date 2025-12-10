
import asyncio
import time

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
    send_alive,
    send_burst_async,
    send_color_message_async,
)

# Pin is PullDown - Needs 3.3v to wake 
button_pin = machine.Pin(BUTTON_GPIO, Pin.IN, pull=Pin.PULL_DOWN)

# set_color = print

async def evt_button(event: asyncio.Event, kleur: str):
    """Handle button events with ESPNow transmission"""
    e = 0
    while True:
        event.clear()
        await event.wait()
        e += 1
        keep_awake()
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


button = Switch(button_pin)


time_to_sleep = 10 

def keep_awake():
    global time_to_sleep
    time_to_sleep = 10 * 60  # Reset to 10 minutes


async def a_ds():
    """Countdown to deep sleep"""
    global time_to_sleep
    # Deep sleep after 10 minutes
    while time_to_sleep > 0:
        msg = f"sleeping in {time_to_sleep} seconds"
        print(msg)
        time_to_sleep -= 1
        await send_alive( msg)
        await asyncio.sleep(1)
    go_to_sleep()



async def run_button():
    print("trying")
    init_espnow()
    await send_alive( "Waking up")
    # Register events to be set on open and close
    button.open_func(None)
    button.close_func(None)
    keep_awake()
    tasks = []
    # Add butten event tasks: 
    # - GREEN for button press/(closed), Off for button released (open)
    tasks.append(asyncio.create_task(evt_button(button.open, "GREEN")))  # Button press
    tasks.append(asyncio.create_task(evt_button(button.close, "OFF")))  # Button release
    # Start other program tasks.
    repl = asyncio.create_task(aiorepl.task())
    ds_t = asyncio.create_task(a_ds())
    try:
        if button_pin.value():
            print("Button is initially pressed")
            # send event immediately
            await send_color_message_async("GREEN")

        #  asyncio.Event()
        print("Running main loop...")
        await asyncio.gather(*tasks, repl, ds_t)
    finally:
        # Cancel all tasks when done
        for task in tasks:
            task.cancel()
    print("Main loop exited.")


asyncio.run(run_button())