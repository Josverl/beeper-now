"""
Basic Async beeper framework with button handling using primitives.pushbutton

Features:
    1. Deep sleep after 2 minutes of inactivity
    2. Activity detection from switch events
    3. Graceful shutdown preparation

- integrate with ESPNow transmit.py for remote triggering

"""

import asyncio
import gc

import aiorepl
import esp32
import espnow
import machine
from machine import Pin
from primitives.switch import Switch

import wifi
from config import BUTTON_GPIO, RECIEVERS, np, set_color
from inactivity import InactivityTimer
from transmit import (
    deactivate_espnow,
    init_espnow,
    send_burst_async,
    send_color_message_async,
)

# +3.3 -> Button-NO --> Button_GPIO
button_pin = Pin(BUTTON_GPIO, Pin.IN, Pin.PULL_DOWN)

# Global exception handler for asyncio
def set_global_exception():
    def handle_exception(loop, context):
        import sys

        sys.print_exception(context["exception"])  # type: ignore
        sys.exit()

    loop = asyncio.get_event_loop()
    loop.set_exception_handler(handle_exception)


# Create a single instance to be used throughout the module
inactivity_timer = InactivityTimer()


async def inactivity_monitor():
    """Monitor for inactivity and go to deep sleep after timeout"""
    print("Starting inactivity monitor...")
    while True:
        remaining = inactivity_timer.remaining_time_ms

        # Show countdown every 10 seconds when less than 1 minute remains
        if remaining <= 60_000 and remaining > 0:  # Less than 60 seconds left
            remaining_sec = remaining // 1000
            if remaining_sec % 10 == 0 or remaining_sec <= 10:
                print(f"Deep sleep in {remaining_sec} seconds...")
                # Flash yellow as warning
                set_color("YELLOW")
                await asyncio.sleep_ms(100)
                set_color("OFF")

        # Check if we've exceeded the inactivity timeout
        if inactivity_timer.is_timeout_reached:
            print("Inactivity timeout reached - preparing for deep sleep...")
            await prepare_for_sleep()
            # This function should not return
        else:
            print("wait some more...") 

        # Check every 1 second
        await asyncio.sleep_ms(1000)


async def prepare_for_sleep():
    """Prepare the system for deep sleep and enter deep sleep mode"""
    print("Preparing for deep sleep...")

    # Show visual indication we're going to sleep
    set_color("PURPLE")
    await asyncio.sleep_ms(500)
    set_color("OFF")
    pins = [Pin(i, Pin.IN, pull=Pin.PULL_DOWN) for i in range(0, 5)]  # Set all pins to input to reduce power

    # esp32.wake_on_gpio((button_pin,), esp32.WAKEUP_ANY_HIGH)  # type: ignore
    esp32.wake_on_gpio((machine.Pin(5),machine.Pin(2)), esp32.WAKEUP_ANY_HIGH)
    print("GPIO wake-up configured")


    print("Entering deep sleep...")
    print("Wake-up: Press reset or power cycle to wake")

    # Turn off all peripherals before sleep
    deactivate_espnow()

    if np:
        set_color("OFF")

    # Enter deep sleep
    machine.deepsleep()


async def evt_button(event: asyncio.Event, kleur: str):
    """Handle button events with ESPNow transmission"""
    e = 0
    while True:
        event.clear()
        await event.wait()
        e += 1
        # Reset activity timer whenever an event is triggered
        inactivity_timer.reset()
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


async def housekeeping():
    """Periodic housekeeping tasks."""
    while True:
        await asyncio.sleep(10)
        gc.collect()
        gc.threshold(gc.mem_free() // 4 + gc.mem_alloc())


# Test for the Switch class (events) with inactivity monitoring
async def a_main():
    set_global_exception()  # Debug aid
    print("Inactivity + ESPNow integration...")

    # Initialize ESPNow
    try:
        init_espnow()
        print("ESPNow initialized successfully")

        # Send initial burst to announce presence
        await send_burst_async()

    except Exception as e:
        print(f"ESPNow initialization failed: {e}")
        print("Continuing without ESPNow functionality")

    # Initialize activity timer
    inactivity_timer.reset()
    sw = Switch(button_pin)
    # Register events to be set on open and close
    sw.open_func(None)
    sw.close_func(None)

    tasks = []
    # Add butten event tasks: 
    # - GREEN for button press/(closed), Off for button released (open)
    tasks.append(asyncio.create_task(evt_button(sw.open, "GREEN")))  # Button press
    tasks.append(asyncio.create_task(evt_button(sw.close, "OFF")))  # Button release

    # Add inactivity monitoring task
    tasks.append(asyncio.create_task(inactivity_monitor()))
    tasks.append(asyncio.create_task(housekeeping()))

    # Start other program tasks.
    repl = asyncio.create_task(aiorepl.task())

    try:
        print("Running main loop...")
        await asyncio.gather(*tasks, repl)
    finally:
        # Cancel all tasks when done
        for task in tasks:
            task.cancel()
    print("Main loop exited.")


def do_beeper_button(timeout_seconds=20 * 60):  # Default 20 minutes
    init_espnow()
    inactivity_timer.timeout_ms = timeout_seconds * 1000
    # DEBUG 
    inactivity_timer.timeout_ms = 5 * 1000
    try:
        asyncio.run(a_main())
    except KeyboardInterrupt:
        print("Interrupted")
    finally:
        asyncio.new_event_loop()


do_beeper_button()
