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
import time

import aiorepl
import esp32
import machine
from machine import Pin
from primitives.switch import Switch

from config import COLORS, NP_PIN, RECIEVERS, np, set_color, signal_led


def set_global_exception():
    def handle_exception(loop, context):
        import sys

        sys.print_exception(context["exception"])  # type: ignore
        sys.exit()

    loop = asyncio.get_event_loop()
    loop.set_exception_handler(handle_exception)


# Global inactivity timer
inactivity_timeout_ms = 2 * 60 * 1000  # 2 minutes in milliseconds
last_activity_time = time.ticks_ms()


def reset_activity_timer():
    """Reset the inactivity timer - call this whenever there's activity"""
    global last_activity_time
    last_activity_time = time.ticks_ms()
    print(f"Activity detected, timer reset at {last_activity_time}")


async def inactivity_monitor():
    """Monitor for inactivity and go to deep sleep after timeout"""
    global last_activity_time

    while True:
        current_time = time.ticks_ms()
        elapsed = time.ticks_diff(current_time, last_activity_time)
        remaining = inactivity_timeout_ms - elapsed

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
        if elapsed >= inactivity_timeout_ms:
            print("Inactivity timeout reached - preparing for deep sleep...")
            await prepare_for_sleep()
            # This function should not return

        # Check every 1 second
        await asyncio.sleep_ms(1000)


async def prepare_for_sleep():
    """Prepare the system for deep sleep and enter deep sleep mode"""
    print("Preparing for deep sleep...")

    # Show visual indication we're going to sleep
    set_color("PURPLE")
    await asyncio.sleep_ms(500)
    set_color("OFF")

    # Configure wake-up sources (similar to your main_button.py)
    motion_gpio = 4
    button_gpio = 5

    try:
        # Try to configure GPIO wake-up if available (custom firmware feature)
        pull = machine.Pin.PULL_DOWN

        motion_pin = machine.Pin(motion_gpio, machine.Pin.IN, pull)
        button_pin = machine.Pin(button_gpio, machine.Pin.IN, pull)

        # Note: wake_on_gpio is a custom firmware feature, check if available
        if hasattr(esp32, "wake_on_gpio") and hasattr(esp32, "WAKEUP_ANY_HIGH"):
            level = esp32.WAKEUP_ANY_HIGH  # type: ignore
            esp32.wake_on_gpio((motion_pin, button_pin), level)  # type: ignore
            print("GPIO wake-up configured")
        else:
            print("Custom GPIO wake-up not available")
            # Try standard ESP32 wake methods
            if hasattr(esp32, "wake_on_ext0"):
                # Use external wake on single pin (button)
                esp32.wake_on_ext0(button_pin, 1)  # type: ignore
                print("EXT0 wake-up configured on button pin")
    except Exception as e:
        print(f"Wake-up configuration failed: {e}")
        # Just go to sleep - system will wake on reset button or power cycle

    print("Entering deep sleep...")
    print("Wake-up: Press reset or power cycle to wake")

    # Turn off all peripherals before sleep
    if np:
        set_color("OFF")

    # Enter deep sleep
    machine.deepsleep()


async def evt_pulse(event: asyncio.Event, kleur: str):
    e = 0
    while True:
        event.clear()
        await event.wait()
        e += 1
        # Reset activity timer whenever an event is triggered
        reset_activity_timer()
        print(f"Event {event} triggered {e} times, pulsing {kleur}")
        set_color(kleur)
        # await asyncio.sleep_ms(500)
        # set_color("OFF")



# Quit test by connecting Pin 0 to ground
async def killer(obj, gpio = 0):
    pin = Pin(gpio, Pin.IN, Pin.PULL_UP)
    while pin.value():
        await asyncio.sleep_ms(50)
    obj.deinit()
    await asyncio.sleep_ms(0)

async def housekeeping():
    """Periodic housekeeping tasks."""
    while True:
        await asyncio.sleep(10)
        gc.collect()
        gc.threshold(gc.mem_free() // 4 + gc.mem_alloc())
        
# Test for the Switch class (events) with inactivity monitoring
async def amain():
    set_global_exception()  # Debug aid
    print("Starting switch event test with 2-minute inactivity timer...")

    # Initialize activity timer
    reset_activity_timer()

    pin = Pin(5, Pin.IN, Pin.PULL_DOWN)
    sw = Switch(pin)
    # Register events to be set on open and close
    sw.open_func(None)
    sw.close_func(None)

    tasks = []
    # Add event pulse tasks
    tasks.append(asyncio.create_task(evt_pulse(sw.close, "RED")))
    tasks.append(asyncio.create_task(evt_pulse(sw.open, "GREEN")))

    # Add inactivity monitoring task
    tasks.append(asyncio.create_task(inactivity_monitor()))
    tasks.append(asyncio.create_task(housekeeping()))

    # Start other program tasks.
    repl = asyncio.create_task(aiorepl.task())

    try:
        await asyncio.gather(*tasks, repl)
    finally:
        # Cancel all tasks when done
        for task in tasks:
            task.cancel()


def test_sw_event(timeout_seconds=30):
    global inactivity_timeout_ms
    inactivity_timeout_ms = timeout_seconds * 1000
    try:
        asyncio.run(amain())
    except KeyboardInterrupt:
        print("Interrupted")
    finally:
        asyncio.new_event_loop()


# Run the test (comment out if you don't want auto-run)
test_sw_event()
