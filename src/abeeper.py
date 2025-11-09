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
import machine
from machine import Pin
from primitives.switch import Switch

from config import COLORS, NP_PIN, RECIEVERS, np, set_color, signal_led
from inactivity import InactivityTimer


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
        inactivity_timer.reset()
        print(f"Event {event} triggered {e} times, pulsing {kleur}")
        set_color(kleur)


async def housekeeping():
    """Periodic housekeeping tasks."""
    while True:
        await asyncio.sleep(10)
        gc.collect()
        gc.threshold(gc.mem_free() // 4 + gc.mem_alloc())


# Test for the Switch class (events) with inactivity monitoring
async def a_main():
    set_global_exception()  # Debug aid
    print("Starting switch event test with 2-minute inactivity timer...")

    # Initialize activity timer
    inactivity_timer.reset()

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


def do_beeper_button(timeout_seconds=20 * 60):  # Default 20 minutes
    inactivity_timer.timeout_ms = timeout_seconds * 1000
    try:
        asyncio.run(a_main())
    except KeyboardInterrupt:
        print("Interrupted")
    finally:
        asyncio.new_event_loop()


do_beeper_button()
