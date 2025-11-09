# main.py
# ESP32-C3 + MPU6050: ga naar deep sleep na X sec geen beweging,
# en wordt weer wakker bij nieuwe beweging via INT-pin (GPIO wake).

import time
from math import e

import esp32
import machine

from config import COLORS, NP_PIN, np, set_color
from mpu6050 import MPU6050

# ---------- Config (pas aan voor jouw board) ----------
# I2C pins (voorbeeld voor veel ESP32-C3 devkits)
I2C_ID = 0
I2C_SCL_PIN = 6
I2C_SDA_PIN = 7
I2C_FREQ = 400_000
MPU_ADDR = 0x68

# INT-pin van de MPU6050 → RTC-capable GPIO op ESP32-C3 (bijv. GPIO4)
MOTION_GPIO = 4
BUTTON_GPIO = 5
INT_ACTIVE_HIGH = True  # moet passen bij INT_PIN_CFG in de driver

# Motion detect parameters
MOTION_THRESHOLD = 8     # More reasonable sensitivity (was 4) 
MOTION_DURATION_MS = 20  # Longer duration (was 10) for stability

# Geen beweging -> naar slaap na deze tijd
NO_MOTION_TIMEOUT_S = 10

# In deep sleep: timer backup (optioneel). Zet op None voor onbeperkt.
DEEPSLEEP_MAX_MS = None  # bv. 6*60*60*1000 voor 6 uur max slaap
DEEPSLEEP_MAX_MS = 20_000

# Zet MPU in low-power accel mode tijdens deep sleep?
USE_LOW_POWER_ACCEL_CYCLE = False

# ------------------------------------------------------


def setup_i2c():
    return machine.I2C(
        I2C_ID,
        scl=machine.Pin(I2C_SCL_PIN, machine.Pin.PULL_UP),
        sda=machine.Pin(I2C_SDA_PIN, machine.Pin.PULL_UP),
        freq=I2C_FREQ,
    )


def configure_wake(
    motion_gpio: int, button_gpio: int
) -> tuple[machine.Pin, machine.Pin]:
    # INT pin configureren (pulldown als INT actief hoog is; anders pullup)
    pull = machine.Pin.PULL_DOWN if INT_ACTIVE_HIGH else machine.Pin.PULL_UP
    # Wake op HIGH of LOW (kies passend bij INT logica)
    level = esp32.WAKEUP_ANY_HIGH if INT_ACTIVE_HIGH else esp32.WAKEUP_ALL_LOW

    # setup pins
    motion_interrupt_pin = machine.Pin(MOTION_GPIO, machine.Pin.IN, pull)
    button_pin = machine.Pin(BUTTON_GPIO, machine.Pin.IN, pull)

    esp32.wake_on_gpio((motion_interrupt_pin, button_pin), level)  # type: ignore

    return motion_interrupt_pin, button_pin


def go_deepsleep(ms=None):
    print("-> Ga naar deep sleep (ms =", ms, ")")
    if ms is None:
        machine.deepsleep()
    else:
        machine.deepsleep(ms)


def test_motion_detection(mpu :MPU6050, motion_interrupt_pin: machine.Pin, duration_s=30):
    """
    Test function to verify motion detection is working correctly.

    Args:
        mpu: MPU6050 instance
        motion_interrupt_pin: The GPIO pin connected to MPU6050 INT
        duration_s: How long to test for (default 30 seconds)
    """
    print(f"\n=== TESTING MOTION DETECTION FOR {duration_s} SECONDS ===")
    print("Try moving the device to test motion detection...")
    print("Expected behavior:")
    print("  - INT pin should go HIGH when motion is detected")
    print("  - Message should appear when motion is detected")
    print("  - INT pin should stay HIGH until interrupt is cleared")
    print()

    # Clear any pending interrupts
    mpu.clear_motion_interrupt()

    start_time = time.ticks_ms()
    motion_count = 0
    last_motion_time = 0
    last_progress_time = 0

    while time.ticks_diff(time.ticks_ms(), start_time) < duration_s * 1000:
        current_time = time.ticks_ms()
        
        # Show progress every 5 seconds
        if time.ticks_diff(current_time, last_progress_time) > 5000:
            elapsed = time.ticks_diff(current_time, start_time) // 1000
            remaining = duration_s - elapsed
            print("[{:2d}s] Testing... {} seconds remaining (detected: {} motions)".format(elapsed, remaining, motion_count))
            last_progress_time = current_time
        current_time = time.ticks_ms()

        # Check if interrupt pin is active
        int_pin_active = motion_interrupt_pin.value() == (1 if INT_ACTIVE_HIGH else 0)

        if int_pin_active:
            # Check if this is a new motion event (debounce)
            if time.ticks_diff(current_time, last_motion_time) > 1000:  # 1000ms debounce
                motion_count += 1
                elapsed_s = time.ticks_diff(current_time, start_time) // 1000
                print(
                    "[{:3d}s] Motion #{} detected!".format(elapsed_s, motion_count)
                )
                print("     INT pin state: {}".format('HIGH' if int_pin_active else 'LOW'))

                # Read and display interrupt status
                int_status = mpu.read_interrupt_status()
                print("     Interrupt status register: 0x{:02X}".format(int_status))

                # Read sensor values
                try:
                    sensors = mpu.read_sensors_scaled()
                    print(
                        "     Accel: X={:6.2f}g Y={:6.2f}g Z={:6.2f}g".format(sensors.AccX, sensors.AccY, sensors.AccZ)
                    )
                except Exception:
                    print("     Could not read sensor values")

                last_motion_time = current_time

                # Clear the interrupt (this should make INT pin go low if latched)
                mpu.clear_motion_interrupt()
                set_color("GREEN")
                time.sleep_ms(100)
                set_color("BLUE")
        else:
            # Show pin state periodically even when no motion
            if time.ticks_diff(current_time, last_progress_time) > 5000:
                int_status = mpu.read_interrupt_status() 
                print("     Current: INT pin={}, Status=0x{:02X}".format(motion_interrupt_pin.value(), int_status))

        time.sleep_ms(100)  # Longer sleep to reduce CPU usage

    print("\\n=== TEST COMPLETE ===")
    print("Total motion events detected: {}".format(motion_count))
    print("Final INT pin state: {}".format('HIGH' if motion_interrupt_pin.value() else 'LOW'))

    if motion_count > 0:
        print("SUCCESS: Motion detection is working!")
    else:
        print("WARNING: No motion detected. Try moving device more or check sensitivity.")

    return motion_count > 0


def main():
    # if machine.reset_cause() == machine.DEEPSLEEP_RESET:
    #     print("Wakker geworden uit deep sleep")
    #     set_color("GREEN")
    # else:
    #     print("Normale start")
    #     set_color("BLUE")

    motion_interrupt_pin, button_pin = configure_wake(MOTION_GPIO, BUTTON_GPIO)

    # I2C + MPU init
    i2c = setup_i2c()
    print("I2C apparaten gevonden:", i2c.scan())
    mpu = MPU6050(i2c, address=MPU_ADDR)

    # Print current configuration for debugging
    mpu.print_configuration()

    # Setup complete motion wake detection with one convenient method
    mpu.setup_motion_wake(
        threshold=MOTION_THRESHOLD,
        duration_ms=MOTION_DURATION_MS,
        active_high=INT_ACTIVE_HIGH,
        low_power=USE_LOW_POWER_ACCEL_CYCLE,
    )

    print("Bewegingsmonitor gestart; timeout =", NO_MOTION_TIMEOUT_S, "s")

    # Optional: Run test function for 15 seconds to verify motion detection
    # Uncomment the next few lines to test motion detection before going into normal operation
    print("\\nStarting motion detection test with reasonable settings...")
    
    # Use reasonable settings for testing (not too sensitive)
    mpu.set_motion_detection_threshold(8)   # Reasonable sensitivity 
    mpu.set_motion_detection_duration(20)   # Longer duration for stability
    mpu.configure_interrupt_pin(active_high=INT_ACTIVE_HIGH, latch=True)
    mpu.enable_motion_interrupt()
    
    # Wait a moment for settings to take effect
    time.sleep_ms(100)
    mpu.clear_motion_interrupt()
    
    print("Motion detection configured:")
    print("  Threshold: 8 (reasonable sensitivity)")
    print("  Duration: 20ms") 
    print("  INT pin: GPIO{}, active {}".format(MOTION_GPIO, 'HIGH' if INT_ACTIVE_HIGH else 'LOW'))
    
    test_success = test_motion_detection(mpu, motion_interrupt_pin, duration_s=15)  # Shorter test
    if not test_success: 
        print("WARNING: Motion detection test failed!")

    print("Starting normal operation...")
    t0 = time.ticks_ms()

    # Event-loop: zolang er af en toe beweging is, blijf wakker.
    while True:
        # Als INT actief is → beweging gedetecteerd
        if motion_interrupt_pin.value() == (1 if INT_ACTIVE_HIGH else 0):
            print("Beweging gedetecteerd (INT) - timer reset")
            # Wissen om latched lijn te laten zakken
            mpu.clear_motion_interrupt()
            t0 = time.ticks_ms()

        # Geen beweging binnen timeout? → deep sleep.
        if time.ticks_diff(time.ticks_ms(), t0) > NO_MOTION_TIMEOUT_S * 1000:
            print(
                "Geen beweging voor", NO_MOTION_TIMEOUT_S, "s → deep sleep voorbereiden"
            )
            # The low-power cycle mode is already enabled if USE_LOW_POWER_ACCEL_CYCLE was True
            # during setup_motion_wake(), so no need to enable it again here
            set_color("OFF")
            # Configureer (optioneel) een max slaapduur als back-up
            go_deepsleep(DEEPSLEEP_MAX_MS)

        time.sleep_ms(50)


# Autostart
main()
