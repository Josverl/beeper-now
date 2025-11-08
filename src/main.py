# main.py
# ESP32-C3 + MPU6050: ga naar deep sleep na X sec geen beweging,
# en wordt weer wakker bij nieuwe beweging via INT-pin (GPIO wake).

import time
from math import e

import esp32
import machine

from config import COLORS, NP_PIN, np, set_color
from mpu6050_motion import MPU6050Motion

# ---------- Config (pas aan voor jouw board) ----------
# I2C pins (voorbeeld voor veel ESP32-C3 devkits)
I2C_ID      = 0
I2C_SCL_PIN = 6
I2C_SDA_PIN = 7
I2C_FREQ    = 400_000
MPU_ADDR    = 0x68

# INT-pin van de MPU6050 → RTC-capable GPIO op ESP32-C3 (bijv. GPIO4)
MOTION_GPIO    = 4
BUTTON_GPIO    = 5
INT_ACTIVE_HIGH = True  # moet passen bij INT_PIN_CFG in de driver

# Motion detect parameters
MOTION_THRESHOLD = 12    # begin met 8..16 (≈ mg-achtig; kalibreren!)
MOTION_DURATION_MS = 40  # 20..80 ms is vaak stabiel

# Geen beweging -> naar slaap na deze tijd
NO_MOTION_TIMEOUT_S = 10

# In deep sleep: timer backup (optioneel). Zet op None voor onbeperkt.
DEEPSLEEP_MAX_MS = None  # bv. 6*60*60*1000 voor 6 uur max slaap
DEEPSLEEP_MAX_MS = 20_000

# Zet MPU in low-power accel mode tijdens deep sleep?
USE_LOW_POWER_ACCEL_CYCLE = False

# ------------------------------------------------------

def setup_i2c():
    return machine.I2C(I2C_ID,
                       scl=machine.Pin(I2C_SCL_PIN, machine.Pin.PULL_UP),
                       sda=machine.Pin(I2C_SDA_PIN, machine.Pin.PULL_UP),
                       freq=I2C_FREQ)

def configure_wake(motion_gpio:int, button_gpio:int) -> tuple[machine.Pin, machine.Pin]:
    # INT pin configureren (pulldown als INT actief hoog is; anders pullup)
    pull = machine.Pin.PULL_DOWN if INT_ACTIVE_HIGH else machine.Pin.PULL_UP
    # Wake op HIGH of LOW (kies passend bij INT logica)
    level = esp32.WAKEUP_ANY_HIGH if INT_ACTIVE_HIGH else esp32.WAKEUP_ALL_LOW

    # setup pins
    motion_interrupt_pin = machine.Pin(MOTION_GPIO, machine.Pin.IN, pull)
    button_pin = machine.Pin(BUTTON_GPIO, machine.Pin.IN, pull)

    esp32.wake_on_gpio((motion_interrupt_pin,button_pin), level) # type: ignore

    return motion_interrupt_pin, button_pin

    

def go_deepsleep(ms=None):
    print("-> Ga naar deep sleep (ms =", ms, ")")
    if ms is None:
        machine.deepsleep()
    else:
        machine.deepsleep(ms)

def main():
    if machine.reset_cause() == machine.DEEPSLEEP_RESET:
        print("Wakker geworden uit deep sleep")
        set_color("GREEN")
    else:
        print("Normale start")
        set_color("BLUE")


    motion_interrupt_pin, button_pin = configure_wake(MOTION_GPIO, BUTTON_GPIO)

    # I2C + MPU init
    i2c = setup_i2c()
    print("I2C apparaten gevonden:", i2c.scan())
    mpu = MPU6050Motion(i2c, addr=MPU_ADDR)

    # Wek sensor en stel accel + motion interrupt in
    mpu.wake()
    mpu.configure_accel(fsr_g=2, dlpf_hz=44)
    mpu.configure_motion_interrupt(
        threshold=MOTION_THRESHOLD,
        duration_ms=MOTION_DURATION_MS,
        int_active_high=INT_ACTIVE_HIGH,
        latch=True,           # latched INT geeft stabiel wake-signaal
        open_drain=False
    )
    # Heel belangrijk: latched INT wissen zodat we niet in een interrupt-loop blijven
    mpu.clear_interrupt()

    print("Bewegingsmonitor gestart; timeout =", NO_MOTION_TIMEOUT_S, "s")
    t0 = time.ticks_ms()

    # Event-loop: zolang er af en toe beweging is, blijf wakker.
    while True:
        # Als INT actief is → beweging gedetecteerd
        if motion_interrupt_pin.value() == (1 if INT_ACTIVE_HIGH else 0):
            print("Beweging gedetecteerd (INT) - timer reset")
            # Wissen om latched lijn te laten zakken
            mpu.clear_interrupt()
            t0 = time.ticks_ms()

        # Geen beweging binnen timeout? → deep sleep.
        if time.ticks_diff(time.ticks_ms(), t0) > NO_MOTION_TIMEOUT_S * 1000:
            print("Geen beweging voor", NO_MOTION_TIMEOUT_S, "s → deep sleep voorbereiden")
            # Zet sensor in low-power accel 'cycle' modus zodat INT actief kan blijven met laag verbruik
            if USE_LOW_POWER_ACCEL_CYCLE:
                mpu.enable_low_power_accel_cycle(lp_wake_rate_hz=5, disable_gyro=True, disable_temp=True)
            set_color("OFF")
            # Configureer (optioneel) een max slaapduur als back-up
            go_deepsleep(DEEPSLEEP_MAX_MS)

        time.sleep_ms(50)

# Autostart
main()