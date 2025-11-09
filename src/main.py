import time

import esp32
import espnow
import machine

import wifi
from config import COLORS, NP_PIN, np, set_color

MOTION_GPIO = 4
BUTTON_GPIO = 5

def configure_wake(
    motion_gpio: int, button_gpio: int
) -> tuple[machine.Pin, machine.Pin]:
    # INT pin configureren (pulldown als INT actief hoog is; anders pullup)
    pull = machine.Pin.PULL_DOWN
    # Wake op HIGH of LOW (kies passend bij INT logica)
    level = esp32.WAKEUP_ANY_HIGH

    # setup pins
    motion_interrupt_pin = machine.Pin(motion_gpio, machine.Pin.IN, pull)
    button_pin = machine.Pin(button_gpio, machine.Pin.IN, pull)

    esp32.wake_on_gpio((motion_interrupt_pin, button_pin), level)  # type: ignore

    return motion_interrupt_pin, button_pin

def main(snooze=5):
    print("wakeup tests")
    sta, ap = wifi.reset()
    e = espnow.ESPNow()
    print("Reset cause:", machine.reset_cause())
    p_motion, p_button = configure_wake(MOTION_GPIO, BUTTON_GPIO)
    print("Motion pin:", p_motion.value())
    print("Button pin:", p_button.value())
    time.sleep(1)
    print(f"getting ready to sleep for {snooze}s")
    for i in range(5):
        set_color("BLUE")
        time.sleep_ms(200)
        set_color("GREEN")
        time.sleep_ms(200)
    if p_button.value() == 1:
        print("Button pressed, staying awake")
        set_color("RED")
        return True
    # prepare to sleep
    e.active(False)
    sta.active(False)                 # Disable the wifi before sleep    
    set_color("OFF")
    # Sleep
    # machine.lightsleep(snooze * 1000)
    machine.deepsleep(snooze * 1000)
    print("should not get here")
    


if __name__ == "__main__":
    while main(10):
        pass