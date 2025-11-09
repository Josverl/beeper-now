This firmware is built using PR: https://github.com/micropython/micropython/pull/17518

# esp32: Add esp32.wake_on_gpio.

Some boards support waking up from deepsleep via GPIO pins (for instance ESP32C3, ESP32C6), but this is not currently supported by MicroPython. This commit adds support for waking with GPIO in a similar interface to waking with ext0, ext1, touch and ulp. This commit adds documentation for this new function as well.

## Flash

esptool --chip esp32c3 --port COM10 -b 921600 write_flash --flash_mode keep --flash_size detect --compress 0x0 esp32_c3_firmware.bin


Tested ESP32_GENERIC_C 3with:

```py 
import machine
import esp32
esp32.wake_on_gpio((machine.Pin(5),machine.Pin(2)), esp32.WAKEUP_ANY_HIGH)
machine.deepsleep()
```

and checked that the relevant pins wake up from deepsleep.