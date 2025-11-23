import time

from . import (
    _DAC_CTRL1_REG,
    _DAC_CTRL2_REG,
    CosineDAC,
    DACManager,
    _reg_get,
)
from .utils import check_hardware_sine_support


def demo():
    """Demonstrate hardware sine wave generation"""
    print("ESP32 Hardware Sine Wave Demo")
    print("=============================")

    # First check if hardware sine is supported
    print("Checking hardware sine wave support...")
    supported, message = check_hardware_sine_support()
    print(f"Support check: {message}")

    if not supported:
        print("\nHardware sine wave is not available!")
        print("Possible solutions:")
        print("1. import machine; machine.reset()")
        print("2. Use software sine wave generation instead")
        return

    hw_sine = None
    try:
        # Create hardware sine generator
        print("\nInitializing hardware sine generator...")
        hw_sine = CosineDAC(pin_num=25, freq=440)

        print("\n1. Starting 440 Hz sine wave...")
        hw_sine.start()
        time.sleep(2)

        print("\n2. Changing to 880 Hz...")
        hw_sine.set_frequency(880)
        time.sleep(2)

        print("\n3. Reducing amplitude to 1/2...")
        hw_sine.set_amplitude_scale(1)
        time.sleep(2)

        print("\n4. Adding 180 degree phase shift...")
        hw_sine.set_phase(180)
        time.sleep(2)

        print("\n5. Back to 0 degree phase...")
        hw_sine.set_phase(0)
        time.sleep(1)

        print("\nStopping hardware sine wave...")
        hw_sine.stop()

        print("\nDemo complete!")

    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
        if hw_sine:
            try:
                hw_sine.stop()
            except Exception:
                pass
    except Exception as e:
        print(f"\nDemo error: {e}")
        print("This might be due to:")
        print("1. DAC driver not properly initialized")
        print("2. Hardware sine generator already in use")
        print("3. ESP32 variant doesn't support hardware sine")
        print("\nTry: import machine; machine.reset()")


# To run the demo manually, call: demo()
