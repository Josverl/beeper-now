
from . import (
    _DAC_CTRL1_REG,
    _DAC_CTRL2_REG,
    CosineDAC,
    DACManager,
    _reg_get,
)


def check_hardware_sine_support():
    """
    Check if hardware sine wave generation is supported

    Returns:
        tuple: (supported, error_message)
    """
    try:
        # Just check if we can access the hardware registers
        # Don't create a DAC object as that can corrupt the DAC state
        ctrl1_val = _reg_get(_DAC_CTRL1_REG)
        ctrl2_val = _reg_get(_DAC_CTRL2_REG)
        print(
            f"Hardware sine registers accessible: CTRL1=0x{ctrl1_val:08x}, CTRL2=0x{ctrl2_val:08x}"
        )
        return True, "Hardware sine wave generator registers are accessible"

    except Exception as e:
        return False, f"Cannot access hardware sine registers: {e}"

def cleanup_all_dacs():
    """Clean up all DAC objects via the singleton manager"""
    manager = DACManager()
    manager.cleanup_all()

# Convenience function to get a hardware sine generator
def get_hardware_sine(pin_num=25, freq=440, scale=0, offset=127):
    """
    Convenience function to get a hardware sine generator

    Args:
        pin_num: GPIO pin number (25/26 for ESP32, 17/18 for ESP32-S2)
        freq: Frequency in Hz
        scale: Amplitude scale (0=full, 1=1/2, 2=1/4, 3=1/8)
        offset: DC offset (0-255)

    Returns:
        HardwareSineDAC instance
    """
    return CosineDAC(pin_num=pin_num, freq=freq, scale=scale, offset=offset)    