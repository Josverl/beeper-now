
from . import (
    _DAC_CTRL1_REG,
    _DAC_CTRL2_REG,
    CosineDAC,
    DACManager,
    _reg_get,
)


def check_hardware_cosine_support():
    """
    Check if hardware cosine wave generation is supported

    Returns:
        tuple: (supported, error_message)
    """
    try:
        # Just check if we can access the hardware registers
        # Don't create a DAC object as that can corrupt the DAC state
        ctrl1_val = _reg_get(_DAC_CTRL1_REG)
        ctrl2_val = _reg_get(_DAC_CTRL2_REG)
        print(
            f"Hardware cosine registers accessible: CTRL1=0x{ctrl1_val:08x}, CTRL2=0x{ctrl2_val:08x}"
        )
        return True, "Hardware cosine wave generator registers are accessible"

    except Exception as e:
        return False, f"Cannot access hardware cosine registers: {e}"

def cleanup_all_dacs():
    """Clean up all DAC objects via the singleton manager"""
    manager = DACManager()
    manager.cleanup_all()
