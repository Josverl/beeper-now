"""
ESP32 Hardware cosine wave Generator

This module provides access to the ESP32's built-in hardware cosine wave generator
for the DAC. This produces much cleaner audio than software-based approaches.

Based on the implementation from:
https://www.i-programmer.info/programming/148-hardware/16391-esp32-in-micropython-using-hardware-registers.html

Hardware Features:
- Single hardware cosine wave generator shared by both DACs
- Each DAC can have independent scale, offset, and phase
- Much higher quality audio than software generation
- Reduced CPU load

GPIO Pins:
- ESP32: DAC1 = GPIO 25 (Channel 1), DAC2 = GPIO 26 (Channel 2)
- ESP32-S2: DAC1 = GPIO 17 (Channel 1), DAC2 = GPIO 18 (Channel 2)
"""

from machine import DAC, Pin, mem32

# Hardware register addresses for DAC cosine wave generator
_DAC_CTRL1_REG = 0x3FF48898  # Frequency and enable control
_DAC_CTRL2_REG = 0x3FF4889C  # Channel selection, scale, phase, offset

# Enable debug tracing
_SILENT = True

class DACManager:
    """
    Singleton manager for DAC objects to avoid ESP_ERR_INVALID_STATE.

    The ESP32 DAC driver doesn't allow creating multiple DAC objects on the same pin.
    This manager ensures only one DAC object per pin is ever created and reuses them.
    """

    _instance = None
    _dac_objects = {}  # pin_num -> DAC object

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def get_dac(self, pin_num):
        """
        Get or reuse DAC object for the specified pin.

        Args:
            pin_num: GPIO pin number (25/26 for ESP32, 17/18 for ESP32-S2)

        Returns:
            DAC object for the pin

        Raises:
            RuntimeError: If DAC creation fails
        """
        if pin_num not in self._dac_objects:
            try:
                _SILENT or print(f"Creating new DAC object for GPIO {pin_num}")
                self._dac_objects[pin_num] = DAC(Pin(pin_num))
            except Exception as e:
                error_msg = str(e)
                if "ESP_ERR_INVALID_STATE" in error_msg or "-259" in error_msg:
                    raise RuntimeError(
                        f"DAC creation failed on GPIO {pin_num}.\n"
                        f"The DAC driver is in an invalid state.\n"
                        f"Solution: import machine; machine.reset()\n"
                        f"Original error: {e}"
                    )
                else:
                    raise RuntimeError(f"Failed to create DAC on GPIO {pin_num}: {e}")
        else:
            _SILENT or print(f"Reusing existing DAC object for GPIO {pin_num}")

        return self._dac_objects[pin_num]

    def release_dac(self, pin_num):
        """
        Release DAC for reuse (sets to safe DC value but keeps object).

        Args:
            pin_num: GPIO pin number
        """
        if pin_num in self._dac_objects:
            # Don't delete - ESP32 doesn't like recreating DAC objects
            # Just set to a safe DC value and keep the object for reuse
            try:
                self._dac_objects[pin_num].write(127)  # Center value
                _SILENT or print(f"Released DAC on GPIO {pin_num} (kept for reuse)")
            except Exception:
                _SILENT or print(f"Warning: Could not set DC value for GPIO {pin_num}")
        else:
            _SILENT or print(f"No DAC object found for GPIO {pin_num} to release")
    def cleanup_all(self):
        """Clean up all DAC objects (sets them to DC center value)"""
        for pin_num in self._dac_objects:
            self.release_dac(pin_num)
        _SILENT or print("All DAC objects set to safe DC values")


def _reg_get(addr):
    """Read hardware register"""
    return mem32[addr]


def _reg_set(addr, value, mask):
    """Set specific bits in hardware register"""
    mem32[addr] = mem32[addr] & ~mask | value & mask


def _channel_from_pin(pin_num):
    """Convert GPIO pin number to DAC channel number"""
    # ESP32: GPIO 25 = DAC1 (Channel 1), GPIO 26 = DAC2 (Channel 2)
    # ESP32-S2: GPIO 17 = DAC1 (Channel 1), GPIO 18 = DAC2 (Channel 2)
    if pin_num == 25 or pin_num == 17:  # DAC1
        return 1
    elif pin_num == 26 or pin_num == 18:  # DAC2
        return 2
    else:
        raise ValueError(
            f"Invalid DAC pin {pin_num}. Use 25/26 (ESP32) or 17/18 (ESP32-S2)"
        )

class CosineDAC:
    """
    Hardware-based cosine wave generator using ESP32's built-in cosine wave generator.

    Uses a singleton DAC manager to avoid ESP_ERR_INVALID_STATE errors from
    creating multiple DAC objects on the same pin.
    """

    def __init__(self, pin_num=25, freq=440, scale=0, offset=127):
        """
        Initialize hardware cosine wave generator

        Args:
            pin_num: GPIO pin number (25/26 for ESP32, 17/18 for ESP32-S2)
            freq: Frequency in Hz
            scale: Amplitude scale (0=full, 1=1/2, 2=1/4, 3=1/8)
            offset: DC offset (0-255)
        """
        self.pin_num = pin_num
        self.channel = _channel_from_pin(pin_num)
        self.freq = freq
        self.scale = scale
        self.offset = offset
        self.running = False

        # Get DAC object from singleton manager
        self.dac_manager = DACManager()
        self.dac = self.dac_manager.get_dac(pin_num)

        # Initialize with DC offset
        self.dac.write(offset)

        print(
            f"Hardware sine DAC initialized on GPIO {pin_num} (Channel {self.channel})"
        )

    def _set_frequency(self, freq):
        """Set cosine wave frequency for this channel"""
        if self.channel < 1 or self.channel > 2:
            raise ValueError("Channel must be 1 or 2")

        # Calculate frequency step value
        # freq = 8MHz * step / 65536
        # step = freq * 65536 / 8000000
        step = int(freq * 65536 / 8000000) & 0xFFFF
        _reg_set(_DAC_CTRL1_REG, step, 0xFFFF)
        _SILENT or print(f"Hardware sine frequency set to {freq} Hz (step: {step})")

    def _set_phase(self, phase):
        """Set phase for this DAC channel"""
        if self.channel < 1 or self.channel > 2:
            raise ValueError("Channel must be 1 or 2")

        if phase not in [2, 3]:
            raise ValueError("Phase must be 2 (0 degrees) or 3 (180 degrees)")

        # Phase bits are at positions 20-21 for channel 1, 22-23 for channel 2
        bit_position = 20 + 2 * (self.channel - 1)
        mask = 0x03 << bit_position
        value = phase << bit_position

        _reg_set(_DAC_CTRL2_REG, value, mask)
        phase_degrees = 0 if phase == 2 else 180
        _SILENT or print(f"Channel {self.channel} phase set to {phase_degrees} degrees")

    def _set_scale(self, scale):
        """Set amplitude scale for this DAC channel"""
        if self.channel < 1 or self.channel > 2:
            raise ValueError("Channel must be 1 or 2")

        if scale < 0 or scale > 3:
            raise ValueError("Scale must be 0-3")

        # Scale bits are at positions 16-17 for channel 1, 18-19 for channel 2
        bit_position = 16 + 2 * (self.channel - 1)
        mask = 0x03 << bit_position
        value = scale << bit_position

        _reg_set(_DAC_CTRL2_REG, value, mask)

        scale_text = ["no scaling", "1/2", "1/4", "1/8"][scale]
        _SILENT or print(f"Channel {self.channel} scale set to {scale_text}")

    def _set_offset(self, offset):
        """Set DC offset for this DAC channel"""
        if self.channel < 1 or self.channel > 2:
            raise ValueError("Channel must be 1 or 2")

        offset = max(0, min(255, offset))

        # Offset bits are at positions 0-7 for channel 1, 8-15 for channel 2
        bit_position = 8 * (self.channel - 1)
        mask = 0xFF << bit_position
        value = offset << bit_position

        _reg_set(_DAC_CTRL2_REG, value, mask)
        _SILENT or print(f"Channel {self.channel} offset set to {offset}")

    def _enable_sine(self, freq):
        """Enable hardware cosine wave generation for this channel"""
        if self.channel < 1 or self.channel > 2:
            raise ValueError("Channel must be 1 or 2")

        # Set frequency
        self._set_frequency(freq)

        # Set default phase (0 degrees)
        self._set_phase(2)

        # Enable tone generator
        _reg_set(_DAC_CTRL1_REG, 0x10000, 0x10000)

        # Select channel - bit 24 for channel 1, bit 25 for channel 2
        if self.channel == 1:
            _reg_set(_DAC_CTRL2_REG, 1 << 24, 1 << 24)
        else:
            _reg_set(_DAC_CTRL2_REG, 1 << 25, 1 << 25)

        _SILENT or print(f"Hardware cosine wave enabled on channel {self.channel} at {freq} Hz")

    def _disable_sine(self):
        """Disable hardware cosine wave generation"""
        # Disable tone generator
        _reg_set(_DAC_CTRL1_REG, 0, 0x10000)
        _SILENT or print("Hardware cosine wave disabled")

    def start(self):
        """Start the hardware cosine wave generation"""
        if self.running:
            _SILENT or print("Hardware cosine wave already running")
            return

        # Configure cosine wave parameters
        self._set_scale(self.scale)
        self._set_offset(self.offset)

        # Enable the sine wave
        self._enable_sine(self.freq)

        self.running = True
        _SILENT or print(f"Hardware cosine wave started: {self.freq} Hz on GPIO {self.pin_num}")

    def stop(self):
        """Stop the hardware cosine wave generation"""
        if not self.running:
            return

        self.running = False

        # Disable cosine wave generation
        self._disable_sine()

        # Set DAC to DC offset value and release for reuse
        self.dac_manager.release_dac(self.pin_num)

        _SILENT or print(f"Hardware cosine wave stopped on GPIO {self.pin_num}")

    def set_frequency(self, freq):
        """Update the cosine wave frequency"""
        self.freq = freq
        if self.running:
            self._set_frequency(freq)
        _SILENT or print(f"Hardware sine frequency updated to {freq} Hz")

    def set_amplitude_scale(self, scale):
        """
        Set amplitude scaling

        Args:
            scale: Scale factor (0=full amplitude, 1=1/2, 2=1/4, 3=1/8)
        """
        self.scale = scale
        if self.running:
            self._set_scale(scale)

        scale_text = ["full", "1/2", "1/4", "1/8"][scale]
        _SILENT or print(f"Hardware sine amplitude set to {scale_text}")

    def set_dc_offset(self, offset):
        """
        Set DC offset

        Args:
            offset: DC offset value (0-255)
        """
        self.offset = offset
        if self.running:
            self._set_offset(offset)
        _SILENT or print(f"Hardware sine DC offset set to {offset}")

    def set_phase(self, phase_degrees):
        """
        Set phase shift

        Args:
            phase_degrees: Phase in degrees (0 or 180 only)
        """
        if phase_degrees == 0:
            phase_value = 2
        elif phase_degrees == 180:
            phase_value = 3
        else:
            raise ValueError("Phase must be 0 or 180 degrees")

        if self.running:
            self._set_phase(phase_value)
        _SILENT or print(f"Hardware sine phase set to {phase_degrees} degrees")


