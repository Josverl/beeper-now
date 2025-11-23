"""
ESP32 DAC Cosine Wave Generator

This module provides functions to generate cosine waves using the ESP32's DAC.
While the ESP-IDF supports hardware cosine wave generation, MicroPython currently
only exposes the basic DAC.write() functionality. This implementation provides
software-generated cosine waves.

GPIO Pins:
- ESP32: DAC1 = GPIO 25, DAC2 = GPIO 26
- ESP32-S2: DAC1 = GPIO 17, DAC2 = GPIO 18

References:
- https://docs.micropython.org/en/latest/esp32/quickref.html#dac-digital-to-analog-conversion
- https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/dac.html
"""

import asyncio
import math
import time
from array import array

from machine import DAC, Pin

# Global DAC instances to prevent conflicts
_active_dacs = {}
_raw_dac_instances = {}  # Track raw DAC objects


def cleanup_dac(pin_num=None):
    """
    Clean up DAC instances to prevent ESP_ERR_INVALID_STATE errors

    Args:
        pin_num: Specific pin to cleanup, or None for all pins
    """
    global _active_dacs, _raw_dac_instances

    if pin_num is None:
        # Cleanup all DACs
        for pin, dac_obj in _active_dacs.items():
            try:
                dac_obj.stop()
            except:
                pass
        _active_dacs.clear()
        _raw_dac_instances.clear()
        print("All DAC instances cleaned up")
    else:
        # Cleanup specific DAC
        if pin_num in _active_dacs:
            try:
                _active_dacs[pin_num].stop()
            except:
                pass
            del _active_dacs[pin_num]
        if pin_num in _raw_dac_instances:
            del _raw_dac_instances[pin_num]
        print(f"DAC on GPIO {pin_num} cleaned up")


def reset_dac_system():
    """Reset the entire DAC system - use this if you get initialization errors"""
    cleanup_dac()

    # Try to force garbage collection to clean up any remaining references
    import gc

    gc.collect()

    # Try to force DAC driver reset by accessing esp32 module
    try:
        import esp32

        # Trigger garbage collection again after esp32 import
        gc.collect()
        print("Attempted low-level DAC driver reset")
    except:
        pass

    print("DAC system reset completed.")


def force_dac_reset():
    """
    Force a complete DAC reset by attempting to recreate the DAC driver state
    This is a more aggressive reset than reset_dac_system()
    """
    global _raw_dac_instances, _active_dacs

    print(">> Forcing complete DAC reset...")

    # Clear all our tracking
    _raw_dac_instances.clear()
    _active_dacs.clear()

    # Force multiple garbage collections
    import gc

    for i in range(3):
        gc.collect()

    # Try to reset ESP32 DAC peripheral state
    try:
        # Import machine to ensure all drivers are loaded
        import machine

        # Force another garbage collection
        gc.collect()

        print("SUCCESS: DAC reset completed - try creating your DAC instance now")
        return True

    except Exception as e:
        print(f"FAILED: Force reset failed: {e}")
        print("NOTE: A machine.reset() is required to fully reset the DAC driver")
        return False


def get_or_create_dac(pin_num):
    """
    Get existing DAC or create new one with proper state management
    """
    global _raw_dac_instances

    # Try to reuse existing DAC if available
    if pin_num in _raw_dac_instances:
        try:
            # Test if the existing DAC still works
            _raw_dac_instances[pin_num].write(127)
            print(f"Reusing existing DAC on GPIO {pin_num}")
            return _raw_dac_instances[pin_num]
        except:
            # DAC is broken, remove it
            del _raw_dac_instances[pin_num]

    # Try to create new DAC
    try:
        dac = DAC(Pin(pin_num))
        _raw_dac_instances[pin_num] = dac
        print(f"Created new DAC on GPIO {pin_num}")
        return dac
    except Exception as e:
        error_msg = str(e)
        if "ESP_ERR_INVALID_STATE" in error_msg or "-259" in error_msg:
            # The issue is that MicroPython's DAC driver maintains internal state
            # that doesn't get cleaned up properly. We need to force a reset.
            print(f"DAC driver is in invalid state. Attempting to force reset...")

            # Try to force cleanup by importing esp32 module and using low-level functions
            try:
                # Force garbage collection
                import gc

                import esp32

                gc.collect()

                # Try again after cleanup
                dac = DAC(Pin(pin_num))
                _raw_dac_instances[pin_num] = dac
                print(f"Successfully recovered DAC on GPIO {pin_num}")
                return dac
            except:
                pass

            raise RuntimeError(
                f"DAC driver on GPIO {pin_num} is in an invalid state.\n"
                f"This is a known MicroPython limitation where the DAC driver\n"
                f"doesn't properly reset its internal state.\n"
                f"\nRequired solution: import machine; machine.reset()\n"
                f"This will restart MicroPython and clear the DAC driver state."
            )
        else:
            raise RuntimeError(f"Failed to initialize DAC on GPIO {pin_num}: {e}")


class CosineWaveDAC:
    """Software-based cosine wave generator using ESP32 DAC"""

    def __init__(self, pin_num=25, sample_rate=8000, wave_freq=440):
        """
        Initialize the cosine wave generator

        Args:
            pin_num: GPIO pin number (25 or 26 for ESP32, 17 or 18 for ESP32-S2)
            sample_rate: Samples per second (Hz)
            wave_freq: Frequency of the cosine wave (Hz)
        """
        global _active_dacs

        self.pin_num = pin_num
        self.sample_rate = sample_rate
        self.wave_freq = wave_freq
        self.amplitude = 127  # Max amplitude for DAC (0-255 range, centered at 127)
        self.offset = 127  # DC offset to center the wave
        self.running = False

        # Check if DAC is already in use by our system
        if pin_num in _active_dacs:
            print(
                f"Warning: DAC on GPIO {pin_num} already exists. Cleaning up previous instance..."
            )
            try:
                _active_dacs[pin_num].stop()
            except:
                pass
            del _active_dacs[pin_num]

        # Initialize DAC using the robust method
        self.dac = get_or_create_dac(pin_num)
        _active_dacs[pin_num] = self  # Register this instance

        # Pre-calculate waveform samples for better performance
        self.samples_per_cycle = int(sample_rate / wave_freq)
        self.wave_table = self._generate_wave_table()
        self.sample_index = 0

    def _generate_wave_table(self):
        """Generate a lookup table of cosine wave samples"""
        # Ensure we have at least one sample per cycle
        if self.samples_per_cycle < 1:
            self.samples_per_cycle = 1
            print(
                f"Warning: Very high frequency ({self.wave_freq} Hz) - using minimum 1 sample per cycle"
            )

        # Prevent extremely large wave tables
        if self.samples_per_cycle > 1000:
            self.samples_per_cycle = 1000
            print(
                f"Warning: Very low frequency ({self.wave_freq} Hz) - limiting to 1000 samples per cycle"
            )

        samples = array("B")  # Byte array for DAC values (0-255)
        for i in range(self.samples_per_cycle):
            angle = 2 * math.pi * i / self.samples_per_cycle
            # Convert cosine (-1 to 1) to DAC range (0 to 255)
            value = int(self.offset + self.amplitude * math.cos(angle))
            value = max(0, min(255, value))  # Clamp to valid DAC range
            samples.append(value)

        if len(samples) == 0:
            # Fallback: create a single DC sample
            samples.append(self.offset)
            print("Warning: Created fallback single-sample wave table")

        print(f"Generated wave table: {len(samples)} samples for {self.wave_freq} Hz")
        return samples

    def set_frequency(self, freq):
        """Update the wave frequency and regenerate wave table"""
        self.wave_freq = freq
        self.samples_per_cycle = int(self.sample_rate / freq)
        self.wave_table = self._generate_wave_table()
        # Reset sample index to prevent out of bounds error
        self.sample_index = 0
        print(f"Wave frequency set to {freq} Hz")

    def set_amplitude(self, amplitude):
        """
        Set the wave amplitude

        Args:
            amplitude: Amplitude (0-127, will be centered around DC offset)
        """
        self.amplitude = max(0, min(127, amplitude))
        self.wave_table = self._generate_wave_table()
        # Reset sample index to prevent out of bounds error
        self.sample_index = 0
        print(f"Amplitude set to {self.amplitude}")

    def start_blocking(self, duration_ms=None):
        """
        Start generating cosine wave (blocking)

        Args:
            duration_ms: Duration in milliseconds (None for infinite)
        """
        print(f"Starting {self.wave_freq} Hz cosine wave on GPIO {self.pin_num}")
        print(f"Sample rate: {self.sample_rate} Hz")
        print("Press Ctrl+C to stop")

        # Safety check: ensure wave table exists and is valid
        if not self.wave_table or len(self.wave_table) == 0:
            print("Error: Invalid wave table, regenerating...")
            self.wave_table = self._generate_wave_table()

        # Ensure sample_index starts at 0
        self.sample_index = 0

        start_time = time.ticks_ms()
        sample_interval_us = int(1_000_000 / self.sample_rate)

        try:
            self.running = True
            while self.running:
                # Double-check bounds (should not be needed but safety first)
                if self.sample_index >= len(self.wave_table):
                    print(
                        f"Warning: sample_index {self.sample_index} >= wave_table length {len(self.wave_table)}"
                    )
                    self.sample_index = 0

                # Output current sample
                self.dac.write(self.wave_table[self.sample_index])

                # Advance to next sample
                self.sample_index = (self.sample_index + 1) % len(self.wave_table)

                # Check duration
                if (
                    duration_ms
                    and time.ticks_diff(time.ticks_ms(), start_time) >= duration_ms
                ):
                    break

                # Wait for next sample
                time.sleep_us(sample_interval_us)

        except KeyboardInterrupt:
            print("\nStopped by user")
        finally:
            self.stop()

    async def start_async(self, duration_ms=None):
        """
        Start generating cosine wave (async)

        Args:
            duration_ms: Duration in milliseconds (None for infinite)
        """
        print(f"Starting {self.wave_freq} Hz cosine wave on GPIO {self.pin_num}")
        print(f"Sample rate: {self.sample_rate} Hz")

        start_time = time.ticks_ms()
        # Use microsecond precision for better timing
        sample_interval_us = 1_000_000 // self.sample_rate
        samples_per_yield = max(1, self.sample_rate // 1000)  # Yield every ~1ms
        
        print(f"Async mode: {samples_per_yield} samples per yield, {sample_interval_us}us per sample")

        try:
            self.running = True
            sample_count = 0
            last_yield_time = time.ticks_us()
            
            while self.running:
                # Ensure sample_index is within bounds
                if self.sample_index >= len(self.wave_table):
                    self.sample_index = 0

                # Output current sample
                self.dac.write(self.wave_table[self.sample_index])

                # Advance to next sample
                self.sample_index = (self.sample_index + 1) % len(self.wave_table)
                sample_count += 1

                # Check duration
                if (
                    duration_ms
                    and time.ticks_diff(time.ticks_ms(), start_time) >= duration_ms
                ):
                    break

                # Yield control periodically, not after every sample
                if sample_count >= samples_per_yield:
                    sample_count = 0
                    
                    # Calculate how long to sleep to maintain sample rate
                    elapsed_us = time.ticks_diff(time.ticks_us(), last_yield_time)
                    expected_us = samples_per_yield * sample_interval_us
                    
                    if elapsed_us < expected_us:
                        sleep_us = expected_us - elapsed_us
                        if sleep_us >= 1000:  # Only sleep if >= 1ms
                            await asyncio.sleep_ms(sleep_us // 1000)
                    
                    last_yield_time = time.ticks_us()
                else:
                    # Tight loop for sample timing, minimal delay
                    time.sleep_us(sample_interval_us)

        except asyncio.CancelledError:
            print("Async cosine wave cancelled")
        finally:
            self.stop()

    def stop(self):
        """Stop the wave generation and set DAC to center value"""
        global _active_dacs

        self.running = False
        try:
            self.dac.write(self.offset)  # Set to DC offset (silence)
        except:
            pass

        # Remove from active DACs
        if self.pin_num in _active_dacs:
            del _active_dacs[self.pin_num]

        print("Cosine wave stopped")

    def sweep_frequency(self, start_freq, end_freq, duration_ms, steps=50):
        """
        Sweep frequency from start_freq to end_freq over duration_ms

        Args:
            start_freq: Starting frequency (Hz)
            end_freq: Ending frequency (Hz)
            duration_ms: Total sweep duration (ms)
            steps: Number of frequency steps
        """
        print(
            f"Frequency sweep: {start_freq} Hz -> {end_freq} Hz over {duration_ms} ms"
        )

        step_duration_ms = duration_ms // steps
        freq_step = (end_freq - start_freq) / steps

        try:
            self.running = True
            for i in range(steps):
                if not self.running:
                    break

                freq = start_freq + i * freq_step
                self.set_frequency(freq)

                # Safety check after frequency change
                if not self.wave_table or len(self.wave_table) == 0:
                    print(f"Warning: Invalid wave table for frequency {freq}")
                    continue

                # Generate wave at this frequency for step duration
                start_time = time.ticks_ms()
                sample_interval_us = int(1_000_000 / self.sample_rate)

                while (
                    time.ticks_diff(time.ticks_ms(), start_time) < step_duration_ms
                    and self.running
                ):
                    # Double-check bounds before accessing array
                    if self.sample_index >= len(self.wave_table):
                        self.sample_index = 0

                    self.dac.write(self.wave_table[self.sample_index])
                    self.sample_index = (self.sample_index + 1) % len(self.wave_table)
                    time.sleep_us(sample_interval_us)

        except KeyboardInterrupt:
            print("\nFrequency sweep stopped by user")
        finally:
            self.stop()
