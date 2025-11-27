# ESP32 Hardware Cosine Wave Generator

This module provides access to the ESP32's built-in hardware cosine wave generator for the DAC, producing much cleaner audio than software-based approaches with reduced CPU load, allowing it to be used to generate tones with asyncio without distortions.


Based on the implementation from [ESP32 in MicroPython - Using Hardware Registers](https://www.i-programmer.info/programming/148-hardware/16391-esp32-in-micropython-using-hardware-registers.html).

## Hardware Supported

### ✅ Chips with DAC Hardware

Only the following ESP32 chips have built-in DAC hardware and support this module:

#### ESP32 (Original)
- **DAC1**: GPIO 25 (Channel 1)
- **DAC2**: GPIO 26 (Channel 2)
- **Clock Source**: RTC_FAST (for cosine generator)
- **Cosine Frequency Range**: ~130 Hz to several MHz (distortion above ~200 KHz)

#### ESP32-S2
- **DAC1**: GPIO 17 (Channel 1)
- **DAC2**: GPIO 18 (Channel 2)
- **Clock Source**: RTC_FAST (for cosine generator)
- **Cosine Frequency Range**: ~130 Hz to several MHz (distortion above ~200 KHz)

> **Note**: This module uses the hardware **cosine wave generator**, which is driven by the RTC_FAST clock. The I2S0 (ESP32) and SPI3 (ESP32-S2) DMA controllers are only used for continuous/streaming DAC modes, not for cosine generation.

### ❌ Chips WITHOUT DAC Hardware

The following ESP32 variants **do not have DAC hardware** and are **NOT supported** by this module:

- **ESP32-C2**: No DAC (RISC-V, Wi-Fi + BLE)
- **ESP32-C3**: No DAC (RISC-V, Wi-Fi + BLE)
- **ESP32-C5**: No DAC (RISC-V, dual-band Wi-Fi 6 + BLE)
- **ESP32-C6**: No DAC (RISC-V, Wi-Fi 6 + BLE + 802.15.4)
- **ESP32-C61**: No DAC (RISC-V, Wi-Fi 6 + BLE)
- **ESP32-S3**: No DAC (Xtensa dual-core, Wi-Fi + BLE with AI acceleration)
- **ESP32-H2**: No DAC (RISC-V, BLE + 802.15.4)
- **ESP32-P4**: No DAC (RISC-V dual-core, high-performance MCU)
- **ESP8266**: No DAC (legacy 2.4 GHz Wi-Fi SoC)

> **Note**: For chips without DAC, consider using:
> - **PWM + Low-pass filter** for analog output
> - **Sigma-Delta Modulation (SDM)** peripheral
> - **LED Control (LEDC)** with hardware PWM filtering
> - **External DAC modules** (e.g., I2S DAC, MCP4725)

### Hardware Features
- Single hardware cosine wave generator shared by both DACs
- Each DAC can have independent scale, offset, and phase configuration
- 8-bit DAC resolution (0-255)
- Hardware cosine wave generator with minimal CPU overhead
- Frequency range: 130 Hz to ~200 KHz recommended (higher frequencies cause distortion)
- Much higher quality audio than software generation
- Significantly reduced CPU load compared to software DAC

## How to Use

### Basic Usage

```python
from hardware_sine import CosineDAC

# Create a cosine wave generator on GPIO 25 (ESP32) or GPIO 17 (ESP32-S2)
dac = CosineDAC(pin_num=25, freq=440, scale=0, offset=127)

# Start generating the cosine wave
dac.start()

# Update parameters while running
dac.set_frequency(880)  # Change frequency to 880 Hz
dac.set_amplitude_scale(1)  # Reduce amplitude to 1/2
dac.set_dc_offset(128)  # Adjust DC offset

# Stop the cosine wave
dac.stop()
```

### Advanced Configuration

```python
# Initialize with custom parameters
dac = CosineDAC(
    pin_num=26,      # Use DAC2 on ESP32
    freq=1000,       # 1 kHz frequency
    scale=2,         # 1/4 amplitude scaling
    offset=100       # DC offset at 100
)

dac.start()

# Set phase shift (0 or 180 degrees only)
dac.set_phase(180)

# Amplitude scaling options:
# - scale=0: Full amplitude
# - scale=1: 1/2 amplitude
# - scale=2: 1/4 amplitude
# - scale=3: 1/8 amplitude
dac.set_amplitude_scale(0)

dac.stop()
```

### Using Multiple DACs

```python
from hardware_sine import CosineDAC

# Both DACs share the same hardware cosine generator
# but can have independent scale, offset, and phase

# DAC1 on GPIO 25
dac1 = CosineDAC(pin_num=25, freq=440, scale=0, offset=127)
dac1.start()

# DAC2 on GPIO 26 with 180-degree phase shift
dac2 = CosineDAC(pin_num=26, freq=440, scale=0, offset=127)
dac2.start()
dac2.set_phase(180)

# Both generate the same frequency but with different phase
# You can also set different scale and offset for each

dac1.stop()
dac2.stop()
```

### Safe DAC Management

The module includes a `DACManager` singleton that prevents `ESP_ERR_INVALID_STATE` errors by reusing DAC objects:

```python
# The module automatically handles DAC object reuse
# You don't need to manually manage this

# If you encounter DAC errors, you can manually reset:
from machine import reset
reset()
```

## Limitations

### Hardware Limitations

1. **Shared Frequency**: Both DAC channels share the same hardware cosine wave generator, meaning they **must operate at the same frequency**. You cannot generate different frequencies on DAC1 and DAC2 simultaneously.

2. **Phase Options**: Phase shift is limited to **0 or 180 degrees only**. No arbitrary phase angles are supported.

3. **Waveform Type**: Only **cosine waves** are supported by the hardware. No sine, square, triangle, or other waveforms.

4. **Amplitude Control**: Amplitude scaling is limited to discrete values:
   - `scale=0`: Full amplitude (no scaling)
   - `scale=1`: 1/2 amplitude
   - `scale=2`: 1/4 amplitude
   - `scale=3`: 1/8 amplitude

5. **DAC Resolution**: 8-bit DAC (0-255 range)

### Software Limitations

1. **DAC Object Reuse**: The ESP32 DAC driver doesn't allow creating multiple DAC objects on the same pin. The module works around this with a singleton `DACManager`, but if errors occur, a hard reset may be required:
   ```python
   from machine import reset
   reset()
   ```

2. **Pin Constraints**: Only specific GPIO pins support DAC functionality:
   - ESP32: GPIO 25 and 26 only
   - ESP32-S2: GPIO 17 and 18 only

3. **Limited Chip Support**: Only **ESP32 (original)** and **ESP32-S2** have DAC hardware. See the "Hardware Supported" section above for the complete list of unsupported chips and alternatives.

### Usage Limitations

1. **Frequency Range**: Practical frequency range depends on the hardware clock and step calculation. Very low or very high frequencies may not be accurate.

2. **No Arbitrary Waveforms**: For complex waveforms or arbitrary wave generation, software-based DAC control is required.

3. **No Dynamic Frequency per Channel**: Since both channels share the hardware generator, calling `set_frequency()` on either DAC will affect both.

## API Reference

### `CosineDAC` Class

#### Constructor
```python
CosineDAC(pin_num=25, freq=440, scale=0, offset=127)
```
- `pin_num`: GPIO pin (25/26 for ESP32, 17/18 for ESP32-S2)
- `freq`: Frequency in Hz
- `scale`: Amplitude scale (0-3)
- `offset`: DC offset (0-255)

#### Methods
- `start()`: Start cosine wave generation
- `stop()`: Stop cosine wave generation
- `set_frequency(freq)`: Update frequency (affects both DACs)
- `set_amplitude_scale(scale)`: Set amplitude scaling (0-3)
- `set_dc_offset(offset)`: Set DC offset (0-255)
- `set_phase(phase_degrees)`: Set phase shift (0 or 180 only)

### `DACManager` Class

Singleton manager for safe DAC object handling (used internally).

## Troubleshooting

### ESP_ERR_INVALID_STATE Error

If you encounter this error, the DAC driver is in an invalid state. Solution:
```python
from machine import reset
reset()
```

### No Output

1. Check that you're using the correct GPIO pins for your ESP32 variant
2. Verify the DAC is started: `dac.start()`
3. Check DC offset and amplitude scale settings
4. Use an oscilloscope or speaker to verify output

### Distorted Output

1. Adjust the amplitude scale to prevent clipping
2. Check DC offset value (127 is center for ±127 range)
3. Ensure frequency is within reasonable range for your application

## License

Based on public domain example from i-programmer.info.
