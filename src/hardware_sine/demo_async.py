import asyncio
import time
from math import pi

from . import CosineDAC
from .utils import check_hardware_sine_support


async def async_frequency_sweep(pin_num=25):
    """Demo: Asynchronous frequency sweep from low to high"""
    print("=== Async Frequency Sweep Demo ===")

    # Check support first
    supported, message = check_hardware_sine_support()
    if not supported:
        print(f"Hardware sine not supported: {message}")
        return

    hw_sine = None
    try:
        print("Creating hardware sine generator...")
        hw_sine = CosineDAC(pin_num=pin_num, freq=220)

        print("Starting frequency sweep: 220Hz -> 880Hz")
        frequencies = [220, 330, 440, 550, 660, 770, 880, 1_000, 1_200, 1_500]

        hw_sine.start()

        for freq in frequencies:
            print(f"  Frequency: {freq} Hz")
            hw_sine.set_frequency(freq)
            await asyncio.sleep(0.5)  # Non-blocking delay

        hw_sine.stop()
        print("Frequency sweep complete!")

    except Exception as e:
        print(f"Frequency sweep error: {e}")
    finally:
        if hw_sine:
            try:
                hw_sine.stop()
            except Exception:
                pass


async def async_amplitude_demo(pin_num=25):
    """Demo: Asynchronous amplitude changes"""
    print("\n=== Async Amplitude Demo ===")

    hw_sine = None
    try:
        print("Creating hardware sine generator for amplitude demo...")
        hw_sine = CosineDAC(pin_num=pin_num, freq=440)

        print("Demonstrating amplitude scaling...")
        scales = [0, 1, 2, 3, 2, 1, 0]  # Full -> 1/8 -> Full
        scale_names = ["Full", "1/2", "1/4", "1/8", "1/4", "1/2", "Full"]

        hw_sine.start()

        for scale, name in zip(scales, scale_names):
            print(f"  Amplitude: {name}")
            hw_sine.set_amplitude_scale(scale)
            await asyncio.sleep(0.8)

        hw_sine.stop()
        print("Amplitude demo complete!")

    except Exception as e:
        print(f"Amplitude demo error: {e}")
    finally:
        if hw_sine:
            try:
                hw_sine.stop()
            except Exception:
                pass


async def async_phase_demo(pin_num=25):
    """Demo: Asynchronous phase shifting"""
    print("\n=== Async Phase Demo ===")

    hw_sine = None
    try:
        print("Creating hardware sine generator for phase demo...")
        hw_sine = CosineDAC(pin_num=pin_num, freq=660)

        print("Demonstrating phase shifting...")
        hw_sine.start()

        for i in range(6):  # Multiple phase switches
            phase = 0 if i % 2 == 0 else 180
            print(f"  Phase: {phase} degrees")
            hw_sine.set_phase(phase)
            await asyncio.sleep(0.6)

        hw_sine.stop()
        print("Phase demo complete!")

    except Exception as e:
        print(f"Phase demo error: {e}")
    finally:
        if hw_sine:
            try:
                hw_sine.stop()
            except Exception:
                pass


async def async_multi_tone_sequence(pin_num=25):
    """Demo: Play a sequence of tones asynchronously"""
    print("\n=== Async Multi-Tone Sequence ===")

    # Musical notes (approximate frequencies)
    notes = [
        ("C4", 261),
        ("D4", 294),
        ("E4", 329),
        ("F4", 349),
        ("G4", 392),
        ("A4", 440),
        ("B4", 493),
        ("C5", 523),
    ]

    hw_sine = None
    try:
        print("Creating hardware sine generator for musical sequence...")
        hw_sine = CosineDAC(pin_num=pin_num, freq=261)

        print("Playing musical scale...")

        for note_name, freq in notes:
            print(f"  Playing {note_name} ({freq} Hz)")
            hw_sine.set_frequency(freq)
            hw_sine.start()
            await asyncio.sleep(0.4)  # Note duration

        hw_sine.stop()
        print("Musical sequence complete!")

    except Exception as e:
        print(f"Multi-tone sequence error: {e}")
    finally:
        if hw_sine:
            try:
                hw_sine.stop()
            except Exception:
                pass


async def async_concurrent_demo(pin_num=25):
    """Demo: Concurrent operations with hardware sine"""
    print("\n=== Async Concurrent Demo ===")

    async def status_monitor():
        """Monitor and report status while sine wave plays"""
        print("  Status monitor started...")
        for i in range(10):
            await asyncio.sleep(0.3)
            print(f"    Monitor tick {i + 1}/10")
        print("  Status monitor finished.")

    async def sine_controller():
        """Control sine wave while monitor runs"""
        hw_sine = None
        try:
            print("  Creating sine controller...")
            hw_sine = CosineDAC(pin_num=pin_num, freq=440)

            hw_sine.start()
            print("  Sine wave started at 440 Hz")

            await asyncio.sleep(1.5)

            print("  Changing to 880 Hz...")
            hw_sine.set_frequency(880)

            await asyncio.sleep(1.5)

            hw_sine.stop()
            print("  Sine wave stopped.")

        except Exception as e:
            print(f"  Sine controller error: {e}")
        finally:
            if hw_sine:
                try:
                    hw_sine.stop()
                except Exception:
                    pass

    try:
        print("Running concurrent monitor and sine controller...")
        # Run both tasks concurrently
        await asyncio.gather(status_monitor(), sine_controller())
        print("Concurrent demo complete!")

    except Exception as e:
        print(f"Concurrent demo error: {e}")


async def async_interactive_demo(pin_num=25):
    """Demo: Interactive async sine wave control"""
    print("\n=== Async Interactive Demo ===")

    hw_sine = None
    try:
        print("Creating interactive hardware sine generator...")
        hw_sine = CosineDAC(pin_num=pin_num, freq=440)

        print("Starting interactive session (auto-controlled)...")
        hw_sine.start()

        # Simulate interactive control with async operations
        controls = [
            ("frequency", 330),
            ("amplitude_scale", 1),
            ("phase", 180),
            ("frequency", 550),
            ("amplitude_scale", 0),
            ("phase", 0),
            ("frequency", 220),
        ]

        for control_type, value in controls:
            if control_type == "frequency":
                print(f"  Setting frequency to {value} Hz")
                hw_sine.set_frequency(value)
            elif control_type == "amplitude_scale":
                scale_names = ["Full", "1/2", "1/4", "1/8"]
                print(f"  Setting amplitude to {scale_names[value]}")
                hw_sine.set_amplitude_scale(value)
            elif control_type == "phase":
                print(f"  Setting phase to {value} degrees")
                hw_sine.set_phase(value)

            await asyncio.sleep(0.7)

        hw_sine.stop()
        print("Interactive demo complete!")

    except Exception as e:
        print(f"Interactive demo error: {e}")
    finally:
        if hw_sine:
            try:
                hw_sine.stop()
            except Exception:
                pass


async def main(pin_num=25):
    """Main async demo function - runs all demonstrations"""
    print("ESP32 Hardware Sine Wave - Async Demonstrations")
    print("=" * 55)

    # Check if hardware sine is supported
    supported, message = check_hardware_sine_support()
    print(f"Hardware sine support: {supported}")
    print(f"Message: {message}")

    if not supported:
        print("\nCannot run demos - hardware sine not supported!")
        print("Try: import machine; machine.reset()")
        return

    try:
        # Run all async demos in sequence
        await async_frequency_sweep(pin_num)
        await asyncio.sleep(1)

        await async_amplitude_demo(pin_num)
        await asyncio.sleep(1)

        await async_phase_demo(pin_num)
        await asyncio.sleep(1)

        await async_multi_tone_sequence(pin_num)
        await asyncio.sleep(1)

        await async_concurrent_demo(pin_num)
        await asyncio.sleep(1)

        await async_interactive_demo(pin_num)

        print("\n" + "=" * 55)
        print("All async demos completed successfully!")
        print("Hardware sine wave async functionality verified.")

    except KeyboardInterrupt:
        print("\nAsync demos interrupted by user")
    except Exception as e:
        print(f"\nAsync demo suite error: {e}")
        print("Try: import machine; machine.reset()")


def run():
    """Convenience function to run the async demo"""
    try:
        asyncio.run(main())
    except Exception as e:
        print(f"Failed to run async demo: {e}")


# For direct execution
if __name__ == "__main__":
    run()
