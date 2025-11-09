# Simple interrupt-based motion detection test
# Run with: mpremote mount ./src exec "exec(open('interrupt_test.py').read())"

import time

import esp
import esp32
import machine

from mpu6050 import MPU6050

# Configuration
I2C_SCL_PIN = 6
I2C_SDA_PIN = 7  
MOTION_GPIO = 4
MPU_ADDR = 0x68

# Global interrupt counter
interrupt_count = 0

def motion_interrupt_handler(pin):
    """Simple interrupt handler"""
    global interrupt_count
    interrupt_count += 1
    print("*** INTERRUPT #{} ***".format(interrupt_count))

print("=== SIMPLE INTERRUPT TEST ===")

# Setup I2C and MPU6050
i2c = machine.I2C(0, scl=machine.Pin(I2C_SCL_PIN), sda=machine.Pin(I2C_SDA_PIN), freq=400000)
print("I2C devices:", i2c.scan())

mpu = MPU6050(i2c, address=MPU_ADDR)

# Setup GPIO with interrupt
motion_pin = machine.Pin(MOTION_GPIO, machine.Pin.IN, machine.Pin.PULL_DOWN)

# Configure motion detection - reasonable sensitivity
print("Configuring motion detection...")
mpu.set_motion_detection_threshold(4)    # Reasonable sensitivity
mpu.set_motion_detection_duration(2)    # Reasonable duration
mpu.configure_interrupt_pin(active_high=True, latch=True, open_drain=False)
mpu.enable_motion_interrupt()
mpu.clear_motion_interrupt()

print("Motion detection configured:")
print("  Threshold: 6")
print("  Duration: 15ms")
print("  Initial pin state:", motion_pin.value())

# Setup interrupt on rising edge
motion_pin.irq(trigger=machine.Pin.IRQ_RISING, handler=motion_interrupt_handler)
print("Interrupt handler attached to GPIO4 rising edge")

print("\\nMove the device now! Monitoring for 20 seconds...")
print("Expected: Pin goes HIGH -> interrupt fires -> message appears\\n")

# Monitor for 20 seconds
start = time.ticks_ms()
last_count = 0

while time.ticks_diff(time.ticks_ms(), start) < 20000:
    current_time = time.ticks_ms()
    elapsed = time.ticks_diff(current_time, start) // 1000
    
    # Check if interrupt count changed
    if interrupt_count != last_count:
        print("[{:2d}s] Interrupt count: {} (pin state: {})".format(elapsed, interrupt_count, motion_pin.value()))
        
        # Clear MPU interrupt to allow next detection
        int_status = mpu.read_interrupt_status()
        print("      MPU status: 0x{:02X}".format(int_status))
        
        # Read acceleration for debugging
        sensors = mpu.read_sensors_scaled()
        print("      Accel: {:.2f}, {:.2f}, {:.2f}\\n".format(sensors.AccX, sensors.AccY, sensors.AccZ))
        
        last_count = interrupt_count
    
    # Show periodic status
    if elapsed % 5 == 0 and elapsed != 0:
        pin_state = motion_pin.value()
        mpu_status = mpu.read_byte(0x3A)
        print("[{:2d}s] Status check - Pin: {}, MPU: 0x{:02X}, Interrupts: {}".format(
            elapsed, pin_state, mpu_status, interrupt_count))
    
    # esp32.gpio_deep_sleep_hold(True)
    machine.deepsleep(1000)
    # time.sleep_ms(100)  # Check every second

print("\\nTest complete!")
print("Total interrupts received:", interrupt_count)

if interrupt_count > 0:
    print("SUCCESS: Interrupt-based motion detection working!")
else:
    print("No interrupts detected. Possible issues:")
    print("- Device not moved enough")
    print("- INT pin not connected to GPIO4") 
    print("- Sensitivity too low")
    print("- Interrupt configuration problem")

# Disable interrupt
motion_pin.irq(handler=None)
print("Interrupt disabled.")