# Debug interrupt pin configuration
# This script checks if the MPU6050 INT pin is configured correctly

import time

import machine

from mpu6050 import MPU6050

# Configuration  
I2C_SCL_PIN = 6
I2C_SDA_PIN = 7  
MOTION_GPIO = 4
MPU_ADDR = 0x68

print("=== INTERRUPT PIN CONFIGURATION DEBUG ===")

# Setup I2C and MPU6050
i2c = machine.I2C(0, scl=machine.Pin(I2C_SCL_PIN), sda=machine.Pin(I2C_SDA_PIN), freq=400000)
mpu = MPU6050(i2c, address=MPU_ADDR)
motion_pin = machine.Pin(MOTION_GPIO, machine.Pin.IN, machine.Pin.PULL_DOWN)

print("Initial register values:")
int_enable = mpu.read_byte(0x38)    # INT_ENABLE
int_cfg = mpu.read_byte(0x37)       # INT_PIN_CFG  
int_status = mpu.read_byte(0x3A)    # INT_STATUS
print("  INT_ENABLE (0x38): 0x{:02X}".format(int_enable))
print("  INT_PIN_CFG (0x37): 0x{:02X}".format(int_cfg))
print("  INT_STATUS (0x3A): 0x{:02X}".format(int_status))
print("  GPIO pin state:", motion_pin.value())

print("\\nConfiguring motion detection step by step...")

# Step 1: Set motion thresholds
print("1. Setting motion threshold and duration...")
mpu.set_motion_detection_threshold(4)
mpu.set_motion_detection_duration(10)
mot_thr = mpu.read_byte(0x1F)
mot_dur = mpu.read_byte(0x20)
print("  Motion threshold: {}".format(mot_thr))
print("  Motion duration: {}ms".format(mot_dur))

# Step 2: Configure interrupt pin - try different configurations
print("\\n2. Configuring interrupt pin...")

# Try configuration 1: Active high, latched, push-pull
print("   Trying: active_high=True, latch=True, open_drain=False")
mpu.configure_interrupt_pin(active_high=True, latch=True, open_drain=False)
int_cfg = mpu.read_byte(0x37)
print("   INT_PIN_CFG result: 0x{:02X}".format(int_cfg))

# Step 3: Enable motion interrupt
print("\\n3. Enabling motion interrupt...")
mpu.enable_motion_interrupt()
int_enable = mpu.read_byte(0x38)
print("   INT_ENABLE result: 0x{:02X} (should have bit 6 set = 0x40)".format(int_enable))

# Step 4: Clear any pending interrupts
print("\\n4. Clearing interrupts...")
int_status = mpu.clear_motion_interrupt()
print("   Cleared interrupt status: 0x{:02X}".format(int_status))

print("\\n5. Final configuration check:")
int_enable = mpu.read_byte(0x38)
int_cfg = mpu.read_byte(0x37)  
int_status = mpu.read_byte(0x3A)
print("  INT_ENABLE: 0x{:02X}".format(int_enable))
print("  INT_PIN_CFG: 0x{:02X}".format(int_cfg))
print("  INT_STATUS: 0x{:02X}".format(int_status))
print("  GPIO pin state:", motion_pin.value())

print("\\n6. Force motion by reading sensors and checking for changes...")
print("Move device now - monitoring for 10 seconds...")

for i in range(10):
    # Read sensors to potentially trigger motion
    sensors = mpu.read_sensors_scaled()
    
    # Check interrupt status
    int_status = mpu.read_byte(0x3A)
    pin_state = motion_pin.value()
    
    motion_bit = bool(int_status & 0x40)
    
    print("[{}s] Pin={}, Status=0x{:02X}, Motion bit={}, Accel: {:.2f},{:.2f},{:.2f}".format(
        i+1, pin_state, int_status, motion_bit, sensors.AccX, sensors.AccY, sensors.AccZ))
    
    if motion_bit or pin_state:
        print("   >>> MOTION DETECTED! <<<")
        # Clear interrupt
        mpu.clear_motion_interrupt()
    
    time.sleep_ms(1000)

print("\\nConfiguration analysis:")
final_int_enable = mpu.read_byte(0x38)
final_int_cfg = mpu.read_byte(0x37)

print("Final INT_ENABLE: 0b{:08b} (0x{:02X})".format(final_int_enable, final_int_enable))
print("  Bit 7 (data ready): {}".format(bool(final_int_enable & 0x80)))
print("  Bit 6 (motion): {}".format(bool(final_int_enable & 0x40)))
print("  Bit 5 (zero motion): {}".format(bool(final_int_enable & 0x20)))
print("  Bit 4 (fifo overflow): {}".format(bool(final_int_enable & 0x10)))

print("Final INT_PIN_CFG: 0b{:08b} (0x{:02X})".format(final_int_cfg, final_int_cfg))
print("  Bit 7 (active low): {}".format(bool(final_int_cfg & 0x80)))
print("  Bit 6 (open drain): {}".format(bool(final_int_cfg & 0x40)))
print("  Bit 5 (latch enable): {}".format(bool(final_int_cfg & 0x20)))
print("  Bit 4 (clear on read): {}".format(bool(final_int_cfg & 0x10)))

print("\\nExpected for active high, latched interrupt:")
print("  INT_ENABLE should be 0x40 (motion interrupt enabled)")
print("  INT_PIN_CFG should be 0x20 (latched, active high, push-pull)")

if final_int_enable == 0x40 and final_int_cfg == 0x20:
    print("\\nConfiguration looks CORRECT!")
else:
    print("\\nConfiguration may have issues!")