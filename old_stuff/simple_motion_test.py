# Simple motion test script
# Run with: mpremote mount ./src exec "exec(open('simple_motion_test.py').read())"

import time

import machine

from mpu6050 import MPU6050

# Configuration
I2C_SCL_PIN = 6
I2C_SDA_PIN = 7  
MOTION_GPIO = 4
MPU_ADDR = 0x68

print("=== SIMPLE MOTION TEST ===")

# Setup I2C and MPU6050
i2c = machine.I2C(0, scl=machine.Pin(I2C_SCL_PIN), sda=machine.Pin(I2C_SDA_PIN), freq=400000)
print("I2C devices:", i2c.scan())

mpu = MPU6050(i2c, address=MPU_ADDR)

# Setup GPIO
motion_pin = machine.Pin(MOTION_GPIO, machine.Pin.IN, machine.Pin.PULL_DOWN)

# Configure with extremely sensitive settings
print("Configuring motion detection...")
mpu.set_motion_detection_threshold(0)    # Most sensitive (try 0)
mpu.set_motion_detection_duration(1)     # Shortest duration 
mpu.configure_interrupt_pin(active_high=True, latch=True)
mpu.enable_motion_interrupt()
mpu.clear_motion_interrupt()

# Check configuration
mot_thr = mpu.read_byte(0x1F) 
mot_dur = mpu.read_byte(0x20)
int_enable = mpu.read_byte(0x38)
int_cfg = mpu.read_byte(0x37)

print("Motion threshold:", mot_thr)
print("Motion duration:", mot_dur)  
print("Int enable: 0x{:02X}".format(int_enable))
print("Int config: 0x{:02X}".format(int_cfg))
print("Initial pin state:", motion_pin.value())

print("\\nMove the device now! Monitoring for 15 seconds...")

# Monitor for 15 seconds
start = time.ticks_ms()
count = 0

while time.ticks_diff(time.ticks_ms(), start) < 15000:
    pin_state = motion_pin.value()
    int_status = mpu.read_byte(0x3A)
    
    if pin_state or (int_status & 0x40):  # Pin high or motion bit set
        count += 1
        elapsed = time.ticks_diff(time.ticks_ms(), start) // 1000
        print("[{:2d}s] MOTION! Pin={}, Status=0x{:02X}".format(elapsed, pin_state, int_status))
        
        # Read acceleration
        sensors = mpu.read_sensors_scaled()
        print("      Accel: {:.2f}, {:.2f}, {:.2f}".format(sensors.AccX, sensors.AccY, sensors.AccZ))
        
        # Clear interrupt
        mpu.clear_motion_interrupt()
        time.sleep_ms(200)  # Debounce
    
    time.sleep_ms(50)

print("\\nTest complete. Motion events:", count)
if count == 0:
    print("No motion detected. Check wiring or try different settings")
else:
    print("Motion detection working!")