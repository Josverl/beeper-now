# Test motion detection algorithm manually
# Check if the issue is with motion detection logic vs interrupt pin

from mpu6050 import MPU6050
import machine
import time
import math

I2C_SCL_PIN = 6
I2C_SDA_PIN = 7  
MPU_ADDR = 0x68

print("=== MOTION DETECTION ALGORITHM TEST ===")

# Setup
i2c = machine.I2C(0, scl=machine.Pin(I2C_SCL_PIN), sda=machine.Pin(I2C_SDA_PIN), freq=400000)
mpu = MPU6050(i2c, address=MPU_ADDR)

print("Testing motion detection at register level...")

# Check if we need to enable accelerometer specifically
print("\\n1. Power management check:")
pwr_mgmt1 = mpu.read_byte(0x6B)  # PWR_MGMT_1
pwr_mgmt2 = mpu.read_byte(0x6C)  # PWR_MGMT_2
print("  PWR_MGMT_1: 0x{:02X}".format(pwr_mgmt1))
print("  PWR_MGMT_2: 0x{:02X}".format(pwr_mgmt2))

# Make sure accelerometer is enabled
if pwr_mgmt2 & 0x38:  # Accel standby bits
    print("  Accelerometer was in standby - enabling...")
    mpu.write_byte(0x6C, pwr_mgmt2 & ~0x38)
    pwr_mgmt2 = mpu.read_byte(0x6C)
    print("  PWR_MGMT_2 now: 0x{:02X}".format(pwr_mgmt2))

# Check accelerometer config
print("\\n2. Accelerometer configuration:")
accel_config = mpu.read_byte(0x1C)   # ACCEL_CONFIG
accel_config2 = mpu.read_byte(0x1D)  # ACCEL_CONFIG2
print("  ACCEL_CONFIG: 0x{:02X}".format(accel_config))
print("  ACCEL_CONFIG2: 0x{:02X}".format(accel_config2))

# Configure motion detection with very sensitive settings
print("\\n3. Configuring ultra-sensitive motion detection:")
mpu.write_byte(0x1F, 1)   # MOT_THR - very sensitive
mpu.write_byte(0x20, 1)   # MOT_DUR - short duration

# Check motion detection control register
mot_detect_ctrl = mpu.read_byte(0x69)
print("  MOT_DETECT_CTRL: 0x{:02X}".format(mot_detect_ctrl))

# Try setting motion detection control register
print("  Setting MOT_DETECT_CTRL to enable motion detection...")
mpu.write_byte(0x69, 0x80)  # Enable motion detection
mot_detect_ctrl = mpu.read_byte(0x69)
print("  MOT_DETECT_CTRL after: 0x{:02X}".format(mot_detect_ctrl))

# Configure interrupt
mpu.write_byte(0x37, 0x20)  # INT_PIN_CFG: latched, active high
mpu.write_byte(0x38, 0x40)  # INT_ENABLE: motion interrupt

print("\\n4. Final register verification:")
registers = [
    (0x1F, "MOT_THR"),
    (0x20, "MOT_DUR"), 
    (0x37, "INT_PIN_CFG"),
    (0x38, "INT_ENABLE"),
    (0x69, "MOT_DETECT_CTRL"),
    (0x6B, "PWR_MGMT_1"),
    (0x6C, "PWR_MGMT_2")
]

for reg, name in registers:
    value = mpu.read_byte(reg)
    print("  {}: 0x{:02X}".format(name, value))

# Clear interrupts and test
print("\\n5. Testing motion detection...")
mpu.read_byte(0x3A)  # Clear interrupt status

print("Move device now! Monitoring for 15 seconds...")

baseline = None
motion_events = 0

for i in range(150):  # 15 seconds at 100ms intervals
    # Read raw accelerometer data
    ax_raw = mpu.read_word2(0x3B)  # ACCEL_XOUT_H/L
    ay_raw = mpu.read_word2(0x3D)  # ACCEL_YOUT_H/L 
    az_raw = mpu.read_word2(0x3F)  # ACCEL_ZOUT_H/L
    
    # Convert to signed values
    if ax_raw > 32767: ax_raw -= 65536
    if ay_raw > 32767: ay_raw -= 65536  
    if az_raw > 32767: az_raw -= 65536
    
    # Calculate magnitude
    magnitude = math.sqrt(ax_raw*ax_raw + ay_raw*ay_raw + az_raw*az_raw)
    
    # Check interrupt status
    int_status = mpu.read_byte(0x3A)
    
    if baseline is None:
        baseline = magnitude
    
    # Calculate change from baseline
    change = abs(magnitude - baseline)
    
    # Check for motion interrupt
    motion_detected = bool(int_status & 0x40)
    
    if motion_detected or change > 1000:  # Arbitrary threshold for manual detection
        motion_events += 1
        elapsed = i / 10
        print("[{:4.1f}s] Motion! INT={}, Status=0x{:02X}, Mag={:.0f}, Change={:.0f}".format(
            elapsed, motion_detected, int_status, magnitude, change))
        
        if motion_detected:
            print("         >>> MPU MOTION INTERRUPT TRIGGERED! <<<")
        
        # Update baseline
        baseline = magnitude
    
    time.sleep_ms(100)

print("\\nResults:")
print("Total motion events detected: {}".format(motion_events))

# Final status check
final_status = mpu.read_byte(0x3A)
print("Final interrupt status: 0x{:02X}".format(final_status))

if motion_events > 0:
    print("Motion detection algorithm appears to work")
else:
    print("Motion detection algorithm may have issues")
    print("Possible problems:")
    print("- Motion detection control register not properly set")
    print("- Accelerometer not properly configured")
    print("- Threshold too high even at value 1")
    print("- Hardware issue with motion detection circuit")