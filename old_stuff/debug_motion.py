# Debug script for motion detection issues
# Run this with: mpremote mount ./src exec "exec(open('debug_motion.py').read())"

import time

import machine

from mpu6050 import MPU6050

# Same config as main.py
I2C_SCL_PIN = 6
I2C_SDA_PIN = 7
MPU_ADDR = 0x68
MOTION_GPIO = 4
INT_ACTIVE_HIGH = True

def debug_motion_detection():
    print("=== MOTION DETECTION DEBUG ===")
    
    # Setup I2C
    i2c = machine.I2C(0, 
                     scl=machine.Pin(I2C_SCL_PIN, machine.Pin.PULL_UP),
                     sda=machine.Pin(I2C_SDA_PIN, machine.Pin.PULL_UP),
                     freq=400000)
    
    print("I2C devices found:", i2c.scan())
    
    # Setup MPU6050
    mpu = MPU6050(i2c, address=MPU_ADDR)
    
    # Setup GPIO pin
    pull = machine.Pin.PULL_DOWN if INT_ACTIVE_HIGH else machine.Pin.PULL_UP
    motion_pin = machine.Pin(MOTION_GPIO, machine.Pin.IN, pull)
    
    print("INT pin configured as: GPIO{}, {}".format(MOTION_GPIO, 'PULL_DOWN' if INT_ACTIVE_HIGH else 'PULL_UP'))
    print("INT expected active state:", 'HIGH' if INT_ACTIVE_HIGH else 'LOW')
    
    # Test 1: Basic sensor readings
    print("\\n=== TEST 1: Basic sensor functionality ===")
    try:
        sensors = mpu.read_sensors_scaled()
        print("Sensor readings OK:")
        print("  Accel: X={:6.2f}g Y={:6.2f}g Z={:6.2f}g".format(sensors.AccX, sensors.AccY, sensors.AccZ))
        print("  Gyro:  X={:6.2f}° Y={:6.2f}° Z={:6.2f}°".format(sensors.GyroX, sensors.GyroY, sensors.GyroZ))
    except Exception as e:
        print("ERROR reading sensors:", e)
        return
    
    # Test 2: Check current registers before setup
    print("\\n=== TEST 2: Register values before setup ===")
    try:
        mot_thr = mpu.read_byte(0x1F)  # MOT_THR
        mot_dur = mpu.read_byte(0x20)  # MOT_DUR  
        int_enable = mpu.read_byte(0x38)  # INT_ENABLE
        int_cfg = mpu.read_byte(0x37)     # INT_PIN_CFG
        int_status = mpu.read_byte(0x3A)  # INT_STATUS
        
        print("  Motion threshold (0x1F):", mot_thr)
        print("  Motion duration (0x20): ", mot_dur)
        print("  Interrupt enable (0x38): 0x{:02X}".format(int_enable))
        print("  Interrupt config (0x37): 0x{:02X}".format(int_cfg))
        print("  Interrupt status (0x3A): 0x{:02X}".format(int_status))
        print("  Initial INT pin state:", motion_pin.value())
        
    except Exception as e:
        print("ERROR reading registers:", e)
        return
    
    # Test 3: Setup motion detection with very sensitive settings
    print("\\n=== TEST 3: Setting up motion detection (very sensitive) ===")
    try:
        # Use very sensitive settings for testing
        mpu.set_motion_detection_threshold(1)  # Very sensitive
        mpu.set_motion_detection_duration(5)   # Short duration 
        mpu.configure_interrupt_pin(active_high=INT_ACTIVE_HIGH, latch=True)
        mpu.enable_motion_interrupt()
        mpu.clear_motion_interrupt()
        
        print("Motion detection configured with sensitive settings:")
        print("  Threshold: 1 (very sensitive)")
        print("  Duration: 5ms (short)")
        if INT_ACTIVE_HIGH:
            print("  Interrupt: latched, active high")
        else:
            print("  Interrupt: latched, active low")
        
    except Exception as e:
        print("ERROR setting up motion detection:", e)
        return
    
    # Test 4: Check registers after setup
    print("\\n=== TEST 4: Register values after setup ===")
    try:
        mot_thr = mpu.read_byte(0x1F)  # MOT_THR
        mot_dur = mpu.read_byte(0x20)  # MOT_DUR  
        int_enable = mpu.read_byte(0x38)  # INT_ENABLE
        int_cfg = mpu.read_byte(0x37)     # INT_PIN_CFG
        int_status = mpu.read_byte(0x3A)  # INT_STATUS
        
        print("  Motion threshold (0x1F):", mot_thr)
        print("  Motion duration (0x20): ", mot_dur)
        print("  Interrupt enable (0x38): 0x{:02X} (bit 6 should be set)".format(int_enable))
        print("  Interrupt config (0x37): 0x{:02X}".format(int_cfg))
        print("  Interrupt status (0x3A): 0x{:02X}".format(int_status))
        print("  INT pin state after setup:", motion_pin.value())
        
        # Check if motion interrupt is properly enabled
        motion_int_enabled = bool(int_enable & (1 << 6))
        print("  Motion interrupt enabled:", motion_int_enabled)
        
    except Exception as e:
        print("ERROR reading registers after setup:", e)
        return
    
    # Test 5: Monitor for a few seconds with debug output
    print("\\n=== TEST 5: Live monitoring (10 seconds) ===")
    print("Try gently shaking or moving the device...")
    
    start_time = time.ticks_ms()
    last_values = None
    motion_count = 0
    
    for i in range(100):  # 10 seconds at 100ms intervals
        try:
            # Read current values
            sensors = mpu.read_sensors_scaled()
            pin_state = motion_pin.value()
            int_status = mpu.read_byte(0x3A)
            
            # Calculate magnitude of acceleration change
            if last_values:
                dx = abs(sensors.AccX - last_values.AccX)
                dy = abs(sensors.AccY - last_values.AccY) 
                dz = abs(sensors.AccZ - last_values.AccZ)
                magnitude = (dx*dx + dy*dy + dz*dz) ** 0.5
                
                # Check for motion
                motion_detected = bool(int_status & (1 << 6))
                pin_active = pin_state == (1 if INT_ACTIVE_HIGH else 0)
                
                if motion_detected or pin_active or magnitude > 0.1:
                    motion_count += 1
                    elapsed = (time.ticks_ms() - start_time) // 1000
                    print("  [{:2d}s] Motion! Pin={}, Status=0x{:02X}, Mag={:.3f}".format(elapsed, pin_state, int_status, magnitude))
                    print("         Accel: X={:6.2f} Y={:6.2f} Z={:6.2f}".format(sensors.AccX, sensors.AccY, sensors.AccZ))
                    
                    # Clear interrupt
                    mpu.clear_motion_interrupt()
                    
            last_values = sensors
            time.sleep_ms(100)
            
        except Exception as e:
            print("ERROR in monitoring loop:", e)
            break
    
    print("\\n=== RESULTS ===")
    print("Motion events detected:", motion_count)
    print("Final INT pin state:", motion_pin.value())
    
    if motion_count == 0:
        print("\\nTROUBLESHOOTING SUGGESTIONS:")
        print("1. Check INT pin wiring - should be connected to GPIO4")
        print("2. Try even lower threshold (try threshold=0)")  
        print("3. Check if accelerometer readings change when moving")
        print("4. Verify pull-up/pull-down resistor configuration")
        print("5. Check if interrupt pin configuration matches expected polarity")

# Run the debug function
debug_motion_detection()