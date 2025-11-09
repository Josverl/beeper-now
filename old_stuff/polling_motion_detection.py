# Motion detection using MPU register polling instead of GPIO interrupts
# This works around possible INT pin wiring issues by polling the MPU interrupt status register

from mpu6050 import MPU6050
import machine
import time
from config import set_color

# Configuration
I2C_SCL_PIN = 6
I2C_SDA_PIN = 7  
MPU_ADDR = 0x68
NO_MOTION_TIMEOUT_S = 30

# Motion detection settings
MOTION_THRESHOLD = 6      # Good sensitivity 
MOTION_DURATION_MS = 15   # Stability duration

def setup_motion_detection():
    """Setup MPU6050 for polling-based motion detection"""
    print("=== POLLING-BASED MOTION DETECTION ===")
    
    # Setup I2C and MPU6050
    i2c = machine.I2C(0, scl=machine.Pin(I2C_SCL_PIN), sda=machine.Pin(I2C_SDA_PIN), freq=400000)
    print("I2C devices:", i2c.scan())
    
    mpu = MPU6050(i2c, address=MPU_ADDR)
    
    print("Configuring motion detection...")
    
    # Configure motion detection (same as before)
    mpu.set_motion_detection_threshold(MOTION_THRESHOLD)
    mpu.set_motion_detection_duration(MOTION_DURATION_MS)
    mpu.enable_motion_detection_hardware()  # Critical register
    mpu.configure_interrupt_pin(active_high=True, latch=True, open_drain=False)
    mpu.enable_motion_interrupt()
    mpu.clear_motion_interrupt()
    
    # Verify configuration
    print("Configuration verified:")
    print("  Threshold:", mpu.read_byte(0x1F))
    print("  Duration: {}ms".format(mpu.read_byte(0x20)))
    print("  MOT_DETECT_CTRL: 0x{:02X}".format(mpu.read_byte(0x69)))
    print("  INT_ENABLE: 0x{:02X}".format(mpu.read_byte(0x38)))
    print("  INT_PIN_CFG: 0x{:02X}".format(mpu.read_byte(0x37)))
    
    return mpu

def test_polling_motion_detection(mpu, test_duration=15):
    """Test motion detection by polling MPU interrupt status register"""
    print("\\n=== TESTING MOTION DETECTION (POLLING) FOR {} SECONDS ===".format(test_duration))
    print("Move the device to test motion detection...")
    print("Method: Polling MPU interrupt status register every 50ms")
    print()
    
    start_time = time.ticks_ms()
    motion_count = 0
    last_report_time = 0
    
    set_color("BLUE")  # Indicate test active
    
    while time.ticks_diff(time.ticks_ms(), start_time) < test_duration * 1000:
        current_time = time.ticks_ms()
        
        # Poll MPU interrupt status register
        int_status = mpu.read_byte(0x3A)  # INT_STATUS register
        motion_detected = bool(int_status & 0x40)  # Check motion interrupt bit
        
        if motion_detected:
            motion_count += 1
            elapsed_s = time.ticks_diff(current_time, start_time) // 1000
            
            print("[{:3d}s] Motion #{} detected! (Status: 0x{:02X})".format(elapsed_s, motion_count, int_status))
            
            # Read sensor data for details
            try:
                sensors = mpu.read_sensors_scaled()
                print("      Accel: X={:6.2f}g Y={:6.2f}g Z={:6.2f}g".format(sensors.AccX, sensors.AccY, sensors.AccZ))
            except Exception:
                print("      Could not read sensor data")
            
            # Clear interrupt by reading status (already done above)
            mpu.clear_motion_interrupt()
            
            # Flash LED to indicate motion
            set_color("GREEN")
            time.sleep_ms(100)
            set_color("BLUE")
            
            # Brief pause to avoid too many rapid detections
            time.sleep_ms(200)
        
        # Report status every 5 seconds
        if time.ticks_diff(current_time, last_report_time) > 5000:
            elapsed_total = time.ticks_diff(current_time, start_time) // 1000
            remaining = test_duration - elapsed_total
            print("[{:3d}s] Status: {} motion events, {} seconds remaining".format(elapsed_total, motion_count, remaining))
            last_report_time = current_time
        
        time.sleep_ms(50)  # Poll every 50ms
    
    set_color("OFF")
    print("\\n=== TEST COMPLETE ===")
    print("Total motion events detected: {}".format(motion_count))
    
    if motion_count > 0:
        print("SUCCESS: Motion detection is working!")
        return True
    else:
        print("No motion detected. Check sensitivity settings or device movement.")
        return False

def run_continuous_monitoring(mpu):
    """Run continuous motion monitoring with sleep timeout"""
    print("\\n=== CONTINUOUS MOTION MONITORING ===")
    print("Timeout: {} seconds without motion -> would enter sleep".format(NO_MOTION_TIMEOUT_S))
    print("Press Ctrl+C to exit\\n")
    
    start_time = time.ticks_ms()
    last_activity_time = time.ticks_ms()
    last_report_time = 0
    motion_count = 0
    
    set_color("BLUE")  # Indicate monitoring active
    
    try:
        while True:
            current_time = time.ticks_ms()
            
            # Poll for motion
            int_status = mpu.read_byte(0x3A)
            motion_detected = bool(int_status & 0x40)
            
            if motion_detected:
                motion_count += 1
                elapsed_s = time.ticks_diff(current_time, start_time) // 1000
                print("[{:3d}s] Motion #{} detected!".format(elapsed_s, motion_count))
                
                # Clear interrupt and update activity time
                mpu.clear_motion_interrupt()
                last_activity_time = current_time
                
                # Flash LED
                set_color("GREEN")
                time.sleep_ms(100)
                set_color("BLUE")
                
                # Brief pause to avoid rapid detections
                time.sleep_ms(200)
            
            # Report status every 10 seconds
            if time.ticks_diff(current_time, last_report_time) > 10000:
                elapsed_total = time.ticks_diff(current_time, start_time) // 1000
                time_since_motion = time.ticks_diff(current_time, last_activity_time) // 1000
                print("[{:3d}s] Status: {} motions, {} sec since last motion".format(
                    elapsed_total, motion_count, time_since_motion))
                last_report_time = current_time
            
            # Check for no motion timeout
            if time.ticks_diff(current_time, last_activity_time) > NO_MOTION_TIMEOUT_S * 1000:
                print("\\nNo motion for {} seconds - would enter deep sleep".format(NO_MOTION_TIMEOUT_S))
                print("(Sleep disabled for testing - press Ctrl+C to exit)")
                
                # Flash red to indicate sleep condition
                for _ in range(3):
                    set_color("RED")
                    time.sleep_ms(200)
                    set_color("OFF")
                    time.sleep_ms(200)
                
                # Reset timeout for continuous monitoring
                last_activity_time = current_time
                set_color("BLUE")
            
            time.sleep_ms(50)  # Poll every 50ms
            
    except KeyboardInterrupt:
        print("\\nMonitoring interrupted by user")
    
    set_color("OFF")
    print("Total motion events detected: {}".format(motion_count))

def main():
    """Main function"""
    try:
        # Setup motion detection
        mpu = setup_motion_detection()
        
        # Run initial test
        print("Running initial motion detection test...")
        if test_polling_motion_detection(mpu, test_duration=10):
            # If test succeeds, run continuous monitoring
            run_continuous_monitoring(mpu)
        else:
            print("Initial test failed - motion detection may need adjustment")
            
    except KeyboardInterrupt:
        print("\\nProgram interrupted")
    except Exception as e:
        print("\\nError:", e)
        import sys
        sys.print_exception(e)
    finally:
        set_color("OFF")

# Run the program
if __name__ == "__main__":
    main()