# Working motion detection with proper interrupt handling
# This script properly configures MPU-6050 motion detection including the critical MOT_DETECT_CTRL register

from mpu6050 import MPU6050
import machine
import time
from config import set_color

# Configuration
I2C_SCL_PIN = 6
I2C_SDA_PIN = 7  
MOTION_GPIO = 4
BUTTON_GPIO = 5
MPU_ADDR = 0x68

# Motion detection settings
MOTION_THRESHOLD = 6      # Good sensitivity 
MOTION_DURATION_MS = 15   # Stability duration
INT_ACTIVE_HIGH = True
NO_MOTION_TIMEOUT_S = 30  # Go to sleep after this many seconds without motion

# Global variables for interrupt handling
motion_detected = False
last_motion_time = 0
motion_count = 0

def motion_interrupt_handler(pin):
    """Interrupt service routine for motion detection"""
    global motion_detected, last_motion_time, motion_count
    current_time = time.ticks_ms()
    
    # Simple debouncing - ignore if too soon after last interrupt
    if time.ticks_diff(current_time, last_motion_time) > 300:  # 300ms debounce
        motion_detected = True
        last_motion_time = current_time
        motion_count += 1

def setup_motion_detection():
    """Setup MPU6050 with proper motion detection configuration"""
    print("=== MOTION DETECTION SETUP ===")
    
    # Setup I2C and MPU6050
    i2c = machine.I2C(0, scl=machine.Pin(I2C_SCL_PIN), sda=machine.Pin(I2C_SDA_PIN), freq=400000)
    print("I2C devices:", i2c.scan())
    
    mpu = MPU6050(i2c, address=MPU_ADDR)
    
    # Setup GPIO pins
    motion_pin = machine.Pin(MOTION_GPIO, machine.Pin.IN, machine.Pin.PULL_DOWN)
    button_pin = machine.Pin(BUTTON_GPIO, machine.Pin.IN, machine.Pin.PULL_UP)
    
    print("Configuring motion detection with proper register settings...")
    
    # Configure motion detection step by step
    mpu.set_motion_detection_threshold(MOTION_THRESHOLD)
    mpu.set_motion_detection_duration(MOTION_DURATION_MS)
    
    # CRITICAL: Enable motion detection hardware (this was missing before!)
    mpu.enable_motion_detection_hardware()
    
    # Configure interrupt pin
    mpu.configure_interrupt_pin(active_high=INT_ACTIVE_HIGH, latch=True, open_drain=False)
    mpu.enable_motion_interrupt()
    mpu.clear_motion_interrupt()
    
    # Setup GPIO interrupt handler
    if INT_ACTIVE_HIGH:
        motion_pin.irq(trigger=machine.Pin.IRQ_RISING, handler=motion_interrupt_handler)
    else:
        motion_pin.irq(trigger=machine.Pin.IRQ_FALLING, handler=motion_interrupt_handler)
    
    # Verify configuration
    print("Motion detection configured:")
    print("  Threshold:", mpu.read_byte(0x1F))
    print("  Duration: {}ms".format(mpu.read_byte(0x20)))
    print("  MOT_DETECT_CTRL: 0x{:02X} (should be 0x80)".format(mpu.read_byte(0x69)))
    print("  INT_ENABLE: 0x{:02X} (should be 0x40)".format(mpu.read_byte(0x38)))
    print("  INT_PIN_CFG: 0x{:02X} (should be 0x20)".format(mpu.read_byte(0x37)))
    print("  GPIO interrupt: {} edge on GPIO{}".format('RISING' if INT_ACTIVE_HIGH else 'FALLING', MOTION_GPIO))
    
    return mpu, motion_pin, button_pin

def test_motion_interrupts(mpu, motion_pin, test_duration=20):
    """Test motion detection for a specified duration"""
    global motion_detected, motion_count
    
    print("\\n=== TESTING MOTION INTERRUPTS FOR {} SECONDS ===".format(test_duration))
    print("Move the device to test motion detection...")
    print("Expected: GPIO interrupt fires when motion detected")
    print()
    
    start_time = time.ticks_ms()
    motion_count = 0  # Reset counter
    last_report_time = 0
    
    set_color("BLUE")  # Indicate test active
    
    while time.ticks_diff(time.ticks_ms(), start_time) < test_duration * 1000:
        current_time = time.ticks_ms()
        
        # Check if interrupt detected motion
        if motion_detected:
            motion_detected = False  # Reset flag
            
            elapsed_s = time.ticks_diff(current_time, start_time) // 1000
            print("[{:3d}s] Motion #{} detected via GPIO interrupt!".format(elapsed_s, motion_count))
            
            # Read sensor data and interrupt status
            try:
                sensors = mpu.read_sensors_scaled()
                int_status = mpu.read_interrupt_status()
                print("      Accel: X={:6.2f}g Y={:6.2f}g Z={:6.2f}g".format(sensors.AccX, sensors.AccY, sensors.AccZ))
                print("      MPU interrupt status: 0x{:02X}".format(int_status))
                print("      GPIO pin state: {}".format(motion_pin.value()))
            except Exception as e:
                print("      Error reading sensor data:", e)
            
            # Clear MPU interrupt to allow next detection
            mpu.clear_motion_interrupt()
            
            # Flash LED to indicate motion
            set_color("GREEN")
            time.sleep_ms(100)
            set_color("BLUE")
        
        # Report status every 5 seconds
        if time.ticks_diff(current_time, last_report_time) > 5000:
            elapsed_total = time.ticks_diff(current_time, start_time) // 1000
            remaining = test_duration - elapsed_total
            print("[{:3d}s] Status: {} motion events, {} seconds remaining".format(elapsed_total, motion_count, remaining))
            last_report_time = current_time
        
        time.sleep_ms(50)
    
    set_color("OFF")
    print("\\n=== TEST COMPLETE ===")
    print("Total motion events detected via interrupts: {}".format(motion_count))
    
    if motion_count > 0:
        print("SUCCESS: Interrupt-based motion detection is working!")
        return True
    else:
        print("No motion detected via interrupts.")
        print("Try moving the device more vigorously.")
        return False

def run_motion_monitoring():
    """Run continuous motion monitoring with sleep timeout"""
    global motion_detected, motion_count
    
    mpu, motion_pin, button_pin = setup_motion_detection()
    
    # First run a test to verify it works
    print("Running initial test...")
    if not test_motion_interrupts(mpu, motion_pin, test_duration=10):
        print("Initial test failed - check hardware setup")
        return
    
    print("\\n=== STARTING CONTINUOUS MONITORING ===")
    print("Timeout: {} seconds without motion -> would go to sleep".format(NO_MOTION_TIMEOUT_S))
    print("Press button (GPIO{}) to exit\\n".format(BUTTON_GPIO))
    
    start_time = time.ticks_ms()
    last_activity_time = time.ticks_ms()
    last_report_time = 0
    total_motion_count = motion_count  # Continue counting from test
    
    set_color("BLUE")  # Indicate monitoring active
    
    while True:
        current_time = time.ticks_ms()
        
        # Check if interrupt detected motion
        if motion_detected:
            motion_detected = False  # Reset flag
            total_motion_count += 1
            
            elapsed_s = time.ticks_diff(current_time, start_time) // 1000
            print("[{:3d}s] Motion #{} detected!".format(elapsed_s, total_motion_count))
            
            # Clear interrupt and update activity time
            mpu.clear_motion_interrupt()
            last_activity_time = current_time
            
            # Flash LED
            set_color("GREEN")
            time.sleep_ms(100)
            set_color("BLUE")
        
        # Check for button press to exit
        if button_pin.value() == 0:  # Button pressed (active low)
            print("\\nButton pressed - exiting...")
            break
        
        # Report status every 10 seconds
        if time.ticks_diff(current_time, last_report_time) > 10000:
            elapsed_total = time.ticks_diff(current_time, start_time) // 1000
            time_since_motion = time.ticks_diff(current_time, last_activity_time) // 1000
            print("[{:3d}s] Status: {} total motions, {} sec since last motion".format(
                elapsed_total, total_motion_count, time_since_motion))
            last_report_time = current_time
        
        # Check for no motion timeout
        if time.ticks_diff(current_time, last_activity_time) > NO_MOTION_TIMEOUT_S * 1000:
            print("\\nNo motion for {} seconds - would enter deep sleep".format(NO_MOTION_TIMEOUT_S))
            print("(Deep sleep disabled for testing)")
            
            # Flash red to indicate sleep condition
            for _ in range(3):
                set_color("RED")
                time.sleep_ms(200)
                set_color("OFF")
                time.sleep_ms(200)
            
            # Reset timeout for continuous monitoring
            last_activity_time = current_time
            set_color("BLUE")
        
        time.sleep_ms(100)
    
    # Cleanup
    motion_pin.irq(handler=None)  # Disable interrupt
    set_color("OFF")
    print("\\nMotion monitoring stopped.")
    print("Total motion events: {}".format(total_motion_count))

# Run the motion monitoring
if __name__ == "__main__":
    try:
        run_motion_monitoring()
    except KeyboardInterrupt:
        print("\\nProgram interrupted")
        set_color("OFF")
    except Exception as e:
        print("\\nError:", e)
        set_color("OFF")