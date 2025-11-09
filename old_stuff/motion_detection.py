# Enhanced motion detection script with interrupt handling
# Based on the working simple_motion_test.py 

import time

import machine

from config import COLORS, set_color
from mpu6050 import MPU6050

# Configuration
I2C_SCL_PIN = 6
I2C_SDA_PIN = 7  
MOTION_GPIO = 4
BUTTON_GPIO = 5
MPU_ADDR = 0x68

# Motion detection settings
MOTION_THRESHOLD = 8      # Reasonable sensitivity
MOTION_DURATION_MS = 20   # Stability duration
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
    if time.ticks_diff(current_time, last_motion_time) > 200:  # 200ms debounce
        motion_detected = True
        last_motion_time = current_time
        motion_count += 1

def setup_hardware():
    """Setup I2C, MPU6050 and GPIO pins"""
    print("=== MOTION DETECTION WITH INTERRUPTS ===")
    
    # Setup I2C and MPU6050
    i2c = machine.I2C(0, scl=machine.Pin(I2C_SCL_PIN), sda=machine.Pin(I2C_SDA_PIN), freq=400000)
    print("I2C devices:", i2c.scan())
    
    mpu = MPU6050(i2c, address=MPU_ADDR)
    
    # Setup GPIO pins
    motion_pin = machine.Pin(MOTION_GPIO, machine.Pin.IN, machine.Pin.PULL_DOWN)
    button_pin = machine.Pin(BUTTON_GPIO, machine.Pin.IN, machine.Pin.PULL_UP)
    
    # Configure motion detection with reasonable sensitivity
    print("Configuring motion detection...")
    mpu.set_motion_detection_threshold(MOTION_THRESHOLD)
    mpu.set_motion_detection_duration(MOTION_DURATION_MS)
    mpu.configure_interrupt_pin(active_high=INT_ACTIVE_HIGH, latch=True, open_drain=False)
    mpu.enable_motion_interrupt()
    mpu.clear_motion_interrupt()
    
    # Setup interrupt handler on rising edge (when motion detected)
    if INT_ACTIVE_HIGH:
        motion_pin.irq(trigger=machine.Pin.IRQ_RISING, handler=motion_interrupt_handler)
    else:
        motion_pin.irq(trigger=machine.Pin.IRQ_FALLING, handler=motion_interrupt_handler)
    
    # Check configuration
    mot_thr = mpu.read_byte(0x1F) 
    mot_dur = mpu.read_byte(0x20)
    int_enable = mpu.read_byte(0x38)
    int_cfg = mpu.read_byte(0x37)
    
    print("Motion detection configured:")
    print("  Threshold:", mot_thr)
    print("  Duration: {}ms".format(mot_dur))
    print("  Int enable: 0x{:02X}".format(int_enable))
    print("  Int config: 0x{:02X}".format(int_cfg))
    print("  GPIO interrupt: {} edge on GPIO{}".format('RISING' if INT_ACTIVE_HIGH else 'FALLING', MOTION_GPIO))
    print("  Initial pin state:", motion_pin.value())
    
    return mpu, motion_pin, button_pin

def monitor_motion(mpu, motion_pin, button_pin):
    """Main motion monitoring loop with interrupt-driven detection"""
    global motion_detected, motion_count
    
    print("\\nStarting motion monitoring...")
    print("Timeout: {} seconds without motion -> sleep".format(NO_MOTION_TIMEOUT_S))
    print("Press button (GPIO{}) to exit\\n".format(BUTTON_GPIO))
    
    start_time = time.ticks_ms()
    last_activity_time = time.ticks_ms()
    last_report_time = 0
    
    set_color("BLUE")  # Indicate monitoring active
    
    while True:
        current_time = time.ticks_ms()
        
        # Check if interrupt detected motion
        if motion_detected:
            motion_detected = False  # Reset flag
            
            elapsed_s = time.ticks_diff(current_time, start_time) // 1000
            print("[{:3d}s] Motion #{} detected via interrupt!".format(elapsed_s, motion_count))
            
            # Read sensor data to show motion details
            try:
                sensors = mpu.read_sensors_scaled()
                print("      Accel: X={:6.2f}g Y={:6.2f}g Z={:6.2f}g".format(sensors.AccX, sensors.AccY, sensors.AccZ))
            except Exception:
                print("      Could not read sensor data")
            
            # Read and clear interrupt status
            int_status = mpu.read_interrupt_status()
            print("      Interrupt status: 0x{:02X}".format(int_status))
            
            # Clear the interrupt to allow next detection
            mpu.clear_motion_interrupt()
            
            # Update activity time
            last_activity_time = current_time
            
            # Flash LED to indicate motion
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
            print("[{:3d}s] Status: {} motions, {} sec since last motion, pin={}".format(
                elapsed_total, motion_count, time_since_motion, motion_pin.value()))
            last_report_time = current_time
        
        # Check for no motion timeout
        if time.ticks_diff(current_time, last_activity_time) > NO_MOTION_TIMEOUT_S * 1000:
            print("\\nNo motion for {} seconds - would go to deep sleep now".format(NO_MOTION_TIMEOUT_S))
            print("(Sleep disabled for testing - press button to exit)")
            
            # Flash red to indicate sleep condition
            for _ in range(3):
                set_color("RED")
                time.sleep_ms(200)
                set_color("OFF")
                time.sleep_ms(200)
            
            # Reset timeout for continuous monitoring
            last_activity_time = current_time
            set_color("BLUE")
        
        # Small delay to prevent busy waiting
        time.sleep_ms(50)
    
    # Cleanup
    set_color("OFF")
    print("\\nMotion monitoring stopped.")
    print("Total motion events detected: {}".format(motion_count))

def test_motion_basic():
    """Simple test without interrupts to verify hardware works"""
    print("=== BASIC MOTION TEST (no interrupts) ===")
    
    # Setup I2C and MPU6050
    i2c = machine.I2C(0, scl=machine.Pin(I2C_SCL_PIN), sda=machine.Pin(I2C_SDA_PIN), freq=400000)
    mpu = MPU6050(i2c, address=MPU_ADDR)
    motion_pin = machine.Pin(MOTION_GPIO, machine.Pin.IN, machine.Pin.PULL_DOWN)
    
    # Configure sensitive settings for testing
    mpu.set_motion_detection_threshold(4)
    mpu.set_motion_detection_duration(10)
    mpu.configure_interrupt_pin(active_high=INT_ACTIVE_HIGH, latch=True)
    mpu.enable_motion_interrupt()
    mpu.clear_motion_interrupt()
    
    print("Testing for 10 seconds - move the device...")
    
    start = time.ticks_ms()
    count = 0
    
    while time.ticks_diff(time.ticks_ms(), start) < 10000:
        pin_state = motion_pin.value()
        int_status = mpu.read_byte(0x3A)
        
        if pin_state or (int_status & 0x40):
            count += 1
            elapsed = time.ticks_diff(time.ticks_ms(), start) // 1000
            print("[{:2d}s] Motion! Pin={}, Status=0x{:02X}".format(elapsed, pin_state, int_status))
            mpu.clear_motion_interrupt()
            time.sleep_ms(500)  # Debounce
        
        time.sleep_ms(100)
    
    print("Basic test complete. Motion events: {}".format(count))
    return count > 0

def main():
    """Main function - choose test mode or full monitoring"""
    print("Motion Detection Test")
    print("1. Basic test (no interrupts)")
    print("2. Full monitoring with interrupts") 
    print("Starting basic test first...\\n")
    
    # First do a basic test to make sure hardware works
    if test_motion_basic():
        print("\\nBasic test PASSED - starting interrupt-based monitoring...\\n")
        
        # Setup hardware with interrupts
        mpu, motion_pin, button_pin = setup_hardware()
        
        # Start monitoring
        monitor_motion(mpu, motion_pin, button_pin)
    else:
        print("\\nBasic test FAILED - check hardware setup")
        print("Possible issues:")
        print("- MPU6050 INT pin not connected to GPIO4")
        print("- Wrong I2C wiring") 
        print("- Device not moving enough for detection")

# Run main function
if __name__ == "__main__":
    main()