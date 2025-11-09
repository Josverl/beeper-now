import collections
import time

from ustruct import unpack

# MPU6050 constants (subset needed for motion detection and power management)
MPU6050_DEFAULT_ADDRESS = 0x68

# Register addresses
MPU6050_RA_SMPLRT_DIV = 0x19
MPU6050_RA_CONFIG = 0x1A
MPU6050_RA_GYRO_CONFIG = 0x1B
MPU6050_RA_ACCEL_CONFIG = 0x1C
# NOTE: MPU-6050 does NOT have free fall detection registers!
# Free fall detection is available in newer chips like MPU-6500
# MPU6050_RA_FF_THR = 0x1D          # Does not exist in MPU-6050
# MPU6050_RA_FF_DUR = 0x1E          # Does not exist in MPU-6050
MPU6050_RA_MOT_THR = 0x1F  # Motion detection threshold
MPU6050_RA_MOT_DUR = 0x20  # Motion detection duration
MPU6050_RA_ZRMOT_THR = 0x21  # Zero motion detection threshold
MPU6050_RA_ZRMOT_DUR = 0x22  # Zero motion detection duration
MPU6050_RA_FIFO_EN = 0x23
MPU6050_RA_INT_PIN_CFG = 0x37
MPU6050_RA_INT_ENABLE = 0x38
MPU6050_RA_INT_STATUS = 0x3A
MPU6050_RA_ACCEL_XOUT_H = 0x3B
MPU6050_RA_GYRO_XOUT_H = 0x43
MPU6050_RA_MOT_DETECT_CTRL = 0x69  # Motion detection control
MPU6050_RA_USER_CTRL = 0x6A
MPU6050_RA_PWR_MGMT_1 = 0x6B
MPU6050_RA_PWR_MGMT_2 = 0x6C
MPU6050_RA_WHO_AM_I = 0x75

# Power management bits
MPU6050_PWR1_DEVICE_RESET_BIT = 7
MPU6050_PWR1_SLEEP_BIT = 6
MPU6050_PWR1_CYCLE_BIT = 5
MPU6050_PWR1_TEMP_DIS_BIT = 3

# Interrupt bits
# NOTE: MPU-6050 does NOT have free fall interrupt - that's in newer chips
MPU6050_INTERRUPT_MOT_BIT = 6  # Motion detection interrupt
MPU6050_INTERRUPT_ZMOT_BIT = 5  # Zero motion interrupt
MPU6050_INTERRUPT_FIFO_OFLOW_BIT = 4  # FIFO overflow interrupt
MPU6050_INTERRUPT_I2C_MST_INT_BIT = 3  # I2C master interrupt
MPU6050_INTERRUPT_DATA_RDY_BIT = 0  # Data ready interrupt

# INT pin configuration bits
MPU6050_INTCFG_INT_LEVEL_BIT = 7
MPU6050_INTCFG_INT_OPEN_BIT = 6
MPU6050_INTCFG_LATCH_INT_EN_BIT = 5
MPU6050_INTCFG_INT_RD_CLEAR_BIT = 4
MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT = 3
MPU6050_INTCFG_FSYNC_INT_EN_BIT = 2

# Motion detection control bits
MPU6050_MOTCTRL_ACCEL_ON_DELAY_BIT = 5
MPU6050_MOTCTRL_ACCEL_ON_DELAY_LENGTH = 2
# NOTE: MPU-6050 does NOT have free fall count bits - that's in newer chips
MPU6050_MOTCTRL_MOT_COUNT_BIT = 1
MPU6050_MOTCTRL_MOT_COUNT_LENGTH = 2

# Accelerometer configuration bits
MPU6050_ACONFIG_XA_ST_BIT = 7
MPU6050_ACONFIG_YA_ST_BIT = 6
MPU6050_ACONFIG_ZA_ST_BIT = 5
MPU6050_ACONFIG_AFS_SEL_BIT = 4
MPU6050_ACONFIG_AFS_SEL_LENGTH = 2
MPU6050_ACONFIG_ACCEL_HPF_BIT = 2
MPU6050_ACONFIG_ACCEL_HPF_LENGTH = 3

# Wake frequency settings for low power mode
MPU6050_WAKE_FREQ_1P25 = 0x0
MPU6050_WAKE_FREQ_2P5 = 0x1
MPU6050_WAKE_FREQ_5 = 0x2
MPU6050_WAKE_FREQ_10 = 0x3

# Clock sources
MPU6050_CLOCK_PLL_XGYRO = 0x01

# Scale ranges
ACCEL_RANGE = [2, 4, 8, 16]
GYRO_RANGE = [250, 500, 1000, 2000]

DEFAULT_SAMPLE_RATE = 0x20


class SensorReadings(
    collections.namedtuple(
        "SensorReadings", ["AccX", "AccY", "AccZ", "Temp", "GyroX", "GyroY", "GyroZ"]
    )
):
    pass


class MPU6050(object):
    """
    Enhanced MPU6050 driver with motion detection and deep sleep wake support.
    Based on py-mpu6050 but extended with interrupt configuration and low-power features.
    """

    stable_reading_timeout = 10
    max_gyro_variance = 5

    def __init__(self, i2c, rate=None, address=None):
        self.rate = rate if rate is not None else DEFAULT_SAMPLE_RATE
        self.address = address if address else MPU6050_DEFAULT_ADDRESS
        self.i2c = i2c

        self.buffer = bytearray(16)
        self.bytebuf = memoryview(self.buffer[0:1])
        self.wordbuf = memoryview(self.buffer[0:2])
        self.sensors = bytearray(14)

        self.init_device()

    def write_byte(self, reg, val):
        self.bytebuf[0] = val
        self.i2c.writeto_mem(self.address, reg, self.bytebuf)

    def read_byte(self, reg):
        self.i2c.readfrom_mem_into(self.address, reg, self.bytebuf)
        return self.bytebuf[0]

    def set_bitfield(self, reg, pos, length, val):
        old = self.read_byte(reg)
        shift = pos - length + 1
        mask = (2**length - 1) << shift
        new = (old & ~mask) | (val << shift)
        self.write_byte(reg, new)

    def set_bit(self, reg, bit, val):
        """Set or clear a single bit in a register"""
        old = self.read_byte(reg)
        new = (old | (1 << bit)) if val else (old & ~(1 << bit))
        self.write_byte(reg, new)

    def read_word(self, reg):
        self.i2c.readfrom_mem_into(self.address, reg, self.wordbuf)
        return unpack(">H", self.wordbuf)[0]

    def read_word2(self, reg):
        self.i2c.readfrom_mem_into(self.address, reg, self.wordbuf)
        return unpack(">h", self.wordbuf)[0]

    def identify(self):
        val = self.read_byte(MPU6050_RA_WHO_AM_I)
        expected = MPU6050_DEFAULT_ADDRESS  # Should be 0x68
        if val != expected:
            raise OSError(
                "No MPU6050 at address 0x{:02x} (got 0x{:02x})".format(
                    self.address, val
                )
            )

    def reset(self):
        self.write_byte(MPU6050_RA_PWR_MGMT_1, (1 << MPU6050_PWR1_DEVICE_RESET_BIT))
        time.sleep_ms(100)
        # After reset, device is in sleep mode, need to wake it
        self.wake()

    def wake(self):
        """Wake the device from sleep mode and set clock source"""
        self.write_byte(MPU6050_RA_PWR_MGMT_1, 0x00)  # Clear sleep bit
        time.sleep_ms(10)
        self.write_byte(MPU6050_RA_PWR_MGMT_1, MPU6050_CLOCK_PLL_XGYRO)
        time.sleep_ms(50)

    def init_device(self):
        self.identify()

        # Wake and set clock source
        self.wake()

        # Enable all sensors
        self.write_byte(MPU6050_RA_PWR_MGMT_2, 0)

        # Set sampling rate
        self.write_byte(MPU6050_RA_SMPLRT_DIV, self.rate)

        # Enable DLPF
        self.write_byte(MPU6050_RA_CONFIG, 1)

        # Set default accel/gyro ranges
        self.set_accel_range(0)  # ±2g
        self.set_gyro_range(0)  # ±250°/s

    def set_gyro_range(self, fsr):
        """Set gyroscope full scale range (0=±250°/s, 1=±500°/s, 2=±1000°/s, 3=±2000°/s)"""
        self.gyro_range = GYRO_RANGE[fsr]
        self.set_bitfield(MPU6050_RA_GYRO_CONFIG, 4, 2, fsr)

    def set_accel_range(self, fsr):
        """Set accelerometer full scale range (0=±2g, 1=±4g, 2=±8g, 3=±16g)"""
        self.accel_range = ACCEL_RANGE[fsr]
        self.set_bitfield(MPU6050_RA_ACCEL_CONFIG, 4, 2, fsr)

    def read_sensors(self):
        """Read raw sensor data"""
        self.i2c.readfrom_mem_into(self.address, MPU6050_RA_ACCEL_XOUT_H, self.sensors)
        data = unpack(">hhhhhhh", self.sensors)
        return SensorReadings(*data)

    def read_sensors_scaled(self):
        """Read sensor data scaled to actual units"""
        data = list(self.read_sensors())
        data[0:3] = [x / (65536 // self.accel_range // 2) for x in data[0:3]]
        data[4:7] = [x / (65536 // self.gyro_range // 2) for x in data[4:7]]
        return SensorReadings(*data)

    # Motion detection methods
    def set_motion_detection_threshold(self, threshold):
        """Set motion detection threshold (0-255, higher = less sensitive)"""
        threshold = max(0, min(threshold, 255))
        self.write_byte(MPU6050_RA_MOT_THR, threshold)

    def set_motion_detection_duration(self, duration):
        """Set motion detection duration (1-255 ms)"""
        duration = max(1, min(duration, 255))
        self.write_byte(MPU6050_RA_MOT_DUR, duration)
        
    def enable_motion_detection_hardware(self):
        """Enable motion detection in hardware - CRITICAL for MPU-6050"""
        # This register is essential for motion detection to work on MPU-6050
        self.write_byte(MPU6050_RA_MOT_DETECT_CTRL, 0x80)

    def enable_motion_interrupt(self, enabled=True):
        """Enable or disable motion detection interrupt"""
        self.set_bit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_MOT_BIT, enabled)

    def disable_motion_interrupt(self):
        """Disable motion detection interrupt"""
        self.enable_motion_interrupt(False)

    # NOTE: MPU-6050 does NOT support free fall detection!
    # Free fall detection is available in newer chips like MPU-6500/MPU-9250
    # The methods below are removed as they don't exist in MPU-6050 hardware

    # Zero motion detection methods
    def set_zero_motion_detection_threshold(self, threshold):
        """
        Set zero motion detection threshold (0-255)
        Higher values = less sensitive to small movements
        Recommended: 4-20 for detecting complete stillness
        """
        threshold = max(0, min(threshold, 255))
        self.write_byte(MPU6050_RA_ZRMOT_THR, threshold)

    def set_zero_motion_detection_duration(self, duration):
        """
        Set zero motion detection duration (1-255 ms)
        Time that acceleration must be below threshold to trigger
        Recommended: 50-255ms (hardware limit is 255ms maximum)
        Note: For longer durations, use software timing instead of hardware register
        """
        duration = max(1, min(duration, 255))
        self.write_byte(MPU6050_RA_ZRMOT_DUR, duration)

    def enable_zero_motion_interrupt(self, enabled=True):
        """Enable or disable zero motion detection interrupt"""
        self.set_bit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_ZMOT_BIT, enabled)

    def disable_zero_motion_interrupt(self):
        """Disable zero motion detection interrupt"""
        self.enable_zero_motion_interrupt(False)

    # Enhanced interrupt configuration for deep sleep wake
    def configure_interrupt_pin(
        self, active_high=True, open_drain=False, latch=True, clear_on_read=False
    ):
        """
        Configure the INT pin behavior for reliable deep sleep wake

        Args:
            active_high: True for active high (recommended for ESP32 ext0 wake), False for active low
            open_drain: True for open drain output, False for push-pull
            latch: True to latch interrupt until cleared, False for 50μs pulse
            clear_on_read: True to clear on any register read, False to clear only on INT_STATUS read
        """
        cfg = 0x00

        if not active_high:
            cfg |= 1 << MPU6050_INTCFG_INT_LEVEL_BIT  # Active low
        if open_drain:
            cfg |= 1 << MPU6050_INTCFG_INT_OPEN_BIT  # Open drain
        if latch:
            cfg |= 1 << MPU6050_INTCFG_LATCH_INT_EN_BIT  # Latched
        if clear_on_read:
            cfg |= 1 << MPU6050_INTCFG_INT_RD_CLEAR_BIT  # Clear on any read

        self.write_byte(MPU6050_RA_INT_PIN_CFG, cfg)

    def read_interrupt_status(self):
        """Read and clear interrupt status register"""
        return self.read_byte(MPU6050_RA_INT_STATUS)

    def clear_motion_interrupt(self):
        """Clear motion interrupt by reading status register"""
        return self.read_interrupt_status()

    def is_motion_interrupt_triggered(self):
        """Check if motion interrupt is currently active"""
        status = self.read_interrupt_status()
        return bool(status & (1 << MPU6050_INTERRUPT_MOT_BIT))

    # NOTE: Free fall detection removed - not supported in MPU-6050

    def is_zero_motion_interrupt_triggered(self):
        """Check if zero motion interrupt is currently active"""
        status = self.read_interrupt_status()
        return bool(status & (1 << MPU6050_INTERRUPT_ZMOT_BIT))

    def is_data_ready_interrupt_triggered(self):
        """Check if data ready interrupt is currently active"""
        status = self.read_interrupt_status()
        return bool(status & (1 << MPU6050_INTERRUPT_DATA_RDY_BIT))

    def get_interrupt_status_details(self):
        """Get detailed breakdown of all interrupt statuses"""
        status = self.read_interrupt_status()
        return {
            # NOTE: Free fall interrupt removed - not supported in MPU-6050
            "motion": bool(status & (1 << MPU6050_INTERRUPT_MOT_BIT)),
            "zero_motion": bool(status & (1 << MPU6050_INTERRUPT_ZMOT_BIT)),
            "fifo_overflow": bool(status & (1 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)),
            "i2c_master": bool(status & (1 << MPU6050_INTERRUPT_I2C_MST_INT_BIT)),
            "data_ready": bool(status & (1 << MPU6050_INTERRUPT_DATA_RDY_BIT)),
            "raw_status": status,
        }

    # Low-power cycle mode for deep sleep
    def enable_low_power_cycle(
        self, wake_frequency=MPU6050_WAKE_FREQ_5, disable_temp=True, disable_gyro=True
    ):
        """
        Enable low-power cycle mode for minimal power consumption during deep sleep
        In this mode, the accelerometer will wake up periodically to check for motion

        Args:
            wake_frequency: Wake frequency (MPU6050_WAKE_FREQ_1P25, _2P5, _5, or _10)
            disable_temp: Disable temperature sensor to save power
            disable_gyro: Disable gyroscope to save power
        """
        pwr1 = self.read_byte(MPU6050_RA_PWR_MGMT_1)

        if disable_temp:
            pwr1 |= 1 << MPU6050_PWR1_TEMP_DIS_BIT

        # Enable cycle mode
        pwr1 |= 1 << MPU6050_PWR1_CYCLE_BIT

        self.write_byte(MPU6050_RA_PWR_MGMT_1, pwr1)

        # Set wake frequency in PWR_MGMT_2
        pwr2 = self.read_byte(MPU6050_RA_PWR_MGMT_2)
        pwr2 = (pwr2 & 0x3F) | ((wake_frequency & 0x03) << 6)  # Bits 7:6

        # Disable gyroscope axes to save power
        if disable_gyro:
            pwr2 |= 0x07  # Disable X, Y, Z gyro

        self.write_byte(MPU6050_RA_PWR_MGMT_2, pwr2)

    def disable_low_power_cycle(self):
        """Disable low-power cycle mode and return to normal operation"""
        pwr1 = self.read_byte(MPU6050_RA_PWR_MGMT_1)
        pwr1 &= ~(1 << MPU6050_PWR1_CYCLE_BIT)  # Clear cycle bit
        pwr1 &= ~(1 << MPU6050_PWR1_TEMP_DIS_BIT)  # Enable temperature
        self.write_byte(MPU6050_RA_PWR_MGMT_1, pwr1)

        # Re-enable all sensors
        self.write_byte(MPU6050_RA_PWR_MGMT_2, 0x00)

    def set_accelerometer_power_mode(self, mode:int = 0):
        """
        Set accelerometer power mode for low power applications
        Args:
            mode: 0=normal, 1=1.25Hz, 2=5Hz, 3=20Hz, 4=40Hz low power modes
        """
        if mode == 0:
            # Normal mode - disable cycle bit
            self.disable_low_power_cycle()
        else:
            # Low power mode - enable cycle and set frequency
            freq_map = {
                1: MPU6050_WAKE_FREQ_1P25,
                2: MPU6050_WAKE_FREQ_2P5,
                3: MPU6050_WAKE_FREQ_5,
                4: MPU6050_WAKE_FREQ_10,
            }
            wake_freq = freq_map.get(mode, MPU6050_WAKE_FREQ_5)
            self.enable_low_power_cycle(wake_frequency=wake_freq)

    def configure_motion_detection_control(self, accel_on_delay=0, mot_count=0):
        """
        Configure motion detection control register for advanced motion detection

        Args:
            accel_on_delay: Delay before accelerometer starts (0-3)
            mot_count: Motion counter decrement (0-3)
        """
        ctrl = 0x00
        ctrl |= (accel_on_delay & 0x03) << MPU6050_MOTCTRL_ACCEL_ON_DELAY_BIT - 1
        ctrl |= (mot_count & 0x03) << MPU6050_MOTCTRL_MOT_COUNT_BIT - 1

        self.write_byte(MPU6050_RA_MOT_DETECT_CTRL, ctrl)

    # Convenience method for complete motion detection setup
    def setup_motion_wake(
        self, threshold=12, duration_ms=40, active_high=True, low_power=True
    ):
        """
        Complete setup for motion detection wake from deep sleep

        Args:
            threshold: Motion detection sensitivity (higher = less sensitive)
            duration_ms: Duration motion must be present to trigger
            active_high: INT pin active high (recommended for ESP32)
            low_power: Enable low-power cycle mode
        """
        # Configure motion detection
        self.set_motion_detection_threshold(threshold)
        self.set_motion_detection_duration(duration_ms)
        
        # CRITICAL: Enable motion detection hardware
        self.enable_motion_detection_hardware()

        # Configure interrupt pin for reliable wake
        self.configure_interrupt_pin(
            active_high=active_high, latch=True, open_drain=False
        )

        # Enable motion interrupt
        self.enable_motion_interrupt()

        # Clear any pending interrupts
        self.clear_motion_interrupt()

        # Enable low power mode if requested
        if low_power:
            self.enable_low_power_cycle()

    def setup_intelligent_wake_system(
        self,
        motion_threshold=12,
        zero_motion_threshold=8,
        zero_motion_duration_ms=200,
        active_high=True,
    ):
        """
        Setup an intelligent wake system that can detect both motion (for wake)
        and zero motion (for sleep preparation)

        Args:
            motion_threshold: Sensitivity for motion detection (wake trigger)
            zero_motion_threshold: Sensitivity for zero motion detection
            zero_motion_duration_ms: How long to wait before triggering zero motion (max 255ms)
            active_high: INT pin active high
        """
        # Configure motion detection for wake
        self.set_motion_detection_threshold(motion_threshold)
        self.set_motion_detection_duration(40)  # 40ms standard

        # Configure zero motion detection for sleep preparation
        # Note: Hardware register limited to 255ms maximum
        self.set_zero_motion_detection_threshold(zero_motion_threshold)
        self.set_zero_motion_detection_duration(min(zero_motion_duration_ms, 255))

        # Configure interrupt pin
        self.configure_interrupt_pin(active_high=active_high, latch=True)

        # Enable both interrupts
        self.enable_motion_interrupt()
        self.enable_zero_motion_interrupt()

        # Clear pending interrupts
        self.clear_motion_interrupt()

    # NOTE: Free fall detection setup removed - not supported in MPU-6050
    # Free fall detection is available in newer chips like MPU-6500/MPU-9250

    def get_power_consumption_estimate(self):
        """
        Get estimated current consumption based on current configuration
        Returns dictionary with consumption estimates in µA
        """
        pwr1 = self.read_byte(MPU6050_RA_PWR_MGMT_1)
        pwr2 = self.read_byte(MPU6050_RA_PWR_MGMT_2)

        consumption = {"gyro": 0, "accel": 0, "temp": 0, "total": 0}

        # Check if in cycle mode
        is_cycle = bool(pwr1 & (1 << MPU6050_PWR1_CYCLE_BIT))
        temp_disabled = bool(pwr1 & (1 << MPU6050_PWR1_TEMP_DIS_BIT))

        # Gyroscope consumption
        gyro_disabled = pwr2 & 0x07  # Bits 2:0
        if gyro_disabled != 0x07:  # Not all gyro axes disabled
            consumption["gyro"] = 3600  # 3.6mA active
        else:
            consumption["gyro"] = 5  # 5µA standby

        # Accelerometer consumption
        if is_cycle:
            # Low power mode consumption depends on wake frequency
            wake_freq = (pwr2 >> 6) & 0x03
            if wake_freq == MPU6050_WAKE_FREQ_1P25:
                consumption["accel"] = 10
            elif wake_freq == MPU6050_WAKE_FREQ_2P5:
                consumption["accel"] = 15
            elif wake_freq == MPU6050_WAKE_FREQ_5:
                consumption["accel"] = 20
            else:  # 10Hz
                consumption["accel"] = 60
        else:
            consumption["accel"] = 500  # Normal mode

        # Temperature sensor
        if not temp_disabled:
            consumption["temp"] = 10  # Estimated

        consumption["total"] = (
            consumption["gyro"] + consumption["accel"] + consumption["temp"]
        )

        return consumption

    def print_configuration(self):
        """Print current device configuration for debugging"""
        print("MPU6050 Configuration:")
        print(f"  Device ID: 0x{self.read_byte(MPU6050_RA_WHO_AM_I):02X}")

        pwr1 = self.read_byte(MPU6050_RA_PWR_MGMT_1)
        pwr2 = self.read_byte(MPU6050_RA_PWR_MGMT_2)
        int_en = self.read_byte(MPU6050_RA_INT_ENABLE)

        print(f"  Sleep mode: {'Enabled' if pwr1 & (1 << 6) else 'Disabled'}")
        print(f"  Cycle mode: {'Enabled' if pwr1 & (1 << 5) else 'Disabled'}")
        print(f"  Temperature: {'Disabled' if pwr1 & (1 << 3) else 'Enabled'}")

        gyro_disabled = pwr2 & 0x07
        print(f"  Gyroscope: {'Disabled' if gyro_disabled == 0x07 else 'Enabled'}")

        print(f"  Motion interrupt: {'Enabled' if int_en & (1 << 6) else 'Disabled'}")
        print(
            f"  Zero motion interrupt: {'Enabled' if int_en & (1 << 5) else 'Disabled'}"
        )

        if pwr1 & (1 << 5):  # Cycle mode
            wake_freq = (pwr2 >> 6) & 0x03
            freq_map = {0: "1.25Hz", 1: "2.5Hz", 2: "5Hz", 3: "10Hz"}
            print(f"  Wake frequency: {freq_map.get(wake_freq, 'Unknown')}")

        consumption = self.get_power_consumption_estimate()
        print(f"  Estimated consumption: {consumption['total']}µA")

    def get_sensor_avg(self, samples, softstart=100):
        """Return the average readings from the sensors over the
        given number of samples.  Discard the first softstart
        samples to give things time to settle."""
        sample = self.read_sensors()
        counters = [0] * 7

        for i in range(samples + softstart):
            # the sleep here is to ensure we read a new sample
            # each time
            time.sleep_ms(2)

            sample = self.read_sensors()
            if i < softstart:
                continue

            for j, val in enumerate(sample):
                counters[j] += val

        return SensorReadings(*[x // samples for x in counters])
