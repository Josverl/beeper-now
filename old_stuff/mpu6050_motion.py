# mpu6050_motion.py
# MicroPython helper voor MPU6050: motion interrupt (INT) + low-power accel.
# Gemaakt voor ESP32-C3; werkt ook op andere ESP32 varianten.

import time

# --- Register adressen (MPU6050) ---
PWR_MGMT_1      = 0x6B
PWR_MGMT_2      = 0x6C
ACCEL_CONFIG    = 0x1C
ACCEL_CONFIG2   = 0x1D
INT_PIN_CFG     = 0x37
INT_ENABLE      = 0x38
INT_STATUS      = 0x3A
MOT_THR         = 0x1F
MOT_DUR         = 0x20
SMPLRT_DIV      = 0x19
CONFIG          = 0x1A
USER_CTRL       = 0x6A
MOT_DETECT_CTRL = 0x69  # niet op alle klonen volledig aanwezig

# PWR_MGMT_1 bits
DEVICE_RESET = 1 << 7
SLEEP        = 1 << 6
CYCLE        = 1 << 5
TEMP_DIS     = 1 << 3
CLKSEL_PLL_X = 0x01

# INT_ENABLE bits
MOT_INT_EN   = 1 << 6

# INT_PIN_CFG bits
INT_LEVEL    = 1 << 7  # 1 = active low, 0 = active high
INT_OPEN     = 1 << 6
LATCH_INT_EN = 1 << 5
INT_RD_CLEAR = 1 << 4  # 1 = clear on any read, 0 = clear on INT_STATUS read

class MPU6050Motion:
    """
    Minimalistische driver gericht op motion interrupt/Wake-on-Motion gebruik.
    """

    def __init__(self, i2c, addr=0x68):
        self.i2c = i2c
        self.addr = addr

    # ---- I2C helpers ----
    def _w(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes([val & 0xFF]))

    def _r(self, reg):
        return self.i2c.readfrom_mem(self.addr, reg, 1)[0]

    # ---- Basis init ----
    def wake(self):
        # Wake uit sleep en gebruik PLL klok voor stabiliteit
        self._w(PWR_MGMT_1, 0x00)
        time.sleep_ms(10)
        self._w(PWR_MGMT_1, CLKSEL_PLL_X)
        time.sleep_ms(50)

    def configure_accel(self, fsr_g=2, dlpf_hz=44):
        """
        fsr_g: 2/4/8/16
        dlpf_hz: 5/10/20/44/94/184 (effectief via ACCEL_CONFIG2 & CONFIG)
        """
        # ±2g..±16g
        fs_map = {2:0, 4:1, 8:2, 16:3}
        self._w(ACCEL_CONFIG, fs_map.get(fsr_g, 0) << 3)
        # DLPF voor accel (grofweg): 44 Hz -> 0x03
        dlpf_map = {5:6, 10:5, 20:4, 44:3, 94:2, 184:1}
        self._w(ACCEL_CONFIG2, dlpf_map.get(dlpf_hz, 3))
        # DLPF voor gyro (CONFIG) mag je ook iets rustiger zetten
        self._w(CONFIG, 0x03)
        # Sample rate divider (optioneel)
        self._w(SMPLRT_DIV, 0x09)  # ~1kHz/(1+9)=100Hz base (met DLPF actief)

    # ---- Motion detect configuratie ----
    def configure_motion_interrupt(self, threshold=8, duration_ms=40,
                                   int_active_high=True, latch=True, open_drain=False):
        """
        threshold: drempel in LSB (~mg schaal; kalibreer in de praktijk)
        duration_ms: tijd dat beweging boven drempel moet zijn
        int_active_high: True = INT hoog bij event (aanrader voor ext0 HIGH wake)
        latch: True = INT blijft actief tot INT_STATUS gelezen is
        """
        # Threshold & duur
        self._w(MOT_THR, max(1, min(threshold, 255)))
        self._w(MOT_DUR, max(1, min(duration_ms, 255)))

        # Optioneel: motion detect control (niet elke kloon ondersteunt deze bits)
        try:
            cur = self._r(MOT_DETECT_CTRL)
            # Bits 7:6 "ACCEL_INTEL" modes; zet conservatief aan als aanwezig
            cur = (cur & ~0xC0) | 0xC0
            self._w(MOT_DETECT_CTRL, cur)
        except OSError:
            pass

        # INT pin gedrag
        cfg = 0x00
        if not int_active_high:
            cfg |= INT_LEVEL       # active low
        if open_drain:
            cfg |= INT_OPEN
        if latch:
            cfg |= LATCH_INT_EN    # latched tot read
            # clear ONLY on reading INT_STATUS om controle te houden
        else:
            cfg |= INT_RD_CLEAR    # clear bij eender welke read
        self._w(INT_PIN_CFG, cfg)

        # Enable alleen de motion interrupt
        self._w(INT_ENABLE, MOT_INT_EN)

    def clear_interrupt(self):
        # Lezen van INT_STATUS wist latched interrupt
        _ = self._r(INT_STATUS)

    # ---- Ultra-low-power accel tijdens MCU deep sleep ----
    def enable_low_power_accel_cycle(self, lp_wake_rate_hz=5, disable_gyro=True, disable_temp=True):
        """
        Zet 'cycle' in PWR_MGMT_1: accel samplet in low-power stand en kan INT blijven geven.
        lp_wake_rate_hz: 1/2/5/40 (datasheet LP_WAKE_CTRL). We kiezen dichtstbij.
        """
        # LP_WAKE_CTRL bits zitten in PWR_MGMT_1[7:6] bij sommige varianten; op 6050 is dit 'CYCLE' + interne timing.
        # Praktisch: CYCLE=1 reduceert stroom, sample-interval wordt door device bepaald.
        pwr1 = self._r(PWR_MGMT_1)
        if disable_temp:
            pwr1 |= TEMP_DIS
        pwr1 |= CYCLE
        self._w(PWR_MGMT_1, pwr1)

        # Gyro assen uit (scheelt stroom)
        if disable_gyro:
            # PWR_MGMT_2 bits 2:0 = DIS_ZYXA, zet alle gyro standby
            self._w(PWR_MGMT_2, 0x07)

    def disable_low_power_cycle(self):
        pwr1 = self._r(PWR_MGMT_1)
        pwr1 &= ~CYCLE
        if pwr1 & TEMP_DIS:
            pwr1 &= ~TEMP_DIS
        self._w(PWR_MGMT_1, pwr1)
        # heractiveer gyro indien nodig
        self._w(PWR_MGMT_2, 0x00)