import time

DEFAULT_TIMEOUT_MS = 20 * 60 * 1000  # 20 minutes in milliseconds


class InactivityTimer:
    """Manages inactivity detection and timeout handling"""

    def __init__(self, timeout_ms: int = DEFAULT_TIMEOUT_MS):
        self._timeout_ms = timeout_ms
        self._last_activity_time = time.ticks_ms()

    @property
    def timeout_ms(self) -> int:
        """Get the inactivity timeout in milliseconds"""
        return self._timeout_ms

    @timeout_ms.setter
    def timeout_ms(self, value: int):
        """Set the inactivity timeout in milliseconds"""
        self._timeout_ms = value

    @property
    def elapsed_time_ms(self) -> int:
        """Get elapsed time since last activity in milliseconds"""
        current_time = time.ticks_ms()
        return time.ticks_diff(current_time, self._last_activity_time)

    @property
    def remaining_time_ms(self) -> int:
        """Get remaining time before timeout in milliseconds"""
        return self._timeout_ms - self.elapsed_time_ms

    @property
    def is_timeout_reached(self) -> bool:
        """Check if the inactivity timeout has been reached"""
        return self.elapsed_time_ms >= self._timeout_ms

    def reset(self):
        """Reset the inactivity timer - call this whenever there's activity"""
        self._last_activity_time = time.ticks_ms()
        print(f"Activity detected, timer reset at {self._last_activity_time}")
