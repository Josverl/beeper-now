import time

# DEFAULT_TIMEOUT_MS = 20 * 60 * 1000  # 20 minutes in milliseconds
DEFAULT_TIMEOUT_MS = 10 * 1000  # 10 seconds in milliseconds


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
        elapsed = time.ticks_diff(time.ticks_ms(), self._last_activity_time)
        print(f"[InactivityTimer] elapsed_time_ms: {elapsed}")
        return elapsed

    @property
    def remaining_time_ms(self) -> int:
        """Get remaining time before timeout in milliseconds"""
        remaining = self._timeout_ms - self.elapsed_time_ms
        result = max(0, remaining)
        print(f"[InactivityTimer] remaining_time_ms: {result} (timeout: {self._timeout_ms})")
        return result

    @property
    def is_timeout_reached(self) -> bool:
        """Check if the inactivity timeout has been reached"""
        current = time.ticks_ms()
        elapsed = time.ticks_diff(current, self._last_activity_time)
        reached = elapsed >= self._timeout_ms
        print(f"[InactivityTimer] is_timeout_reached: {reached} (elapsed: {elapsed}, timeout: {self._timeout_ms}, current: {current}, last: {self._last_activity_time})")
        return reached

    def reset(self):
        """Reset the inactivity timer - call this whenever there's activity"""
        self._last_activity_time = time.ticks_ms()
        print(f"Activity detected, timer reset at {self._last_activity_time}")
