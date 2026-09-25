"""Safe non-blocking keyboard teleoperation for the G1 simulation.

The keyboard and ROS integrations are kept separate from :class:`TeleopState`
so the safety rules can be checked without a running ROS installation.
"""

from __future__ import annotations

from dataclasses import dataclass
import os
import select
import sys
import time
from typing import Optional


PUBLISH_HZ = 20.0
WATCHDOG_TIMEOUT = 0.30
DEFAULT_SPEED = 0.50
MIN_SPEED = 0.05
MAX_SPEED = 2.00
SPEED_STEP = 0.10


@dataclass(frozen=True)
class MotionCommand:
    """The planar velocity that should be sent to ``/cmd_vel``."""

    linear_x: float = 0.0
    angular_z: float = 0.0


class TeleopState:
    """Pure keyboard-to-command state with a 0.30 second safety watchdog."""

    _DIRECTIONS = {
        "UP": MotionCommand(linear_x=1.0),
        "DOWN": MotionCommand(linear_x=-1.0),
        "LEFT": MotionCommand(angular_z=1.0),
        "RIGHT": MotionCommand(angular_z=-1.0),
        "w": MotionCommand(linear_x=1.0),
        "s": MotionCommand(linear_x=-1.0),
        "a": MotionCommand(angular_z=1.0),
        "d": MotionCommand(angular_z=-1.0),
    }

    def __init__(self, speed: float = DEFAULT_SPEED, watchdog_timeout: float = WATCHDOG_TIMEOUT):
        self.speed = min(MAX_SPEED, max(MIN_SPEED, speed))
        self.watchdog_timeout = watchdog_timeout
        self._direction = MotionCommand()
        self._last_direction_at: Optional[float] = None

    def handle_key(self, key: str, now: float) -> bool:
        """Handle one normalized key and return whether the program should quit."""

        if key in ("q", "Q"):
            self.stop()
            return True
        if key == " ":
            self.stop()
            return False
        if key in ("+", "="):
            self.speed = min(MAX_SPEED, self.speed + SPEED_STEP)
            return False
        if key == "-":
            self.speed = max(MIN_SPEED, self.speed - SPEED_STEP)
            return False

        direction = self._DIRECTIONS.get(key)
        if direction is not None:
            self._direction = direction
            self._last_direction_at = now
        return False

    def stop(self) -> None:
        """Clear motion immediately."""

        self._direction = MotionCommand()
        self._last_direction_at = None

    def command_at(self, now: float) -> MotionCommand:
        """Return the current command, applying the keyboard watchdog."""

        if self._last_direction_at is None or now - self._last_direction_at >= self.watchdog_timeout:
            return MotionCommand()
        return MotionCommand(
            linear_x=self._direction.linear_x * self.speed,
            angular_z=self._direction.angular_z * self.speed,
        )


class KeyReader:
    """Cross-platform, non-blocking terminal reader with restoration support."""

    def __init__(self, stream=None):
        self.stream = stream or sys.stdin
        self._fd: Optional[int] = None
        self._old_terminal = None
        self._windows = os.name == "nt"

    def __enter__(self) -> "KeyReader":
        if self._windows:
            return self

        import termios
        import tty

        if not self.stream.isatty():
            raise RuntimeError("g1_teleop requires an interactive terminal")
        self._fd = self.stream.fileno()
        self._old_terminal = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.restore()

    def restore(self) -> None:
        """Restore terminal settings after normal or exceptional shutdown."""

        if self._old_terminal is not None and self._fd is not None:
            import termios

            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_terminal)
            self._old_terminal = None

    def read_key(self) -> Optional[str]:
        """Read one key without waiting; arrows are returned as UP/DOWN/LEFT/RIGHT."""

        if self._windows:
            return self._read_windows_key()
        return self._read_posix_key()

    def _read_windows_key(self) -> Optional[str]:
        import msvcrt

        if not msvcrt.kbhit():
            return None
        key = msvcrt.getwch()
        if key in ("\x00", "\xe0"):
            return {
                "H": "UP",
                "P": "DOWN",
                "K": "LEFT",
                "M": "RIGHT",
            }.get(msvcrt.getwch())
        return key

    def _read_posix_key(self) -> Optional[str]:
        if self._fd is None:
            raise RuntimeError("KeyReader must be used as a context manager")
        ready, _, _ = select.select([self._fd], [], [], 0.0)
        if not ready:
            return None

        key = os.read(self._fd, 1).decode(errors="ignore")
        if key != "\x1b":
            return key or None

        # Arrow keys arrive as ESC [ A/B/C/D. The short timeout keeps the
        # reader non-blocking while allowing the rest of one escape sequence.
        sequence = ""
        deadline = time.monotonic() + 0.02
        while len(sequence) < 2 and time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            ready, _, _ = select.select([self._fd], [], [], remaining)
            if ready:
                sequence += os.read(self._fd, 1).decode(errors="ignore")
        return {
            "[A": "UP",
            "[B": "DOWN",
            "[C": "RIGHT",
            "[D": "LEFT",
        }.get(sequence)


def main(args=None) -> None:
    """Run the 20 Hz ROS 2 teleoperation loop."""

    import rclpy
    from geometry_msgs.msg import Twist

    rclpy.init(args=args)
    node = rclpy.create_node("g1_teleop")
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    state = TeleopState()
    period = 1.0 / PUBLISH_HZ

    def publish(command: MotionCommand) -> None:
        message = Twist()
        message.linear.x = command.linear_x
        message.angular.z = command.angular_z
        publisher.publish(message)

    try:
        with KeyReader() as reader:
            next_publish = time.monotonic()
            try:
                while rclpy.ok():
                    key = reader.read_key()
                    while key is not None:
                        if state.handle_key(key, time.monotonic()):
                            return
                        key = reader.read_key()

                    now = time.monotonic()
                    if now >= next_publish:
                        publish(state.command_at(now))
                        next_publish = now + period
                    rclpy.spin_once(node, timeout_sec=0.0)
                    time.sleep(min(0.005, max(0.0, next_publish - time.monotonic())))
            finally:
                state.stop()
                publish(MotionCommand())
                rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
