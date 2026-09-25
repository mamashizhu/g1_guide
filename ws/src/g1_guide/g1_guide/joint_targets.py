"""Pure-Python validation and rate limiting for joint position targets.

This module deliberately contains no controller or ROS integration.  It guards
references; it does not compute effort or implement a PD loop.
"""

import math
from collections.abc import Mapping, Sequence


class JointTargetGuard:
    """Validate joint targets and publish bounded reference steps.

    ``limits`` iteration order defines the order returned by :meth:`step`.
    Feedback must contain every joint, while commands may update any non-empty
    subset.  All times are monotonic wall-clock seconds supplied by the caller.
    """

    def __init__(
        self,
        limits,
        max_velocity=0.3,
        command_timeout=0.6,
        feedback_timeout=0.5,
    ):
        if not isinstance(limits, Mapping) or not limits:
            raise ValueError("limits must be a non-empty ordered mapping")
        self.names = tuple(limits.keys())
        if any(not isinstance(name, str) or not name for name in self.names):
            raise ValueError("joint names must be non-empty strings")
        self._index = {name: index for index, name in enumerate(self.names)}
        self._limits = []
        for name in self.names:
            pair = limits[name]
            if not isinstance(pair, Sequence) or isinstance(pair, (str, bytes)) or len(pair) != 2:
                raise ValueError("each joint limit must be a (lower, upper) pair")
            lower, upper = pair
            self._finite(lower, "lower limit")
            self._finite(upper, "upper limit")
            if lower >= upper:
                raise ValueError("each lower limit must be below its upper limit")
            self._limits.append((float(lower), float(upper)))

        self.max_velocity = self._positive_finite(max_velocity, "max_velocity")
        self.command_timeout = self._positive_finite(command_timeout, "command_timeout")
        self.feedback_timeout = self._positive_finite(feedback_timeout, "feedback_timeout")

        self.rejected_count = 0
        self._status = "waiting_feedback"
        self._feedback_positions = None
        self._feedback_time = None
        self._reference = None
        self._target = None
        self._last_step_time = None
        self._last_command_time = None
        self._last_time = None
        self._fault = False
        self._timed_out = False

    @property
    def status(self):
        """Current guard state; before the first feedback it is waiting_feedback."""
        return self._status

    @staticmethod
    def _finite(value, label):
        try:
            finite = math.isfinite(value)
        except (TypeError, ValueError, OverflowError):
            finite = False
        if not finite:
            raise ValueError("{} must be finite".format(label))

    @classmethod
    def _positive_finite(cls, value, label):
        cls._finite(value, label)
        if value <= 0:
            raise ValueError("{} must be positive".format(label))
        return float(value)

    @staticmethod
    def _validated_names_positions(names, positions, *, allow_subset, known_names):
        if isinstance(names, (str, bytes)) or not isinstance(names, Sequence):
            raise ValueError("names must be a sequence of joint names")
        if isinstance(positions, (str, bytes)) or not isinstance(positions, Sequence):
            raise ValueError("positions must be a sequence")
        if len(names) != len(positions):
            raise ValueError("names and positions must have equal lengths")
        if not names and not allow_subset:
            raise ValueError("feedback must contain every joint")
        if allow_subset and not names:
            raise ValueError("command must contain at least one joint")
        if any(not isinstance(name, str) for name in names):
            raise ValueError("joint names must be strings")
        if len(set(names)) != len(names):
            raise ValueError("joint names must not contain duplicates")
        if any(name not in known_names for name in names):
            raise ValueError("unknown or invalid joint name")
        values = []
        for value in positions:
            JointTargetGuard._finite(value, "joint position")
            values.append(float(value))
        return tuple(names), tuple(values)

    def _check_time(self, now):
        self._finite(now, "now")
        now = float(now)
        if self._last_time is not None and now < self._last_time:
            self._enter_fault()
            raise ValueError("monotonic time moved backwards; guard entered fault")
        return now

    def _enter_fault(self):
        self._fault = True
        self._target = None
        self._last_command_time = None
        self._timed_out = False
        self._status = "fault"

    def _in_limits(self, index, value):
        lower, upper = self._limits[index]
        return lower <= value <= upper

    def feedback(self, names, positions, now):
        """Accept one complete, reorderable, in-limit joint-state sample."""
        now = self._check_time(now)
        self._last_time = now
        names, positions = self._validated_names_positions(
            names, positions, allow_subset=False, known_names=self._index
        )
        if len(names) != len(self.names) or set(names) != set(self.names):
            raise ValueError("feedback must contain exactly the configured joints")

        ordered = [0.0] * len(self.names)
        for name, position in zip(names, positions):
            index = self._index[name]
            if not self._in_limits(index, position):
                self._enter_fault()
                self._last_time = now
                raise ValueError("feedback position for {} is outside its limits".format(name))
            ordered[index] = position

        recovering = self._fault
        self._feedback_positions = ordered
        self._feedback_time = now
        self._last_time = now
        if self._reference is None or recovering:
            # Recovery deliberately discards any old command and re-anchors to
            # measured state, so a fault cannot resume a latent trajectory.
            self._reference = list(ordered)
            self._target = None
            self._last_command_time = None
            self._last_step_time = now
            self._fault = False
            self._timed_out = False
            self._status = "hold_idle"
        elif self._last_command_time is None:
            self._status = "hold_timeout" if self._timed_out else "hold_idle"

    def command(self, names, positions, now):
        """Atomically update a non-empty subset of targets, or reject it."""
        try:
            now = self._check_time(now)
            # Rejected calls still advance the observed clock watermark, but
            # never refresh command_timeout or alter the staged target.
            self._last_time = now
            names, positions = self._validated_names_positions(
                names, positions, allow_subset=True, known_names=self._index
            )
            if self._fault:
                raise ValueError("guard is faulted; valid feedback is required to recover")
            if self._feedback_time is None or self._reference is None:
                raise ValueError("a complete feedback sample is required before commands")
            if now - self._feedback_time > self.feedback_timeout:
                raise ValueError("feedback is stale; command rejected")
            for name, position in zip(names, positions):
                index = self._index[name]
                if not self._in_limits(index, position):
                    raise ValueError("target for {} is outside its limits".format(name))

            new_target = list(self._target if self._target is not None else self._reference)
            for name, position in zip(names, positions):
                new_target[self._index[name]] = position
        except (ValueError, TypeError, OverflowError):
            self.rejected_count += 1
            raise

        # Commit only after every item, timestamp and feedback condition passed.
        self._target = new_target
        self._last_command_time = now
        self._timed_out = False
        self._last_time = now
        self._status = "tracking"

    def step(self, now):
        """Return complete bounded positions, their velocities, and state."""
        now = self._check_time(now)
        if self._reference is None or self._feedback_time is None:
            raise ValueError("step requires an initial complete feedback sample")
        self._last_time = now

        if self._fault:
            self._status = "fault"
            return list(self._reference), [0.0] * len(self.names), self._status

        if now - self._feedback_time > self.feedback_timeout:
            self._status = "stale_feedback"
            self._last_step_time = now
            return list(self._reference), [0.0] * len(self.names), self._status

        if self._last_command_time is None:
            self._status = "hold_timeout" if self._timed_out else "hold_idle"
            self._last_step_time = now
            return list(self._reference), [0.0] * len(self.names), self._status

        if now - self._last_command_time > self.command_timeout:
            self._target = None
            self._last_command_time = None
            self._timed_out = True
            self._status = "hold_timeout"
            self._last_step_time = now
            return list(self._reference), [0.0] * len(self.names), self._status

        raw_dt = max(0.0, now - self._last_step_time)
        dt = min(raw_dt, 0.05)
        previous = list(self._reference)
        velocities = [0.0] * len(self.names)
        if dt > 0.0:
            max_delta = self.max_velocity * dt
            for index, target in enumerate(self._target):
                delta = target - previous[index]
                if abs(delta) <= max_delta:
                    self._reference[index] = target
                else:
                    self._reference[index] = previous[index] + math.copysign(max_delta, delta)
                velocities[index] = (self._reference[index] - previous[index]) / dt

        if self._reference == self._target:
            self._status = "holding"
        else:
            self._status = "tracking"
        self._last_step_time = now
        return list(self._reference), velocities, self._status
