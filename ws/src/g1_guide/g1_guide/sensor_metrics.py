"""ROS-independent sensor validation, summaries, and freshness tracking."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Iterable, Optional


@dataclass(frozen=True)
class StreamHealth:
    """Current user-visible health for one subscribed stream."""

    status: str
    age_sec: Optional[float]
    header_stamp_sec: Optional[float]
    detail: str


class StreamTracker:
    """Track callback validity and monotonic receipt age for a ROS topic."""

    def __init__(self, stale_after_sec: float) -> None:
        threshold = float(stale_after_sec)
        if not math.isfinite(threshold) or threshold <= 0.0:
            raise ValueError("stale_after_sec must be a finite positive number")
        self.stale_after_sec = threshold
        self._received: Optional[float] = None
        self._stamp: Optional[float] = None
        self._invalid_detail: Optional[str] = None

    def update(
        self,
        received_monotonic: float,
        header_stamp_sec: Any = None,
        *,
        valid: bool = True,
        detail: str = "received",
    ) -> None:
        try:
            received = float(received_monotonic)
        except (TypeError, ValueError):
            received = math.nan
        if not math.isfinite(received) or received < 0.0:
            self._invalid_detail = "invalid monotonic receipt time"
            return
        self._received = received
        self._stamp = _finite_stamp(header_stamp_sec)
        if header_stamp_sec is not None and self._stamp is None:
            valid = False
            detail = "invalid/non-finite ROS header timestamp"
        self._invalid_detail = None if valid else str(detail or "invalid message")

    def mark_invalid(
        self, received_monotonic: float, detail: str, header_stamp_sec: Any = None
    ) -> None:
        self.update(
            received_monotonic,
            header_stamp_sec,
            valid=False,
            detail=detail,
        )

    def snapshot(self, now_monotonic: float) -> StreamHealth:
        if self._received is None:
            if self._invalid_detail is not None:
                return StreamHealth("invalid", None, self._stamp, self._invalid_detail)
            return StreamHealth("missing", None, None, "no message received")
        try:
            now = float(now_monotonic)
        except (TypeError, ValueError):
            now = math.nan
        age = now - self._received
        if not math.isfinite(age) or age < 0.0:
            return StreamHealth("invalid", None, self._stamp, "monotonic clock moved backwards")
        if self._invalid_detail is not None:
            return StreamHealth("invalid", age, self._stamp, self._invalid_detail)
        if age > self.stale_after_sec:
            return StreamHealth("stale", age, self._stamp, "no recent valid message")
        return StreamHealth("fresh", age, self._stamp, "valid message")


@dataclass(frozen=True)
class MetricResult:
    valid: bool
    detail: str
    values: dict[str, Any]


def dashboard_requires_display(
    snapshot_path: Optional[str], snapshot_joints_path: Optional[str]
) -> bool:
    """Whether the dashboard invocation needs an interactive display."""
    return not bool(snapshot_path or snapshot_joints_path)


def _finite_stamp(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        stamp = float(value)
    except (TypeError, ValueError):
        return None
    return stamp if math.isfinite(stamp) and stamp >= 0.0 else None


def _components(value: Any, names: Iterable[str]) -> Optional[tuple[float, ...]]:
    try:
        result = tuple(float(getattr(value, name)) for name in names)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None
    return result if all(math.isfinite(item) for item in result) else None


def summarize_laser_scan(scan: Any) -> MetricResult:
    """Validate ranges and return finite-hit points in the scan frame."""
    try:
        angle_min = float(scan.angle_min)
        angle_increment = float(scan.angle_increment)
        range_min = float(scan.range_min)
        range_max = float(scan.range_max)
        ranges = tuple(float(value) for value in scan.ranges)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return MetricResult(False, "missing or malformed LaserScan fields", {})
    if not ranges or not all(math.isfinite(v) for v in (angle_min, angle_increment, range_min, range_max)):
        return MetricResult(False, "empty ranges or non-finite scan metadata", {})
    if angle_increment == 0.0 or range_min < 0.0 or range_max <= range_min:
        return MetricResult(False, "invalid scan angle/range limits", {})

    usable: list[tuple[int, float]] = []
    finite_hits: list[tuple[int, float]] = []
    invalid_count = 0
    for index, distance in enumerate(ranges):
        if math.isinf(distance) and distance > 0:
            usable.append((index, range_max))
        elif math.isfinite(distance) and range_min <= distance <= range_max:
            usable.append((index, distance))
            finite_hits.append((index, distance))
        else:
            invalid_count += 1
    if not usable:
        return MetricResult(False, "no ranges within sensor limits", {"invalid_count": invalid_count})
    points = tuple(
        (distance * math.cos(angle_min + index * angle_increment),
         distance * math.sin(angle_min + index * angle_increment))
        for index, distance in finite_hits
    )
    nearest = min((distance for _, distance in finite_hits), default=None)
    return MetricResult(
        True,
        "valid scan",
        {
            "sample_count": len(ranges),
            "usable_count": len(usable),
            "invalid_count": invalid_count,
            "hit_count": len(finite_hits),
            "nearest_m": nearest,
            "range_max_m": range_max,
            "points_xy": points,
        },
    )


def summarize_imu(message: Any) -> MetricResult:
    try:
        orientation = _components(message.orientation, ("x", "y", "z", "w"))
        angular = _components(message.angular_velocity, ("x", "y", "z"))
        linear = _components(message.linear_acceleration, ("x", "y", "z"))
    except AttributeError:
        return MetricResult(False, "missing IMU fields", {})
    if orientation is None or angular is None or linear is None:
        return MetricResult(False, "IMU contains missing or non-finite values", {})
    covariance = getattr(message, "orientation_covariance", [0.0])
    try:
        orientation_available = not (len(covariance) > 0 and covariance[0] == -1.0)
    except (TypeError, IndexError):
        orientation_available = True
    norm = math.sqrt(sum(component * component for component in orientation))
    if orientation_available and (not math.isfinite(norm) or norm < 1e-9):
        return MetricResult(False, "IMU orientation quaternion has zero norm", {})
    return MetricResult(
        True,
        "valid IMU",
        {
            "orientation_xyzw": orientation if orientation_available else None,
            "angular_velocity_xyz": angular,
            "linear_acceleration_xyz": linear,
        },
    )


def summarize_odometry(message: Any) -> MetricResult:
    try:
        pose = message.pose.pose
        twist = message.twist.twist
        position = _components(pose.position, ("x", "y", "z"))
        orientation = _components(pose.orientation, ("x", "y", "z", "w"))
        linear = _components(twist.linear, ("x", "y", "z"))
        angular = _components(twist.angular, ("x", "y", "z"))
    except AttributeError:
        return MetricResult(False, "missing Odometry fields", {})
    if any(value is None for value in (position, orientation, linear, angular)):
        return MetricResult(False, "Odometry contains missing or non-finite values", {})
    assert position is not None and orientation is not None
    assert linear is not None and angular is not None
    q_norm = math.sqrt(sum(component * component for component in orientation))
    if q_norm < 1e-9:
        return MetricResult(False, "Odometry orientation quaternion has zero norm", {})
    return MetricResult(
        True,
        "valid odometry",
        {"position_xyz": position, "orientation_xyzw": orientation,
         "linear_velocity_xyz": linear, "angular_velocity_xyz": angular},
    )


def summarize_joint_state(message: Any, display_limit: int = 8) -> MetricResult:
    try:
        names = tuple(str(name) for name in message.name)
        positions = tuple(float(value) for value in message.position)
        velocities = tuple(float(value) for value in message.velocity)
        efforts = tuple(float(value) for value in message.effort)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return MetricResult(False, "missing or malformed JointState fields", {})
    if not names or len(names) != len(positions):
        return MetricResult(False, "joint names/positions are empty or mismatched", {})
    if len(set(names)) != len(names) or any(not name for name in names):
        return MetricResult(False, "joint names are empty or duplicated", {})
    if not all(math.isfinite(value) for value in positions):
        return MetricResult(False, "joint positions contain non-finite values", {})
    if velocities and (len(velocities) != len(names) or not all(math.isfinite(v) for v in velocities)):
        return MetricResult(False, "joint velocities are mismatched or non-finite", {})
    if efforts and (len(efforts) != len(names) or not all(math.isfinite(v) for v in efforts)):
        return MetricResult(False, "joint efforts are mismatched or non-finite", {})
    shown = min(max(0, int(display_limit)), len(names))
    return MetricResult(
        True,
        "valid joint state",
        {
            "count": len(names),
            "position_min_rad": min(positions),
            "position_max_rad": max(positions),
            "joints": tuple((names[i], positions[i]) for i in range(shown)),
            "remaining_count": len(names) - shown,
        },
    )


def dashboard_vector_rows(label: str, values: Iterable[float]) -> tuple[str, str]:
    """Return short ASCII label/value rows for a fixed-width dashboard column."""
    ascii_label = str(label).encode("ascii", "replace").decode("ascii")
    vector = tuple(float(value) for value in values)
    return ascii_label, "[" + ", ".join(f"{value:+.3f}" for value in vector) + "]"


def dashboard_joint_line(name: str, position: float, max_name_chars: int = 24) -> str:
    """Format a bounded ASCII joint row so long ROS names cannot cross columns."""
    safe_name = str(name).encode("ascii", "replace").decode("ascii")
    limit = max(4, int(max_name_chars))
    if len(safe_name) > limit:
        safe_name = safe_name[: limit - 3] + "..."
    return f"{safe_name}: {float(position):+.3f} rad"
