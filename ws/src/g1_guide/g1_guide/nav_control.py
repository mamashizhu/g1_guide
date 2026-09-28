"""ROS-independent planar navigation command and safety gate."""

from __future__ import annotations

from dataclasses import dataclass
import math


# 2026-09-25 raised from 0.35 m/s / 0.8 rad/s; the real G1 walks ~1 m/s.
MAX_LINEAR_SPEED = 0.6
MAX_ANGULAR_SPEED = 1.2
OBSTACLE_STOP_DISTANCE = 0.80
# Turning in place sweeps only the robot's own footprint (arms at home reach
# ~0.35 m), so it stays allowed while the nearest return in any direction is
# farther than this.  Forbidding it deadlocked the robot facing an obstacle.
ROTATE_CLEARANCE = 0.50
# Backing off: slow reverse, only while nothing is this close behind.
REVERSE_SPEED = 0.15
REAR_STOP_DISTANCE = 0.60
ARRIVAL_DISTANCE = 0.25
ARRIVAL_YAW_ERROR = 0.20
TURN_IN_PLACE_ERROR = 0.35
_HEADING_GAIN = 1.5


@dataclass(frozen=True)
class Pose2D:
    """A planar position and heading, with yaw expressed in radians."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Velocity:
    """A bounded planar velocity command and its controller state."""

    linear_x: float
    angular_z: float
    status: str


def _zero(status: str) -> Velocity:
    return Velocity(linear_x=0.0, angular_z=0.0, status=status)


def _wrapped_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _finite_pose(pose: Pose2D) -> bool:
    return all(math.isfinite(value) for value in (pose.x, pose.y, pose.yaw))


def _bounded_angular(error: float) -> float:
    return max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, _HEADING_GAIN * error))


def compute_command(
    pose: Pose2D,
    waypoint: Pose2D,
    goal: Pose2D,
    forward_clearance: float,
    *,
    final: bool = False,
    arrival_distance: float = ARRIVAL_DISTANCE,
    turn_clearance: float = 0.0,
) -> Velocity:
    """Track a waypoint while enforcing speed, obstacle, and arrival limits.

    ``waypoint`` guides travel along the planned path. ``goal`` supplies the
    terminal position and heading; the terminal heading is aligned only after
    entering the goal position tolerance. A short intermediate waypoint stops
    here so the caller can advance its path index; only a final waypoint may
    use the goal direction as a replacement.

    ``arrival_distance`` is the goal position tolerance.  With an estimated
    pose (AMCL) the true stop error is this plus the localization error, so
    the SLAM map mode passes a tighter value.

    ``turn_clearance`` is the nearest return in any direction.  Below
    ``OBSTACLE_STOP_DISTANCE`` forward, linear motion stops, but turning in
    place is still allowed when ``turn_clearance`` reaches
    ``ROTATE_CLEARANCE``, so a replanned path can lead the robot away.  The
    default 0.0 means unknown and keeps a blocked robot fully stopped.
    """

    try:
        if not all(_finite_pose(item) for item in (pose, waypoint, goal)):
            return _zero("invalid")
        if (
            math.isnan(forward_clearance)
            or forward_clearance == -math.inf
            or forward_clearance < 0.0
        ):
            return _zero("invalid")
        if type(final) is not bool:
            return _zero("invalid")
        if not (math.isfinite(arrival_distance) and arrival_distance > 0.0):
            return _zero("invalid")
        if math.isnan(turn_clearance) or turn_clearance < 0.0:
            return _zero("invalid")
    except (AttributeError, TypeError, ValueError, OverflowError):
        return _zero("invalid")

    try:
        command = _compute_validated(pose, waypoint, goal, forward_clearance, final,
                                     arrival_distance)
    except (TypeError, ValueError, OverflowError):
        # Finite but extreme values can still overflow during subtraction or
        # angle wrapping. Treat those derived invalid values as unsafe too.
        return _zero("invalid")

    if forward_clearance >= OBSTACLE_STOP_DISTANCE or command.status == "arrived":
        return command
    # Forward is blocked: never translate.  A pure turn is kept only when the
    # robot's surroundings are clear; aligned with an obstacle ahead is blocked.
    if (command.linear_x == 0.0 and command.angular_z != 0.0
            and command.status == "aligning" and turn_clearance >= ROTATE_CLEARANCE):
        return command
    return _zero("blocked")


def _compute_validated(
    pose: Pose2D,
    waypoint: Pose2D,
    goal: Pose2D,
    forward_clearance: float,
    final: bool,
    arrival_distance: float = ARRIVAL_DISTANCE,
) -> Velocity:
    goal_distance = math.hypot(goal.x - pose.x, goal.y - pose.y)
    goal_yaw_error = _wrapped_angle(goal.yaw - pose.yaw)

    if final and goal_distance <= arrival_distance and abs(goal_yaw_error) <= ARRIVAL_YAW_ERROR:
        return _zero("arrived")

    if final and goal_distance <= arrival_distance:
        return Velocity(0.0, _bounded_angular(goal_yaw_error), "aligning")

    waypoint_distance = math.hypot(waypoint.x - pose.x, waypoint.y - pose.y)
    if waypoint_distance <= ARRIVAL_DISTANCE:
        if not final:
            return _zero("aligning")
        target_x, target_y = goal.x, goal.y
    else:
        target_x, target_y = waypoint.x, waypoint.y

    target_dx = target_x - pose.x
    target_dy = target_y - pose.y
    if math.hypot(target_dx, target_dy) <= 1e-12:
        # Coincident path and goal points leave no useful travel bearing; use
        # the final heading only when this is the terminal waypoint.
        if final:
            return Velocity(0.0, _bounded_angular(goal_yaw_error), "aligning")
        return _zero("aligning")

    heading_error = _wrapped_angle(math.atan2(target_dy, target_dx) - pose.yaw)
    angular = _bounded_angular(heading_error)
    if abs(heading_error) > TURN_IN_PLACE_ERROR:
        return Velocity(0.0, angular, "aligning")

    # Linear speed tapers only when closing on the goal. Path waypoints are
    # ~0.2 m apart, so tapering on the waypoint would cap cruising speed near
    # 0.3 m/s; sharp corners still stop and turn in place above.
    target_distance = math.hypot(target_dx, target_dy)
    linear = min(MAX_LINEAR_SPEED, target_distance if final else goal_distance)
    return Velocity(linear, angular, "moving")


def back_off(front_clearance: float, rear_clearance: float, target_clearance: float) -> Velocity:
    """Reverse away from an obstacle until the planner has room again.

    Used after a stop left the robot closer to an obstacle than the planning
    clearance: it can neither get a plan nor turn.  Returns ``backing_up``
    while reversing, ``blocked`` once ``front_clearance`` reaches
    ``target_clearance`` (done) or the rear is not clear (cannot help).
    """
    values = (front_clearance, rear_clearance, target_clearance)
    if any(math.isnan(v) or v < 0.0 for v in values):
        return _zero("invalid")
    if front_clearance >= target_clearance or rear_clearance < REAR_STOP_DISTANCE:
        return _zero("blocked")
    return Velocity(-REVERSE_SPEED, 0.0, "backing_up")
