"""ROS-independent navigation decisions shared by the node and the offline sim.

``NavigatorCore`` holds the goal, the planned path and the recovery state.
The ROS node (``exhibition_nav``) feeds it poses and projected laser points
and publishes what it returns; ``tools/nav_sim.py`` drives the very same
object with a kinematic robot and ray-cast laser, far faster than Gazebo.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

from .nav_control import (ARRIVAL_DISTANCE, MAX_ANGULAR_SPEED, MAX_LINEAR_SPEED, Pose2D,
                          back_off, compute_command)
from .nav_grid import OccupancyGrid, _box, _RectangleIndex


REPLAN_WALL_INTERVAL_S = 8.0
# After a failed plan or a blocked stop, retry at this pace instead of running
# A* on every 10 Hz tick while waiting for the way to clear.
REPLAN_RETRY_S = 1.0
# Backing off stops once nothing in this front cone is within the planning
# clearance plus this margin.
BACK_OFF_HALF_ANGLE = 1.05
BACK_OFF_MARGIN = 0.10
# Waypoints this close to the robot count as reached.
WAYPOINT_REACHED = 0.25


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class StaticGeometry:
    """Answers whether a laser endpoint lies on configured static geometry.

    Covers the room boundary, exhibits, obstacles, the baked hall walls
    (``nav_obstacles``) and static visitors, the same set the planner grid
    treats as static.  Missing the hall walls made every wall return a
    "dynamic" point, so a robot near a wall could never plan a path.
    """

    def __init__(self, config: dict, tolerance: float = 0.25):
        room = config["room"]
        self.inner_x = (float(room["width"])-float(room["wall_thickness"]))/2
        self.inner_y = (float(room["depth"])-float(room["wall_thickness"]))/2
        self.tolerance = tolerance
        rectangles = []
        for field_name in ("exhibits", "obstacles", "nav_obstacles", "people"):
            for item in config.get(field_name, ()):
                if field_name == "people" and item.get("kind") != "static":
                    continue
                box = _box(item.get("position"), item.get("size"), item.get("yaw", 0.0))
                if box is not None:
                    rectangles.append(box)
        self._index = _RectangleIndex(rectangles, tolerance)

    def hit(self, point: tuple[float, float]) -> bool:
        x, y = point
        if (abs(abs(x)-self.inner_x) <= self.tolerance
                or abs(abs(y)-self.inner_y) <= self.tolerance):
            return True
        return self._index.blocks(x, y)


def known_static_hit(point: tuple[float, float], config: dict, tolerance: float = 0.25) -> bool:
    """Whether a laser endpoint coincides with configured static geometry."""
    return StaticGeometry(config, tolerance).hit(point)


def forward_clearance(pose, points: list[tuple[float, float]], half_angle: float = 0.52) -> float:
    candidates = []
    for x, y in points:
        dx, dy = x-pose.x, y-pose.y
        distance = math.hypot(dx, dy)
        if distance > 1e-6 and abs(wrap_angle(math.atan2(dy, dx)-pose.yaw)) <= half_angle:
            candidates.append(distance)
    return min(candidates, default=math.inf)


def nearest_distance(pose, points: list[tuple[float, float]]) -> float:
    """Nearest return in any direction: what an in-place turn could touch."""
    return min((math.hypot(x-pose.x, y-pose.y) for x, y in points), default=math.inf)


def nearest_static_clear(grid, point: tuple[float, float],
                         max_distance: float = 0.6) -> tuple[float, float] | None:
    """Closest point, searched on rings every 5 cm, outside static inflation."""
    steps = int(round(max_distance / 0.05))
    for ring in range(1, steps + 1):
        radius = ring * 0.05
        for index in range(32):
            angle = index * math.pi / 16
            candidate = (point[0] + radius*math.cos(angle), point[1] + radius*math.sin(angle))
            if grid.is_clear(*candidate):
                return candidate
    return None


def plan_with_escape(grid, start: tuple[float, float], goal: tuple[float, float],
                     dynamic_points: list[tuple[float, float]]):
    """Plan around dynamic points; if they swallow the start, plan out anyway.

    A robot that stopped within the planning clearance of an obstacle would
    otherwise never get a plan and stay blocked forever.  The fallback drops
    only returns inside that clearance around the start; the controller's
    forward gate still refuses to drive into them.

    The same holds for static geometry: path following cuts corners, so the
    robot can end up a few centimetres inside the inflation of an exhibit the
    lidar cannot see (the 0.9 m plinths sit below it).  Then the plan starts
    from the nearest clear point instead.
    """
    if not grid.is_clear(*start):
        snapped = nearest_static_clear(grid, start)
        if snapped is None:
            return None
        rest = plan_with_escape(grid, snapped, goal, dynamic_points)
        return [start, *rest] if rest else None
    plan = grid.plan(start, goal, dynamic_points=dynamic_points)
    if plan is not None or grid.is_clear(*start, dynamic_points):
        return plan
    far = [point for point in dynamic_points
           if math.hypot(point[0]-start[0], point[1]-start[1]) > grid.clearance]
    return grid.plan(start, goal, dynamic_points=far)


def _rounded(value: float, digits: int = 3):
    return round(value, digits) if math.isfinite(value) else None


@dataclass
class Command:
    """One tick's velocity and, when there is news, the status payload."""

    linear_x: float = 0.0
    angular_z: float = 0.0
    status: dict | None = field(default=None)


class NavigatorCore:
    def __init__(self, static: dict, stations: dict, arrival_distance: float = ARRIVAL_DISTANCE):
        # The follower stops below 0.80 m forward clearance; plan with
        # 0.85 m robot-centre clearance to avoid boundary deadlocks.
        self.grid = OccupancyGrid(static, extra_margin=0.30)
        self.static_geometry = StaticGeometry(static)
        self.stations = stations
        self.arrival_distance = float(arrival_distance)
        self.scan_points: list[tuple[float, float]] = []
        self.dynamic_points: list[tuple[float, float]] = []
        self.scan_origin: tuple[float, float] | None = None
        self.goal_key: str | None = None
        self.path: list[tuple[float, float]] = []
        self.path_index = 0
        self.last_plan_at = -math.inf
        self.replans = 0
        self.backing_up = False

    def payload(self, state: str, **details) -> dict:
        return {"state": state, "goal_key": self.goal_key, "replans": self.replans,
                "path_points": len(self.path), **details}

    def set_goal(self, key: str) -> dict:
        """Accept a station key, or 0/cancel/q; returns the status payload."""
        key = key.strip()
        self.path = []
        self.path_index = 0
        self.backing_up = False
        if key in ("0", "cancel", "q"):
            self.goal_key = None
            return self.payload("cancelled")
        if key not in self.stations:
            self.goal_key = None
            return self.payload("invalid_goal", requested=key)
        self.goal_key = key
        self.last_plan_at = -math.inf
        return self.payload("requested")

    def set_scan(self, points: list[tuple[float, float]],
                 origin: tuple[float, float] | None = None) -> None:
        """Laser endpoints in the global frame, floor returns already removed."""
        self.scan_points = points
        self.dynamic_points = [point for point in points if not self.static_geometry.hit(point)]
        self.scan_origin = origin

    def step(self, pose: Pose2D, now: float) -> Command:
        if self.goal_key is None:
            return Command()
        station = self.stations[self.goal_key]
        goal = Pose2D(float(station["x"]), float(station["y"]), float(station["yaw"]))
        distance_to_goal = math.hypot(pose.x-goal.x, pose.y-goal.y)
        if self.backing_up:
            command = self._back_off(pose, distance_to_goal)
            if command is not None:
                return command
        # The planner is refreshed as fresh LiDAR obstacles enter its map.
        if (not self.path and now-self.last_plan_at > REPLAN_RETRY_S) or (
                self.path and now-self.last_plan_at > REPLAN_WALL_INTERVAL_S):
            plan = plan_with_escape(self.grid, (pose.x, pose.y), (goal.x, goal.y),
                                    self.dynamic_points)
            self.last_plan_at = now
            if plan is None:
                self.path = []
                self._start_back_off(pose)
                return Command(status=self._plan_failure(pose, goal, distance_to_goal))
            self.path = plan
            self.path_index = 0
            self.replans += 1
        if not self.path:
            # Waiting out REPLAN_RETRY_S after a failed plan or a stop.
            return Command()
        while self.path_index < len(self.path)-1 and math.hypot(
                pose.x-self.path[self.path_index][0],
                pose.y-self.path[self.path_index][1]) <= WAYPOINT_REACHED:
            self.path_index += 1
        waypoint_xy = self.path[self.path_index]
        waypoint = Pose2D(float(waypoint_xy[0]), float(waypoint_xy[1]), goal.yaw)
        clearance = forward_clearance(pose, self.scan_points)
        final = self.path_index == len(self.path)-1
        velocity = compute_command(pose, waypoint, goal, clearance, final=final,
                                   arrival_distance=self.arrival_distance,
                                   turn_clearance=nearest_distance(pose, self.scan_points))
        linear = max(-MAX_LINEAR_SPEED, min(MAX_LINEAR_SPEED, float(velocity.linear_x)))
        angular = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, float(velocity.angular_z)))
        if not all(math.isfinite(v) for v in (linear, angular)):
            return Command(status=self.payload("invalid_command"))
        status = self.payload(velocity.status, distance_m=round(distance_to_goal, 3),
                              clearance_m=_rounded(clearance), waypoint_index=self.path_index)
        if velocity.status == "blocked":
            self.path = []
            self._start_back_off(pose)
        if velocity.status == "arrived":
            self.goal_key = None
            linear = angular = 0.0
        return Command(linear, angular, status)

    def _back_off_target(self) -> float:
        return self.grid.clearance + BACK_OFF_MARGIN

    def _start_back_off(self, pose: Pose2D) -> None:
        """Too close to replan or turn: reverse first (checked each step)."""
        if forward_clearance(pose, self.scan_points, BACK_OFF_HALF_ANGLE) < self._back_off_target():
            self.backing_up = True

    def _back_off(self, pose: Pose2D, distance_to_goal: float) -> Command | None:
        """One back-off step, or None once done or impossible."""
        front = forward_clearance(pose, self.scan_points, BACK_OFF_HALF_ANGLE)
        rear = forward_clearance(Pose2D(pose.x, pose.y, pose.yaw+math.pi), self.scan_points)
        command = back_off(front, rear, self._back_off_target())
        if command.status == "backing_up":
            return Command(command.linear_x, 0.0, self.payload(
                "backing_up", distance_m=round(distance_to_goal, 3),
                front_m=_rounded(front), rear_m=_rounded(rear)))
        self.backing_up = False
        self.path = []
        if front >= self._back_off_target():
            # Room regained: replan right away.  Otherwise the rear is not
            # clear either; keep the REPLAN_RETRY_S pace and wait.
            self.last_plan_at = -math.inf
        return None

    def _plan_failure(self, pose: Pose2D, goal: Pose2D, distance_to_goal: float) -> dict:
        nearest = nearest_distance(pose, self.scan_points)
        goal_nearest = min(self.scan_points, default=(math.inf, math.inf),
                           key=lambda p: math.hypot(p[0]-goal.x, p[1]-goal.y))
        return self.payload(
            "blocked", distance_m=round(distance_to_goal, 3),
            scan_points=len(self.scan_points), dynamic_points=len(self.dynamic_points),
            nearest_scan_m=_rounded(nearest),
            nearest_goal_scan_point=[_rounded(v) for v in goal_nearest],
            nearest_goal_scan_m=_rounded(math.hypot(goal_nearest[0]-goal.x,
                                                    goal_nearest[1]-goal.y)),
            scan_origin=[round(v, 3) for v in self.scan_origin]
            if self.scan_origin is not None else None,
            start_dynamic_clear=self.grid.is_clear(pose.x, pose.y, self.dynamic_points),
            goal_dynamic_clear=self.grid.is_clear(goal.x, goal.y, self.dynamic_points),
            static_path_exists=self.grid.plan((pose.x, pose.y), (goal.x, goal.y)) is not None)
