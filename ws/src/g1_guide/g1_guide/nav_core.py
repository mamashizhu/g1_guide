"""ROS-independent navigation decisions shared by the node and the offline sim.

``NavigatorCore`` holds the goal, the planned path and the recovery state.
The ROS node (``exhibition_nav``) feeds it poses and projected laser points
and publishes what it returns; ``tools/nav_sim.py`` drives the very same
object with a kinematic robot and ray-cast laser, far faster than Gazebo.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

from .nav_control import (ARRIVAL_DISTANCE, MAX_ANGULAR_SPEED, MAX_LINEAR_SPEED,
                          ROTATE_CLEARANCE, Pose2D, back_off, compute_command)
from .follow_core import (FOLLOW_DISTANCE, FRESH_S, HOLD_BAND, LOST_S, VISION_FRESH_S,
                          PersonFollower, vision_persons)
from .nav_grid import OccupancyGrid, _box, _DynamicPointIndex, _RectangleIndex


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
# When fresh laser points cut the path ahead, replan early (at most this
# often) instead of driving on until the forward gate forces a stop.
REPLAN_BLOCKED_MIN_S = 1.0
# Obstacle memory: dynamic points stay in the planner for this long after
# they were last seen, unless a later beam passes through their spot.
# Without it the robot forgot an obstacle as soon as it turned away (or it
# fell out of view), replanned back towards it, saw it again, and looped.
MEMORY_S = 15.0
MEMORY_CELL = 0.10
MEMORY_SELF_RADIUS = 0.60   # own arms and shoulders, never remembered
MEMORY_CLEAR_MARGIN = 0.30  # a beam must reach this far past a point to clear it
MEMORY_OPEN_RANGE = 7.5     # a beam with no return saw free space this far
# 讲解模式「面向观众」(PLAN M5.5): after arriving, turn in place to the centre
# of the people within FACE_RADIUS, as reported by /guide/people.
PRESENT_MODES = ("point", "face")
FACE_RADIUS = 3.0
FACE_YAW_TOLERANCE = 0.07
FACE_TIMEOUT_S = 8.0
PEOPLE_STALE_S = 1.5
# 跟随 (goal key ``follow`` / ``follow:<lidar id>``), target from follow_core.
FOLLOW_START_S = 3.0        # no target found within this: ``no_target``
FOLLOW_RESUME = 0.2         # holding: walk again once the person is this much past the band
FOLLOW_REPLAN_MOVE = 0.3    # replan once the follow goal moved this far
# Visitors walk 0.8-1.2 m/s and the touring 0.6 m/s fell ever further behind
# (offline: 9 m gap after 40 s); the real G1 walks ~1 m/s.
FOLLOW_MAX_SPEED = 0.8
FOLLOW_REPLAN_MIN_S = 0.5
FOLLOW_ARRIVAL = 0.30
FOLLOW_FACE_ERROR = 0.15    # holding: turn to the person beyond this heading error
FOLLOW_SELF_CLEAR = 0.6     # dynamic points this close to the target are its own body
FOLLOW_STALE_S = 0.5        # remembered, not in the latest scans (5 Hz): may be the target's old body
SEARCH_STANDOFF = 0.6       # searching: stop this short of where the target was last seen
SEARCH_LOOK_S = 2.0         # then look the way they were heading this long ...
SEARCH_SWEEP_STEP = 0.4     # ... then sweep round (heading error -> 0.6 rad/s)
FOLLOW_STATES = ("following", "follow_waiting", "follow_searching", "follow_starting",
                 "follow_blocked")


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class StaticGeometry:
    """Answers whether a laser endpoint lies on configured static geometry.

    Covers the room boundary, exhibits, obstacles, the baked hall walls
    (``nav_obstacles``) and static visitors, the same set the planner grid
    treats as static.  Missing the hall walls made every wall return a
    "dynamic" point, so a robot near a wall could never plan a path.

    ``include_people=False`` leaves the static visitors out, for people
    detection, which must see them.
    """

    def __init__(self, config: dict, tolerance: float = 0.25, include_people: bool = True):
        room = config["room"]
        self.inner_x = (float(room["width"])-float(room["wall_thickness"]))/2
        self.inner_y = (float(room["depth"])-float(room["wall_thickness"]))/2
        self.tolerance = tolerance
        rectangles = []
        for field_name in ("exhibits", "obstacles", "nav_obstacles", "people"):
            for item in config.get(field_name, ()):
                if field_name == "people" and (not include_people or item.get("kind") != "static"):
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
        self.memory: dict[tuple[int, int], tuple[float, float, float]] = {}
        self.scan_origin: tuple[float, float] | None = None
        self.goal_key: str | None = None
        self.path: list[tuple[float, float]] = []
        self.path_index = 0
        self.last_plan_at = -math.inf
        self.replans = 0
        self.backing_up = False
        self.present_mode = "point"
        self.people: list[dict] = []
        self.people_at = -math.inf
        self.facing_yaw: float | None = None
        self.facing_since = 0.0
        self.follower = PersonFollower()
        self.follow_request: int | None = None
        self.follow_since: float | None = None
        self.follow_goal: tuple[float, float] | None = None
        self.follow_holding = False
        self.follow_plan_failed = False
        self.search_look_since: float | None = None
        self.vision = []
        self.vision_at = -math.inf

    def set_present_mode(self, mode: str) -> bool:
        if mode not in PRESENT_MODES:
            return False
        self.present_mode = mode
        return True

    def set_people(self, people: list[dict], now: float) -> None:
        """People (dicts with world ``x``, ``y``) from the latest /guide/people."""
        self.people = list(people)
        self.people_at = now
        if self.goal_key == "follow":
            self.follower.observe_people(self.people, now)

    def set_vision(self, persons: list[dict], pose_at_image: Pose2D, now: float) -> None:
        """YOLO persons from /guide/vision with the robot pose at the frame's stamp."""
        self.vision = vision_persons(persons, pose_at_image)
        self.vision_at = now
        if self.goal_key == "follow":
            self.follower.observe_vision(self.vision, now, self.scan_points)

    def people_near(self, pose: Pose2D, now: float) -> int | None:
        """Head count within FACE_RADIUS; None without fresh people data."""
        if now - self.people_at > PEOPLE_STALE_S:
            return None
        return sum(p.get("group", 1) for p in self.people
                   if math.hypot(p["x"] - pose.x, p["y"] - pose.y) <= FACE_RADIUS)

    def payload(self, state: str, **details) -> dict:
        return {"state": state, "goal_key": self.goal_key, "replans": self.replans,
                "path_points": len(self.path), **details}

    def set_goal(self, key: str) -> dict:
        """Accept a station key, ``face`` (turn to the people here), or
        0/cancel/q; returns the status payload."""
        key = key.strip()
        self.path = []
        self.path_index = 0
        self.backing_up = False
        self.facing_yaw = None
        self.follower.reset()
        if key == "follow" or key.startswith("follow:"):
            try:
                self.follow_request = int(key[7:]) if key[7:] else None
            except ValueError:
                self.goal_key = None
                return self.payload("invalid_goal", requested=key)
            self.goal_key = "follow"
            self.follow_since = None
            self.follow_goal = None
            self.follow_holding = False
            self.follow_plan_failed = False
            self.search_look_since = None
            self.last_plan_at = -math.inf
            return self.payload("requested", follow_id=self.follow_request)
        if key == "face":
            self.goal_key = key
            return self.payload("facing")
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
                 origin: tuple[float, float] | None = None, now: float | None = None) -> None:
        """Laser endpoints in the global frame, floor returns already removed.

        With ``origin`` and ``now``, dynamic points are also remembered (see
        MEMORY_S); ``dynamic_points`` is then the remembered set.
        """
        self.scan_points = points
        current = [point for point in points if not self.static_geometry.hit(point)]
        self.scan_origin = origin
        if origin is None or now is None:
            self.dynamic_points = current
            return
        self._update_memory(points, current, origin, now)
        self.dynamic_points = [(x, y) for x, y, _ in self.memory.values()]

    def _update_memory(self, points, current, origin, now) -> None:
        ox, oy = origin
        nearest = [math.inf] * 360
        for x, y in points:
            index = int(math.degrees(math.atan2(y-oy, x-ox))) % 360
            nearest[index] = min(nearest[index], math.hypot(x-ox, y-oy))
        kept = {}
        for key, (x, y, seen) in self.memory.items():
            if now - seen > MEMORY_S:
                continue
            distance = math.hypot(x-ox, y-oy)
            beam = nearest[int(math.degrees(math.atan2(y-oy, x-ox))) % 360]
            if math.isinf(beam):
                if distance < MEMORY_OPEN_RANGE:
                    continue
            elif beam > distance + MEMORY_CLEAR_MARGIN:
                continue
            kept[key] = (x, y, seen)
        for x, y in current:
            if math.hypot(x-ox, y-oy) <= MEMORY_SELF_RADIUS:
                continue
            kept[(round(x / MEMORY_CELL), round(y / MEMORY_CELL))] = (x, y, now)
        self.memory = kept

    def step(self, pose: Pose2D, now: float) -> Command:
        if self.goal_key is None:
            return Command()
        if self.goal_key == "follow":
            return self._follow_step(pose, now)
        if self.goal_key == "face" or self.facing_yaw is not None:
            return self._face_step(pose, now)
        station = self.stations[self.goal_key]
        goal = Pose2D(float(station["x"]), float(station["y"]), float(station["yaw"]))
        distance_to_goal = math.hypot(pose.x-goal.x, pose.y-goal.y)
        if self.backing_up:
            command = self._back_off(pose, distance_to_goal)
            if command is not None:
                return command
        # The planner is refreshed as fresh LiDAR obstacles enter its map.
        if (not self.path and now-self.last_plan_at > REPLAN_RETRY_S) or (
                self.path and now-self.last_plan_at > REPLAN_WALL_INTERVAL_S) or (
                self.path and now-self.last_plan_at > REPLAN_BLOCKED_MIN_S
                and self.grid.path_blocked([(pose.x, pose.y), *self.path[self.path_index:]],
                                           self.dynamic_points, (pose.x, pose.y))):
            plan = plan_with_escape(self.grid, (pose.x, pose.y), (goal.x, goal.y),
                                    self.dynamic_points)
            self.last_plan_at = now
            if plan is None:
                self.path = []
                self._start_back_off(pose)
                return Command(status=self._plan_failure(pose, goal, distance_to_goal))
            self.path = self.grid.shortcut(plan, self.dynamic_points)
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
            if self.present_mode == "face" and self._start_facing(pose, now):
                return Command(status=self.payload("facing", distance_m=round(distance_to_goal, 3)))
            return Command(status=self._finish(pose, now, "arrived", faced=False,
                                               distance_m=round(distance_to_goal, 3)))
        return Command(linear, angular, status)

    # ---- 跟随 ----
    def _follow_step(self, pose: Pose2D, now: float) -> Command:
        """Keep FOLLOW_DISTANCE from the target; search where it was last seen;
        give up after LOST_S.  Never ends on its own while the target is seen."""
        f = self.follower
        if self.follow_since is None:
            self.follow_since = now
        if not f.active:
            people = self.people if now - self.people_at <= PEOPLE_STALE_S else None
            vision = self.vision if now - self.vision_at <= VISION_FRESH_S else None
            if not f.start(pose, now, people, vision, self.scan_points, self.follow_request):
                if now - self.follow_since > FOLLOW_START_S:
                    return Command(status=self._finish_follow("no_target", people=people is not None,
                                                              vision=vision is not None))
                return Command(status=self.payload("follow_starting"))
        age = f.age(now)
        if age > LOST_S:
            return Command(status=self._finish_follow("follow_lost", unseen_s=round(age, 1)))
        tx, ty = f.predict(now)
        distance = math.hypot(tx - pose.x, ty - pose.y)
        details = dict(f.summary(), distance_m=round(distance, 2), unseen_s=round(age, 1))
        if age > FRESH_S:
            # Searching: go to where the target was last seen and look around.
            lx, ly = f.x, f.y
            goal = None
            if math.hypot(lx - pose.x, ly - pose.y) > SEARCH_STANDOFF + FOLLOW_ARRIVAL:
                goal = self._follow_goal(pose, lx, ly, SEARCH_STANDOFF, now)
            if goal is not None:
                self.search_look_since = None
                return self._follow_drive(pose, goal, (lx, ly), now, "follow_searching", details)
            # There (or cannot get closer): look where they were heading, then
            # sweep all round -- the camera sees only 70 degrees.
            if self.search_look_since is None:
                self.search_look_since = now
            if now - self.search_look_since > SEARCH_LOOK_S:
                return self._follow_turn(pose, pose.yaw + SEARCH_SWEEP_STEP, "follow_searching", details)
            heading = math.atan2(f.vy, f.vx) if math.hypot(f.vx, f.vy) > 0.2 else math.atan2(
                ly - pose.y, lx - pose.x)
            return self._follow_turn(pose, heading, "follow_searching", details)
        self.search_look_since = None
        band = FOLLOW_DISTANCE + HOLD_BAND + (FOLLOW_RESUME if self.follow_holding else 0.0)
        if distance <= band:
            self.follow_holding = True
            return self._follow_turn(pose, math.atan2(ty - pose.y, tx - pose.x),
                                     "follow_waiting", details)
        self.follow_holding = False
        goal = self._follow_goal(pose, tx, ty, FOLLOW_DISTANCE, now)
        if goal is None:
            return self._follow_turn(pose, math.atan2(ty - pose.y, tx - pose.x),
                                     "follow_blocked", details)
        return self._follow_drive(pose, goal, (tx, ty), now, "following", details)

    def _follow_obstacles(self, now: float) -> list[tuple[float, float]]:
        """Dynamic points minus the target's body and the remembered returns of
        it along the floor it walked on.  A return seen in the current scans is
        kept even beside that trail: someone standing there is real."""
        f = self.follower
        if self.memory:
            points = [(x, y) for x, y, seen in self.memory.values()
                      if now - seen <= FOLLOW_STALE_S or not f.near_trail(x, y, now)]
        else:
            points = self.dynamic_points
        return [(x, y) for x, y in points if math.hypot(x - f.x, y - f.y) > FOLLOW_SELF_CLEAR]

    def _follow_goal(self, pose: Pose2D, tx: float, ty: float, standoff: float,
                     now: float) -> tuple[float, float] | None:
        """First clear point on the line from the target back towards the robot,
        ``standoff`` or farther from the target (a wall beside the person can
        swallow the plain standoff point)."""
        distance = math.hypot(pose.x - tx, pose.y - ty)
        if distance < 1e-6:
            return None
        ux, uy = (pose.x - tx) / distance, (pose.y - ty) / distance
        index = _DynamicPointIndex(tuple(self._follow_obstacles(now)), self.grid.clearance)
        s = standoff
        while s <= max(standoff, distance) + 1e-9 and s <= standoff + 2.0:
            candidate = (tx + ux * s, ty + uy * s)
            if self.grid._clear_with_dynamic(*candidate, index):
                return candidate
            s += 0.1
        return nearest_static_clear(self.grid, (tx + ux * standoff, ty + uy * standoff), 1.0)

    def _follow_turn(self, pose: Pose2D, heading: float, state: str, details: dict) -> Command:
        """Stand still, turned towards ``heading`` (when turning is safe)."""
        self.path = []
        self.backing_up = False
        error = wrap_angle(heading - pose.yaw)
        angular = 0.0
        if abs(error) > FOLLOW_FACE_ERROR and nearest_distance(pose, self.scan_points) >= ROTATE_CLEARANCE:
            angular = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, 1.5 * error))
        return Command(0.0, angular, self.payload(state, **details))

    def _follow_drive(self, pose: Pose2D, goal: tuple[float, float], target: tuple[float, float],
                      now: float, state: str, details: dict) -> Command:
        if self.backing_up:
            command = self._back_off(pose, math.hypot(goal[0] - pose.x, goal[1] - pose.y))
            if command is not None:
                return command
        obstacles = self._follow_obstacles(now)
        moved = (self.follow_goal is None
                 or math.hypot(goal[0] - self.follow_goal[0], goal[1] - self.follow_goal[1]) > FOLLOW_REPLAN_MOVE)
        since = now - self.last_plan_at
        if (not self.path and since > (REPLAN_RETRY_S if self.follow_plan_failed else 0.0)) or (
                self.path and moved and since > FOLLOW_REPLAN_MIN_S) or (
                self.path and since > REPLAN_WALL_INTERVAL_S) or (
                self.path and since > REPLAN_BLOCKED_MIN_S and self.grid.path_blocked(
                    [(pose.x, pose.y), *self.path[self.path_index:]], obstacles, (pose.x, pose.y))):
            plan = plan_with_escape(self.grid, (pose.x, pose.y), goal, obstacles)
            self.last_plan_at = now
            self.follow_goal = goal
            self.follow_plan_failed = plan is None
            if plan is None:
                self.path = []
                self._start_back_off(pose)
                return Command(status=self.payload("follow_blocked", **details))
            self.path = self.grid.shortcut(plan, obstacles)
            self.path_index = 0
            self.replans += 1
        if not self.path:
            return Command(status=self.payload("follow_blocked", **details))
        while self.path_index < len(self.path) - 1 and math.hypot(
                pose.x - self.path[self.path_index][0],
                pose.y - self.path[self.path_index][1]) <= WAYPOINT_REACHED:
            self.path_index += 1
        end = self.path[-1]
        face = math.atan2(target[1] - end[1], target[0] - end[0])
        goal_pose = Pose2D(float(end[0]), float(end[1]), face)
        waypoint = Pose2D(float(self.path[self.path_index][0]), float(self.path[self.path_index][1]), face)
        clearance = forward_clearance(pose, self.scan_points)
        velocity = compute_command(pose, waypoint, goal_pose, clearance,
                                   final=self.path_index == len(self.path) - 1,
                                   arrival_distance=FOLLOW_ARRIVAL,
                                   turn_clearance=nearest_distance(pose, self.scan_points),
                                   max_linear=FOLLOW_MAX_SPEED)
        if velocity.status == "arrived":
            # The end of a stale path, not necessarily the person: plan again
            # on the next tick (the hold band decides about waiting).
            self.path = []
            self.follow_goal = None
            return self._follow_turn(pose, math.atan2(target[1] - pose.y, target[0] - pose.x),
                                     state, details)
        if velocity.status == "blocked":
            self.path = []
            self.follow_plan_failed = True
            self._start_back_off(pose)
            return Command(status=self.payload("follow_blocked", clearance_m=_rounded(clearance), **details))
        linear = max(-FOLLOW_MAX_SPEED, min(FOLLOW_MAX_SPEED, float(velocity.linear_x)))
        angular = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, float(velocity.angular_z)))
        if not all(math.isfinite(v) for v in (linear, angular)):
            return Command(status=self.payload("invalid_command"))
        return Command(linear, angular, self.payload(state, motion=velocity.status, **details))

    def _finish_follow(self, state: str, **details) -> dict:
        payload = self.payload(state, **self.follower.summary(), **details)
        self.goal_key = None
        self.path = []
        self.backing_up = False
        self.follower.reset()
        return payload

    def _start_facing(self, pose: Pose2D, now: float) -> bool:
        from .people_core import face_heading

        if now - self.people_at > PEOPLE_STALE_S:
            return False
        heading = face_heading(self.people, pose.x, pose.y, FACE_RADIUS)
        if heading is None:
            return False
        self.facing_yaw = heading
        self.facing_since = now
        return True

    def _face_step(self, pose: Pose2D, now: float) -> Command:
        """Turn in place towards the people; ends in ``arrived`` (or ``faced``
        for the bare ``face`` command) whether or not anyone was there."""
        done = "faced" if self.goal_key == "face" else "arrived"
        if self.facing_yaw is None and not self._start_facing(pose, now):
            return Command(status=self._finish(pose, now, done, faced=False))
        error = wrap_angle(self.facing_yaw - pose.yaw)
        if abs(error) <= FACE_YAW_TOLERANCE:
            return Command(status=self._finish(pose, now, done, faced=True))
        if (now - self.facing_since > FACE_TIMEOUT_S
                or nearest_distance(pose, self.scan_points) < ROTATE_CLEARANCE):
            return Command(status=self._finish(pose, now, done, faced=False))
        angular = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, 1.5 * error))
        return Command(0.0, angular, self.payload("facing", yaw_error_deg=round(math.degrees(error), 1)))

    def _finish(self, pose: Pose2D, now: float, state: str, **details) -> dict:
        payload = self.payload(state, present_mode=self.present_mode,
                               people_near=self.people_near(pose, now), **details)
        self.goal_key = None
        self.facing_yaw = None
        return payload

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
