"""Static exhibition occupancy grid and deterministic 8-connected A* planner."""

from __future__ import annotations

import heapq
import math
from typing import Any, Iterable, Mapping, Sequence

from .scene_config import world_footprint


_SQRT2 = math.sqrt(2.0)


def _finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _point(value: Any) -> tuple[float, float] | None:
    try:
        if isinstance(value, (str, bytes)) or len(value) != 2:
            return None
        x, y = value
    except (TypeError, ValueError):
        return None
    if not _finite_number(x) or not _finite_number(y):
        return None
    return float(x), float(y)


def _box(position: Any, size: Any, yaw: Any = 0.0) -> tuple[float, float, float, float] | None:
    try:
        if len(position) < 2 or len(size) < 2:
            return None
        x, y = position[0], position[1]
    except (TypeError, IndexError):
        return None
    if not all(_finite_number(v) for v in (x, y)):
        return None
    extent = world_footprint({"size": size, "yaw": yaw})
    if extent is None:
        return None
    sx, sy = extent
    return float(x) - sx / 2, float(y) - sy / 2, float(x) + sx / 2, float(y) + sy / 2


class _RectangleIndex:
    """Uniform grid over static rectangles, expanded by the clearance radius.

    The hall is described by a few hundred baked rectangles, so a linear scan
    per query would dominate both grid construction and path smoothing.
    """

    def __init__(self, rectangles: Sequence[tuple[float, float, float, float]],
                 clearance: float):
        self._clearance = clearance
        self._clearance2 = clearance * clearance
        self._bucket = max(2.0 * clearance, 1.0)
        buckets: dict[tuple[int, int], list[tuple[float, float, float, float]]] = {}
        for rectangle in rectangles:
            left, bottom, right, top = rectangle
            for ix in range(math.floor((left - clearance) / self._bucket),
                            math.floor((right + clearance) / self._bucket) + 1):
                for iy in range(math.floor((bottom - clearance) / self._bucket),
                                math.floor((top + clearance) / self._bucket) + 1):
                    buckets.setdefault((ix, iy), []).append(rectangle)
        self._buckets = buckets

    def blocks(self, x: float, y: float) -> bool:
        key = (math.floor(x / self._bucket), math.floor(y / self._bucket))
        for left, bottom, right, top in self._buckets.get(key, ()):
            dx = max(left - x, 0.0, x - right)
            dy = max(bottom - y, 0.0, y - top)
            if dx * dx + dy * dy <= self._clearance2 + 1e-12:
                return True
        return False


class _DynamicPointIndex:
    """Spatial hash for exact-radius queries against a scan point set."""

    def __init__(self, points: tuple[tuple[float, float], ...], radius: float):
        self._radius2 = radius * radius
        # Keep the hash cell at least as large as the tolerance used by the
        # exact distance check, including the zero-clearance edge case.
        self._bucket_size = max(radius + 1e-6, 1e-6)
        buckets: dict[tuple[int, int], list[tuple[float, float]]] = {}
        for point in points:
            key = self._key(*point)
            buckets.setdefault(key, []).append(point)
        self._buckets = buckets

    def _key(self, x: float, y: float) -> tuple[int, int]:
        return math.floor(x / self._bucket_size), math.floor(y / self._bucket_size)

    def blocks(self, x: float, y: float) -> bool:
        bx, by = self._key(x, y)
        for ix in range(bx - 1, bx + 2):
            for iy in range(by - 1, by + 2):
                for px, py in self._buckets.get((ix, iy), ()):
                    if (x - px) ** 2 + (y - py) ** 2 <= self._radius2 + 1e-12:
                        return True
        return False

    def segment_blocks(self, start: tuple[float, float], end: tuple[float, float]) -> bool:
        """Exact test: is any point within the radius of the segment?"""
        if not self._buckets:
            return False
        (ax, ay), (bx_, by_) = start, end
        dx, dy = bx_ - ax, by_ - ay
        length2 = dx * dx + dy * dy
        reach = int(math.ceil(math.sqrt(length2) / 2 / self._bucket_size)) + 1
        mx, my = self._key((ax + bx_) / 2, (ay + by_) / 2)
        for ix in range(mx - reach, mx + reach + 1):
            for iy in range(my - reach, my + reach + 1):
                for px, py in self._buckets.get((ix, iy), ()):
                    t = 0.0 if length2 == 0.0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length2))
                    if (ax + t * dx - px) ** 2 + (ay + t * dy - py) ** 2 <= self._radius2 + 1e-12:
                        return True
        return False


class OccupancyGrid:
    """2-D collision grid built from the exhibition JSON-compatible mapping.

    Rectangles are expanded by the robot's circular clearance radius using the
    exact point-to-AABB distance. Room walls are represented by an inset free
    boundary with the same clearance applied.
    """

    def __init__(self, config: Mapping[str, Any], resolution: float = 0.20,
                 extra_margin: float = 0.10):
        if not isinstance(config, Mapping):
            raise ValueError("config must be a mapping")
        if not _finite_number(resolution) or resolution <= 0:
            raise ValueError("resolution must be finite and positive")
        if not _finite_number(extra_margin) or extra_margin < 0:
            raise ValueError("extra_margin must be finite and non-negative")
        room = config.get("room")
        if not isinstance(room, Mapping):
            raise ValueError("config.room must be a mapping")
        width, depth, wall = room.get("width"), room.get("depth"), room.get("wall_thickness")
        radius = config.get("robot_clearance_radius")
        if not all(_finite_number(v) for v in (width, depth, wall, radius)):
            raise ValueError("room dimensions and robot_clearance_radius must be finite numbers")
        if width <= 0 or depth <= 0 or wall < 0 or radius < 0:
            raise ValueError("room dimensions and robot_clearance_radius are out of range")

        self.resolution = float(resolution)
        self.clearance = float(radius) + float(extra_margin)
        self._xmin = -float(width) / 2 + float(wall) / 2 + self.clearance
        self._xmax = float(width) / 2 - float(wall) / 2 - self.clearance
        self._ymin = -float(depth) / 2 + float(wall) / 2 + self.clearance
        self._ymax = float(depth) / 2 - float(wall) / 2 - self.clearance
        if self._xmin >= self._xmax or self._ymin >= self._ymax:
            raise ValueError("clearance leaves no navigable room")
        self._nx = int(math.floor((self._xmax - self._xmin) / self.resolution)) + 1
        self._ny = int(math.floor((self._ymax - self._ymin) / self.resolution)) + 1
        if self._nx * self._ny > 2_000_000:
            raise ValueError("resolution creates an impractically large occupancy grid")

        # ``nav_obstacles`` carries the imported hall's walls, baked into
        # rectangles offline; ``people`` contributes only its rigid visitors,
        # since walking actors have no collision for the lidar to return.
        self._rectangles: list[tuple[float, float, float, float]] = []
        for field in ("exhibits", "obstacles", "nav_obstacles", "people"):
            objects = config.get(field, ())
            if not isinstance(objects, Sequence) or isinstance(objects, (str, bytes)):
                raise ValueError(f"config.{field} must be a sequence")
            for index, item in enumerate(objects):
                if not isinstance(item, Mapping):
                    raise ValueError(f"config.{field}[{index}] must be a mapping")
                if field == "people" and item.get("kind") != "static":
                    continue
                rectangle = _box(item.get("position"), item.get("size"), item.get("yaw", 0.0))
                if rectangle is None:
                    raise ValueError(f"config.{field}[{index}] needs finite position and positive size")
                self._rectangles.append(rectangle)
        self._index = _RectangleIndex(self._rectangles, self.clearance)

        # Static geometry never changes: A* edge checks against it are cached.
        self._static_edges: dict[tuple[tuple[int, int], tuple[int, int]], bool] = {}
        self._blocked = {
            (ix, iy)
            for ix in range(self._nx)
            for iy in range(self._ny)
            if not self._static_clear(*self._cell_to_world((ix, iy)))
        }

    def _cell_to_world(self, cell: tuple[int, int]) -> tuple[float, float]:
        return self._xmin + cell[0] * self.resolution, self._ymin + cell[1] * self.resolution

    def _world_to_cell(self, point: tuple[float, float]) -> tuple[int, int] | None:
        x, y = point
        if x < self._xmin or x > self._xmax or y < self._ymin or y > self._ymax:
            return None
        ix = int(round((x - self._xmin) / self.resolution))
        iy = int(round((y - self._ymin) / self.resolution))
        if not (0 <= ix < self._nx and 0 <= iy < self._ny):
            return None
        return ix, iy

    def _static_clear(self, x: float, y: float) -> bool:
        if not (self._xmin <= x <= self._xmax and self._ymin <= y <= self._ymax):
            return False
        return not self._index.blocks(x, y)

    @staticmethod
    def _dynamic(dynamic_points: Iterable[Any]) -> tuple[tuple[float, float], ...] | None:
        try:
            result = []
            for value in dynamic_points:
                point = _point(value)
                if point is None:
                    return None
                result.append(point)
            return tuple(result)
        except (TypeError, ValueError):
            return None

    def _clear_with_dynamic(self, x: float, y: float,
                            dynamic_index: _DynamicPointIndex) -> bool:
        if not self._static_clear(x, y):
            return False
        return not dynamic_index.blocks(x, y)

    def is_clear(self, x: float, y: float, dynamic_points: Iterable[Any] = ()) -> bool:
        if not _finite_number(x) or not _finite_number(y):
            return False
        dynamic = self._dynamic(dynamic_points)
        if dynamic is None:
            return False
        return self._clear_with_dynamic(float(x), float(y), _DynamicPointIndex(dynamic, self.clearance))

    def _segment_clear(self, start: tuple[float, float], end: tuple[float, float],
                       dynamic: tuple[tuple[float, float], ...] | _DynamicPointIndex) -> bool:
        dynamic_index = (dynamic if isinstance(dynamic, _DynamicPointIndex)
                         else _DynamicPointIndex(dynamic, self.clearance))
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        steps = max(1, int(math.ceil(distance / (self.resolution / 8))))
        return all(self._clear_with_dynamic(
            start[0] + (end[0] - start[0]) * i / steps,
            start[1] + (end[1] - start[1]) * i / steps, dynamic_index)
            for i in range(steps + 1))

    def _static_segment_clear(self, start: tuple[float, float], end: tuple[float, float]) -> bool:
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        steps = max(1, int(math.ceil(distance / (self.resolution / 8))))
        return all(self._static_clear(
            start[0] + (end[0] - start[0]) * i / steps,
            start[1] + (end[1] - start[1]) * i / steps)
            for i in range(steps + 1))

    def _edge_clear(self, current: tuple[int, int], neighbor: tuple[int, int],
                    dynamic_index: _DynamicPointIndex) -> bool:
        key = (current, neighbor) if current <= neighbor else (neighbor, current)
        static = self._static_edges.get(key)
        a, b = self._cell_to_world(current), self._cell_to_world(neighbor)
        if static is None:
            static = self._static_edges[key] = self._static_segment_clear(a, b)
        return static and not dynamic_index.segment_blocks(a, b)

    def _nearest_connectable(self, point: tuple[float, float],
                             dynamic: _DynamicPointIndex) -> tuple[int, int] | None:
        if not self._clear_with_dynamic(*point, dynamic):
            return None
        approx = self._world_to_cell(point)
        if approx is None:
            return None
        candidates = []
        for ix in range(max(0, approx[0] - 3), min(self._nx, approx[0] + 4)):
            for iy in range(max(0, approx[1] - 3), min(self._ny, approx[1] + 4)):
                cell = (ix, iy)
                world = self._cell_to_world(cell)
                if cell not in self._blocked and self._clear_with_dynamic(*world, dynamic) and self._segment_clear(point, world, dynamic):
                    candidates.append((math.dist(point, world), cell))
        return min(candidates)[1] if candidates else None

    def _line_clear(self, start: tuple[float, float], end: tuple[float, float],
                    dynamic_index: _DynamicPointIndex) -> bool:
        """Straight segment clear of static geometry (sampled every 5 cm) and
        of dynamic points (exact distance), with the full robot clearance."""
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        steps = max(1, int(math.ceil(distance / (self.resolution / 4))))
        if not all(self._static_clear(start[0] + (end[0] - start[0]) * i / steps,
                                      start[1] + (end[1] - start[1]) * i / steps)
                   for i in range(steps + 1)):
            return False
        return not dynamic_index.segment_blocks(start, end)

    def shortcut(self, path: Sequence[tuple[float, float]],
                 dynamic_points: Iterable[Any] = ()) -> list[tuple[float, float]]:
        """Drop grid vertices while the straight line to a later one stays clear.

        A* on an 8-connected grid returns a staircase of 0.2 m steps with 45°
        kinks; following it meant stopping to turn every few steps.  The
        result keeps only real corners and is exactly as clear as the input.
        """
        points = [tuple(map(float, point)) for point in path]
        dynamic = self._dynamic(dynamic_points)
        if len(points) < 3 or dynamic is None:
            return points
        dynamic_index = _DynamicPointIndex(dynamic, self.clearance)
        result = [points[0]]
        i = 0
        while i < len(points) - 1:
            j = i + 1
            while j + 1 < len(points) and self._line_clear(points[i], points[j + 1], dynamic_index):
                j += 1
            result.append(points[j])
            i = j
        return result

    def path_blocked(self, path: Sequence[tuple[float, float]], dynamic_points: Iterable[Any],
                     robot: tuple[float, float], horizon: float = 4.0) -> bool:
        """Whether dynamic points now cut the next ``horizon`` metres of path.

        Points within the clearance of ``robot`` are left out: the robot is
        already next to them and the controller's forward gate handles them;
        counting them would replan on every scan.
        """
        dynamic = self._dynamic(dynamic_points)
        if dynamic is None:
            return True
        far = tuple(p for p in dynamic
                    if math.hypot(p[0] - robot[0], p[1] - robot[1]) > self.clearance)
        dynamic_index = _DynamicPointIndex(far, self.clearance)
        remaining = horizon
        for start, end in zip(path, path[1:]):
            length = math.hypot(end[0] - start[0], end[1] - start[1])
            if length > remaining:
                scale = remaining / length
                end = (start[0] + (end[0] - start[0]) * scale, start[1] + (end[1] - start[1]) * scale)
            if dynamic_index.segment_blocks(start, end):
                return True
            remaining -= length
            if remaining <= 0.0:
                break
        return False

    def plan(self, start: Any, goal: Any,
             dynamic_points: Iterable[Any] = ()) -> list[tuple[float, float]] | None:
        start_point, goal_point = _point(start), _point(goal)
        dynamic = self._dynamic(dynamic_points)
        if start_point is None or goal_point is None or dynamic is None:
            return None
        dynamic_index = _DynamicPointIndex(dynamic, self.clearance)
        if not self._clear_with_dynamic(*start_point, dynamic_index) or not self._clear_with_dynamic(*goal_point, dynamic_index):
            return None
        if start_point == goal_point:
            return [start_point]

        source = self._nearest_connectable(start_point, dynamic_index)
        target = self._nearest_connectable(goal_point, dynamic_index)
        if source is None or target is None:
            return None

        def heuristic(cell: tuple[int, int]) -> float:
            dx, dy = abs(cell[0] - target[0]), abs(cell[1] - target[1])
            return max(dx, dy) + (_SQRT2 - 1) * min(dx, dy)

        frontier: list[tuple[float, float, tuple[int, int]]] = [(heuristic(source), 0.0, source)]
        costs = {source: 0.0}
        parents: dict[tuple[int, int], tuple[int, int]] = {}
        directions = ((1, 0), (-1, 0), (0, 1), (0, -1),
                      (1, 1), (1, -1), (-1, 1), (-1, -1))
        while frontier:
            _, cost, current = heapq.heappop(frontier)
            if cost > costs.get(current, math.inf) + 1e-12:
                continue
            if current == target:
                break
            for dx, dy in directions:
                neighbor = current[0] + dx, current[1] + dy
                if not (0 <= neighbor[0] < self._nx and 0 <= neighbor[1] < self._ny):
                    continue
                if neighbor in self._blocked:
                    continue
                if not self._edge_clear(current, neighbor, dynamic_index):
                    continue
                if dx and dy:
                    side_a, side_b = (current[0] + dx, current[1]), (current[0], current[1] + dy)
                    if side_a in self._blocked or side_b in self._blocked:
                        continue
                    if not self._clear_with_dynamic(*self._cell_to_world(side_a), dynamic_index):
                        continue
                    if not self._clear_with_dynamic(*self._cell_to_world(side_b), dynamic_index):
                        continue
                step_cost = _SQRT2 if dx and dy else 1.0
                new_cost = cost + step_cost
                if new_cost + 1e-12 >= costs.get(neighbor, math.inf):
                    continue
                costs[neighbor] = new_cost
                parents[neighbor] = current
                heapq.heappush(frontier, (new_cost + heuristic(neighbor), new_cost, neighbor))
        else:
            return None

        cells = [target]
        while cells[-1] != source:
            cells.append(parents[cells[-1]])
        cells.reverse()
        grid_points = [self._cell_to_world(cell) for cell in cells]
        if not self._segment_clear(start_point, grid_points[0], dynamic_index):
            return None
        if not self._segment_clear(grid_points[-1], goal_point, dynamic_index):
            return None
        path = [start_point]
        if grid_points[0] != start_point:
            path.append(grid_points[0])
        path.extend(grid_points[1:])
        if path[-1] != goal_point:
            path.append(goal_point)
        return path
