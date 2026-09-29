"""ROS-independent people detection from the MID-360 point cloud (PLAN M5.2).

Returns that are not floor, not configured static geometry (walls, exhibits)
and not the robot itself are grouped into clusters; a cluster tall enough to
be a person becomes a detection with its position and height.  A tracker
keeps stable ids across frames so the dialog can tell "someone left" from
"the lidar missed a frame".

Height caveat: the G1's MID-360 hangs upside down in the head (about 1.27 m
up) and sees only about 7 degrees above the horizon, so an adult's head is
usually outside the field of view.  A cluster whose top reaches that ceiling
is marked ``height_clipped`` -- its height is a lower bound -- and never
counts as a child.  Children are shorter than the lidar and are measured in
full, which is what the child/adult split needs.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math

import numpy as np

from .nav_core import StaticGeometry, wrap_angle

CHILD_MAX_HEIGHT = 1.30
# MID-360 vertical field of view is -7..+52 deg; mounted upside down that is
# +7 deg upward at most, less the ~3 deg mount pitch on one side.
LIDAR_UP_ANGLE = 0.071


@dataclass(frozen=True)
class PeopleParams:
    min_height: float = 0.15       # below: floor returns
    max_height: float = 2.3
    # The robot's own arms: a pointing gesture puts the hand ~0.5 m out in
    # front (seen in Gazebo as a 0.97 m "child"); nobody stands that close.
    self_radius: float = 0.75
    range_max: float = 6.0
    cell: float = 0.30             # clustering grid; touching cells form one cluster
    min_points: int = 4
    min_top: float = 0.70          # shorter clusters are bags, benches, plinth edges
    max_top: float = 2.05          # taller ones are door frames, hall fittings
    person_width: float = 0.70     # a wider cluster is counted as several people
    max_extent: float = 2.5        # wider still: unmodelled furniture, not people
    static_tolerance: float = 0.15
    clip_margin: float = 0.08


@dataclass(frozen=True)
class Detection:
    x: float
    y: float
    height: float
    height_clipped: bool
    group: int                     # estimated number of people in the cluster
    points: int


def height_ceiling(lidar_z: float, distance: float, up_angle: float = LIDAR_UP_ANGLE) -> float:
    """Highest world height the lidar can see at a horizontal distance."""
    return lidar_z + distance * math.tan(up_angle)


def detect_people(points: np.ndarray, static: StaticGeometry, robot_xy: tuple[float, float],
                  lidar_z: float, floor_top: float = 0.0,
                  params: PeopleParams = PeopleParams()) -> list[Detection]:
    """People-shaped clusters in world-frame points (N x 3)."""
    if points.size == 0:
        return []
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    height = points[:, 2] - floor_top
    points = points[(height >= params.min_height) & (height <= params.max_height)]
    distance = np.hypot(points[:, 0] - robot_xy[0], points[:, 1] - robot_xy[1])
    points = points[(distance > params.self_radius) & (distance <= params.range_max)]
    if points.size == 0:
        return []
    keep = np.array([not static.hit((float(x), float(y))) for x, y in points[:, :2]])
    points = points[keep]
    if points.size == 0:
        return []

    cells = np.floor(points[:, :2] / params.cell).astype(int)
    members: dict[tuple[int, int], list[int]] = {}
    for index, (cx, cy) in enumerate(cells):
        members.setdefault((int(cx), int(cy)), []).append(index)
    seen: set[tuple[int, int]] = set()
    detections = []
    for start in members:
        if start in seen:
            continue
        stack, cluster = [start], []
        seen.add(start)
        while stack:
            cx, cy = stack.pop()
            cluster.extend(members[(cx, cy)])
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    neighbour = (cx + dx, cy + dy)
                    if neighbour in members and neighbour not in seen:
                        seen.add(neighbour)
                        stack.append(neighbour)
        detection = _as_detection(points[cluster], robot_xy, lidar_z, floor_top, params)
        if detection is not None:
            detections.append(detection)
    return detections


def _as_detection(cluster: np.ndarray, robot_xy, lidar_z: float, floor_top: float,
                  params: PeopleParams) -> Detection | None:
    if len(cluster) < params.min_points:
        return None
    top_index = int(np.argmax(cluster[:, 2]))
    top = float(cluster[top_index, 2] - floor_top)
    if top < params.min_top or top > params.max_top:
        return None
    xy = cluster[:, :2]
    extent = float(np.max(np.ptp(xy, axis=0)))
    if extent > params.max_extent:
        return None
    x, y = (float(v) for v in xy.mean(axis=0))
    top_distance = math.hypot(cluster[top_index, 0] - robot_xy[0], cluster[top_index, 1] - robot_xy[1])
    clipped = cluster[top_index, 2] >= height_ceiling(lidar_z, top_distance) - params.clip_margin
    group = max(1, math.ceil(extent / params.person_width - 0.2))
    return Detection(x, y, round(top, 3), bool(clipped), group, len(cluster))


@dataclass
class Track:
    track_id: int
    x: float
    y: float
    first_seen: float
    last_seen: float
    group: int = 1
    heights: deque = field(default_factory=lambda: deque(maxlen=15))

    @property
    def height(self) -> float:
        return max(h for h, _clipped in self.heights)

    @property
    def height_clipped(self) -> bool:
        return any(clipped for _h, clipped in self.heights)

    @property
    def child(self) -> bool:
        return self.group == 1 and not self.height_clipped and self.height < CHILD_MAX_HEIGHT


class PeopleTracker:
    """Greedy nearest-neighbour tracking; a track survives short dropouts."""

    def __init__(self, gate: float = 0.8, keep_s: float = 0.6):
        self.gate = gate
        self.keep_s = keep_s
        self.tracks: list[Track] = []
        self._next_id = 1

    def update(self, detections: list[Detection], now: float) -> list[Track]:
        pairs = sorted(((math.hypot(t.x - d.x, t.y - d.y), ti, di)
                        for ti, t in enumerate(self.tracks) for di, d in enumerate(detections)))
        used_tracks, used_detections = set(), set()
        for distance, ti, di in pairs:
            if distance > self.gate or ti in used_tracks or di in used_detections:
                continue
            used_tracks.add(ti)
            used_detections.add(di)
            self._apply(self.tracks[ti], detections[di], now)
        for di, detection in enumerate(detections):
            if di not in used_detections:
                track = Track(self._next_id, detection.x, detection.y, now, now)
                self._next_id += 1
                self._apply(track, detection, now)
                self.tracks.append(track)
        self.tracks = [t for t in self.tracks if now - t.last_seen <= self.keep_s]
        return self.tracks

    @staticmethod
    def _apply(track: Track, detection: Detection, now: float) -> None:
        track.x, track.y = detection.x, detection.y
        track.last_seen = now
        track.group = detection.group
        track.heights.append((detection.height, detection.height_clipped))


def snapshot(tracks: list[Track], robot: tuple[float, float, float], now: float,
             frame: str = "odom") -> dict:
    """The /guide/people payload: everyone the tracker currently holds."""
    rx, ry, ryaw = robot
    people = []
    for t in sorted(tracks, key=lambda t: math.hypot(t.x - rx, t.y - ry)):
        people.append({
            "id": t.track_id,
            "x": round(t.x, 2), "y": round(t.y, 2),
            "distance": round(math.hypot(t.x - rx, t.y - ry), 2),
            "bearing_deg": round(math.degrees(wrap_angle(math.atan2(t.y - ry, t.x - rx) - ryaw)), 1),
            "height": round(t.height, 2),
            "height_clipped": t.height_clipped,
            "child": t.child,
            "group": t.group,
            "seen_s": round(now - t.first_seen, 1),
        })
    return {"frame": frame, "robot": [round(rx, 3), round(ry, 3), round(ryaw, 3)],
            "people": people}


def count_within(payload: dict, radius: float = 3.0) -> int:
    """Number of people within ``radius`` of the robot in a /guide/people payload."""
    return sum(p.get("group", 1) for p in payload.get("people", ()) if p["distance"] <= radius)


def face_heading(people, x: float, y: float, radius: float = 3.0) -> float | None:
    """World heading from (x, y) to the centre of the people within ``radius``."""
    near = [p for p in people if math.hypot(p["x"] - x, p["y"] - y) <= radius]
    if not near:
        return None
    weight = sum(p.get("group", 1) for p in near)
    cx = sum(p["x"] * p.get("group", 1) for p in near) / weight
    cy = sum(p["y"] * p.get("group", 1) for p in near) / weight
    return math.atan2(cy - y, cx - x)


def face_yaw(payload: dict, radius: float = 3.0) -> float | None:
    """``face_heading`` from the robot pose stored in a /guide/people payload."""
    rx, ry, _ = payload.get("robot", (0.0, 0.0, 0.0))
    return face_heading(payload.get("people", ()), rx, ry, radius)
