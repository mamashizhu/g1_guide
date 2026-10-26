"""ROS-independent target estimate for following a visitor (YOLO + lidar).

``PersonFollower`` keeps the world position of the one person being followed.
Two sources feed it:

* lidar people (/guide/people, ``people_core``): world x/y with a track id at
  ~5 Hz, all around the robot.  This is the position the robot steers by.
* YOLO persons (/guide/vision, ``vision/vision_core.py``): a bearing and a
  ByteTrack id per person in the front camera.  No distance, 70 deg wide.

The camera decides *who*: at the start, the person in front of the camera; and
when the lidar track breaks (occlusion, two people crossing, the greedy lidar
tracker swapping ids), the YOLO id says which of the lidar people is still the
target.  With the simple 2-D lidar (no /guide/people) the YOLO bearing plus
the laser range along it gives the position instead.

Every bearing is world-frame here: the caller passes the robot pose at the
camera frame's stamp, so a robot turning during the detection latency does not
smear the target sideways.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

FOLLOW_DISTANCE = 1.2      # stand this far from the person
HOLD_BAND = 0.3            # within FOLLOW_DISTANCE + this: stand still and face them
START_RADIUS = 3.0         # who can be picked by a bare "follow"
START_HALF_ANGLE = math.radians(50)
MATCH_BEARING = math.radians(8)    # YOLO and lidar person this close in bearing = same person
MERGED_MATCH_BEARING = math.radians(25)   # ... for a lidar cluster of several people
SWAP_BEARING = math.radians(15)    # tracked lidar id this far off the YOLO target = id swapped
SWAP_LATERAL = 0.4         # ... and this far sideways (m): close by, a step is many degrees
SWAP_VISION_S = 0.35       # only a camera fix this fresh may overrule the lidar id
TRACK_GATE = 1.0           # a new lidar id within this of the prediction may be the target
SEARCH_GATE = 2.0          # ... after the target has been out of sight for a while
FRESH_S = 0.8              # a measurement this old still counts as "seen"
VISION_FRESH_S = 1.0
PEOPLE_FRESH_S = 1.5       # older lidar people data: assume the simple 2-D lidar
LOST_S = 15.0              # unseen this long: give up (a full search sweep takes ~10 s)
WALK_MAX = 1.2             # m/s: how far a target unseen for t s can have walked
REACQUIRE_MAX = 8.0        # ... capped here
MERGE_WAIT_S = 1.5         # after a merge, wait this long for both people to show up again
SPLIT_APART = 0.7          # ... as two lidar people at least this far apart
YOLO_KEEP_S = 1.0          # the target's YOLO id missing this long: another id may take over
MAX_SPEED = 1.8            # velocity estimate clamp (m/s)
VELOCITY_WINDOW_S = 1.0    # velocity = displacement over about this long (single
                           # fixes from bearing + laser range are noisy)
PREDICT_MAX_S = 1.5
TRAIL_S = 15.0             # remembered target positions (same span as the obstacle memory)
TRAIL_CELL = 0.25          # trail kept on this grid; "near" = the 3x3 cells around a point
SCAN_BEARING = math.radians(3)     # laser returns this close to a YOLO bearing measure its range
BODY_RADIUS = 0.15         # laser hits the front of the body; the centre is this much farther
SCAN_RANGE = (0.3, 7.0)


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class VisionPerson:
    """One YOLO person, already turned into a world bearing from ``origin``."""

    bearing: float                     # world frame, rad
    origin: tuple[float, float]        # robot position at the frame's stamp
    track_id: int | None = None
    conf: float = 0.0


def vision_persons(persons: list[dict], pose_at_image) -> list[VisionPerson]:
    """/guide/vision ``persons`` (robot-frame ``bearing_deg``, left positive)."""
    origin = (float(pose_at_image.x), float(pose_at_image.y))
    out = []
    for person in persons:
        try:
            bearing = wrap(float(pose_at_image.yaw) + math.radians(float(person["bearing_deg"])))
        except (KeyError, TypeError, ValueError):
            continue
        track = person.get("track_id")
        out.append(VisionPerson(bearing, origin, int(track) if track is not None else None,
                                float(person.get("conf", 0.0))))
    return out


def bearing_from(origin: tuple[float, float], x: float, y: float) -> float:
    return math.atan2(y - origin[1], x - origin[0])


def scan_range(points, origin: tuple[float, float], bearing: float) -> float | None:
    """Nearest laser return along a world bearing (simple-lidar mode)."""
    best = None
    for x, y in points:
        distance = math.hypot(x - origin[0], y - origin[1])
        if not SCAN_RANGE[0] <= distance <= SCAN_RANGE[1]:
            continue
        if abs(wrap(bearing_from(origin, x, y) - bearing)) <= SCAN_BEARING:
            best = distance if best is None else min(best, distance)
    return best


class PersonFollower:
    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self.active = False
        self.lidar_id: int | None = None
        self.yolo_id: int | None = None
        self.x = self.y = 0.0
        self.vx = self.vy = 0.0
        self.seen_at = -math.inf
        self.source = ""
        self.people_at = -math.inf
        self.lidar_seen_at = -math.inf
        self.yolo_seen_at = -math.inf
        self.vision_target: VisionPerson | None = None
        self.vision_at = -math.inf
        self.trail: dict[tuple[int, int], float] = {}
        self.recent: deque = deque()
        # (t, x, y, vx, vy) when the target's lidar cluster merged with someone else's
        self.merge_from: tuple[float, float, float, float, float] | None = None
        self.merged_at = -math.inf
        self.swaps = 0          # lidar id corrected by YOLO
        self.reacquired = 0     # target picked up again under a new lidar id

    # ---- state ----
    def age(self, now: float) -> float:
        return now - self.seen_at

    def predict(self, now: float) -> tuple[float, float]:
        dt = max(0.0, min(now - self.seen_at, PREDICT_MAX_S))
        return self.x + self.vx * dt, self.y + self.vy * dt

    def summary(self) -> dict:
        return {"target_id": self.lidar_id, "yolo_id": self.yolo_id, "source": self.source,
                "target": [round(self.x, 2), round(self.y, 2)],
                "id_swaps": self.swaps, "reacquired": self.reacquired}

    def near_trail(self, x: float, y: float, now: float) -> bool:
        """Where the target walked recently (within ~1.5 cells): free floor,
        whatever the obstacle memory still holds of the target's own body."""
        cx, cy = round(x / TRAIL_CELL), round(y / TRAIL_CELL)
        return any(now - self.trail.get((cx + dx, cy + dy), -math.inf) <= TRAIL_S
                   for dx in (-1, 0, 1) for dy in (-1, 0, 1))

    def _update(self, x: float, y: float, now: float, source: str,
                keep_velocity: bool = False) -> None:
        if keep_velocity:
            self.recent.clear()
        elif not self.active or now - self.seen_at >= 2.0:
            self.recent.clear()
        self.recent.append((now, x, y))
        while len(self.recent) > 2 and now - self.recent[1][0] >= VELOCITY_WINDOW_S:
            self.recent.popleft()
        t0, x0, y0 = self.recent[0]
        if now - t0 >= VELOCITY_WINDOW_S / 2:
            self.vx, self.vy = (x - x0) / (now - t0), (y - y0) / (now - t0)
            speed = math.hypot(self.vx, self.vy)
            if speed > MAX_SPEED:
                self.vx, self.vy = self.vx * MAX_SPEED / speed, self.vy * MAX_SPEED / speed
        elif len(self.recent) == 1 and not keep_velocity:
            self.vx = self.vy = 0.0
        self.x, self.y, self.seen_at, self.source = x, y, now, source
        if source != "vision":
            self.lidar_seen_at = now
        self.active = True
        if source != "predicted":   # only where someone was seen is walked floor
            self.trail[(round(x / TRAIL_CELL), round(y / TRAIL_CELL))] = now
        if len(self.trail) > 400:
            self.trail = {k: t for k, t in self.trail.items() if now - t <= TRAIL_S}

    def _vision_fresh(self, now: float) -> VisionPerson | None:
        if self.vision_target is not None and now - self.vision_at <= VISION_FRESH_S:
            return self.vision_target
        return None

    # ---- start ----
    def start(self, pose, now: float, people: list[dict] | None, vision: list[VisionPerson] | None,
              scan_points=(), person_id: int | None = None) -> bool:
        """Pick the target; ``people`` None = no lidar people data (simple lidar)."""
        self.reset()
        self.people_at = now if people is not None else -math.inf
        if person_id is not None and people is None:
            return False   # that visitor is a /guide/people id: wait for the lidar
        if people is not None and person_id is not None:
            match = [p for p in people if p.get("id") == person_id]
            if not match:
                return False
            person = match[0]
            self.lidar_id = person_id
            seen = self._match_vision(vision or (), person["x"], person["y"])
            if seen is not None:
                self._set_vision(seen, now)
            self._update(person["x"], person["y"], now, "lidar")
            return True
        if people is not None:
            front = []
            for p in people:
                distance = math.hypot(p["x"] - pose.x, p["y"] - pose.y)
                relative = abs(wrap(bearing_from((pose.x, pose.y), p["x"], p["y"]) - pose.yaw))
                if distance <= START_RADIUS and relative <= START_HALF_ANGLE:
                    front.append((relative, distance, p))
            if not front:
                return False
            if vision:
                # The camera picks the person (most nearly straight ahead), the
                # lidar adds the position.  Going the other way round, a lidar
                # cluster of two people standing together has its centre
                # between them and may match either one's bearing.
                for v in sorted(vision, key=lambda v: abs(wrap(v.bearing - pose.yaw))):
                    if abs(wrap(v.bearing - pose.yaw)) > START_HALF_ANGLE:
                        break
                    near = [(abs(wrap(bearing_from(v.origin, p["x"], p["y"]) - v.bearing)), d, p)
                            for _rel, d, p in front]
                    # A cluster of several people is centred between them.
                    near = [item for item in near if item[0] <= (
                        MERGED_MATCH_BEARING if int(item[2].get("group", 1)) > 1 else MATCH_BEARING * 1.5)]
                    if near:
                        _off, _d, person = min(near, key=lambda item: item[0])
                        self.lidar_id = person.get("id")
                        self._set_vision(v, now)
                        self._update(person["x"], person["y"], now, "lidar+vision")
                        return True
            # Lidar only: the person most nearly straight ahead (whoever says
            # "follow me" faces the robot; the nearest may be a bystander).
            _rel, _d, person = min(front, key=lambda item: item[0])
            self.lidar_id = person.get("id")
            self._update(person["x"], person["y"], now, "lidar")
            return True
        # Simple 2-D lidar: YOLO bearing plus the laser range along it.
        best = None
        for v in vision or ():
            distance = scan_range(scan_points, v.origin, v.bearing)
            if distance is None or distance > START_RADIUS:
                continue
            relative = abs(wrap(v.bearing - pose.yaw))
            if best is None or relative < best[0]:
                best = (relative, v, distance)
        if best is None:
            return False
        _rel, v, distance = best
        self._set_vision(v, now)
        x, y = self._along(v, distance)
        self._update(x, y, now, "vision")
        return True

    # ---- observations ----
    def observe_people(self, people: list[dict], now: float) -> None:
        self.people_at = now
        if not self.active:
            return
        pred = self.predict(now)
        seen = self._vision_fresh(now)
        target = next((p for p in people if self.lidar_id is not None and p.get("id") == self.lidar_id),
                      None)
        if target is None:
            target = self._pick(people, pred, self._gate(now, seen is not None), seen)
            if target is not None and target.get("id") != self.lidar_id:
                self.reacquired += 1
                self.lidar_id = target.get("id")
        if target is not None and int(target.get("group", 1)) > 1:
            # Merged with someone standing close (one lidar cluster): its centre
            # is between the two and the id may go to either when they part.
            # Hold the velocity from before and decide at the split.
            if self.merge_from is None:
                self.merge_from = (now, self.x, self.y, self.vx, self.vy)
            self.merged_at = now
            self._update(target["x"], target["y"], now, "lidar(merged)", keep_velocity=True)
            return
        if self.merge_from is not None:
            _t0, _x0, _y0, vx0, vy0 = self.merge_from
            split, sure = self._after_merge(people, now, seen)
            if not sure and now - self.merged_at <= MERGE_WAIT_S:
                # Only one of the two is visible yet (the other hidden behind
                # them): either could be the target.  Coast on the prediction.
                self._update(*self._merge_prediction(now), now, "predicted", keep_velocity=True)
                return
            self.merge_from = None
            if split is not None and split.get("id") != self.lidar_id:
                self.lidar_id = split.get("id")
                self.swaps += 1
                target = split
            # The pre-merge velocity, not the jumps of the merged centre.
            self.recent.clear()
            self.vx, self.vy = vx0, vy0
            if target is None:
                return
            if seen is None:
                self._update(target["x"], target["y"], now, "lidar", keep_velocity=True)
                return
            # With YOLO, the bearing check below still applies to the choice.
        if target is not None and seen is not None and now - self.vision_at <= SWAP_VISION_S:
            off = abs(wrap(bearing_from(seen.origin, target["x"], target["y"]) - seen.bearing))
            lateral = math.hypot(target["x"] - seen.origin[0], target["y"] - seen.origin[1]) * math.sin(
                min(off, math.pi / 2))
            if off > SWAP_BEARING and lateral > SWAP_LATERAL:
                # The lidar id went to someone else: YOLO knows better -- if
                # someone else is where the camera sees the target.  If not,
                # keep the lidar id (the camera may have lost the target).
                better = self._pick(people, pred, TRACK_GATE + 0.5, seen, exclude=self.lidar_id)
                if better is not None:
                    self.lidar_id = better.get("id")
                    self.swaps += 1
                    # A jump from the wrong person to the right one is no motion.
                    self._update(better["x"], better["y"], now, "lidar+vision", keep_velocity=True)
                    return
        if target is not None:
            self._update(target["x"], target["y"], now, "lidar+vision" if seen else "lidar")

    def observe_vision(self, persons: list[VisionPerson], now: float, scan_points=()) -> None:
        if not self.active:
            return
        chosen = None
        if self.yolo_id is not None:
            chosen = next((v for v in persons if v.track_id == self.yolo_id), None)
            if chosen is None and now - self.yolo_seen_at <= YOLO_KEEP_S:
                return   # the target is hidden for a moment; do not grab a neighbour
        by_id = chosen is not None
        searching = self.age(now) > FRESH_S * 2
        if chosen is None and not searching:
            px, py = self.predict(now)
            limit = MATCH_BEARING if self.age(now) <= FRESH_S else MATCH_BEARING * 2
            near = [(abs(wrap(bearing_from(v.origin, px, py) - v.bearing)), v) for v in persons]
            near = [(off, v) for off, v in near if off <= limit]
            if near:
                chosen = min(near, key=lambda item: item[0])[1]
        elif chosen is None:
            # Out of sight for a while (and ByteTrack has a new id by now): the
            # person nearest to where the target was last seen, if they can
            # have walked there meanwhile.  The bearing to a stale prediction
            # says little once the target turned a corner.
            gate = self._gate(now, True)
            best = None
            for v in persons:
                distance = scan_range(scan_points, v.origin, v.bearing)
                if distance is None:
                    continue
                x, y = self._along(v, distance)
                off = math.hypot(x - self.x, y - self.y)
                if off <= gate and (best is None or off < best[0]):
                    best = (off, v)
            if best is not None:
                chosen, by_id = best[1], True
        if chosen is None:
            return
        self._set_vision(chosen, now)
        if now - self.lidar_seen_at <= FRESH_S:
            return   # the lidar people give the position
        # No lidar people (simple lidar), or the target is beyond their range
        # or hidden from them: YOLO bearing plus the laser range along it.
        distance = scan_range(scan_points, chosen.origin, chosen.bearing)
        if distance is None:
            return
        x, y = self._along(chosen, distance)
        px, py = self.predict(now)
        # The same ByteTrack id is the same person wherever they walked to.
        if by_id or math.hypot(x - px, y - py) <= self._gate(now, True) + 0.5:
            self._update(x, y, now, "vision")

    # ---- helpers ----
    def _merge_prediction(self, now: float) -> tuple[float, float]:
        """Where the target is while merged: the pre-merge velocity carried on,
        or (no velocity yet) the merged centre last seen."""
        t0, x0, y0, vx, vy = self.merge_from
        if math.hypot(vx, vy) < 0.2:
            return self.x, self.y
        dt = min(now - t0, 4.0)
        return x0 + vx * dt, y0 + vy * dt

    def _after_merge(self, people, now: float,
                     seen: VisionPerson | None) -> tuple[dict | None, bool]:
        """At a split: which single person is the target, and whether that is
        sure.  YOLO first (its id usually survives the lidar merge): the one
        in the camera target's bearing.  Else the one nearest to the
        prediction, sure only once both people are visible again."""
        single = [p for p in people if int(p.get("group", 1)) == 1]
        if seen is not None:
            match = [(abs(wrap(bearing_from(seen.origin, p["x"], p["y"]) - seen.bearing)), i, p)
                     for i, p in enumerate(single)]
            match = sorted(m for m in match if m[0] <= MATCH_BEARING * 1.5)
            if match:
                return match[0][2], True
        px, py = self._merge_prediction(now)
        near = sorted((math.hypot(p["x"] - px, p["y"] - py), i, p) for i, p in enumerate(single))
        near = [p for d, _i, p in near if d <= TRACK_GATE + 0.5]
        if not near:
            return None, False
        # Two separate people: a lidar track that just lost its detection is
        # still listed for a moment at its old spot, right next to the other.
        both = any(math.hypot(p["x"] - near[0]["x"], p["y"] - near[0]["y"]) >= SPLIT_APART
                   for p in near[1:])
        return near[0], both

    def _gate(self, now: float, confirmed: bool) -> float:
        """How far from the prediction a re-found target may be.  Confirmed by
        a YOLO bearing: as far as a person can have walked meanwhile; lidar
        alone (anyone could be there): SEARCH_GATE."""
        age = self.age(now)
        if age <= FRESH_S * 2:
            return TRACK_GATE
        if confirmed:
            return min(REACQUIRE_MAX, TRACK_GATE + WALK_MAX * age)
        return SEARCH_GATE

    def _set_vision(self, seen: VisionPerson, now: float) -> None:
        self.vision_target = seen
        self.vision_at = now
        if seen.track_id is not None:
            self.yolo_id = seen.track_id
            self.yolo_seen_at = now

    @staticmethod
    def _along(seen: VisionPerson, distance: float) -> tuple[float, float]:
        r = distance + BODY_RADIUS
        return seen.origin[0] + r * math.cos(seen.bearing), seen.origin[1] + r * math.sin(seen.bearing)

    @staticmethod
    def _match_vision(vision, x: float, y: float) -> VisionPerson | None:
        best = None
        for v in vision:
            off = abs(wrap(bearing_from(v.origin, x, y) - v.bearing))
            if off <= MATCH_BEARING and (best is None or off < best[0]):
                best = (off, v)
        return None if best is None else best[1]

    @staticmethod
    def _pick(people, pred, gate: float, seen: VisionPerson | None, exclude=None) -> dict | None:
        """Lidar person nearest the prediction; with YOLO, only one in its bearing."""
        best = None
        for p in people:
            if exclude is not None and p.get("id") == exclude:
                continue
            distance = math.hypot(p["x"] - pred[0], p["y"] - pred[1])
            if distance > gate:
                continue
            if seen is not None and abs(wrap(bearing_from(seen.origin, p["x"], p["y"])
                                             - seen.bearing)) > MATCH_BEARING * 1.5:
                continue
            if best is None or distance < best[0]:
                best = (distance, p)
        return None if best is None else best[1]
