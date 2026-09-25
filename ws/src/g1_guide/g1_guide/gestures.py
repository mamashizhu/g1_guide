"""Keyframe gesture library and playback engine for the guide's arms (no ROS).

``config/gestures.yaml`` defines named poses and gestures; ``GestureEngine``
turns a gesture into a smooth 14-joint trajectory sampled at any time.  The
joint order is ``arm_kinematics.ALL_ARM_JOINTS`` (left 7, then right 7).
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from pathlib import Path

import yaml

from g1_guide.arm_kinematics import ALL_ARM_JOINTS, SIDES, mirror

KEEP = "keep"
THEN_CHOICES = ("home", "hold", "resume")


class GestureError(ValueError):
    """A gesture request or definition that cannot be played."""


@dataclass(frozen=True)
class Frame:
    pose: str
    time: float
    hold: float = 0.0


@dataclass(frozen=True)
class Gesture:
    name: str
    frames: tuple[Frame, ...]
    loop: bool = False
    then: str = "home"
    jitter_deg: float = 0.0
    kind: str = "keyframes"


def _positive(value, label, allow_zero=False) -> float:
    number = float(value)
    if not math.isfinite(number) or number < 0 or (number == 0 and not allow_zero):
        raise GestureError(f"{label} must be {'non-negative' if allow_zero else 'positive'}")
    return number


class GestureLibrary:
    """Validated poses (radians, per-side convention) and gestures."""

    def __init__(self, data: dict):
        if not isinstance(data, dict) or "poses" not in data or "gestures" not in data:
            raise GestureError("gesture file needs 'poses' and 'gestures'")
        raw_poses = data["poses"]
        # arm spec per pose and side: 7 radians in that side's convention, or KEEP
        self.poses: dict[str, dict[str, list[float] | str]] = {}
        for name in raw_poses:
            self.poses[name] = {side: self._arm(raw_poses, name, side, set()) for side in SIDES}
        if "home" not in self.poses:
            raise GestureError("a 'home' pose is required")
        if any(value == KEEP for value in self.poses["home"].values()):
            raise GestureError("'home' must define both arms")

        self.gestures: dict[str, Gesture] = {}
        for name, spec in data["gestures"].items():
            self.gestures[name] = self._gesture(name, spec)
        if "home" not in self.gestures:
            raise GestureError("a 'home' gesture is required")

    @classmethod
    def from_file(cls, path: Path) -> "GestureLibrary":
        return cls(yaml.safe_load(Path(path).read_text(encoding="utf-8")))

    def _arm(self, raw, name, side, seen):
        if name not in raw:
            raise GestureError(f"unknown pose {name!r}")
        if (name, side) in seen:
            raise GestureError(f"pose reference loop at {name!r}")
        value = raw[name].get(side) if isinstance(raw[name], dict) else None
        if value is None:
            raise GestureError(f"pose {name!r} does not define the {side} arm")
        if value == KEEP:
            return KEEP
        if isinstance(value, str):
            return self._arm(raw, value, side, seen | {(name, side)})
        if not isinstance(value, list) or len(value) != 7:
            raise GestureError(f"pose {name!r} {side} arm needs 7 angles")
        degrees = [float(v) for v in value]
        if not all(math.isfinite(v) for v in degrees):
            raise GestureError(f"pose {name!r} has a non-finite angle")
        radians = [math.radians(v) for v in degrees]
        return mirror(radians) if side == "right" else radians

    def _gesture(self, name, spec) -> Gesture:
        if not isinstance(spec, dict):
            raise GestureError(f"gesture {name!r} must be a mapping")
        then = spec.get("then", "home")
        if then not in THEN_CHOICES:
            raise GestureError(f"gesture {name!r}: then must be one of {THEN_CHOICES}")
        jitter = _positive(spec.get("jitter_deg", 0.0), f"{name}.jitter_deg", allow_zero=True)
        if spec.get("kind") == "point":
            frame = Frame("point", _positive(spec.get("time", 1.2), f"{name}.time"),
                          _positive(spec.get("hold", 0.0), f"{name}.hold", allow_zero=True))
            return Gesture(name, (frame,), then=then, kind="point")
        frames = spec.get("frames")
        if not frames:
            raise GestureError(f"gesture {name!r} has no frames")
        parsed = []
        for frame in frames:
            if frame.get("pose") not in self.poses:
                raise GestureError(f"gesture {name!r} uses unknown pose {frame.get('pose')!r}")
            parsed.append(Frame(frame["pose"], _positive(frame.get("time"), f"{name}.time"),
                                _positive(frame.get("hold", 0.0), f"{name}.hold", allow_zero=True)))
        loop = bool(spec.get("loop", False))
        return Gesture(name, tuple(parsed), loop=loop, then="hold" if loop else then,
                       jitter_deg=jitter)

    def pose_values(self, name: str, current: list[float]) -> tuple[list[float], list[bool]]:
        """14 target values for a pose; ``keep`` arms copy ``current``.

        The second list flags joints that follow the pose (not kept)."""
        values, moving = [], []
        for index, side in enumerate(SIDES):
            arm = self.poses[name][side]
            if arm == KEEP:
                values.extend(current[index * 7:(index + 1) * 7])
                moving.extend([False] * 7)
            else:
                values.extend(arm)
                moving.extend([True] * 7)
        return values, moving

    def home(self) -> list[float]:
        return self.pose_values("home", [0.0] * 14)[0]


def smoothstep(x: float) -> float:
    x = min(1.0, max(0.0, x))
    return x * x * (3 - 2 * x)


@dataclass
class _Segment:
    target: list[float]
    duration: float
    hold: float


class GestureEngine:
    """Plays one gesture at a time; ``sample(now)`` returns 14 joint targets."""

    def __init__(self, library: GestureLibrary, limits: dict[str, tuple[float, float]],
                 *, max_speed: float = 1.5, rng: random.Random | None = None):
        self.library = library
        self.limits = [limits[name] for name in ALL_ARM_JOINTS]
        self.max_speed = _positive(max_speed, "max_speed")
        self.rng = rng or random.Random()
        self.values = self._clamp(library.home())
        self.active: str | None = None
        self._resume: str | None = None
        self._segments: list[_Segment] = []
        self._index = 0
        self._segment_start = 0.0
        self._segment_from = list(self.values)
        self._point_values: list[float] | None = None

    def _clamp(self, values):
        return [min(hi - 0.01, max(lo + 0.01, v)) for v, (lo, hi) in zip(values, self.limits)]

    def _build(self, gesture: Gesture) -> list[_Segment]:
        if gesture.kind == "point":
            frame = gesture.frames[0]
            return [_Segment(self._clamp(self._point_values), frame.time, frame.hold)]
        segments = []
        jitter = math.radians(gesture.jitter_deg)
        reference = list(self.values)
        for frame in gesture.frames:
            target, moving = self.library.pose_values(frame.pose, reference)
            if jitter:
                target = [v + self.rng.uniform(-jitter, jitter) if m else v
                          for v, m in zip(target, moving)]
            segments.append(_Segment(self._clamp(target), frame.time, frame.hold))
        return segments

    def start(self, name: str, now: float, *, point_values: list[float] | None = None) -> None:
        gesture = self.library.gestures.get(name)
        if gesture is None:
            raise GestureError(f"unknown gesture {name!r}; known: {', '.join(sorted(self.library.gestures))}")
        if gesture.kind == "point":
            if point_values is None or len(point_values) != 14:
                raise GestureError("point needs 14 joint values from the IK")
            self._point_values = list(point_values)
        if gesture.then == "resume" and self.active and self.library.gestures[self.active].loop:
            self._resume = self.active
        elif gesture.then != "resume":
            self._resume = None
        self.active = name
        self._begin(gesture, now)

    def _begin(self, gesture: Gesture, now: float) -> None:
        self._segments = self._build(gesture)
        first = self._segments[0]
        distance = max(abs(a - b) for a, b in zip(first.target, self.values))
        first.duration = max(first.duration, distance / self.max_speed)
        self._index = 0
        self._segment_start = now
        self._segment_from = list(self.values)

    def sample(self, now: float) -> list[float]:
        while self.active is not None:
            segment = self._segments[self._index]
            elapsed = now - self._segment_start
            if elapsed < segment.duration + segment.hold:
                blend = smoothstep(elapsed / segment.duration)
                self.values = [a + (b - a) * blend for a, b in zip(self._segment_from, segment.target)]
                return list(self.values)
            self.values = list(segment.target)
            self._segment_from = list(segment.target)
            self._segment_start += segment.duration + segment.hold
            self._index += 1
            if self._index < len(self._segments):
                continue
            self._finish(self._segment_start)
        return list(self.values)

    def _finish(self, now: float) -> None:
        gesture = self.library.gestures[self.active]
        if gesture.loop:
            # New jitter every round; the round starts where the last one ended.
            self._segments = self._build(gesture)
            self._index = 0
            return
        if gesture.then == "home":
            self.active = "home"
            self._begin(self.library.gestures["home"], now)
        elif gesture.then == "resume" and self._resume:
            name, self._resume = self._resume, None
            self.active = name
            self._begin(self.library.gestures[name], now)
        else:
            self.active = None

    @property
    def idle(self) -> bool:
        return self.active is None
