"""Pure-numpy forward/inverse kinematics for the two G1 arms.

The chain is read from a URDF (the official source or the generated glide
model); joints not listed in ``values`` are treated as held at zero, which is
exactly how the glide model welds the legs and waist.  Positions are expressed
in ``base_link`` when present, otherwise in the URDF root link.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np


# Order used everywhere (gesture YAML, player, tests): shoulder pitch/roll/yaw,
# elbow, wrist roll/pitch/yaw.
ARM_JOINT_SUFFIXES = (
    "shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
    "wrist_roll", "wrist_pitch", "wrist_yaw",
)
SIDES = ("left", "right")
# Right = mirror of left across the sagittal plane: joints whose axis is
# along x or z change sign, joints about y keep it.
MIRROR_SIGNS = (1, -1, -1, 1, -1, 1, -1)
# Pointing ray tip: a few centimetres past the rubber hand's origin.
HAND_TIP_OFFSET = (0.08, 0.0, 0.0)


def arm_joint_names(side: str) -> tuple[str, ...]:
    if side not in SIDES:
        raise ValueError(f"side must be one of {SIDES}")
    return tuple(f"{side}_{suffix}_joint" for suffix in ARM_JOINT_SUFFIXES)


ALL_ARM_JOINTS = arm_joint_names("left") + arm_joint_names("right")


def mirror(values):
    """Map a 7-vector of one arm onto the other arm."""
    return [sign * value for sign, value in zip(MIRROR_SIGNS, values)]


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _axis_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    t = 1 - c
    return np.array([
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ])


def _transform(rotation: np.ndarray, translation) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


@dataclass(frozen=True)
class _Joint:
    name: str
    parent: str
    origin: np.ndarray
    axis: np.ndarray
    movable: bool
    lower: float
    upper: float


class ArmKinematics:
    """Forward kinematics and a small pointing IK for both arms."""

    def __init__(self, urdf_text: str):
        root = ET.fromstring(urdf_text)
        self._by_child: dict[str, _Joint] = {}
        self.limits: dict[str, tuple[float, float]] = {}
        for joint in root.findall("joint"):
            origin = joint.find("origin")
            xyz = [float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()]
            rpy = [float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()]
            axis_element = joint.find("axis")
            axis = np.array([float(v) for v in (axis_element.get("xyz") if axis_element is not None else "1 0 0").split()])
            axis = axis / np.linalg.norm(axis)
            movable = joint.get("type") in ("revolute", "continuous")
            limit = joint.find("limit")
            lower = float(limit.get("lower", "-inf")) if limit is not None else -math.inf
            upper = float(limit.get("upper", "inf")) if limit is not None else math.inf
            parsed = _Joint(joint.get("name"), joint.find("parent").get("link"),
                            _transform(_rpy_matrix(*rpy), xyz), axis, movable, lower, upper)
            self._by_child[joint.find("child").get("link")] = parsed
            if movable:
                self.limits[parsed.name] = (lower, upper)
        missing = [name for name in ALL_ARM_JOINTS if name not in self.limits]
        if missing:
            raise ValueError("URDF arm joints are missing or welded: " + ", ".join(missing))

    @classmethod
    def from_file(cls, path: Path) -> "ArmKinematics":
        return cls(Path(path).read_text(encoding="utf-8"))

    def link_pose(self, link: str, values: dict[str, float]) -> np.ndarray:
        """4x4 pose of ``link`` in the root frame for the given joint values."""
        chain = []
        while link in self._by_child:
            joint = self._by_child[link]
            chain.append(joint)
            link = joint.parent
        pose = np.eye(4)
        for joint in reversed(chain):
            pose = pose @ joint.origin
            if joint.movable:
                pose = pose @ _transform(_axis_matrix(joint.axis, values.get(joint.name, 0.0)), (0, 0, 0))
        return pose

    def shoulder(self, side: str) -> np.ndarray:
        # The pitch joint frame origin does not depend on any arm joint.
        return self.link_pose(f"{side}_shoulder_pitch_link", {})[:3, 3]

    def hand_tip(self, side: str, arm: dict[str, float]) -> np.ndarray:
        pose = self.link_pose(f"{side}_wrist_yaw_link", arm)
        palm = self._by_child.get(f"{side}_rubber_hand")
        tip = np.array([*HAND_TIP_OFFSET, 1.0])
        if palm is not None:
            pose = pose @ palm.origin
        return (pose @ tip)[:3]

    def elbow(self, side: str, arm: dict[str, float]) -> np.ndarray:
        return self.link_pose(f"{side}_elbow_link", arm)[:3, 3]

    def pointing_error(self, side: str, arm: dict[str, float], target) -> float:
        """Angle (rad) between shoulder→hand tip and shoulder→target."""
        start = self.shoulder(side)
        ray = self.hand_tip(side, arm) - start
        goal = np.asarray(target, dtype=float) - start
        cosine = float(np.dot(ray, goal) / (np.linalg.norm(ray) * np.linalg.norm(goal)))
        return math.acos(max(-1.0, min(1.0, cosine)))

    def clamp(self, name: str, value: float, margin: float = 0.02) -> float:
        lower, upper = self.limits[name]
        return min(upper - margin, max(lower + margin, value))

    def solve_point(self, side: str, target, *, elbow: float = 1.35,
                    wrist=(0.0, 0.0, 0.0), iterations: int = 80) -> tuple[list[float], float]:
        """Shoulder angles that aim the (almost straight) arm at ``target``.

        Damped least squares on the 3 shoulder joints; the elbow and wrist are
        held so the hand stays in line with the forearm.  Returns the 7 arm
        values and the remaining pointing error in radians.
        """

        names = arm_joint_names(side)
        start = self.shoulder(side)
        goal = np.asarray(target, dtype=float) - start
        if np.linalg.norm(goal) < 1e-6:
            raise ValueError("target coincides with the shoulder")
        goal = goal / np.linalg.norm(goal)
        # Seed: aim the upper arm with pitch (forward/up) and roll (sideways).
        horizontal = math.hypot(goal[0], goal[2])
        sign = 1.0 if side == "left" else -1.0
        q = np.array([
            -math.atan2(goal[0], -goal[2]) if horizontal > 1e-6 else 0.0,
            sign * max(0.0, sign * math.asin(max(-1.0, min(1.0, goal[1])))),
            0.0,
        ])
        fixed = [elbow, *wrist]

        def values(shoulder):
            return dict(zip(names, [*shoulder, *fixed]))

        def residual(shoulder):
            ray = self.hand_tip(side, values(shoulder)) - start
            return ray / np.linalg.norm(ray) - goal

        for _ in range(iterations):
            error = residual(q)
            if np.linalg.norm(error) < 1e-4:
                break
            jacobian = np.zeros((3, 3))
            for column in range(3):
                step = np.zeros(3)
                step[column] = 1e-5
                jacobian[:, column] = (residual(q + step) - error) / 1e-5
            # Light pull of shoulder yaw toward 0 keeps the elbow from twisting.
            damping = np.diag([1e-3, 1e-3, 5e-2])
            delta = np.linalg.solve(jacobian.T @ jacobian + damping, -jacobian.T @ error)
            q = q + np.clip(delta, -0.3, 0.3)
            q = np.array([self.clamp(n, v) for n, v in zip(names[:3], q)])
        result = [*q.tolist(), *fixed]
        return result, self.pointing_error(side, values(q), start + goal)
