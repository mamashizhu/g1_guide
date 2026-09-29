"""Hand gestures from MediaPipe hand landmarks (PLAN M5.1).

``classify`` names one hand's pose; ``GestureTracker`` turns the per-frame
poses into events, each emitted once while held:
  palm       五指张开、手不动      → 停止讲解 / 停车
  wave       五指张开、左右摆      → 打招呼 / 再见
  thumbs_up  只竖拇指、朝上        → 肯定（「好的」）
  one..four  伸出 1–4 根手指（不含拇指）→ 去 1–4 号展位
Five spread fingers are the palm, so counting stops at four; stations 5 and 6
are chosen by voice.
"""

from __future__ import annotations

from collections import deque
import math

import numpy as np

WRIST, THUMB_MCP, THUMB_IP, THUMB_TIP, INDEX_MCP = 0, 2, 3, 4, 5
FINGERS = ((6, 8), (10, 12), (14, 16), (18, 20))   # (pip, tip) of index..pinky
COUNTS = {1: "one", 2: "two", 3: "three", 4: "four"}
COUNT_KEYS = {"one": "1", "two": "2", "three": "3", "four": "4"}
HOLD_S = 0.5          # a static pose must last this long (and 2 frames)
WAVE_WINDOW_S = 2.0
WAVE_SPAN = 0.6       # wrist sweep, in palm widths
WAVE_REVERSALS = 2


def _distance(a, b) -> float:
    return float(math.hypot(a[0] - b[0], a[1] - b[1]))


def extended_fingers(points: np.ndarray) -> list[bool]:
    """Index..pinky: tip farther from the wrist than the middle joint."""
    wrist = points[WRIST]
    return [_distance(points[tip], wrist) > 1.15 * _distance(points[pip], wrist)
            for pip, tip in FINGERS]


def thumb_extended(points: np.ndarray) -> bool:
    return _distance(points[THUMB_TIP], points[INDEX_MCP]) > 1.3 * _distance(points[THUMB_IP], points[INDEX_MCP])


def classify(points) -> str | None:
    """21 normalised (x, y[, z]) landmarks of one hand; image y grows downward."""
    points = np.asarray(points, dtype=float)[:, :2]
    fingers = extended_fingers(points)
    thumb = thumb_extended(points)
    count = sum(fingers)
    if count == 4 and thumb:
        return "palm"
    if count == 0 and thumb:
        # The thumb must point up: mostly vertical from its base to its tip.
        dx, dy = points[THUMB_TIP] - points[THUMB_MCP]
        if -dy > 2 * abs(dx) and -dy > 0.8 * _distance(points[WRIST], points[INDEX_MCP]):
            return "thumbs_up"
        return None
    if not thumb and fingers[:count] == [True] * count and count in COUNTS:
        return COUNTS[count]   # counting uses the fingers in order, index first
    return None


def palm_width(points) -> float:
    points = np.asarray(points, dtype=float)
    return _distance(points[INDEX_MCP], points[17])


class GestureTracker:
    """Per-frame poses in, one event per held gesture out."""

    def __init__(self):
        self.pose: str | None = None
        self.since = 0.0
        self.frames = 0
        self.emitted = False
        self.wrists: deque = deque()   # (t, wrist x / palm width) while the palm is open

    def update(self, points, now: float) -> str | None:
        pose = classify(points) if points is not None else None
        if pose == "palm":
            self.wrists.append((now, float(points[WRIST][0]) / max(palm_width(points), 1e-6)))
        else:
            self.wrists.clear()
        while self.wrists and now - self.wrists[0][0] > WAVE_WINDOW_S:
            self.wrists.popleft()
        if pose != self.pose:
            self.pose, self.since, self.frames, self.emitted = pose, now, 0, False
        self.frames += 1
        if pose is None or self.emitted:
            return None
        if pose == "palm" and self._waving():
            self.emitted = True
            return "wave"
        if self.frames >= 2 and now - self.since >= HOLD_S and not (pose == "palm" and self._moving()):
            self.emitted = True
            return pose
        return None

    def _sweep(self) -> list[float]:
        return [x for _, x in self.wrists]

    def _moving(self) -> bool:
        xs = self._sweep()
        return len(xs) >= 2 and max(xs) - min(xs) > WAVE_SPAN / 2

    def _waving(self) -> bool:
        xs = self._sweep()
        if len(xs) < 4 or max(xs) - min(xs) < WAVE_SPAN:
            return False
        steps = [b - a for a, b in zip(xs, xs[1:]) if abs(b - a) > 0.05]
        reversals = sum(1 for a, b in zip(steps, steps[1:]) if a * b < 0)
        return reversals >= WAVE_REVERSALS


class HandDetector:
    """MediaPipe HandLandmarker on BGR frames; returns the largest hand's landmarks."""

    def __init__(self):
        from pathlib import Path

        import mediapipe as mp
        from mediapipe.tasks.python import BaseOptions, vision

        self._mp = mp
        model = Path(__file__).resolve().parent / "models/hand_landmarker.task"
        self.landmarker = vision.HandLandmarker.create_from_options(vision.HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=str(model)), num_hands=2,
            min_hand_detection_confidence=0.5))

    def detect(self, image_bgr) -> np.ndarray | None:
        rgb = np.ascontiguousarray(image_bgr[:, :, ::-1])
        result = self.landmarker.detect(self._mp.Image(image_format=self._mp.ImageFormat.SRGB, data=rgb))
        if not result.hand_landmarks:
            return None
        hands = [np.array([(p.x, p.y, p.z) for p in hand]) for hand in result.hand_landmarks]
        return max(hands, key=palm_width)
