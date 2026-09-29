"""PLAN M5.1 gesture classification and debouncing, on synthetic hand landmarks."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from gestures import COUNT_KEYS, GestureTracker, classify  # noqa: E402


def hand(fingers=(False, False, False, False), thumb="in", dx=0.0):
    """21 landmarks in image coordinates (y down), palm facing the camera."""
    p = np.zeros((21, 3))
    p[0] = (0.50, 0.80, 0)
    p[1] = (0.40, 0.75, 0)
    p[2] = (0.36, 0.70, 0)
    p[3], p[4] = {"in": ((0.40, 0.68, 0), (0.44, 0.70, 0)),
                  "out": ((0.32, 0.66, 0), (0.26, 0.62, 0)),
                  "up": ((0.40, 0.55, 0), (0.40, 0.45, 0))}[thumb]
    for i, (x, out) in enumerate(zip((0.44, 0.48, 0.52, 0.56), fingers)):
        base = 5 + 4 * i
        p[base] = (x, 0.65, 0)
        ys = (0.58, 0.53, 0.48) if out else (0.60, 0.64, 0.68)
        for k, y in enumerate(ys):
            p[base + 1 + k] = (x, y, 0)
    p[:, 0] += dx
    return p


def test_poses():
    assert classify(hand((True,) * 4, "out")) == "palm"
    assert classify(hand(thumb="up")) == "thumbs_up"
    assert classify(hand(thumb="out")) is None                    # thumb sideways: not "up"
    assert classify(hand()) is None                               # fist
    assert classify(hand((True, False, False, False))) == "one"
    assert classify(hand((True, True, False, False))) == "two"
    assert classify(hand((True, True, True, False))) == "three"
    assert classify(hand((True, True, True, True))) == "four"
    assert classify(hand((False, True, False, False))) is None    # middle finger alone
    assert COUNT_KEYS["three"] == "3"


def run(frames, dt=0.25):
    tracker = GestureTracker()
    return [e for i, f in enumerate(frames) if (e := tracker.update(f, i * dt))]


def test_static_gesture_needs_hold_and_fires_once():
    three = hand((True, True, True, False))
    assert run([three]) == []
    assert run([three] * 8) == ["three"]
    assert run([three] * 3 + [None] + [three] * 3) == ["three", "three"]


def test_flicker_between_poses_fires_nothing():
    a, b = hand((True, False, False, False)), hand((True, True, False, False))
    assert run([a, b] * 6) == []


def test_wave_versus_still_palm():
    palm = [hand((True,) * 4, "out", dx) for dx in (0, 0.08, 0, -0.08, 0, 0.08, 0, -0.08)]
    assert run(palm, dt=0.2) == ["wave"]
    assert run([hand((True,) * 4, "out")] * 6) == ["palm"]
