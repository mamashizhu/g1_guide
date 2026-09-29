"""讲解模式「面向观众」in NavigatorCore (PLAN M5.3)."""

import math
from pathlib import Path

from g1_guide.nav_control import Pose2D
from g1_guide.nav_core import NavigatorCore, wrap_angle
from g1_guide.scene_config import load_scene

CONFIG = load_scene(Path(__file__).resolve().parents[1] / "config/exhibition_simple.json")
STATIONS = {e["station"]["key"]: e["station"] for e in CONFIG["exhibits"]}


def at_station(key):
    s = STATIONS[key]
    return Pose2D(float(s["x"]), float(s["y"]), float(s["yaw"]))


def run(core, pose, key, people=(), t0=0.0, limit=20.0):
    """Step the core with an ideal robot; returns (last status, pose)."""
    core.set_goal(key)
    t = t0
    status = None
    while t < t0 + limit:
        core.set_people([{"x": x, "y": y} for x, y in people], t)
        command = core.step(pose, t)
        if command.status is not None:
            status = command.status
            if status["state"] in ("arrived", "faced"):
                return status, pose
        yaw = pose.yaw + command.angular_z * 0.1
        pose = Pose2D(pose.x + command.linear_x * 0.1 * math.cos(pose.yaw),
                      pose.y + command.linear_x * 0.1 * math.sin(pose.yaw), wrap_angle(yaw))
        t += 0.1
    raise AssertionError(f"no arrival, last {status}")


def core():
    c = NavigatorCore(CONFIG, STATIONS)
    c.set_scan([], None)
    return c


def test_point_mode_keeps_station_heading():
    pose = at_station("3")
    status, end = run(core(), pose, "3", people=[(pose.x + 1.0, pose.y + 1.5)])
    assert status["present_mode"] == "point" and not status["faced"]
    assert abs(wrap_angle(end.yaw - pose.yaw)) < 0.25
    assert status["people_near"] == 1


def test_face_mode_turns_to_people_then_arrives():
    c = core()
    assert c.set_present_mode("face")
    pose = at_station("3")
    people = [(pose.x - 1.0, pose.y - 1.5), (pose.x - 2.0, pose.y - 0.5)]
    status, end = run(c, pose, "3", people)
    assert status["state"] == "arrived" and status["faced"] and status["people_near"] == 2
    cx, cy = (sum(p[i] for p in people) / 2 for i in (0, 1))
    assert abs(math.degrees(wrap_angle(math.atan2(cy - end.y, cx - end.x) - end.yaw))) < 5.0
    assert status["goal_key"] == "3"


def test_face_mode_with_nobody_arrives_facing_the_exhibit():
    c = core()
    c.set_present_mode("face")
    pose = at_station("3")
    status, end = run(c, pose, "3")
    assert status["state"] == "arrived" and not status["faced"] and status["people_near"] == 0
    assert abs(wrap_angle(end.yaw - pose.yaw)) < 0.25


def test_stale_people_are_unknown():
    c = core()
    c.set_present_mode("face")
    c.set_people([{"x": 0.0, "y": 0.0}], -10.0)
    pose = at_station("3")
    c.set_goal("3")
    status = None
    for step in range(50):
        command = c.step(pose, step * 0.1)
        status = command.status or status
        if status and status["state"] == "arrived":
            break
    assert status["people_near"] is None and not status["faced"]


def test_face_command_alone_ends_in_faced():
    c = core()
    pose = at_station("3")
    status, end = run(c, pose, "face", people=[(pose.x, pose.y - 2.0)])
    assert status["state"] == "faced" and status["faced"] and status["goal_key"] == "face"
    assert abs(math.degrees(wrap_angle(-math.pi / 2 - end.yaw))) < 5.0
    assert c.goal_key is None


def test_unknown_mode_rejected():
    assert not core().set_present_mode("dance")
