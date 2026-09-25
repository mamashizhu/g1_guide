"""Gesture library, playback engine, pointing IK and glide-model arm release."""

from __future__ import annotations

import importlib.util
import math
import random
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from g1_guide.arm_kinematics import (  # noqa: E402
    ALL_ARM_JOINTS, ArmKinematics, arm_joint_names, mirror)
from g1_guide.gesture_player import parse_request, point_pose, world_to_base  # noqa: E402
from g1_guide.gestures import GestureEngine, GestureError, GestureLibrary  # noqa: E402

SOURCE_URDF = PACKAGE_ROOT.parent / "g1_description" / "g1_29dof_mode_11.urdf"
GESTURES = PACKAGE_ROOT / "config" / "gestures.yaml"
BRIDGE = PACKAGE_ROOT / "config" / "bridge.yaml"
GENERATOR_PATH = PACKAGE_ROOT / "scripts" / "make_glide_urdf.py"


def _generator():
    spec = importlib.util.spec_from_file_location("make_glide_urdf_gestures", GENERATOR_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


KINEMATICS = ArmKinematics.from_file(SOURCE_URDF)
LIBRARY = GestureLibrary.from_file(GESTURES)


def _engine(seed=1):
    return GestureEngine(LIBRARY, KINEMATICS.limits, rng=random.Random(seed))


def _run(engine, start, end, step=0.02):
    frames, t = [], start
    while t <= end:
        frames.append(engine.sample(t))
        t += step
    return frames


class KinematicsTest(unittest.TestCase):
    def test_mirror_is_symmetric(self):
        left = [-0.3, 0.4, 0.2, 0.5, -0.6, 0.1, 0.2]
        l_tip = KINEMATICS.hand_tip("left", dict(zip(arm_joint_names("left"), left)))
        r_tip = KINEMATICS.hand_tip("right", dict(zip(arm_joint_names("right"), mirror(left))))
        np.testing.assert_allclose(l_tip * [1, -1, 1], r_tip, atol=2e-3)

    def test_point_ik_error_below_one_degree(self):
        for side, target in (("right", (1.85, -0.1, 0.96)), ("left", (1.0, 1.5, 0.3)),
                             ("right", (0.5, -2.0, 1.5)), ("right", (2.0, 0.0, -0.5))):
            values, error = KINEMATICS.solve_point(side, target)
            self.assertLess(math.degrees(error), 1.0, (side, target))
            for name, value in zip(arm_joint_names(side), values):
                lower, upper = KINEMATICS.limits[name]
                self.assertTrue(lower <= value <= upper, name)


class LibraryTest(unittest.TestCase):
    def test_all_poses_within_joint_limits_and_clear_of_torso(self):
        for name in LIBRARY.poses:
            values, moving = LIBRARY.pose_values(name, LIBRARY.home())
            for joint, value in zip(ALL_ARM_JOINTS, values):
                lower, upper = KINEMATICS.limits[joint]
                self.assertTrue(lower < value < upper, f"{name}: {joint}={math.degrees(value):.0f}°")
            for index, side in enumerate(("left", "right")):
                arm = dict(zip(arm_joint_names(side), values[index * 7:index * 7 + 7]))
                tip = KINEMATICS.hand_tip(side, arm)
                # Hands must stay out of the torso box (|y| < 0.1 near x = 0).
                inside = abs(tip[1]) < 0.1 and -0.12 < tip[0] < 0.15 and -0.1 < tip[2] < 0.45
                self.assertFalse(inside, f"{name}: {side} hand inside torso at {tip.round(2)}")

    def test_wave_raises_right_hand_above_shoulder(self):
        values, _ = LIBRARY.pose_values("wave_up", LIBRARY.home())
        tip = KINEMATICS.hand_tip("right", dict(zip(arm_joint_names("right"), values[7:])))
        self.assertGreater(tip[2], KINEMATICS.shoulder("right")[2] + 0.1)

    def test_home_in_file_matches_generator_initial_position(self):
        home = _generator()._home_pose(list(ALL_ARM_JOINTS))
        for joint, value in zip(ALL_ARM_JOINTS, LIBRARY.home()):
            self.assertAlmostEqual(home[joint], value)

    def test_invalid_definitions_are_rejected(self):
        data = yaml.safe_load(GESTURES.read_text(encoding="utf-8"))
        data["gestures"]["broken"] = {"frames": [{"pose": "nope", "time": 1}]}
        with self.assertRaises(GestureError):
            GestureLibrary(data)
        data = yaml.safe_load(GESTURES.read_text(encoding="utf-8"))
        data["poses"]["bad"] = {"left": [0] * 6, "right": "home"}
        with self.assertRaises(GestureError):
            GestureLibrary(data)


class EngineTest(unittest.TestCase):
    def test_wave_is_smooth_and_returns_home(self):
        engine = _engine()
        engine.start("wave", 0.0)
        frames = _run(engine, 0.0, 12.0)
        steps = [max(abs(a - b) for a, b in zip(f0, f1)) for f0, f1 in zip(frames, frames[1:])]
        # 20 ms samples: no joint jumps more than 3 rad/s * 0.02 s.
        self.assertLess(max(steps), 0.06)
        self.assertTrue(engine.idle)
        np.testing.assert_allclose(frames[-1], engine._clamp(LIBRARY.home()), atol=1e-9)

    def test_first_transition_respects_max_speed(self):
        engine = _engine()
        engine.start("invite", 0.0)
        frames = _run(engine, 0.0, 0.6, step=0.01)
        speed = max(max(abs(a - b) for a, b in zip(f0, f1)) / 0.01 for f0, f1 in zip(frames, frames[1:]))
        self.assertLess(speed, 1.5 * 1.6)  # smoothstep peak = 1.5x average

    def test_explain_loop_repeats_with_jitter(self):
        engine = _engine(seed=3)
        engine.start("explain_loop", 0.0)
        period = sum(f.time + f.hold for f in LIBRARY.gestures["explain_loop"].frames)
        first = _run(engine, 0.0, period * 3)
        self.assertEqual(engine.active, "explain_loop")
        a = first[int(period / 0.02 * 1.5)]
        b = first[int(period / 0.02 * 2.5)]
        self.assertNotEqual(a, b)  # same phase, different random jitter

    def test_emphasize_resumes_explain_loop_and_keeps_left_arm(self):
        engine = _engine()
        engine.start("explain_loop", 0.0)
        _run(engine, 0.0, 2.0)
        left_before = engine.values[:7]
        engine.start("emphasize", 2.0)
        mid = _run(engine, 2.0, 2.3)[-1]
        np.testing.assert_allclose(mid[:7], left_before)
        _run(engine, 2.3, 5.0)
        self.assertEqual(engine.active, "explain_loop")

    def test_point_holds_ik_pose(self):
        engine = _engine()
        values, side, _ = point_pose(KINEMATICS, LIBRARY.home(), (1.85, -0.1, 0.96))
        self.assertEqual(side, "right")
        engine.start("point", 0.0, point_values=values)
        final = _run(engine, 0.0, 8.0)[-1]
        arm = dict(zip(arm_joint_names("right"), final[7:]))
        self.assertLess(math.degrees(KINEMATICS.pointing_error("right", arm, (1.85, -0.1, 0.96))), 10)
        with self.assertRaises(GestureError):
            engine.start("point", 9.0)

    def test_unknown_gesture_rejected(self):
        with self.assertRaises(GestureError):
            _engine().start("dance", 0.0)


class PlayerHelpersTest(unittest.TestCase):
    def test_parse_request(self):
        self.assertEqual(parse_request(" wave\n"), {"name": "wave"})
        self.assertEqual(parse_request('{"name": "point", "exhibit": "E03"}')["exhibit"], "E03")
        with self.assertRaises(GestureError):
            parse_request("")

    def test_world_to_base_applies_yaw(self):
        half = math.pi / 4  # yaw 90°: world +y is base +x
        quaternion = (0.0, 0.0, math.sin(half), math.cos(half))
        base = world_to_base((1.0, 3.0, 1.75), (1.0, 1.0, 0.79), quaternion)
        np.testing.assert_allclose(base, (2.0, 0.0, 0.96), atol=1e-9)

    def test_left_arm_for_targets_on_the_left(self):
        _, side, _ = point_pose(KINEMATICS, LIBRARY.home(), (0.5, 1.5, 0.5))
        self.assertEqual(side, "left")


class GlideModelArmsTest(unittest.TestCase):
    def test_generated_model_releases_only_arm_joints(self):
        generator = _generator()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "glide.urdf"
            generator.generate(SOURCE_URDF, output, PACKAGE_ROOT.parents[1])
            root = ET.parse(output).getroot()
        movable = {j.get("name") for j in root.findall("joint") if j.get("type") != "fixed"}
        self.assertEqual(movable, set(ALL_ARM_JOINTS))
        plugins = root.findall("./gazebo/plugin[@name='gz::sim::systems::JointPositionController']")
        self.assertEqual(sorted(p.findtext("joint_name") for p in plugins), sorted(ALL_ARM_JOINTS))
        for plugin in plugins:
            self.assertEqual(plugin.findtext("use_velocity_commands"), "true")
            self.assertEqual(plugin.findtext("topic"), f"/g1/arm/{plugin.findtext('joint_name')}/cmd_pos")
        children = {j.get("name"): j.find("child").get("link") for j in root.findall("joint")}
        for joint in ALL_ARM_JOINTS:
            self.assertEqual(root.findtext(f"./gazebo[@reference='{children[joint]}']/gravity"), "false")
        states = root.find("./gazebo/plugin[@name='gz::sim::systems::JointStatePublisher']")
        self.assertEqual(states.findtext("topic"), "/joint_states")
        # The generated model is also the IK model used at runtime.
        ArmKinematics(ET.tostring(root, encoding="unicode"))

    def test_bridge_forwards_every_arm_command(self):
        entries = yaml.safe_load(BRIDGE.read_text(encoding="utf-8"))
        topics = {e["ros_topic_name"]: e for e in entries}
        for joint in ALL_ARM_JOINTS:
            entry = topics[f"/g1/arm/{joint}/cmd_pos"]
            self.assertEqual((entry["direction"], entry["gz_type_name"]), ("ROS_TO_GZ", "gz.msgs.Double"))
        self.assertEqual(topics["/joint_states"]["ros_type_name"], "sensor_msgs/msg/JointState")


if __name__ == "__main__":
    unittest.main()
