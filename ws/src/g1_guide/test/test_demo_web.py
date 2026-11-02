"""网页展示控制台：demo_web 的纯函数、跟拍相机注入、/nav/view。"""

from __future__ import annotations

import json
import os
import math
import tempfile
import threading
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from g1_guide.demo_web import (Stream, depth_to_bgr, image_to_bgr, list_media, parse_range,
                               safe_join)
from g1_guide.exhibition_nav import view_payload
from g1_guide.lidar_variants import CHASE_CAMERA_TOPIC, with_chase_camera
from g1_guide.nav_control import Pose2D
from g1_guide.nav_core import NavigatorCore
from g1_guide.scene_config import load_scene

PACKAGE = Path(__file__).resolve().parents[1]
URDF = ('<?xml version="1.0"?>\n<robot name="g1">\n  <link name="base_link" />\n'
        '  <link name="pelvis" />\n</robot>\n')


class ImageConversionTest(unittest.TestCase):
    def test_rgb_is_swapped_to_bgr_and_row_padding_dropped(self):
        rows = np.zeros((2, 8), dtype=np.uint8)          # width 2, step 8 (2 bytes padding)
        rows[0, 0:3] = (10, 20, 30)
        image = image_to_bgr(rows.tobytes(), 2, 2, 8, "rgb8")
        self.assertEqual(image.shape, (2, 2, 3))
        self.assertEqual(tuple(image[0, 0]), (30, 20, 10))

    def test_unknown_encoding_is_rejected(self):
        with self.assertRaises(ValueError):
            image_to_bgr(b"\0" * 12, 2, 2, 6, "yuv422")

    def test_depth_invalid_pixels_are_black_and_near_is_bright(self):
        depth = np.array([[0.5, 5.5], [math.inf, math.nan]], dtype=np.float32)
        image = depth_to_bgr(depth.tobytes(), 2, 2, 8, "32FC1")
        self.assertEqual(image.shape, (2, 2, 3))
        self.assertEqual(int(image[1, 0].sum()), 0)
        self.assertEqual(int(image[1, 1].sum()), 0)
        self.assertGreater(int(image[0, 0].sum()), 0)
        self.assertFalse(np.array_equal(image[0, 0], image[0, 1]))


class HttpHelpersTest(unittest.TestCase):
    def test_safe_join_blocks_escape(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.assertEqual(safe_join(root, "/a/b.mp4"), Path(os.path.abspath(root / "a/b.mp4")))
            self.assertIsNone(safe_join(root, "../secret"))
            self.assertIsNone(safe_join(root, "a/%2e%2e/%2e%2e/x"))

    def test_parse_range(self):
        self.assertEqual(parse_range("bytes=0-99", 1000), (0, 99))
        self.assertEqual(parse_range("bytes=500-", 1000), (500, 999))
        self.assertEqual(parse_range("bytes=-100", 1000), (900, 999))
        self.assertEqual(parse_range("bytes=900-5000", 1000), (900, 999))
        self.assertIsNone(parse_range("bytes=1000-", 1000))
        self.assertIsNone(parse_range(None, 1000))
        self.assertIsNone(parse_range("items=0-1", 1000))

    def test_list_media_groups_by_policy(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            demo = root / "mujoco/policies/g1_x/demo"
            demo.mkdir(parents=True)
            (demo / "demo.mp4").write_bytes(b"x")
            (root / "mujoco/policies/g1_x/eval_a.jsonl").write_text("{}\n")
            (root / "mujoco/policies/g1_x/model.pt").write_bytes(b"x")
            items = list_media(root)
            self.assertEqual({i["path"] for i in items},
                             {"mujoco/policies/g1_x/demo/demo.mp4", "mujoco/policies/g1_x/eval_a.jsonl"})
            self.assertTrue(all(i["group"] == "g1_x" for i in items))
            (demo / "demo_web.mp4").write_bytes(b"h264")
            paths = {i["path"] for i in list_media(root)}
            self.assertIn("mujoco/policies/g1_x/demo/demo_web.mp4", paths)
            self.assertNotIn("mujoco/policies/g1_x/demo/demo.mp4", paths)


class StreamTest(unittest.TestCase):
    def test_wanted_while_watched_and_briefly_after(self):
        stream = Stream("front", "/front_camera/image", "rgb", 5.0)
        self.assertFalse(stream.wanted(1e9))
        stream.join()
        self.assertTrue(stream.wanted(1e9))
        stream.leave()
        self.assertTrue(stream.wanted(stream.last_client_at + 1.0))
        self.assertFalse(stream.wanted(stream.last_client_at + 10.0))

    def test_wait_returns_new_frame(self):
        stream = Stream("front", "/front_camera/image", "rgb", 5.0)
        threading.Timer(0.05, stream.publish, args=(b"jpeg", 1.0)).start()
        frame, seq = stream.wait(0, timeout=2.0)
        self.assertEqual((frame, seq), (b"jpeg", 1))


class ChaseCameraTest(unittest.TestCase):
    def test_camera_added_on_pelvis(self):
        root = ET.fromstring(with_chase_camera(URDF, "-1 0 2 0 0.5 0", 8))
        gazebo = [g for g in root.findall("gazebo") if g.get("reference") == "pelvis"]
        self.assertEqual(len(gazebo), 1)
        sensor = gazebo[0].find("sensor")
        self.assertEqual(sensor.get("type"), "camera")
        self.assertEqual(sensor.findtext("topic"), CHASE_CAMERA_TOPIC)
        self.assertEqual(sensor.findtext("pose"), "-1 0 2 0 0.5 0")
        self.assertEqual(sensor.findtext("update_rate"), "8")

    def test_idempotent_and_needs_pelvis(self):
        once = with_chase_camera(URDF)
        self.assertEqual(with_chase_camera(once), once)
        no_pelvis = '<robot name="x"><link name="base_link"/></robot>'
        self.assertEqual(with_chase_camera(no_pelvis), no_pelvis)

    def test_bad_pose_rejected(self):
        with self.assertRaises(ValueError):
            with_chase_camera(URDF, "1 2 3")
        with self.assertRaises(ValueError):
            with_chase_camera(URDF, "1 2 3 4 5 nan")

    def test_generated_glide_model_accepts_camera(self):
        model = PACKAGE / "urdf/g1_glide.urdf"
        if not model.is_file():
            self.skipTest("generated model not built")
        text = with_chase_camera(model.read_text(encoding="utf-8"))
        self.assertIn('name="chase_camera"', text)
        ET.fromstring(text.encode("utf-8"))


class NavViewTest(unittest.TestCase):
    def setUp(self):
        config = load_scene(PACKAGE / "config/exhibition_simple.json")
        stations = {e["station"]["key"]: e["station"] for e in config["exhibits"]}
        self.core = NavigatorCore(config, stations)

    def test_idle_view_has_pose_and_no_path(self):
        view = view_payload(self.core, Pose2D(1.0, 2.0, 0.5), "odom")
        self.assertEqual(view["pose"], [1.0, 2.0, 0.5])
        self.assertEqual(view["path"], [])
        json.dumps(view, allow_nan=False)

    def test_path_starts_at_robot_and_points_are_thinned(self):
        self.core.goal_key = "1"
        self.core.path = [(float(i), 0.0) for i in range(5)]
        self.core.path_index = 2
        self.core.scan_points = [(0.01 * i, 1.0) for i in range(1000)]
        view = view_payload(self.core, Pose2D(1.5, 0.0, 0.0), "map", scan_limit=100)
        self.assertEqual(view["path"], [[1.5, 0.0], [2.0, 0.0], [3.0, 0.0], [4.0, 0.0]])
        self.assertLessEqual(len(view["scan"]), 100)
        self.assertIsNone(view_payload(self.core, None, "odom")["pose"])


if __name__ == "__main__":
    unittest.main()
