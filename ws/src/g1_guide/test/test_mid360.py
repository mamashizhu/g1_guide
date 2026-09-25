"""真机雷达版: 3D MID-360 model generation and cloud flattening."""

from __future__ import annotations

import math
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from g1_guide.mid360_to_scan import flatten_points, quaternion_matrix
from test_model_generation import GENERATOR, _write_source

IDENTITY = (np.eye(3), np.zeros(3))


def _generate(lidar_model: str) -> ET.Element:
    with tempfile.TemporaryDirectory() as directory:
        workspace = Path(directory) / "g1_ws"
        output = Path(directory) / "generated.urdf"
        GENERATOR.generate(_write_source(workspace), output, workspace, lidar_model)
        return ET.parse(output).getroot()


class Mid360ModelTests(unittest.TestCase):
    def test_mid360_is_3d_with_real_field_of_view(self):
        root = _generate("mid360")
        lidar = root.find("./gazebo[@reference='mid360_link']/sensor")
        self.assertEqual(lidar.findtext("topic"), "/mid360")
        self.assertEqual(lidar.findtext("lidar/scan/vertical/samples"), "30")
        self.assertAlmostEqual(math.degrees(float(lidar.findtext("lidar/scan/vertical/min_angle"))), -7.0)
        self.assertAlmostEqual(math.degrees(float(lidar.findtext("lidar/scan/vertical/max_angle"))), 52.0)
        self.assertEqual(lidar.findtext("lidar/range/max"), "40.0")
        # The official inverted mount is kept, so the sensor looks downward.
        mount = root.find("./joint[@name='mid360_joint']/origin")
        self.assertTrue(mount.get("rpy").startswith("3.14159"))

    def test_mid360_model_has_no_simulated_2d_nav_lidar(self):
        root = _generate("mid360")
        self.assertIsNone(root.find("./gazebo[@reference='nav_lidar_link']"))
        # The level frame stays: mid360_to_scan publishes /nav_scan in it.
        self.assertIsNotNone(root.find("./joint[@name='nav_lidar_joint']"))

    def test_simple_model_is_unchanged(self):
        root = _generate("simple")
        lidar = root.find("./gazebo[@reference='mid360_link']/sensor")
        self.assertEqual(lidar.findtext("topic"), "/scan")
        self.assertIsNone(lidar.find("lidar/scan/vertical"))
        self.assertIsNotNone(root.find("./gazebo[@reference='nav_lidar_link']/sensor"))

    def test_unknown_lidar_model_rejected(self):
        with self.assertRaises(ValueError):
            _generate("velodyne")


class FlattenTests(unittest.TestCase):
    NAV = (np.eye(3), np.array([0.0, 0.0, 1.3]))

    def test_low_obstacle_floor_and_self_hits(self):
        points = np.array([
            [2.0, 0.0, 0.5],    # bench top ahead: kept
            [3.0, 0.0, 0.0],    # floor: dropped
            [0.2, 0.0, 1.0],    # own body: dropped
            [0.0, 4.0, 1.0],    # wall to the left: kept
            [0.0, 5.0, 1.0],    # farther wall behind it: nearest wins
        ])
        ranges = flatten_points(points, IDENTITY, self.NAV, floor_top=0.0)
        self.assertEqual(len(ranges), 360)
        self.assertAlmostEqual(ranges[180], 2.0)   # bearing 0 is beam 180
        self.assertAlmostEqual(ranges[270], 4.0)   # bearing +90°
        self.assertEqual(sum(math.isfinite(r) for r in ranges), 2)

    def test_inverted_sensor_frame_is_applied(self):
        # Sensor upside down 1.3 m up: a point 0.8 m "above" it in its own
        # frame is 0.5 m above the floor in the world.
        flip = (quaternion_matrix(1.0, 0.0, 0.0, 0.0), np.array([0.0, 0.0, 1.3]))
        ranges = flatten_points(np.array([[1.5, 0.0, 0.8]]), flip, self.NAV, floor_top=0.0)
        self.assertAlmostEqual(ranges[180], 1.5)

    def test_empty_cloud(self):
        ranges = flatten_points(np.zeros((0, 3)), IDENTITY, self.NAV, floor_top=0.0)
        self.assertTrue(all(math.isinf(r) for r in ranges))


if __name__ == "__main__":
    unittest.main()
