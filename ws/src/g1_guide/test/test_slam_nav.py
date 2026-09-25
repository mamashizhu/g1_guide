"""SLAM 地图导航: odometry drift, map-to-obstacle conversion, tour summary."""

import math
import unittest
from pathlib import Path

from g1_guide.nav_grid import OccupancyGrid
from g1_guide.odom_drift import DriftModel, step
from g1_guide.slam_explore import summarize
from g1_guide.slam_map import FREE, OCCUPIED, GridMap, navigation_scene, occupied_rectangles
from test_mid360 import _generate


def walk(model, moves, dt=0.1):
    """Drive the truth through ``moves`` and return (truth, drifted)."""
    truth = drifted = (0.0, 0.0, 0.0)
    for forward, turn in moves:
        x, y, yaw = truth
        after = (x + forward * math.cos(yaw), y + forward * math.sin(yaw), yaw + turn)
        drifted = step(drifted, truth, after, dt, model)
        truth = after
    return truth, drifted


class OdomDriftTests(unittest.TestCase):
    def test_zero_model_reproduces_truth(self):
        truth, drifted = walk(DriftModel(), [(0.03, 0.01)] * 200)
        for a, b in zip(truth, drifted):
            self.assertAlmostEqual(a, b, places=9)

    def test_distance_scale(self):
        _truth, drifted = walk(DriftModel(linear_scale=0.03), [(0.03, 0.0)] * 100)
        self.assertAlmostEqual(drifted[0], 3.09)

    def test_heading_bias_only_while_moving(self):
        _t, still = walk(DriftModel(yaw_bias=0.01), [(0.0, 0.0)] * 100)
        self.assertEqual(still[2], 0.0)
        _t, moving = walk(DriftModel(yaw_bias=0.01), [(0.03, 0.0)] * 100)
        self.assertAlmostEqual(moving[2], 0.1)


def room_map():
    """8 x 6 m room of 0.1 m cells with border walls and a 1 x 1 m block."""
    w, h = 80, 60
    cells = []
    for row in range(h):
        for col in range(w):
            border = row in (0, h - 1) or col in (0, w - 1)
            block = 35 <= col < 45 and 25 <= row < 35
            cells.append(OCCUPIED if border or block else FREE)
    return GridMap(w, h, 0.1, (-4.0, -3.0, 0.0), cells)


class NavigationSceneTests(unittest.TestCase):
    def test_rectangles_cover_exactly_the_occupied_cells(self):
        grid = room_map()
        rects = occupied_rectangles(grid)
        area = sum((r - l) * (t - b) for l, b, r, t in rects)
        self.assertAlmostEqual(area / 0.01, grid.count(OCCUPIED))
        self.assertLessEqual(len(rects), 6)   # 4 walls + block, merged

    def test_planner_avoids_mapped_block_and_ignores_layout(self):
        scene = {"room": {"width": 99.0, "depth": 99.0, "wall_thickness": 0.1, "floor_top": 0.0},
                 "robot_clearance_radius": 0.3,
                 "exhibits": [{"position": [2.0, 0.0], "size": [1.0, 1.0],
                               "station": {"key": "1", "x": 3.0, "y": 0.0}}]}
        static = navigation_scene(room_map(), scene)
        self.assertEqual(static["exhibits"], [])
        grid = OccupancyGrid(static, extra_margin=0.1)
        self.assertFalse(grid.is_clear(0.0, 0.0))       # mapped block
        self.assertTrue(grid.is_clear(2.0, 0.0))        # layout-only exhibit: not in the map
        self.assertFalse(grid.is_clear(3.9, 0.0))       # mapped east wall
        path = grid.plan((-3.0, 0.0), (3.0, 0.0))
        self.assertIsNotNone(path)
        self.assertTrue(all(math.hypot(x, y) > 0.5 for x, y in path))


class ArrivalDistanceTests(unittest.TestCase):
    def test_tighter_tolerance_keeps_driving(self):
        from g1_guide.nav_control import Pose2D, compute_command
        pose = Pose2D(0.0, 0.0, 0.0)
        goal = Pose2D(0.2, 0.0, 0.0)
        self.assertEqual(compute_command(pose, goal, goal, 5.0, final=True).status, "arrived")
        tight = compute_command(pose, goal, goal, 5.0, final=True, arrival_distance=0.15)
        self.assertEqual(tight.status, "moving")
        self.assertEqual(compute_command(pose, goal, goal, 5.0, final=True,
                                         arrival_distance=float("nan")).status, "invalid")


class MapSceneGuardTests(unittest.TestCase):
    def test_map_from_another_hall_is_refused(self):
        import tempfile
        from g1_guide.slam_map import check_map_scene, write_map_meta
        with tempfile.TemporaryDirectory() as tmp:
            map_yaml = Path(tmp) / "m.yaml"
            with self.assertRaises(ValueError):          # no record at all
                check_map_scene(map_yaml, "exhibition.json")
            write_map_meta(map_yaml, "/any/dir/exhibition_simple.json", "mid360")
            check_map_scene(map_yaml, "config/exhibition_simple.json")
            with self.assertRaises(ValueError):
                check_map_scene(map_yaml, "config/exhibition.json")


class TourReportTests(unittest.TestCase):
    def test_summarize(self):
        self.assertEqual(summarize([0.1, 0.3]), {"mean_m": 0.2, "max_m": 0.3, "samples": 2})
        self.assertIsNone(summarize([])["max_m"])


class TruthOdomModelTests(unittest.TestCase):
    def _odometry(self, lidar):
        root = _generate(lidar)
        plugin = root.find(".//plugin[@name='gz::sim::systems::OdometryPublisher']")
        return {child.tag: child.text for child in plugin}

    def test_mid360_publishes_truth_without_tf(self):
        odom = self._odometry("mid360")
        self.assertEqual(odom["odom_topic"], "/odom_truth")
        self.assertEqual(odom["publish_tf"], "false")

    def test_simple_keeps_exact_odom_tf(self):
        odom = self._odometry("simple")
        self.assertEqual((odom["odom_topic"], odom["odom_frame"], odom["publish_tf"]),
                         ("/odom", "odom", "true"))


if __name__ == "__main__":
    unittest.main()
