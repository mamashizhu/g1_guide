import math
import unittest
from pathlib import Path

from g1_guide.nav_grid import OccupancyGrid
from g1_guide.scene_config import load_scene


CONFIG_PATH = Path(__file__).parents[1] / "config" / "exhibition.json"


def load_config():
    return load_scene(CONFIG_PATH)


class OccupancyGridTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config()
        self.grid = OccupancyGrid(self.config)
        self.spawn = (self.config["spawn"]["x"], self.config["spawn"]["y"])

    @staticmethod
    def obstacle_grid():
        return OccupancyGrid({
            "room": {"width": 12.0, "depth": 8.0, "wall_thickness": 0.1},
            "robot_clearance_radius": 0.55,
            "exhibits": [],
            "obstacles": [{"position": [0.0, 0.0, 0.0], "size": [1.0, 1.0, 1.0]}],
        })

    def test_six_station_routes_keep_exact_endpoints_and_avoid_collisions(self):
        for exhibit in self.config["exhibits"]:
            station = exhibit["station"]
            goal = (station["x"], station["y"])
            with self.subTest(key=station["key"]):
                path = self.grid.plan(self.spawn, goal)
                self.assertIsNotNone(path)
                self.assertEqual(path[0], self.spawn)
                self.assertEqual(path[-1], goal)
                self.assertTrue(all(self.grid.is_clear(*point) for point in path))
                for first, second in zip(path, path[1:]):
                    self.assertTrue(self.grid._segment_clear(first, second, ()))

    def test_path_avoids_inflated_rectangles(self):
        grid = self.obstacle_grid()
        start, goal = (-4.0, 0.0), (4.0, 0.0)
        path = grid.plan(start, goal)
        self.assertIsNotNone(path)
        self.assertTrue(any(abs(y) > 0.8 for _, y in path))
        self.assertTrue(all(grid.is_clear(*point) for point in path))
        for first, second in zip(path, path[1:]):
            self.assertTrue(grid._segment_clear(first, second, ()))

    def test_dynamic_point_forces_detour_and_blocks_endpoint(self):
        # Across the open north gallery, where a detour is possible but a
        # straight line is not once a scan return lands in the middle.
        start, goal, obstacle = (-5.5, 8.0), (5.5, 8.0), (0.0, 8.0)
        baseline = self.grid.plan(start, goal)
        detour = self.grid.plan(start, goal, dynamic_points=(obstacle,))
        self.assertIsNotNone(baseline)
        self.assertIsNotNone(detour)
        self.assertGreater(max(abs(y - 8.0) for _, y in detour), 0.6)
        self.assertTrue(all(self.grid.is_clear(*p, dynamic_points=(obstacle,)) for p in detour))
        for first, second in zip(detour, detour[1:]):
            self.assertTrue(self.grid._segment_clear(first, second, (obstacle,)))
        self.assertIsNone(self.grid.plan(start, obstacle, dynamic_points=(obstacle,)))

    def test_two_hundred_scan_points_preserve_collision_free_station_route(self):
        # A full lidar sweep around a spot the tour does not pass through.
        center = (-5.5, 8.0)
        scan_points = tuple(
            (center[0] + 1.2 * math.cos(2 * math.pi * i / 200),
             center[1] + 1.2 * math.sin(2 * math.pi * i / 200))
            for i in range(200)
        )
        station = self.config["exhibits"][4]["station"]
        goal = (station["x"], station["y"])
        path = self.grid.plan(self.spawn, goal, dynamic_points=scan_points)
        self.assertIsNotNone(path)
        self.assertEqual(path[0], self.spawn)
        self.assertEqual(path[-1], goal)
        self.assertTrue(all(self.grid.is_clear(*point, dynamic_points=scan_points) for point in path))
        for first, second in zip(path, path[1:]):
            self.assertTrue(self.grid._segment_clear(first, second, scan_points))

    def test_diagonal_steps_do_not_cut_blocked_corners(self):
        grid = self.obstacle_grid()
        path = grid.plan((-4.0, 0.0), (4.0, 0.0))
        self.assertIsNotNone(path)
        diagonal_steps = 0
        for first, second in zip(path[1:-1], path[2:]):
            a, b = grid._world_to_cell(first), grid._world_to_cell(second)
            if a is None or b is None:
                continue
            dx, dy = b[0] - a[0], b[1] - a[1]
            if abs(dx) == 1 and abs(dy) == 1:
                diagonal_steps += 1
                side_x = grid._cell_to_world((a[0] + dx, a[1]))
                side_y = grid._cell_to_world((a[0], a[1] + dy))
                self.assertTrue(grid.is_clear(*side_x))
                self.assertTrue(grid.is_clear(*side_y))
        self.assertGreater(diagonal_steps, 0)

    @staticmethod
    def corridor_grid():
        """A room narrow enough that one scan barrier can seal it off."""
        return OccupancyGrid({
            "room": {"width": 12.0, "depth": 4.0, "wall_thickness": 0.1},
            "robot_clearance_radius": 0.55,
            "exhibits": [],
            "obstacles": [],
        })

    def test_unreachable_and_invalid_inputs_fail_closed(self):
        corridor = self.corridor_grid()
        barrier = [(0.0, y) for y in [i / 10 for i in range(-20, 21)]]
        self.assertIsNone(corridor.plan((-3, 0), (3, 0), dynamic_points=barrier))
        self.assertIsNone(self.grid.plan((math.nan, 0), (1, 1)))
        self.assertIsNone(self.grid.plan((0, 0), (999, 999)))
        self.assertFalse(self.grid.is_clear(math.inf, 0))
        self.assertFalse(self.grid.is_clear(0, 0, dynamic_points=((0, math.nan),)))

    def test_hall_walls_and_dynamic_points_block_clearance(self):
        # The hall is imported as a mesh, so its walls reach the planner only
        # through the baked rectangles; a gap there would show up right here.
        self.assertGreater(len(self.config["nav_obstacles"]), 100)
        self.assertTrue(self.grid.is_clear(*self.spawn))
        for station in (exhibit["station"] for exhibit in self.config["exhibits"]):
            self.assertTrue(self.grid.is_clear(station["x"], station["y"]))
        for outside in ((0.0, 11.5), (-9.8, 0.0), (10.2, 0.0), (0.0, -11.5)):
            self.assertFalse(self.grid.is_clear(*outside))
        self.assertFalse(self.grid.is_clear(self.spawn[0] + 0.5, self.spawn[1],
                                            dynamic_points=(self.spawn,)))

    def test_invalid_constructor_arguments_raise_value_error(self):
        with self.assertRaises(ValueError):
            OccupancyGrid({}, resolution=0)
        with self.assertRaises(ValueError):
            OccupancyGrid(None)


if __name__ == "__main__":
    unittest.main()
