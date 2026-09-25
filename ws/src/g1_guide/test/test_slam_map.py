import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from g1_guide.slam_explore import tour_order
from g1_guide.slam_map import (FREE, OCCUPIED, UNKNOWN, GridMap, compare_map,
                               detected_objects, from_occupancy_grid,
                               ground_truth_rectangles, load_map, named_objects,
                               render_overlay, save_map)


def box_map(offset: float = 0.0) -> GridMap:
    """A 4 m x 4 m room of 0.1 m cells with walls on the border, shifted by ``offset``."""

    width = height = 40
    cells = []
    for row in range(height):
        for col in range(width):
            border = row in (0, height - 1) or col in (0, width - 1)
            cells.append(OCCUPIED if border else FREE)
    cells[width + 1] = UNKNOWN
    return GridMap(width, height, 0.1, (-2.0 + offset, -2.0, 0.0), cells)


# Walls 0.1 m thick along the border of the 4 m room.
WALLS = [(-2.0, -2.0, 2.0, -1.9), (-2.0, 1.9, 2.0, 2.0),
         (-2.0, -2.0, -1.9, 2.0), (1.9, -2.0, 2.0, 2.0)]


class SlamMapTests(unittest.TestCase):
    def test_save_and_load_round_trip(self):
        grid = box_map()
        with tempfile.TemporaryDirectory() as tmp:
            pgm, yaml = save_map(grid, Path(tmp) / "room")
            self.assertTrue(pgm.read_bytes().startswith(b"P5\n40 40\n255\n"))
            self.assertIn("image: room.pgm", yaml.read_text(encoding="utf-8"))
            loaded = load_map(yaml)
        self.assertEqual(loaded.cells, grid.cells)
        self.assertEqual(loaded.origin, grid.origin)
        self.assertAlmostEqual(loaded.resolution, 0.1)

    def test_pgm_rows_are_top_down(self):
        grid = GridMap(2, 2, 1.0, (0.0, 0.0, 0.0), [OCCUPIED, FREE, FREE, FREE])
        with tempfile.TemporaryDirectory() as tmp:
            pgm, _ = save_map(grid, Path(tmp) / "m")
            pixels = pgm.read_bytes()[-4:]
        # Grid row 0 (bottom) is the last PGM row.
        self.assertEqual(list(pixels), [254, 254, 0, 254])

    def test_aligned_map_passes(self):
        report = compare_map(box_map(), WALLS)
        self.assertTrue(report["pass"])
        self.assertEqual(report["p95_m"], 0.0)
        self.assertEqual(report["occupied_cells"], 156)

    def test_shifted_map_fails(self):
        report = compare_map(box_map(offset=0.5), WALLS)
        self.assertFalse(report["pass"])
        self.assertGreater(report["p95_m"], 0.2)

    def test_empty_map_does_not_pass(self):
        grid = GridMap(2, 1, 0.1, (0.0, 0.0, 0.0), [FREE, UNKNOWN])
        self.assertFalse(compare_map(grid, WALLS)["pass"])

    def test_from_occupancy_grid_thresholds(self):
        q = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
        message = SimpleNamespace(
            info=SimpleNamespace(width=4, height=1, resolution=0.05,
                                 origin=SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0),
                                                        orientation=q)),
            data=[-1, 0, 50, 100])
        grid = from_occupancy_grid(message)
        self.assertEqual(grid.cells, [UNKNOWN, FREE, UNKNOWN, OCCUPIED])
        self.assertEqual(grid.origin, (1.0, 2.0, 0.0))

    def test_ground_truth_uses_rotated_footprints_and_skips_actors(self):
        config = {
            "exhibits": [{"position": [0.0, 0.0], "size": [2.0, 0.2], "yaw": math.pi / 2}],
            "people": [{"kind": "actor", "position": [5.0, 5.0], "size": [0.5, 0.5]},
                       {"kind": "static", "position": [3.0, 3.0], "size": [0.4, 0.4]}],
        }
        rects = ground_truth_rectangles(config)
        self.assertEqual(len(rects), 2)
        left, bottom, right, top = rects[0]
        self.assertAlmostEqual(right - left, 0.2)
        self.assertAlmostEqual(top - bottom, 2.0)

    def test_box_room_walls_are_ground_truth(self):
        room = {"width": 4.0, "depth": 2.0, "wall_thickness": 0.2}
        self.assertEqual(len(ground_truth_rectangles({"room": room})), 4)
        self.assertEqual(ground_truth_rectangles({"room": room, "hall": {}}), [])

    def test_low_object_missing_from_map_is_reported(self):
        config = {"room": {"width": 4.0, "depth": 4.0, "wall_thickness": 0.1},
                  "obstacles": [{"name": "bench", "position": [0.0, 0.0], "size": [1.0, 0.5]}]}
        seen = detected_objects(box_map(), named_objects(config))
        self.assertTrue(seen["wall_north"])
        self.assertFalse(seen["bench"])

    def test_overlay_is_png(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = render_overlay(box_map(), WALLS, Path(tmp) / "o.png")
            self.assertTrue(path.read_bytes().startswith(b"\x89PNG\r\n\x1a\n"))


class TourOrderTests(unittest.TestCase):
    STATIONS = {"1": {"x": 5.0, "y": 0.0}, "2": {"x": 1.0, "y": 0.0}, "3": {"x": 2.0, "y": 0.0}}

    def test_nearest_first(self):
        self.assertEqual(tour_order(self.STATIONS, (0.0, 0.0)), ["2", "3", "1"])

    def test_requested_order(self):
        self.assertEqual(tour_order(self.STATIONS, (0.0, 0.0), "3, 1 2"), ["3", "1", "2"])

    def test_unknown_key_rejected(self):
        with self.assertRaises(ValueError):
            tour_order(self.STATIONS, (0.0, 0.0), "9")


if __name__ == "__main__":
    unittest.main()
