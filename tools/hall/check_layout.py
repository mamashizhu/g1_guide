"""Validate a candidate exhibition layout against the museum mesh.

Two modes.  ``--point x y`` reports how much room a spot has and whether the
robot can reach it; ``--config FILE`` walks a full ``exhibition.json`` and
checks every spawn point, station, exhibit footprint and person against the
mesh-derived grid.  This is the ground truth the JSON layout is tuned to.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from bake_nav_obstacles import CELL, dilate, flood, occupancy

HERE = Path(__file__).resolve().parent


class Hall:
    def __init__(self, radius: float):
        points = np.load(HERE / "museum_points.npy")
        self.occ, self.lo = occupancy(points)
        self.radius = radius
        self.centres = flood(~dilate(self.occ, int(round(radius / CELL))),
                             (-self.lo[1], -self.lo[0]))
        ys, xs = np.nonzero(self.occ)
        self._obstacles = np.column_stack(((xs + self.lo[0]) * CELL,
                                           (ys + self.lo[1]) * CELL))

    def cell(self, x: float, y: float) -> tuple[int, int]:
        return (int(round(y / CELL)) - self.lo[1], int(round(x / CELL)) - self.lo[0])

    def reachable(self, x: float, y: float) -> bool:
        row, col = self.cell(x, y)
        rows, cols = self.centres.shape
        return 0 <= row < rows and 0 <= col < cols and bool(self.centres[row, col])

    def clearance(self, x: float, y: float) -> float:
        d = np.hypot(self._obstacles[:, 0] - x, self._obstacles[:, 1] - y)
        return float(d.min())

    def footprint_free(self, x: float, y: float, sx: float, sy: float) -> bool:
        """True when no mesh obstacle falls inside the footprint rectangle."""
        inside = ((np.abs(self._obstacles[:, 0] - x) <= sx / 2 + CELL / 2)
                  & (np.abs(self._obstacles[:, 1] - y) <= sy / 2 + CELL / 2))
        return not bool(inside.any())


def report_point(hall: Hall, x: float, y: float, label: str = "") -> None:
    mark = "OK " if hall.reachable(x, y) else "!! "
    print(f"{mark}{label:<22} ({x:6.2f},{y:6.2f})  clearance {hall.clearance(x, y):5.2f} m  "
          f"reachable={hall.reachable(x, y)}")


def check_config(hall: Hall, path: Path) -> int:
    config = json.loads(path.read_text(encoding="utf-8"))
    problems = 0

    spawn = config["spawn"]
    report_point(hall, spawn["x"], spawn["y"], "spawn")
    if not hall.reachable(spawn["x"], spawn["y"]):
        problems += 1

    for exhibit in config["exhibits"]:
        eid = exhibit["exhibit_id"]
        px, py = exhibit["position"][0], exhibit["position"][1]
        sx, sy = exhibit["size"][0], exhibit["size"][1]
        station = exhibit["station"]
        if exhibit["kind"] != "poster" and not hall.footprint_free(px, py, sx, sy):
            print(f"!! {eid} footprint overlaps the hall mesh at ({px}, {py})")
            problems += 1
        report_point(hall, station["x"], station["y"], f"{eid} station {station['key']}")
        if not hall.reachable(station["x"], station["y"]):
            problems += 1
        # The robot must be looking at the exhibit from its station.
        want = math.atan2(py - station["y"], px - station["x"])
        delta = abs((want - station["yaw"] + math.pi) % (2 * math.pi) - math.pi)
        if delta > math.radians(12):
            print(f"!! {eid} station yaw {station['yaw']:.3f} does not face the exhibit "
                  f"(want {want:.3f})")
            problems += 1
        distance = math.dist((px, py), (station["x"], station["y"]))
        if not 1.2 <= distance <= 3.5:
            print(f"!! {eid} station is {distance:.2f} m from the exhibit (want 1.2-3.5)")
            problems += 1

    for item in config.get("obstacles", ()):
        px, py = item["position"][0], item["position"][1]
        size = item["size"]
        if not hall.footprint_free(px, py, size[0], size[1]):
            print(f"!! obstacle {item['name']} overlaps the hall mesh at ({px}, {py})")
            problems += 1
        report_point(hall, px, py, f"obstacle {item['name']}")

    for person in config.get("people", ()):
        name = person["name"]
        if person["kind"] == "static":
            px, py = person["position"]
            size = person["size"]
            if not hall.footprint_free(px, py, size[0], size[1]):
                print(f"!! visitor {name} overlaps the hall mesh at ({px}, {py})")
                problems += 1
            report_point(hall, px, py, f"visitor {name}")
            continue
        # Walking actors do not collide, but a route through a wall looks wrong.
        for step, (px, py) in enumerate(person["waypoints"]):
            report_point(hall, px, py, f"{name} wp{step}")
            if not hall.reachable(px, py):
                problems += 1

    print(f"\n{problems} problem(s)")
    return 1 if problems else 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--radius", type=float, default=0.55)
    parser.add_argument("--point", type=float, nargs=2, action="append", default=[])
    parser.add_argument("--config", type=Path)
    args = parser.parse_args()

    hall = Hall(args.radius)
    for x, y in args.point:
        report_point(hall, x, y)
    if args.config:
        return check_config(hall, args.config)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
