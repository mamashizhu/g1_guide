"""Plan the whole tour with the real planner to prove the new hall is usable.

Builds the same ``OccupancyGrid`` the navigation node builds, then plans from
the spawn point to every station and around the station loop.  A layout change
that walls off an exhibit fails here instead of in Gazebo.
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[2] / "ws/src/g1_guide"
sys.path.insert(0, str(PACKAGE))

from g1_guide.nav_grid import OccupancyGrid           # noqa: E402
from g1_guide.scene_config import load_scene          # noqa: E402


def main() -> int:
    config = load_scene(PACKAGE / "config/exhibition.json")
    started = time.time()
    grid = OccupancyGrid(config, extra_margin=0.30)
    print(f"grid built in {time.time() - started:.2f} s "
          f"from {len(config.get('nav_obstacles', ()))} baked hall rectangles")

    spawn = (config["spawn"]["x"], config["spawn"]["y"])
    stations = [exhibit["station"] for exhibit in config["exhibits"]]
    failures = 0

    for station in stations:
        started = time.time()
        path = grid.plan(spawn, (station["x"], station["y"]))
        elapsed = time.time() - started
        if path is None:
            print(f"  FAIL spawn -> station {station['key']}: unreachable")
            failures += 1
            continue
        length = sum(math.dist(path[i], path[i + 1]) for i in range(len(path) - 1))
        print(f"  ok   spawn -> station {station['key']} "
              f"({station['x']:5.1f},{station['y']:5.1f}): "
              f"{length:5.1f} m over {len(path):3d} points in {elapsed:.2f} s")

    for current, following in zip(stations, stations[1:] + stations[:1]):
        if grid.plan((current["x"], current["y"]),
                     (following["x"], following["y"])) is None:
            print(f"  FAIL station {current['key']} -> {following['key']}: unreachable")
            failures += 1

    print(f"\n{failures} failure(s)")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
