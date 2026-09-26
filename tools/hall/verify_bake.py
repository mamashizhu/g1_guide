"""Check that the baked rectangles confine the robot exactly like the mesh.

Rasterises ``nav_obstacles.json`` back onto the probe grid and compares the
flood-filled reachable set against the one derived from the mesh itself.  Any
cell reachable through the rectangles but not through the mesh is a hole the
planner would drive into a wall.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from bake_nav_obstacles import CELL, dilate, flood, occupancy

HERE = Path(__file__).resolve().parent
RADIUS = 0.55
SEED = (0.0, 0.0)


def main() -> int:
    points = np.load(HERE / "museum_points.npy")
    occ, lo = occupancy(points)
    radius_cells = int(round(RADIUS / CELL))
    seed = (int(round(SEED[1] / CELL)) - lo[1], int(round(SEED[0] / CELL)) - lo[0])
    truth = flood(~dilate(occ, radius_cells), seed)

    baked = np.zeros_like(occ)
    data = json.loads((HERE / "nav_obstacles.json").read_text(encoding="utf-8"))
    for rect in data["nav_obstacles"]:
        cx, cy = rect["position"]
        sx, sy = rect["size"]
        x0 = int(round((cx - sx / 2) / CELL)) - lo[0]
        y0 = int(round((cy - sy / 2) / CELL)) - lo[1]
        baked[y0:y0 + int(round(sy / CELL)), x0:x0 + int(round(sx / CELL))] = True

    replay = flood(~dilate(baked, radius_cells), seed)
    leak = replay & ~truth
    lost = truth & ~replay
    print(f"mesh reachable      : {truth.sum() * CELL * CELL:8.2f} m^2")
    print(f"baked reachable     : {replay.sum() * CELL * CELL:8.2f} m^2")
    print(f"LEAK (through walls): {leak.sum() * CELL * CELL:8.2f} m^2  <- must be 0")
    print(f"lost (over-blocked) : {lost.sum() * CELL * CELL:8.2f} m^2")
    if leak.any():
        ys, xs = np.nonzero(leak)
        print("  leak bbox x", round(float((xs.min() + lo[0]) * CELL), 2),
              "..", round(float((xs.max() + lo[0]) * CELL), 2),
              " y", round(float((ys.min() + lo[1]) * CELL), 2),
              "..", round(float((ys.max() + lo[1]) * CELL), 2))
    return 1 if leak.any() else 0


if __name__ == "__main__":
    raise SystemExit(main())
