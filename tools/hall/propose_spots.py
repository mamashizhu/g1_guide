"""Propose exhibit anchors inside the museum: wall faces plus their stations.

A usable poster spot needs a flat run of wall, open floor in front of it, and
a viewing station the robot can actually stand on and reach.  This searches
the mesh-derived grid for all three at once so the layout in
``config/exhibition.json`` is measured rather than guessed.
"""

from __future__ import annotations

import argparse

import numpy as np

from bake_nav_obstacles import CELL, dilate, flood, occupancy
from pathlib import Path

HERE = Path(__file__).resolve().parent
DIRECTIONS = {"+y": (0, 1), "-y": (0, -1), "+x": (1, 0), "-x": (-1, 0)}
YAW = {"+y": 1.5707963267948966, "-y": -1.5707963267948966, "+x": 0.0, "-x": 3.141592653589793}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--radius", type=float, default=0.55)
    parser.add_argument("--standoff", type=float, default=2.0,
                        help="distance from wall face to the viewing station")
    parser.add_argument("--run", type=float, default=2.6,
                        help="required flat wall length, metres")
    parser.add_argument("--top", type=int, default=14)
    args = parser.parse_args()

    points = np.load(HERE / "museum_points.npy")
    occ, lo = occupancy(points)
    radius_cells = int(round(args.radius / CELL))
    centres = flood(~dilate(occ, radius_cells),
                    (-lo[1], -lo[0]))                      # seed at world origin
    rows, cols = occ.shape
    stand = int(round(args.standoff / CELL))
    half_run = int(round(args.run / CELL)) // 2

    def cell_ok(y: int, x: int, grid: np.ndarray) -> bool:
        return 0 <= y < rows and 0 <= x < cols and bool(grid[y, x])

    candidates = []
    for label, (dx, dy) in DIRECTIONS.items():
        px, py = -dy, dx                                    # along-wall direction
        for y in range(rows):
            for x in range(cols):
                if not occ[y, x]:
                    continue
                # Free floor from just off the wall out to the station.
                if any(not cell_ok(y + dy * k, x + dx * k, ~occ) for k in range(1, stand + 1)):
                    continue
                sy, sx = y + dy * stand, x + dx * stand
                if not cell_ok(sy, sx, centres):
                    continue
                # The same profile must hold across the whole poster width.
                span = range(-half_run, half_run + 1)
                if any(not cell_ok(y + py * t, x + px * t, occ) for t in span):
                    continue
                if any(not cell_ok(sy + py * t, sx + px * t, centres) for t in span):
                    continue
                openness = 0
                while cell_ok(y + dy * (stand + openness + 1),
                              x + dx * (stand + openness + 1), centres):
                    openness += 1
                candidates.append((openness, label,
                                   round(float((x + lo[0]) * CELL), 2),
                                   round(float((y + lo[1]) * CELL), 2),
                                   round(float((sx + lo[0]) * CELL), 2),
                                   round(float((sy + lo[1]) * CELL), 2)))

    candidates.sort(reverse=True)
    print(f"{len(candidates)} raw candidates (wall run {args.run} m, standoff {args.standoff} m)\n")
    print(f"{'facing':>6} {'wall x':>7} {'wall y':>7} {'station x':>10} {'station y':>10} "
          f"{'open m':>7}  yaw")
    chosen: list[tuple] = []
    for openness, label, wx, wy, sx, sy in candidates:
        if any(lab == label and abs(sx - cx) < args.run and abs(sy - cy) < args.run
               for _o, lab, _wx, _wy, cx, cy in chosen):
            continue
        chosen.append((openness, label, wx, wy, sx, sy))
        print(f"{label:>6} {wx:7.2f} {wy:7.2f} {sx:10.2f} {sy:10.2f} "
              f"{openness * CELL:7.2f}  {YAW[label]:+.4f}")
        if len(chosen) >= args.top:
            break
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
