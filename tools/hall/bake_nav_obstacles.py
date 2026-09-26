"""Bake the museum mesh into axis-aligned rectangles for the navigation grid.

Gazebo renders the hall from its COLLADA mesh, but ``nav_grid.OccupancyGrid``
plans against rectangles.  This tool derives those rectangles once, offline,
so the planner's static map matches the mesh the robot actually drives in.

Only the boundary of the region the robot can reach is baked: geometry the
robot could never touch without first crossing that boundary is dropped, which
keeps the rectangle count small.  Output is JSON on stdout.
"""

from __future__ import annotations

import argparse
import json
from collections import deque
from pathlib import Path

import numpy as np

CACHE = Path(__file__).resolve().parent / "museum_points.npy"
BAND = (0.20, 1.80)
CELL = 0.10


def occupancy(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    band = points[(points[:, 2] > BAND[0]) & (points[:, 2] < BAND[1])]
    lo = np.floor(points[:, :2].min(axis=0) / CELL).astype(int)
    hi = np.ceil(points[:, :2].max(axis=0) / CELL).astype(int)
    occ = np.zeros((hi[1] - lo[1] + 1, hi[0] - lo[0] + 1), dtype=bool)
    idx = np.floor(band[:, :2] / CELL).astype(int) - lo
    occ[idx[:, 1], idx[:, 0]] = True
    return occ, lo


def dilate(mask: np.ndarray, radius_cells: int) -> np.ndarray:
    if radius_cells <= 0:
        return mask.copy()
    out = mask.copy()
    rows, cols = mask.shape
    ys, xs = np.nonzero(mask)
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dy * dy + dx * dx > radius_cells * radius_cells:
                continue
            ny, nx = ys + dy, xs + dx
            keep = (ny >= 0) & (ny < rows) & (nx >= 0) & (nx < cols)
            out[ny[keep], nx[keep]] = True
    return out


def flood(free: np.ndarray, seed: tuple[int, int]) -> np.ndarray:
    reach = np.zeros_like(free)
    if not free[seed]:
        raise SystemExit(f"seed cell {seed} is not free")
    reach[seed] = True
    queue = deque([seed])
    rows, cols = free.shape
    while queue:
        y, x = queue.popleft()
        for ny, nx in ((y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)):
            if 0 <= ny < rows and 0 <= nx < cols and free[ny, nx] and not reach[ny, nx]:
                reach[ny, nx] = True
                queue.append((ny, nx))
    return reach


def largest_rectangle(mask: np.ndarray) -> tuple[int, int, int, int, int]:
    """Return (area, y0, x0, height, width) of the largest all-True rectangle."""
    rows, cols = mask.shape
    heights = np.zeros(cols, dtype=int)
    best = (0, 0, 0, 0, 0)
    for y in range(rows):
        heights = np.where(mask[y], heights + 1, 0)
        stack: list[tuple[int, int]] = []          # (start column, height)
        for x in range(cols + 1):
            h = heights[x] if x < cols else 0
            start = x
            while stack and stack[-1][1] >= h:
                sx, sh = stack.pop()
                area = sh * (x - sx)
                if area > best[0]:
                    best = (area, y - sh + 1, sx, sh, x - sx)
                start = sx
            stack.append((start, h))
    return best


def decompose(mask: np.ndarray, min_cells: int) -> list[tuple[int, int, int, int]]:
    """Greedily cover ``mask`` with rectangles, largest first."""
    remaining = mask.copy()
    rectangles = []
    while True:
        area, y0, x0, height, width = largest_rectangle(remaining)
        if area < min_cells:
            break
        rectangles.append((y0, x0, height, width))
        remaining[y0:y0 + height, x0:x0 + width] = False
    return rectangles


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--radius", type=float, default=0.55)
    parser.add_argument("--seed", type=float, nargs=2, default=(0.0, 0.0))
    parser.add_argument("--reach-margin", type=float, default=1.6,
                        help="how far beyond the reachable region to keep geometry")
    parser.add_argument("--min-area", type=float, default=0.09,
                        help="drop rectangles smaller than this many m^2")
    args = parser.parse_args()

    points = np.load(CACHE)
    occ, lo = occupancy(points)
    radius_cells = int(round(args.radius / CELL))

    # Where the robot's centre may go, then the footprint it can sweep.
    centres = flood(~dilate(occ, radius_cells),
                    (int(round(args.seed[1] / CELL)) - lo[1],
                     int(round(args.seed[0] / CELL)) - lo[0]))
    sweep = dilate(centres, radius_cells)

    # Bake only the obstacle shell around that sweep; anything further away is
    # unreachable by construction and would just inflate the rectangle count.
    shell = occ & dilate(sweep, int(round(args.reach_margin / CELL)))
    rectangles = decompose(shell, max(1, int(round(args.min_area / (CELL * CELL)))))

    covered = np.zeros_like(shell)
    out = []
    for index, (y0, x0, height, width) in enumerate(rectangles):
        covered[y0:y0 + height, x0:x0 + width] = True
        cx = (lo[0] + x0 + width / 2.0) * CELL
        cy = (lo[1] + y0 + height / 2.0) * CELL
        out.append({"name": f"hall_{index:03d}",
                    "position": [round(cx, 3), round(cy, 3)],
                    "size": [round(width * CELL, 3), round(height * CELL, 3)]})

    missed = int((shell & ~covered).sum())
    ys, xs = np.nonzero(centres)
    summary = {
        "source": "OpenRobotics/iscas_museum mesh, baked by tools/hall/bake_nav_obstacles.py",
        "cell": CELL,
        "clearance_radius": args.radius,
        "reachable_area_m2": round(float(centres.sum()) * CELL * CELL, 2),
        "reachable_x": [round(float((xs.min() + lo[0]) * CELL), 2),
                        round(float((xs.max() + lo[0]) * CELL), 2)],
        "reachable_y": [round(float((ys.min() + lo[1]) * CELL), 2),
                        round(float((ys.max() + lo[1]) * CELL), 2)],
        "rectangles": len(out),
        "uncovered_cells": missed,
    }
    print(json.dumps({"summary": summary, "nav_obstacles": out}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
