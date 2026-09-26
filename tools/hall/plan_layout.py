"""Analyse the museum occupancy map and report where exhibits can go.

Consumes the point cache written by ``probe_museum.py``.  Builds an occupancy
grid, inflates it by the robot clearance radius, keeps the free region the
robot can actually reach from a seed, and prints that region plus the best
open spots.  Everything here is measurement only -- no world files are
written.
"""

from __future__ import annotations

import argparse
from collections import deque
from pathlib import Path

import numpy as np

CACHE = Path(__file__).resolve().parent / "museum_points.npy"
BAND = (0.20, 1.80)
CELL = 0.10


def build_grid(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    band = points[(points[:, 2] > BAND[0]) & (points[:, 2] < BAND[1])]
    lo = np.floor(points[:, :2].min(axis=0) / CELL).astype(int)
    hi = np.ceil(points[:, :2].max(axis=0) / CELL).astype(int)
    occ = np.zeros((hi[1] - lo[1] + 1, hi[0] - lo[0] + 1), dtype=bool)
    idx = np.floor(band[:, :2] / CELL).astype(int) - lo
    occ[idx[:, 1], idx[:, 0]] = True
    return occ, lo


def inflate(occ: np.ndarray, radius_cells: int) -> np.ndarray:
    """Dilate obstacles with a disc of ``radius_cells``."""
    out = occ.copy()
    offsets = [(dy, dx)
               for dy in range(-radius_cells, radius_cells + 1)
               for dx in range(-radius_cells, radius_cells + 1)
               if dy * dy + dx * dx <= radius_cells * radius_cells]
    rows, cols = occ.shape
    ys, xs = np.nonzero(occ)
    for dy, dx in offsets:
        ny, nx = ys + dy, xs + dx
        keep = (ny >= 0) & (ny < rows) & (nx >= 0) & (nx < cols)
        out[ny[keep], nx[keep]] = True
    return out


def flood(free: np.ndarray, seed: tuple[int, int]) -> np.ndarray:
    reach = np.zeros_like(free)
    if not free[seed]:
        return reach
    queue = deque([seed])
    reach[seed] = True
    rows, cols = free.shape
    while queue:
        y, x = queue.popleft()
        for ny, nx in ((y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)):
            if 0 <= ny < rows and 0 <= nx < cols and free[ny, nx] and not reach[ny, nx]:
                reach[ny, nx] = True
                queue.append((ny, nx))
    return reach


def clearance(free: np.ndarray) -> np.ndarray:
    """Chamfer distance in cells from each free cell to the nearest obstacle."""
    big = free.size + 1
    dist = np.where(free, big, 0).astype(np.int32)
    rows, cols = dist.shape
    for y in range(rows):
        for x in range(cols):
            if dist[y, x] == 0:
                continue
            best = dist[y, x]
            if y:
                best = min(best, dist[y - 1, x] + 1)
            if x:
                best = min(best, dist[y, x - 1] + 1)
            dist[y, x] = best
    for y in range(rows - 1, -1, -1):
        for x in range(cols - 1, -1, -1):
            if dist[y, x] == 0:
                continue
            best = dist[y, x]
            if y + 1 < rows:
                best = min(best, dist[y + 1, x] + 1)
            if x + 1 < cols:
                best = min(best, dist[y, x + 1] + 1)
            dist[y, x] = best
    return dist


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--radius", type=float, default=0.55,
                        help="robot clearance radius in metres")
    parser.add_argument("--seed", type=float, nargs=2, default=(0.0, 0.0),
                        help="seed point x y inside the hall")
    parser.add_argument("--stride", type=int, default=2,
                        help="print every Nth cell (map readability)")
    args = parser.parse_args()

    points = np.load(CACHE)
    occ, lo = build_grid(points)
    radius_cells = int(round(args.radius / CELL))
    blocked = inflate(occ, radius_cells)
    free = ~blocked
    seed = (int(round(args.seed[1] / CELL)) - lo[1], int(round(args.seed[0] / CELL)) - lo[0])
    reach = flood(free, seed)
    area = reach.sum() * CELL * CELL
    print(f"grid {occ.shape}  cell {CELL} m  clearance radius {args.radius} m")
    print(f"reachable area from {tuple(args.seed)}: {area:.1f} m^2")

    dist = clearance(reach)
    ys, xs = np.nonzero(reach)
    world_x = (xs + lo[0]) * CELL
    world_y = (ys + lo[1]) * CELL
    print(f"reachable x range {world_x.min():.2f} .. {world_x.max():.2f}")
    print(f"reachable y range {world_y.min():.2f} .. {world_y.max():.2f}")

    order = np.argsort(-dist[ys, xs])[:12]
    print("\nmost open reachable spots (x, y, distance-to-obstacle m):")
    for k in order:
        print(f"  ({world_x[k]:6.2f}, {world_y[k]:6.2f})  {dist[ys[k], xs[k]] * CELL:.2f}")

    stride = args.stride
    print("\n'R' reachable  '+' free but cut off  '#' obstacle (uninflated)\n")
    rows, cols = occ.shape
    for row in range(rows - 1, -1, -stride):
        line = []
        for col in range(0, cols, stride):
            block_r = reach[row:row + stride, col:col + stride].any()
            block_f = free[row:row + stride, col:col + stride].any()
            block_o = occ[row:row + stride, col:col + stride].any()
            line.append("R" if block_r else ("+" if block_f else ("#" if block_o else " ")))
        print(f"{(row + lo[1]) * CELL:7.1f} |{''.join(line)}")
    print(f"        +{'-' * len(range(0, cols, stride))}")
    print(f"        x {lo[0] * CELL:.1f} .. {(lo[0] + cols) * CELL:.1f}, "
          f"one char = {stride * CELL:.1f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
