"""Save, load and check SLAM occupancy maps against the exhibition layout.

The map files follow the nav2 map_server convention (trinary PGM + YAML), so a
map saved here loads in map_server/RViz and vice versa.  Cells use the ROS
``nav_msgs/OccupancyGrid`` order: index ``row * width + col`` with row 0 at the
map origin (bottom), values -1 unknown, 0 free, 100 occupied.

The check answers PLAN M2.1: "墙体偏差 < 0.2 m".  Every occupied cell in the
SLAM map is measured against the nearest rectangle the scene says is solid
(baked hall walls, exhibits, static visitors).  Walking actors have no
collision, so the lidar never sees them and they are left out.
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

from .scene_config import load_scene, world_footprint

FREE, UNKNOWN, OCCUPIED = 0, -1, 100
_PGM_FREE, _PGM_UNKNOWN, _PGM_OCCUPIED = 254, 205, 0
# 0.196 (ROS 1 default) keeps the 205 "unknown" shade unknown: 1 - 205/255 = 0.19608.
OCCUPIED_THRESH, FREE_THRESH = 0.65, 0.196


@dataclass
class GridMap:
    width: int
    height: int
    resolution: float
    origin: tuple[float, float, float]
    cells: list[int]

    def __post_init__(self):
        if self.width <= 0 or self.height <= 0 or len(self.cells) != self.width * self.height:
            raise ValueError("map size does not match its cell count")
        if not (math.isfinite(self.resolution) and self.resolution > 0):
            raise ValueError("map resolution must be positive")
        if abs(self.origin[2]) > 1e-9:
            raise ValueError("rotated map origins are not supported")

    def cell_center(self, index: int) -> tuple[float, float]:
        row, col = divmod(index, self.width)
        return (self.origin[0] + (col + 0.5) * self.resolution,
                self.origin[1] + (row + 0.5) * self.resolution)

    def count(self, value: int) -> int:
        return sum(1 for cell in self.cells if cell == value)


def _classify(value: int) -> int:
    if value < 0:
        return UNKNOWN
    if value >= OCCUPIED_THRESH * 100:
        return OCCUPIED
    if value <= FREE_THRESH * 100:
        return FREE
    return UNKNOWN


def from_occupancy_grid(message) -> GridMap:
    """Convert a ``nav_msgs/OccupancyGrid`` message (duck-typed)."""

    info = message.info
    origin = info.origin
    q = origin.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return GridMap(int(info.width), int(info.height), float(info.resolution),
                   (float(origin.position.x), float(origin.position.y), yaw),
                   [_classify(int(v)) for v in message.data])


def save_map(grid: GridMap, stem: str | Path) -> tuple[Path, Path]:
    """Write ``<stem>.pgm`` and ``<stem>.yaml`` (map_server trinary format)."""

    stem = Path(stem)
    stem.parent.mkdir(parents=True, exist_ok=True)
    pgm, yaml = stem.with_suffix(".pgm"), stem.with_suffix(".yaml")
    pixels = bytearray()
    # PGM rows run top to bottom, grid rows bottom to top.
    for row in range(grid.height - 1, -1, -1):
        for cell in grid.cells[row * grid.width:(row + 1) * grid.width]:
            pixels.append(_PGM_OCCUPIED if cell == OCCUPIED else
                          _PGM_FREE if cell == FREE else _PGM_UNKNOWN)
    pgm.write_bytes(f"P5\n{grid.width} {grid.height}\n255\n".encode("ascii") + bytes(pixels))
    ox, oy, oyaw = grid.origin
    yaml.write_text(
        f"image: {pgm.name}\n"
        "mode: trinary\n"
        f"resolution: {grid.resolution:.6g}\n"
        f"origin: [{ox:.6g}, {oy:.6g}, {oyaw:.6g}]\n"
        "negate: 0\n"
        f"occupied_thresh: {OCCUPIED_THRESH}\n"
        f"free_thresh: {FREE_THRESH}\n", encoding="utf-8")
    return pgm, yaml


def _read_pgm(path: Path) -> tuple[int, int, int, bytes]:
    data = path.read_bytes()
    tokens: list[bytes] = []
    pos = 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":
            pos = data.index(b"\n", pos) + 1
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    if tokens[0] != b"P5":
        raise ValueError(f"{path}: only binary PGM (P5) is supported")
    width, height, maxval = (int(t) for t in tokens[1:])
    if maxval > 255:
        raise ValueError(f"{path}: 16-bit PGM is not supported")
    pixels = data[pos + 1:pos + 1 + width * height]
    if len(pixels) != width * height:
        raise ValueError(f"{path}: truncated image")
    return width, height, maxval, pixels


def _parse_yaml(text: str) -> dict:
    """Parse the flat ``key: value`` YAML map_server writes."""

    result: dict = {}
    for line in text.splitlines():
        line = line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = (part.strip() for part in line.split(":", 1))
        if value.startswith("["):
            result[key] = [float(v) for v in value.strip("[]").split(",")]
        else:
            try:
                result[key] = float(value)
            except ValueError:
                result[key] = value.strip("'\"")
    return result


def load_map(yaml_path: str | Path) -> GridMap:
    yaml_path = Path(yaml_path)
    meta = _parse_yaml(yaml_path.read_text(encoding="utf-8"))
    width, height, maxval, pixels = _read_pgm(yaml_path.parent / str(meta["image"]))
    negate = bool(meta.get("negate", 0))
    occupied = float(meta.get("occupied_thresh", OCCUPIED_THRESH))
    free = float(meta.get("free_thresh", FREE_THRESH))
    cells = [UNKNOWN] * (width * height)
    for i, value in enumerate(pixels):
        p = value / maxval
        occ = p if negate else 1.0 - p
        row, col = divmod(i, width)
        index = (height - 1 - row) * width + col
        cells[index] = OCCUPIED if occ > occupied else FREE if occ < free else UNKNOWN
    origin = meta.get("origin", [0.0, 0.0, 0.0])
    return GridMap(width, height, float(meta["resolution"]),
                   (float(origin[0]), float(origin[1]), float(origin[2])), cells)


def ground_truth_rectangles(config: dict) -> list[tuple[float, float, float, float]]:
    """Solid footprints of the scene as (left, bottom, right, top)."""

    rectangles = []
    room = config.get("room")
    if room and "hall" not in config:
        # Generated box room: four walls centred on the room border.
        # An imported hall mesh brings its own walls via nav_obstacles.
        w, d, t = float(room["width"]), float(room["depth"]), float(room["wall_thickness"])
        rectangles += [(-w / 2, d / 2 - t / 2, w / 2, d / 2 + t / 2),
                       (-w / 2, -d / 2 - t / 2, w / 2, -d / 2 + t / 2),
                       (w / 2 - t / 2, -d / 2, w / 2 + t / 2, d / 2),
                       (-w / 2 - t / 2, -d / 2, -w / 2 + t / 2, d / 2)]
    for field in ("nav_obstacles", "exhibits", "obstacles", "people"):
        for item in config.get(field, ()):
            if field == "people" and item.get("kind") != "static":
                continue
            extent = world_footprint(item)
            position = item.get("position")
            if extent is None or not position:
                continue
            x, y = float(position[0]), float(position[1])
            rectangles.append((x - extent[0] / 2, y - extent[1] / 2,
                               x + extent[0] / 2, y + extent[1] / 2))
    return rectangles


def named_objects(config: dict) -> list[tuple[str, tuple[float, float, float, float]]]:
    """Walls and objects by name, for the per-object "was it mapped" check."""

    named = []
    room = config.get("room")
    if room and "hall" not in config:
        names = ("wall_north", "wall_south", "wall_east", "wall_west")
        named += list(zip(names, ground_truth_rectangles({"room": room})))
    for field in ("exhibits", "obstacles", "people"):
        for item in config.get(field, ()):
            rects = ground_truth_rectangles({field: [item]})
            if rects:
                name = item.get("exhibit_id") or item.get("name") or field
                named.append((str(name), rects[0]))
    return named


def detected_objects(grid: GridMap, named, tolerance: float = 0.2) -> dict[str, bool]:
    """Whether each object has at least one occupied map cell on or near it."""

    result = {}
    res = grid.resolution
    ox, oy = grid.origin[0], grid.origin[1]
    for name, (left, bottom, right, top) in named:
        c0 = max(0, math.floor((left - tolerance - ox) / res))
        c1 = min(grid.width - 1, math.floor((right + tolerance - ox) / res))
        r0 = max(0, math.floor((bottom - tolerance - oy) / res))
        r1 = min(grid.height - 1, math.floor((top + tolerance - oy) / res))
        result[name] = any(grid.cells[r * grid.width + c] == OCCUPIED
                           for r in range(r0, r1 + 1) for c in range(c0, c1 + 1))
    return result


class _NearestRectangle:
    """Distance to the nearest rectangle, bucketed so a map check stays fast."""

    def __init__(self, rectangles: Sequence[tuple[float, float, float, float]], reach: float = 1.0):
        self._reach = reach
        self._buckets: dict[tuple[int, int], list] = {}
        for rect in rectangles:
            left, bottom, right, top = rect
            for ix in range(math.floor((left - reach) / reach), math.floor((right + reach) / reach) + 1):
                for iy in range(math.floor((bottom - reach) / reach), math.floor((top + reach) / reach) + 1):
                    self._buckets.setdefault((ix, iy), []).append(rect)

    def distance(self, x: float, y: float) -> float:
        """Exact distance up to ``reach``; ``inf`` beyond it."""

        best = math.inf
        for left, bottom, right, top in self._buckets.get(
                (math.floor(x / self._reach), math.floor(y / self._reach)), ()):
            dx = max(left - x, 0.0, x - right)
            dy = max(bottom - y, 0.0, y - top)
            best = min(best, math.hypot(dx, dy))
        return best if best <= self._reach else math.inf


def _percentile(values: Sequence[float], q: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, max(0, math.ceil(q * len(ordered)) - 1))]


def compare_map(grid: GridMap, rectangles: Sequence[tuple[float, float, float, float]],
                tolerance: float = 0.2) -> dict:
    """Wall deviation of the SLAM map against the scene layout.

    ``p95_m`` is the headline number: 95 % of occupied cells lie within it of a
    real wall.  A single stray cell (e.g. a scan through a doorway frame) must
    not fail the whole map, which is why the pass rule is a percentile.
    """

    index = _NearestRectangle(rectangles)
    errors = [index.distance(*grid.cell_center(i))
              for i, cell in enumerate(grid.cells) if cell == OCCUPIED]
    finite = [e for e in errors if math.isfinite(e)]
    capped = [min(e, 1.0) for e in errors]
    area = grid.resolution ** 2
    within = sum(1 for e in errors if e <= tolerance)
    p95 = _percentile(capped, 0.95)
    return {
        "occupied_cells": len(errors),
        "free_area_m2": round(grid.count(FREE) * area, 2),
        "mean_m": round(sum(capped) / len(capped), 4) if capped else None,
        "p95_m": round(p95, 4) if capped else None,
        "max_m": round(max(finite), 4) if finite else None,
        "beyond_1m": len(errors) - len(finite),
        "within_tolerance_ratio": round(within / len(errors), 4) if errors else None,
        "tolerance_m": tolerance,
        "pass": bool(errors) and p95 <= tolerance,
    }


def _png(width: int, height: int, rgb: bytes) -> bytes:
    def chunk(kind: bytes, body: bytes) -> bytes:
        return (struct.pack(">I", len(body)) + kind + body
                + struct.pack(">I", zlib.crc32(kind + body) & 0xFFFFFFFF))
    stride = width * 3
    raw = b"".join(b"\x00" + rgb[r * stride:(r + 1) * stride] for r in range(height))
    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def render_overlay(grid: GridMap, rectangles: Iterable[tuple[float, float, float, float]],
                   path: str | Path, tolerance: float = 0.2, scale: int = 2) -> Path:
    """PNG overlay: scene walls green, SLAM walls black (red if off by > tolerance)."""

    index = _NearestRectangle(list(rectangles))
    w, h = grid.width * scale, grid.height * scale
    image = bytearray(w * h * 3)
    colors = {FREE: (250, 250, 250), UNKNOWN: (200, 200, 205)}
    for i, cell in enumerate(grid.cells):
        x, y = grid.cell_center(i)
        distance = index.distance(x, y)
        if cell == OCCUPIED:
            color = (20, 20, 20) if distance <= tolerance else (220, 30, 30)
        elif distance == 0.0:
            color = (120, 200, 120)
        else:
            color = colors[cell]
        row, col = divmod(i, grid.width)
        for dy in range(scale):
            base = ((grid.height - 1 - row) * scale + dy) * w * 3 + col * scale * 3
            image[base:base + scale * 3] = bytes(color) * scale
    path = Path(path)
    path.write_bytes(_png(w, h, bytes(image)))
    return path


def check_map(yaml_path: str | Path, scene_path: str | Path, tolerance: float = 0.2,
              overlay: str | Path | None = None) -> dict:
    grid = load_map(yaml_path)
    config = load_scene(scene_path)
    rectangles = ground_truth_rectangles(config)
    report = compare_map(grid, rectangles, tolerance)
    detected = detected_objects(grid, named_objects(config), tolerance)
    report["objects_detected"] = [name for name, seen in detected.items() if seen]
    report["objects_missed"] = [name for name, seen in detected.items() if not seen]
    report["map"] = str(yaml_path)
    if overlay is not None:
        report["overlay"] = str(render_overlay(grid, rectangles, overlay, tolerance))
    return report


def default_scene() -> Path:
    """The scene JSON next to this package (source tree or installed share)."""

    source = Path(__file__).resolve().parents[1] / "config" / "exhibition.json"
    if source.is_file():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("g1_guide")) / "config" / "exhibition.json"


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="检查 SLAM 地图与展馆布局的墙体偏差")
    parser.add_argument("map_yaml")
    parser.add_argument("--scene", default=None, help="展馆配置，默认 config/exhibition.json")
    parser.add_argument("--tolerance", type=float, default=0.2)
    parser.add_argument("--overlay", default=None, help="叠加图 PNG；默认与地图同名 _overlay.png")
    args = parser.parse_args(argv)
    overlay = args.overlay or str(Path(args.map_yaml).with_name(Path(args.map_yaml).stem + "_overlay.png"))
    report = check_map(args.map_yaml, args.scene or default_scene(), args.tolerance, overlay)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
