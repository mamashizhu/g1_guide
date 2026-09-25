"""Shared loading and geometry helpers for the exhibition scene description.

The scene JSON is read by both the launch file (to generate the world) and the
navigation node (to build the occupancy grid), so the two must agree on what a
footprint is.  Keeping that agreement here means a rotated poster cannot be
drawn in one place and planned around in another.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence


def load_scene(path: str | Path) -> dict:
    """Read a scene JSON and inline the baked hall obstacles it references.

    ``nav_obstacles_file`` is resolved relative to the scene file so the
    package can ship the 300-odd baked rectangles separately from the handful
    of exhibit entries a human edits.
    """

    scene_path = Path(path)
    config = json.loads(scene_path.read_text(encoding="utf-8"))
    reference = config.pop("nav_obstacles_file", None)
    if reference is None:
        return config
    if not isinstance(reference, str) or Path(reference).name != reference:
        raise ValueError("nav_obstacles_file must be a filename next to the scene file")
    payload = json.loads((scene_path.parent / reference).read_text(encoding="utf-8"))
    baked = payload.get("nav_obstacles")
    if not isinstance(baked, list):
        raise ValueError(f"{reference}: nav_obstacles must be a list")
    config["nav_obstacles"] = list(config.get("nav_obstacles", ())) + baked
    return config


def world_footprint(item: Mapping[str, Any]) -> tuple[float, float] | None:
    """Return the axis-aligned (size_x, size_y) an item occupies in the world.

    ``size`` is authored in the item's own frame; a poster hung on a side wall
    carries a yaw, so its width and thickness swap in world axes.  Arbitrary
    yaws fall back to the bounding box of the rotated rectangle, which is
    conservative and therefore safe for clearance checks.
    """

    size = item.get("size")
    if not isinstance(size, Sequence) or isinstance(size, (str, bytes)) or len(size) < 2:
        return None
    try:
        sx, sy = float(size[0]), float(size[1])
    except (TypeError, ValueError):
        return None
    if not (math.isfinite(sx) and math.isfinite(sy)) or sx <= 0.0 or sy <= 0.0:
        return None
    yaw = item.get("yaw", 0.0)
    if isinstance(yaw, bool) or not isinstance(yaw, (int, float)) or not math.isfinite(yaw):
        return None
    cos, sin = abs(math.cos(float(yaw))), abs(math.sin(float(yaw)))
    return sx * cos + sy * sin, sx * sin + sy * cos


__all__ = ["load_scene", "world_footprint"]
