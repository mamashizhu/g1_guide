"""Rasterise the ISCAS museum mesh into a top-down occupancy map.

Reads the Fuel-cached COLLADA file, applies each visual-scene node matrix and
the model.sdf yaw, then samples every triangle so that large flat walls are
covered rather than reduced to their four corners.  Cells touched by geometry
inside the robot's height band are obstacles; cells touched only near the
ground are walkable floor.
"""

from __future__ import annotations

import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
DAE = Path.home() / ".gz/fuel/fuel.gazebosim.org/openrobotics/models/iscas_museum/1/meshes/ISCAS_museum.dae"
MODEL_YAW = -1.57          # from model.sdf <pose>
BAND = (0.20, 1.80)        # obstacle height band the robot cares about
CELL = 0.25
STEP = 0.10                # triangle sampling pitch, metres
CACHE = Path(__file__).resolve().parent / "museum_points.npy"


def tag(name: str) -> str:
    return f"{{{NS['c']}}}{name}"


def geometry_triangles(geo):
    """Return (V, 3) vertex array and (T, 3) index array for one geometry."""
    sources = {}
    for src in geo.iter(tag("source")):
        fa = src.find("c:float_array", NS)
        if fa is not None:
            sources[src.get("id")] = np.fromstring(fa.text, sep=" ")
    position_ids = {}
    for vert in geo.iter(tag("vertices")):
        for inp in vert.findall("c:input", NS):
            if inp.get("semantic") == "POSITION":
                position_ids[vert.get("id")] = inp.get("source")[1:]

    faces = []
    verts = None
    for prim in list(geo.iter(tag("triangles"))) + list(geo.iter(tag("polylist"))):
        inputs = prim.findall("c:input", NS)
        stride = max(int(i.get("offset", 0)) for i in inputs) + 1
        offset = None
        for inp in inputs:
            if inp.get("semantic") == "VERTEX":
                offset = int(inp.get("offset", 0))
                key = inp.get("source")[1:]
                verts = sources[position_ids.get(key, key)].reshape(-1, 3)
        p = prim.find("c:p", NS)
        if offset is None or p is None:
            continue
        idx = np.fromstring(p.text, sep=" ", dtype=float).astype(int)[offset::stride]
        vcount = prim.find("c:vcount", NS)
        if vcount is None:
            faces.append(idx.reshape(-1, 3))
        else:
            counts = np.fromstring(vcount.text, sep=" ", dtype=float).astype(int)
            cursor = 0
            for count in counts:                      # fan-triangulate polygons
                poly = idx[cursor:cursor + count]
                cursor += count
                for k in range(1, count - 1):
                    faces.append(np.array([[poly[0], poly[k], poly[k + 1]]]))
    if verts is None or not faces:
        return None, None
    return verts, np.vstack(faces)


def sample_triangles(verts, faces):
    """Yield points covering each triangle at roughly STEP spacing."""
    a, b, c = verts[faces[:, 0]], verts[faces[:, 1]], verts[faces[:, 2]]
    edge1, edge2 = b - a, c - a
    longest = np.maximum(np.linalg.norm(edge1, axis=1), np.linalg.norm(edge2, axis=1))
    n = np.clip(np.ceil(longest / STEP).astype(int), 1, 240)
    out = [a, b, c]
    for divisions in np.unique(n):
        sel = n == divisions
        if divisions == 1:
            continue
        grid = [(i / divisions, j / divisions)
                for i in range(divisions + 1) for j in range(divisions + 1 - i)]
        uv = np.array(grid)
        out.append((a[sel][:, None, :]
                    + edge1[sel][:, None, :] * uv[None, :, 0, None]
                    + edge2[sel][:, None, :] * uv[None, :, 1, None]).reshape(-1, 3))
    return np.vstack(out)


def main() -> int:
    root = ET.parse(DAE).getroot()
    geometries = {}
    for geo in root.iter(tag("geometry")):
        verts, faces = geometry_triangles(geo)
        if verts is not None:
            geometries[geo.get("id")] = (verts, faces)

    cy, sy = math.cos(MODEL_YAW), math.sin(MODEL_YAW)
    yaw = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])

    chunks = []
    for node in root.iter(tag("node")):
        inst = node.find("c:instance_geometry", NS)
        mat_el = node.find("c:matrix", NS)
        if inst is None or mat_el is None:
            continue
        entry = geometries.get(inst.get("url")[1:])
        if entry is None:
            continue
        m = np.fromstring(mat_el.text, sep=" ").reshape(4, 4)
        pts = sample_triangles(*entry)
        chunks.append((pts @ m[:3, :3].T + m[:3, 3]) @ yaw.T)
    if not chunks:
        print("no geometry found", file=sys.stderr)
        return 1
    pts = np.vstack(chunks)
    CACHE.parent.mkdir(parents=True, exist_ok=True)
    np.save(CACHE, pts.astype(np.float32))
    print(f"cached -> {CACHE}")

    print(f"sampled points: {len(pts)}")
    print("bbox min:", np.round(pts.min(axis=0), 2))
    print("bbox max:", np.round(pts.max(axis=0), 2))

    lo = np.floor(pts[:, :2].min(axis=0) / CELL).astype(int)
    hi = np.ceil(pts[:, :2].max(axis=0) / CELL).astype(int)
    shape = (hi[1] - lo[1] + 1, hi[0] - lo[0] + 1)
    occ = np.zeros(shape, dtype=bool)
    flr = np.zeros(shape, dtype=bool)
    for mask, grid in (((pts[:, 2] > BAND[0]) & (pts[:, 2] < BAND[1]), occ),
                       (pts[:, 2] <= BAND[0], flr)):
        idx = np.floor(pts[mask][:, :2] / CELL).astype(int) - lo
        grid[idx[:, 1], idx[:, 0]] = True

    print(f"\ncell = {CELL} m, rows top = +y, '#' obstacle  '.' floor  ' ' void\n")
    header = "".join(str(abs(int((lo[0] + c) * CELL)) % 10) if (lo[0] + c) % 4 == 0 else " "
                     for c in range(shape[1]))
    print(f"        |{header}")
    for row in range(shape[0] - 1, -1, -1):
        y = (lo[1] + row) * CELL
        line = "".join("#" if occ[row, col] else ("." if flr[row, col] else " ")
                       for col in range(shape[1]))
        print(f"{y:7.1f} |{line}")
    print(f"        +{'-' * shape[1]}")
    print(f"        x from {lo[0] * CELL:.1f} to {hi[0] * CELL:.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
