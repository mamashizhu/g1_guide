"""Bisect the real generated world by removing one group of entities at a time.

``bisect_load.py`` cleared every imported model, so the hang comes from what
those minimal probes left out: the exhibits, the walking actors, or the
rendering sensor.  This strips each group from the real world and reports
which removal lets the server step.
"""

from __future__ import annotations

import copy
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parents[1] / "ws/src/g1_guide"
sys.path.insert(0, str(PACKAGE))

from g1_guide.exhibition_world import generate_world      # noqa: E402
from g1_guide.scene_config import load_scene              # noqa: E402

ITERATIONS = 50
TIMEOUT = 90


def strip(world: ET.Element, predicate) -> int:
    removed = 0
    for child in list(world):
        if predicate(child):
            world.remove(child)
            removed += 1
    return removed


def _use_ogre(world: ET.Element) -> int:
    changed = 0
    for plugin in world.findall("plugin"):
        engine = plugin.find("render_engine")
        if engine is not None:
            engine.text = "ogre"
            changed += 1
    return changed


def _no_shadows(world: ET.Element) -> int:
    changed = 0
    for scene in world.findall("scene"):
        shadows = scene.find("shadows")
        if shadows is not None:
            shadows.text = "false"
            changed += 1
    for light in world.findall("light"):
        cast = light.find("cast_shadows")
        if cast is not None:
            cast.text = "false"
            changed += 1
    return changed


VARIANTS = {
    "full": lambda w: 0,
    "no_actors": lambda w: strip(w, lambda e: e.tag == "actor"),
    "no_overview_camera": lambda w: strip(
        w, lambda e: e.tag == "model" and e.get("name") == "exhibition_overview"),
    "no_sensors_plugin": lambda w: strip(
        w, lambda e: e.tag == "plugin" and "Sensors" in (e.get("name") or "")) + strip(
        w, lambda e: e.tag == "model" and e.get("name") == "exhibition_overview"),
    "no_exhibits": lambda w: strip(
        w, lambda e: e.tag == "model" and str(e.get("name")).startswith("exhibit_")),
    "no_stations": lambda w: strip(
        w, lambda e: e.tag == "model" and str(e.get("name")).startswith("station_")),
    "no_people": lambda w: strip(w, lambda e: e.tag == "actor") + strip(
        w, lambda e: e.tag == "include" and e.findtext("name") != "exhibition_hall"),
    "one_actor": lambda w: strip(
        w, lambda e: e.tag == "actor" and e.get("name") == "walker_corridor"),
    "only_corridor_actor": lambda w: strip(
        w, lambda e: e.tag == "actor" and e.get("name") == "walker_gallery"),
    "ogre_engine": _use_ogre,
    "ogre_engine_no_shadows": lambda w: _use_ogre(w) + _no_shadows(w),
}


def main() -> int:
    config = load_scene(PACKAGE / "config/exhibition.json")
    base = ET.fromstring(generate_world(config, PACKAGE / "assets/exhibition"))
    only = sys.argv[1:]

    for name, mutate in VARIANTS.items():
        if only and name not in only:
            continue
        root = copy.deepcopy(base)
        removed = mutate(root.find("world"))
        path = HERE / f"variant_{name}.sdf"
        path.write_bytes(ET.tostring(root, encoding="utf-8", xml_declaration=True))
        env = dict(os.environ, GZ_PARTITION=f"hallvar_{name}")
        started = time.time()
        try:
            done = subprocess.run(
                ["gz", "sim", "-s", "-r", "--iterations", str(ITERATIONS), str(path)],
                capture_output=True, text=True, timeout=TIMEOUT, env=env)
            ok, elapsed = done.returncode == 0, time.time() - started
            note = ""
        except subprocess.TimeoutExpired:
            ok, elapsed, note = False, time.time() - started, "hung"
        print(f"{'STEPS ' if ok else 'HANGS '} {name:<20} removed={removed:<3} "
              f"{elapsed:6.1f} s  {note}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
