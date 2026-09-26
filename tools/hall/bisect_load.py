"""Find which imported model stops the Gazebo server from stepping.

The full world loads its meshes and then hangs before physics runs, so
``SceneBroadcaster`` never sees any state.  This builds one minimal world per
suspect model and reports whether the server reaches its iteration limit.
Each run uses its own GZ_PARTITION so it cannot disturb other sessions.
"""

from __future__ import annotations

import os
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ITERATIONS = 50
TIMEOUT = 90

HEADER = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="probe">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <model name="floor"><static>true</static><pose>0 0 -0.05 0 0 0</pose>
      <link name="link"><collision name="c"><geometry><box><size>22 24 0.1</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>22 24 0.1</size></box></geometry></visual></link></model>
"""
FOOTER = "  </world>\n</sdf>\n"


def include(name: str, uri: str, pose: str = "0 0 0 0 0 0", static: bool = True) -> str:
    return (f"    <include><name>{name}</name><uri>{uri}</uri><pose>{pose}</pose>"
            f"<static>{str(static).lower()}</static></include>\n")


FUEL = "https://fuel.gazebosim.org/1.0/OpenRobotics/models"
CASES = {
    "floor_only": "",
    "hall": include("hall", f"{FUEL}/iscas_museum"),
    "casual_female": include("p1", f"{FUEL}/Casual female", "5.5 4.5 0 0 0 0"),
    "female_visitor": include("p2", f"{FUEL}/FemaleVisitor", "-6 5 0 0 0 0"),
    "standing_person": include("p3", f"{FUEL}/Standing person", "0.9 5.6 0 0 0 0"),
    "all_people": (include("p1", f"{FUEL}/Casual female", "5.5 4.5 0 0 0 0")
                   + include("p2", f"{FUEL}/FemaleVisitor", "-6 5 0 0 0 0")
                   + include("p3", f"{FUEL}/Standing person", "0.9 5.6 0 0 0 0")),
}


def run(name: str, body: str) -> tuple[bool, float, str]:
    world = HERE / f"probe_{name}.sdf"
    world.write_text(HEADER + body + FOOTER, encoding="utf-8")
    env = dict(os.environ, GZ_PARTITION=f"hallprobe_{name}")
    started = time.time()
    try:
        done = subprocess.run(
            ["gz", "sim", "-s", "-r", "--iterations", str(ITERATIONS), str(world)],
            capture_output=True, text=True, timeout=TIMEOUT, env=env)
    except subprocess.TimeoutExpired:
        return False, time.time() - started, "hung"
    elapsed = time.time() - started
    tail = [line for line in (done.stderr + done.stdout).splitlines()
            if "rror" in line or "arn" in line]
    return done.returncode == 0, elapsed, (tail[-1][:110] if tail else "")


def main() -> int:
    only = sys.argv[1:]
    for name, body in CASES.items():
        if only and name not in only:
            continue
        ok, elapsed, note = run(name, body)
        print(f"{'STEPS ' if ok else 'HANGS '} {name:<18} {elapsed:6.1f} s   {note}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
