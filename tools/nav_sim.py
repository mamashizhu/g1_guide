#!/usr/bin/env python3
"""离线导航仿真：不开 Gazebo，几秒钟跑完所有展位组合，自动判定卡死。

机器人按独轮车模型积分（假设速度指令立即生效），导航雷达用光线投射对场景矩形生成
（360 线、0.12–8 m、5 Hz，只计入跨过雷达高度的物体，和 URDF 一致）。决策逻辑
直接用 g1_guide.nav_core.NavigatorCore，与 ROS 节点完全同一份代码。

用法（WSL 里，不需要 source ROS）：
  python3 tools/nav_sim.py                       # 简单展馆，所有展位两两互达
  python3 tools/nav_sim.py --scene hall          # 导入的大展馆
  python3 tools/nav_sim.py --tour 1 3 5 2 -v     # 从出生点依次去这些展位，打印状态变化
  python3 tools/nav_sim.py --tour 3 --box 0.5,0  # 额外放 0.8 m 箱子（可多个）
  python3 tools/nav_sim.py --tour 3 --box-ahead 2:1.3   # 第 2 秒时在机器人正前方 1.3 m 放箱子

判定：到达 = arrived；卡死 = 连续 20 s（仿真时间）没离开半径 0.5 m 的范围；
超时 = 150 s 未到；碰撞 = 机器人中心离障碍物表面 < 0.30 m。
"""

from __future__ import annotations

import argparse
import itertools
import math
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ws/src/g1_guide"))

from g1_guide.nav_control import Pose2D  # noqa: E402
from g1_guide.nav_core import NavigatorCore  # noqa: E402
from g1_guide.nav_grid import _box  # noqa: E402
from g1_guide.scene_config import load_scene  # noqa: E402

SCENES = {"simple": ROOT / "ws/src/g1_guide/config/exhibition_simple.json",
          "hall": ROOT / "ws/src/g1_guide/config/exhibition.json"}
DT = 0.1                 # 节点 tick 周期
SCAN_PERIOD = 0.2        # 导航雷达 5 Hz
BEAMS = 360
RANGE_MIN, RANGE_MAX = 0.12, 8.0
LIDAR_ABOVE_BASE = 0.47  # nav_lidar_joint
BOX_SIZE = 0.8           # tools/spawn_box.sh 的箱子
BODY_RADIUS = 0.30
STUCK_WINDOW, STUCK_RADIUS, LEG_TIMEOUT = 20.0, 0.5, 150.0


def scene_boxes(config: dict, lidar_z: float) -> tuple[np.ndarray, np.ndarray]:
    """(all static AABBs, the subset the lidar plane intersects)."""
    everything, seen = [], []
    room = config["room"]
    hx, hy, t = float(room["width"])/2, float(room["depth"])/2, float(room["wall_thickness"])
    walls = [(-hx, -hy, hx, -hy+t), (-hx, hy-t, hx, hy), (-hx, -hy, -hx+t, hy), (hx-t, -hy, hx, hy)]
    everything += walls
    seen += walls
    for field in ("exhibits", "obstacles", "nav_obstacles", "people"):
        for item in config.get(field, ()):
            if field == "people" and item.get("kind") != "static":
                continue
            box = _box(item.get("position"), item.get("size"), item.get("yaw", 0.0))
            if box is None:
                continue
            everything.append(box)
            position, size = item.get("position", ()), item.get("size", ())
            if len(position) >= 3 and len(size) >= 3:
                low, high = float(position[2]) - float(size[2])/2, float(position[2]) + float(size[2])/2
                if not low <= lidar_z <= high:
                    continue
            seen.append(box)
    return np.array(everything, float), np.array(seen, float)


def cast(boxes: np.ndarray, x: float, y: float) -> np.ndarray:
    """Ranges of BEAMS rays from (x, y) against AABBs (slab method)."""
    angles = -math.pi + np.arange(BEAMS) * (2*math.pi/BEAMS)
    dx, dy = np.cos(angles)[:, None], np.sin(angles)[:, None]
    with np.errstate(divide="ignore", invalid="ignore"):
        tx1, tx2 = (boxes[None, :, 0]-x)/dx, (boxes[None, :, 2]-x)/dx
        ty1, ty2 = (boxes[None, :, 1]-y)/dy, (boxes[None, :, 3]-y)/dy
    tmin = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
    tmax = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
    hit = (tmax >= np.maximum(tmin, 0.0)) & np.isfinite(tmin)
    distance = np.where(hit, np.maximum(tmin, 0.0), np.inf).min(axis=1)
    return angles, distance


def surface_distance(boxes: np.ndarray, x: float, y: float) -> float:
    ddx = np.maximum.reduce([boxes[:, 0]-x, np.zeros(len(boxes)), x-boxes[:, 2]])
    ddy = np.maximum.reduce([boxes[:, 1]-y, np.zeros(len(boxes)), y-boxes[:, 3]])
    return float(np.hypot(ddx, ddy).min())


def box_at(x: float, y: float) -> tuple[float, float, float, float]:
    h = BOX_SIZE/2
    return (x-h, y-h, x+h, y+h)


class Sim:
    def __init__(self, scene: str, boxes: list[tuple[float, float]], box_ahead=None):
        self.config = load_scene(SCENES[scene])
        spawn = self.config["spawn"]
        self.pose = Pose2D(float(spawn["x"]), float(spawn["y"]), float(spawn["yaw"]))
        lidar_z = float(spawn["z"]) + LIDAR_ABOVE_BASE
        self.all_boxes, self.seen_boxes = scene_boxes(self.config, lidar_z)
        for bx, by in boxes:
            self.add_box(bx, by)
        self.box_ahead = box_ahead  # (sim time, distance) or None
        stations = {e["station"]["key"]: e["station"] for e in self.config["exhibits"]}
        self.core = NavigatorCore(self.config, stations)
        self.t = 0.0
        self.next_scan = 0.0

    def add_box(self, x: float, y: float) -> None:
        box = np.array([box_at(x, y)])
        self.all_boxes = np.vstack([self.all_boxes, box])
        self.seen_boxes = np.vstack([self.seen_boxes, box])

    def scan(self) -> None:
        angles, ranges = cast(self.seen_boxes, self.pose.x, self.pose.y)
        keep = (ranges >= RANGE_MIN) & (ranges <= RANGE_MAX)
        # Lidar angles are in the robot frame; cast() already used world angles,
        # which is the same set of rays for a full 360° scan.
        points = [(self.pose.x + r*math.cos(a), self.pose.y + r*math.sin(a))
                  for a, r in zip(angles[keep], ranges[keep])]
        self.core.set_scan(points, (self.pose.x, self.pose.y))

    def leg(self, key: str, verbose: bool) -> dict:
        status = self.core.set_goal(key)
        start_t = self.t
        anchor, anchor_t = (self.pose.x, self.pose.y), self.t
        states: dict[str, int] = {}
        last_state, travelled, min_surface = None, 0.0, math.inf
        history: list[str] = []
        result = "timeout"
        while self.t - start_t < LEG_TIMEOUT:
            if self.box_ahead and self.t >= self.box_ahead[0]:
                d = self.box_ahead[1]
                self.add_box(self.pose.x + d*math.cos(self.pose.yaw), self.pose.y + d*math.sin(self.pose.yaw))
                history.append(f"  t={self.t:6.1f}  放箱子于正前方 {d} m")
                self.box_ahead = None
            if self.t >= self.next_scan:
                self.scan()
                self.next_scan += SCAN_PERIOD
            command = self.core.step(self.pose, self.t)
            if command.status is not None:
                status = command.status
                state = status["state"]
                states[state] = states.get(state, 0) + 1
                if state != last_state:
                    extra = {k: v for k, v in status.items()
                             if k in ("distance_m", "clearance_m", "front_m", "rear_m",
                                      "start_dynamic_clear", "goal_dynamic_clear", "static_path_exists")}
                    history.append(f"  t={self.t:6.1f}  ({self.pose.x:6.2f},{self.pose.y:6.2f},"
                                   f"{math.degrees(self.pose.yaw):5.0f}°)  {state:11s} {extra}")
                    last_state = state
                if state == "arrived":
                    result = "arrived"
                    break
            v, w = command.linear_x, command.angular_z
            yaw = self.pose.yaw + w*DT
            nx = self.pose.x + v*DT*math.cos(self.pose.yaw + w*DT/2)
            ny = self.pose.y + v*DT*math.sin(self.pose.yaw + w*DT/2)
            travelled += math.hypot(nx-self.pose.x, ny-self.pose.y)
            self.pose = Pose2D(nx, ny, math.atan2(math.sin(yaw), math.cos(yaw)))
            self.t += DT
            min_surface = min(min_surface, surface_distance(self.all_boxes, nx, ny))
            if min_surface < BODY_RADIUS:
                result = "collision"
                break
            if math.hypot(nx-anchor[0], ny-anchor[1]) > STUCK_RADIUS:
                anchor, anchor_t = (nx, ny), self.t
            elif self.t - anchor_t > STUCK_WINDOW:
                result = "stuck"
                break
        if result != "arrived":
            self.core.set_goal("0")
        out = dict(key=key, result=result, sim_s=round(self.t-start_t, 1), path_m=round(travelled, 1),
                   min_surface_m=round(min_surface, 2), states=states,
                   pose=(round(self.pose.x, 2), round(self.pose.y, 2), round(math.degrees(self.pose.yaw))))
        if verbose or result != "arrived":
            out["history"] = history
        return out


def report(label: str, leg: dict) -> bool:
    ok = leg["result"] == "arrived"
    mark = "OK " if ok else "!! "
    print(f"{mark}{label:12s} {leg['result']:9s} {leg['sim_s']:6.1f}s  路程 {leg['path_m']:5.1f} m  "
          f"最近障碍 {leg['min_surface_m']:.2f} m  终点位姿 {leg['pose']}  {leg['states']}")
    for line in leg.get("history", [])[-25:]:
        print(line)
    return ok


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scene", choices=SCENES, default="simple")
    parser.add_argument("--tour", nargs="*", help="从出生点依次前往的展位；不给则测所有两两组合")
    parser.add_argument("--box", action="append", default=[], help="x,y：放一个 0.8 m 箱子")
    parser.add_argument("--box-ahead", help="t:d：第 t 秒在机器人正前方 d 米放箱子（只用于 --tour）")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()
    boxes = [tuple(map(float, b.split(","))) for b in args.box]
    ahead = tuple(map(float, args.box_ahead.split(":"))) if args.box_ahead else None
    wall = time.monotonic()
    failures = 0
    if args.tour is not None:
        sim = Sim(args.scene, boxes, ahead)
        previous = "出生点"
        for key in args.tour:
            failures += not report(f"{previous}→{key}", sim.leg(key, args.verbose))
            previous = key
    else:
        keys = sorted(e["station"]["key"] for e in load_scene(SCENES[args.scene])["exhibits"])
        for key in keys:
            failures += not report(f"出生点→{key}", Sim(args.scene, boxes).leg(key, args.verbose))
        for a, b in itertools.permutations(keys, 2):
            sim = Sim(args.scene, boxes)
            if sim.leg(a, False)["result"] != "arrived":
                print(f"-- {a}→{b}        跳过（出生点→{a} 已失败）")
                continue
            failures += not report(f"{a}→{b}", sim.leg(b, args.verbose))
    print(f"\n{'全部通过' if not failures else f'{failures} 段失败'}，墙钟 {time.monotonic()-wall:.1f} s")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
