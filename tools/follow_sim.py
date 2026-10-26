#!/usr/bin/env python3
"""离线跟随仿真：不开 Gazebo，测 NavigatorCore 的「跟随」(goal key follow)。

和 nav_sim.py 同一套运动学机器人和光线投射雷达；场景里的人是会走动的 0.5×0.4 m 方块
（导航雷达看得见）。两种人员感知按真机方式模拟：
  雷达人群：5 Hz，视线可达且 6 m 内的人，加 5 cm 噪声后交给 people_core.PeopleTracker
            （和 people_tracker 节点同一个跟踪器，会因遮挡断轨、换 ID）；相距 0.75 m 以内的
            人合成一个检测（people_core 按 0.3 m 栅格聚类），分开时 ID 可能落到另一个人身上；
  YOLO：    前置相机 ±35°、8 m，视线可达，10 Hz，延迟 0.25 s，方位加 1° 噪声；
            轨迹 ID 模拟 ByteTrack：连续可见时不变，丢失 1 s 以上换新 ID。
--lidar simple 时没有雷达人群，只能靠 YOLO 方位 + 2D 雷达测距。
人会绕开机器人：离机器人 < 0.9 m 时向侧面让开（0.5 m/s），之后慢慢回到原路线。

用法（WSL，不需要 ROS）：
  python3 tools/follow_sim.py                # 跑全部场景
  python3 tools/follow_sim.py walk cross -v  # 指定场景，打印状态变化
场景：
  walk        领路人从机器人前方出发，经过几个展位走一大圈（大展馆）
  lidar_only  同 walk，没有相机
  vision_only 同 walk，简易 2D 雷达（没有雷达人群）
  cross       领路人走路时，另一个人从领路人身后 0.6 m 横穿（雷达里两人短暂合成一团）
  swap        另一个人和领路人并排走（间距 0.6 m，雷达里是一团）2 m 后向左拐开 1.5 m
  stop_go     领路人停 10 s，再掉头从机器人身边走过
  vanish      领路人中途消失 → 机器人去最后位置找，10 s 后报 follow_lost
  detour      领路人走人能过、机器人过不去的窄口（大展馆 5→1 号展位；简单展馆中间隔墙的缝），不限间距，只要最后追上

判定：不碰撞（机器人中心离障碍物、离人 ≥ 0.30 m）；领路人走动时间距不超过 5 m；领路人停下后
25 s 内机器人进入 follow_waiting、离领路人 ≤ 1.9 m 且朝向误差 ≤ 20°；全程跟的是领路人
（结束时目标估计离领路人 < 0.6 m）；vanish 场景要求最终 follow_lost。
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import random
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent / "ws/src/g1_guide"))

from nav_sim import (BODY_RADIUS, DT, LIDAR_ABOVE_BASE, PERSON_SIZE, RANGE_MAX, RANGE_MIN,  # noqa: E402
                     SCAN_PERIOD, SCENES, cast, scene_boxes, surface_distance)
from g1_guide.follow_core import FOLLOW_DISTANCE, HOLD_BAND  # noqa: E402
from g1_guide.nav_control import Pose2D  # noqa: E402
from g1_guide.nav_core import NavigatorCore  # noqa: E402
from g1_guide.nav_grid import OccupancyGrid  # noqa: E402
from g1_guide.people_core import Detection, PeopleTracker  # noqa: E402
from g1_guide.scene_config import load_scene  # noqa: E402

PEOPLE_PERIOD = 0.2
PEOPLE_RANGE = 6.0
PEOPLE_MERGE = 0.75
VISION_PERIOD = 0.1
VISION_LATENCY = 0.25
VISION_HALF_FOV = math.radians(35)
VISION_RANGE = 8.0
YOLO_GAP_S = 1.0
WARMUP_S = 1.0             # sensors only, before the follow command
AVOID_DISTANCE = 0.9       # people keep this far from the robot ...
AVOID_STEP = 0.05          # ... sidestepping at 0.5 m/s
AVOID_RETURN = 0.02        # back onto their route at 0.2 m/s
WALK_SPEED = 0.5
SETTLE_S = 25.0
MAX_GAP = 5.0
DONE_DISTANCE = FOLLOW_DISTANCE + HOLD_BAND + 0.4
DONE_FACE_DEG = 20.0


class Walker:
    """A person walking a polyline at a fixed speed, with pauses."""

    def __init__(self, name: str, route: list[tuple[float, float]], speed: float = WALK_SPEED,
                 start_t: float = 0.0, pauses: dict[int, float] | None = None,
                 vanish_t: float | None = None):
        self.name, self.route, self.speed, self.start_t = name, route, speed, start_t
        self.pauses = pauses or {}       # waypoint index -> seconds to stand there
        self.vanish_t = vanish_t
        self.segments = []               # (t0, t1, a, b)
        t = start_t
        for i in range(len(route) - 1):
            t += self.pauses.get(i, 0.0)
            a, b = route[i], route[i + 1]
            duration = math.hypot(b[0] - a[0], b[1] - a[1]) / speed
            self.segments.append((t, t + duration, a, b))
            t += duration
        self.end_t = t

    def present(self, t: float) -> bool:
        return self.vanish_t is None or t < self.vanish_t

    def moving(self, t: float) -> bool:
        return any(t0 <= t < t1 for t0, t1, _a, _b in self.segments)

    def at(self, t: float) -> tuple[float, float]:
        position = self.route[0]
        for t0, t1, a, b in self.segments:
            if t < t0:
                return position
            if t < t1:
                s = (t - t0) / (t1 - t0)
                return a[0] + (b[0] - a[0]) * s, a[1] + (b[1] - a[1]) * s
            position = b
        return position


def person_box(x: float, y: float) -> tuple[float, float, float, float]:
    w, d = PERSON_SIZE[0] / 2, PERSON_SIZE[1] / 2
    return (x - w, y - d, x + w, y + d)


def line_of_sight(boxes: np.ndarray, a: tuple[float, float], b: tuple[float, float],
                  skip: tuple[float, float, float, float] | None = None) -> bool:
    """No box (except ``skip``) between a and b."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return True
    for box in boxes:
        if skip is not None and np.allclose(box, skip):
            continue
        with np.errstate(divide="ignore", invalid="ignore"):
            tx1, tx2 = (box[0] - a[0]) / dx, (box[2] - a[0]) / dx
            ty1, ty2 = (box[1] - a[1]) / dy, (box[3] - a[1]) / dy
        tmin = max(min(tx1, tx2), min(ty1, ty2))
        tmax = min(max(tx1, tx2), max(ty1, ty2))
        if np.isfinite(tmin) and tmax >= max(tmin, 0.0) and tmin < 1.0 - 0.3 / length:
            return False
    return True


class FollowSim:
    def __init__(self, scene: str, walkers: list[Walker], lidar: str = "mid360", vision: bool = True,
                 seed: int = 1):
        self.config = load_scene(SCENES[scene])
        spawn = self.config["spawn"]
        self.pose = Pose2D(float(spawn["x"]), float(spawn["y"]), float(spawn["yaw"]))
        lidar_z = float(spawn["z"]) + LIDAR_ABOVE_BASE
        self.all_boxes, self.seen_boxes = scene_boxes(self.config, lidar_z)
        stations = {e["station"]["key"]: e["station"] for e in self.config["exhibits"]}
        self.core = NavigatorCore(self.config, stations)
        self.walkers = walkers
        self.lidar, self.use_vision = lidar, vision
        self.tracker = PeopleTracker()
        self.rng = random.Random(seed)
        self.t = 0.0
        self.next_scan = self.next_people = self.next_vision = 0.0
        self.pose_log: list[tuple[float, Pose2D]] = []
        self.vision_queue: list[tuple[float, list[dict], Pose2D]] = []
        self.yolo_ids: dict[str, tuple[int, float]] = {}   # walker -> (id, last seen)
        self.offsets: dict[str, tuple[float, float]] = {}  # walker -> sidestep around the robot
        self.next_yolo_id = 1

    def people_boxes(self) -> list[tuple[str, tuple[float, float, float, float]]]:
        return [(w.name, person_box(*self.pos(w))) for w in self.walkers if w.present(self.t)]

    def pos(self, w: Walker) -> tuple[float, float]:
        x, y = w.at(self.t)
        ox, oy = self.offsets.get(w.name, (0.0, 0.0))
        return x + ox, y + oy

    def avoid_robot(self) -> None:
        """People step aside around the robot (a blind walker would walk
        straight through a robot standing in its way), and drift back onto
        their route once clear of it."""
        for w in self.walkers:
            ox, oy = self.offsets.get(w.name, (0.0, 0.0))
            x, y = self.pos(w)
            distance = math.hypot(x - self.pose.x, y - self.pose.y)
            if distance < AVOID_DISTANCE:
                ahead = w.at(self.t + 0.5)
                hx, hy = ahead[0] - (x - ox), ahead[1] - (y - oy)
                norm = math.hypot(hx, hy)
                if norm < 1e-6:   # standing: step straight away
                    px, py = (x - self.pose.x) / max(distance, 1e-6), (y - self.pose.y) / max(distance, 1e-6)
                else:
                    px, py = -hy / norm, hx / norm
                    if (x - self.pose.x) * px + (y - self.pose.y) * py < 0:
                        px, py = -px, -py
                ox, oy = ox + px * AVOID_STEP, oy + py * AVOID_STEP
            else:
                size = math.hypot(ox, oy)
                if size > 1e-6:
                    shrink = max(0.0, size - AVOID_RETURN) / size
                    nx, ny = x - ox + ox * shrink, y - oy + oy * shrink
                    if math.hypot(nx - self.pose.x, ny - self.pose.y) >= AVOID_DISTANCE:
                        ox, oy = ox * shrink, oy * shrink
            self.offsets[w.name] = (ox, oy)

    def scan(self) -> None:
        boxes = np.vstack([self.seen_boxes, *[np.array([b]) for _n, b in self.people_boxes()]])
        angles, ranges = cast(boxes, self.pose.x, self.pose.y)
        keep = (ranges >= RANGE_MIN) & (ranges <= RANGE_MAX)
        points = [(self.pose.x + r * math.cos(a), self.pose.y + r * math.sin(a))
                  for a, r in zip(angles[keep], ranges[keep])]
        self.core.set_scan(points, (self.pose.x, self.pose.y), self.t)

    def visible(self, name: str, box) -> bool:
        others = [b for n, b in self.people_boxes() if n != name]
        boxes = np.vstack([self.seen_boxes, *[np.array([b]) for b in others]]) if others else self.seen_boxes
        cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
        return line_of_sight(boxes, (self.pose.x, self.pose.y), (cx, cy))

    def people(self) -> None:
        centres = []
        for name, box in self.people_boxes():
            cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
            if math.hypot(cx - self.pose.x, cy - self.pose.y) > PEOPLE_RANGE or not self.visible(name, box):
                continue
            centres.append([cx, cy, 1])
        # people_core clusters touching 0.3 m cells: two people this close are
        # one detection at their middle, and the tracker may hand the id to
        # either of them when they part.
        merged = True
        while merged:
            merged = False
            for i in range(len(centres)):
                for j in range(i + 1, len(centres)):
                    a, b = centres[i], centres[j]
                    if math.hypot(a[0] - b[0], a[1] - b[1]) <= PEOPLE_MERGE:
                        n = a[2] + b[2]
                        centres[i] = [(a[0] * a[2] + b[0] * b[2]) / n, (a[1] * a[2] + b[1] * b[2]) / n, n]
                        del centres[j]
                        merged = True
                        break
                if merged:
                    break
        detections = [Detection(x + self.rng.gauss(0, 0.05), y + self.rng.gauss(0, 0.05), 1.7, True, n, 30)
                      for x, y, n in centres]
        tracks = self.tracker.update(detections, self.t)
        self.core.set_people([{"id": t.track_id, "x": t.x, "y": t.y, "group": t.group}
                              for t in tracks], self.t)

    def capture(self) -> None:
        persons = []
        for name, box in self.people_boxes():
            cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
            distance = math.hypot(cx - self.pose.x, cy - self.pose.y)
            bearing = math.atan2(cy - self.pose.y, cx - self.pose.x) - self.pose.yaw
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))
            if distance > VISION_RANGE or abs(bearing) > VISION_HALF_FOV or not self.visible(name, box):
                continue
            track, last = self.yolo_ids.get(name, (None, -math.inf))
            if track is None or self.t - last > YOLO_GAP_S:
                track = self.next_yolo_id
                self.next_yolo_id += 1
            self.yolo_ids[name] = (track, self.t)
            persons.append({"bearing_deg": math.degrees(bearing) + self.rng.gauss(0, 1.0),
                            "track_id": track, "conf": 0.8})
        self.vision_queue.append((self.t + VISION_LATENCY, persons, self.pose))

    def deliver_vision(self) -> None:
        while self.vision_queue and self.vision_queue[0][0] <= self.t:
            _due, persons, pose = self.vision_queue.pop(0)
            self.core.set_vision(persons, pose, self.t)

    def sense(self) -> None:
        if self.t >= self.next_scan:
            self.scan()
            self.next_scan += SCAN_PERIOD
        if self.lidar == "mid360" and self.t >= self.next_people:
            self.people()
            self.next_people += PEOPLE_PERIOD
        if self.use_vision and self.t >= self.next_vision:
            self.capture()
            self.next_vision += VISION_PERIOD
        self.deliver_vision()

    def collision(self) -> float:
        boxes = np.vstack([self.all_boxes, *[np.array([b]) for _n, b in self.people_boxes()]])
        return surface_distance(boxes, self.pose.x, self.pose.y)

    def run(self, leader: str, duration: float, verbose: bool) -> dict:
        lead = next(w for w in self.walkers if w.name == leader)
        # Sensors run a while before the command, as on the robot (the camera
        # streams all the time; "follow" never meets an empty vision buffer).
        while self.t < WARMUP_S:
            self.sense()
            self.t += DT
        status = self.core.set_goal("follow")
        history, last_state = [], None
        states: dict[str, int] = {}
        gaps, min_surface = [], math.inf
        result = None
        done_at = None
        while self.t < duration:
            self.sense()
            command = self.core.step(self.pose, self.t)
            if command.status is not None:
                status = command.status
                state = status["state"]
                states[state] = states.get(state, 0) + 1
                if state != last_state:
                    lx, ly = self.pos(lead)
                    history.append(f"  t={self.t:6.1f} 机器人({self.pose.x:6.2f},{self.pose.y:6.2f},"
                                   f"{math.degrees(self.pose.yaw):5.0f}°) 领路人({lx:6.2f},{ly:6.2f}) "
                                   f"{state:16s} id={status.get('target_id')} yolo={status.get('yolo_id')} "
                                   f"src={status.get('source')} d={status.get('distance_m')}")
                    last_state = state
                if state in ("follow_lost", "no_target"):
                    result = state
                    break
            v, w = command.linear_x, command.angular_z
            yaw = self.pose.yaw + w * DT
            nx = self.pose.x + v * DT * math.cos(self.pose.yaw + w * DT / 2)
            ny = self.pose.y + v * DT * math.sin(self.pose.yaw + w * DT / 2)
            self.pose = Pose2D(nx, ny, math.atan2(math.sin(yaw), math.cos(yaw)))
            self.t += DT
            min_surface = min(min_surface, self.collision())
            if min_surface < BODY_RADIUS:
                result = "collision"
                break
            self.avoid_robot()
            if lead.present(self.t):
                lx, ly = self.pos(lead)
                gap = math.hypot(lx - nx, ly - ny)
                if lead.moving(self.t):
                    gaps.append(gap)
                if verbose and round(self.t / DT) % 40 == 0:
                    history.append(f"  t={self.t:6.1f}    间距 {gap:5.2f} m  机器人({nx:6.2f},{ny:6.2f}) "
                                   f"领路人({lx:6.2f},{ly:6.2f}) 速度 {v:.2f}  {status.get('state')}"
                                   f" {status.get('motion', '')}")
                if self.t > lead.end_t and done_at is None:
                    err = math.degrees(abs(math.atan2(math.sin(math.atan2(ly - ny, lx - nx) - self.pose.yaw),
                                                      math.cos(math.atan2(ly - ny, lx - nx) - self.pose.yaw))))
                    if (status.get("state") == "follow_waiting" and gap <= DONE_DISTANCE
                            and err <= DONE_FACE_DEG and command.linear_x == 0 and abs(command.angular_z) < 1e-9):
                        done_at = self.t
                        f = self.core.follower
                        on_target = math.hypot(f.x - lx, f.y - ly) < 0.6
                        result = "ok" if on_target else "wrong_target"
                        history.append(f"  t={self.t:6.1f} 停稳：间距 {gap:.2f} m，朝向误差 {err:.0f}°，"
                                       f"目标估计离领路人 {math.hypot(f.x - lx, f.y - ly):.2f} m")
                        break
                    if self.t - lead.end_t > SETTLE_S:
                        result = "not_settled"
                        break
        summary = self.core.follower.summary()
        return dict(result=result or "timeout", sim_s=round(self.t, 1), states=states,
                    gap_mean=round(float(np.mean(gaps)), 2) if gaps else None,
                    gap_max=round(float(np.max(gaps)), 2) if gaps else None,
                    min_surface=round(min_surface, 2), swaps=summary["id_swaps"],
                    reacquired=summary["reacquired"], history=history)


def plan_route(config: dict, points: list[tuple[float, float]],
               narrow: bool = False) -> list[tuple[float, float]]:
    """A polyline through ``points`` the robot could also take (same clearance
    as its planner); ``narrow`` = a person's route (0.45 m off things), which
    may squeeze through gaps the robot has to go around."""
    grid = (OccupancyGrid({**config, "robot_clearance_radius": 0.45}, extra_margin=0.0) if narrow
            else OccupancyGrid(config, extra_margin=0.30))
    route = [points[0]]
    for a, b in zip(points, points[1:]):
        path = grid.plan(a, b)
        if path is None:
            raise SystemExit(f"领路人路线规划失败：{a} → {b}")
        route += [tuple(p) for p in grid.shortcut(path, [])[1:]]
    return route


def scenarios(scene: str) -> dict:
    config = load_scene(SCENES[scene])
    spawn = config["spawn"]
    sx, sy = float(spawn["x"]), float(spawn["y"])
    start = (sx, sy + 1.8)            # spawn yaw is +y in both scenes
    if scene == "hall":
        # Out of the south corridor into the open middle room, then north past
        # station 5 to station 1 (the robot planner reaches all of these).
        tour = plan_route(config, [start, (1.5, -2.5), (3.4, 5.0), (-3.7, 9.1)])
        detour = plan_route(config, [start, (1.5, -2.5), (3.4, 5.0), (-3.7, 9.1)], narrow=True)
        straight = plan_route(config, [start, (1.5, -2.5), (4.5, -2.5)])
        room = [(1.5, -2.5), (4.5, -2.5), (1.0, -0.8)]
    else:
        # Round the central partition by the side (its middle gap is one
        # planner cell wide: the "detour" case, not a fair "walk").
        tour = plan_route(config, [start, (-6.0, -2.0), (-6.0, 3.0), (3.0, 3.5)])
        detour = plan_route(config, [start, (0.0, 3.0), (-5.0, 3.0)], narrow=True)
        straight = plan_route(config, [start, (-1.0, -2.0), (-6.5, -2.0)])
        room = [(-1.0, -2.0), (-5.0, -2.0), (-2.0, -1.5)]
    walk = lambda **kw: Walker("leader", tour, **kw)  # noqa: E731
    out = {
        "walk": dict(walkers=[walk()], leader="leader"),
        "lidar_only": dict(walkers=[walk()], leader="leader", vision=False),
        "vision_only": dict(walkers=[walk()], leader="leader", lidar="simple"),
        "detour": dict(walkers=[Walker("leader", detour)], leader="leader", max_gap=None),
    }
    lead = Walker("leader", straight)
    # cross: once the leader walks along the last leg, someone crosses 0.6 m
    # behind them (between leader and robot), perpendicular, at 1 m/s.
    t0 = lead.segments[-1][0] + 3.0
    (ax, ay), (bx, by) = straight[-2], straight[-1]
    length = math.hypot(bx - ax, by - ay)
    ux, uy = (bx - ax) / length, (by - ay) / length
    t_cross = t0 + 0.6 / WALK_SPEED     # the leader is 0.6 m past the crossing point by then
    mx, my = lead.at(t0)
    out["cross"] = dict(walkers=[lead, Walker("other", [(mx - uy * 3.0, my + ux * 3.0),
                                                        (mx + uy * 3.0, my - ux * 3.0)],
                                              speed=1.0, start_t=t_cross - 3.0)], leader="leader")
    # swap: on the last leg a second person walks beside the leader, 0.6 m to
    # the left, for 2 m, then turns off to the left.
    sx0, sy0 = ax - uy * 0.6, ay + ux * 0.6
    sx1, sy1 = sx0 + ux * 2.0, sy0 + uy * 2.0
    out["swap"] = dict(walkers=[Walker("leader", straight),
                                Walker("other", [(sx0, sy0), (sx1, sy1), (sx1 - uy * 1.5, sy1 + ux * 1.5)],
                                       start_t=lead.segments[-1][0])], leader="leader")
    # stop_go: stand 10 s, then walk on and come back past the robot.
    out["stop_go"] = dict(walkers=[Walker("leader", plan_route(config, [start, *room]),
                                          pauses={len(plan_route(config, [start, room[0]])) - 1: 10.0})],
                          leader="leader")
    out["vanish"] = dict(walkers=[Walker("leader", straight, vanish_t=lead.segments[-1][0] + 2.0)],
                         leader="leader", expect="follow_lost")
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("names", nargs="*")
    parser.add_argument("--scene", choices=SCENES, default="hall")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()
    table = scenarios(args.scene)
    names = args.names or list(table)
    failures, wall = 0, time.monotonic()
    for name in names:
        spec = dict(table[name])
        expect = spec.pop("expect", "ok")
        spec.pop("max_gap", None)
        leader = spec.pop("leader")
        walkers = spec.pop("walkers")
        sim = FollowSim(args.scene, walkers, **spec)
        duration = max(w.end_t for w in walkers) + SETTLE_S + 15.0
        out = sim.run(leader, duration, args.verbose)
        max_gap = spec_gap if (spec_gap := table[name].get("max_gap", MAX_GAP)) else math.inf
        ok = out["result"] == expect and (out["gap_max"] is None or out["gap_max"] <= max_gap
                                          or expect != "ok")
        failures += not ok
        print(f"{'OK ' if ok else '!! '}{name:11s} {out['result']:12s} {out['sim_s']:6.1f}s  "
              f"间距 均值 {out['gap_mean']} 最大 {out['gap_max']} m  最近障碍 {out['min_surface']} m  "
              f"ID 纠正 {out['swaps']} 重捕 {out['reacquired']}  {out['states']}")
        if args.verbose or not ok:
            for line in out["history"][-30:]:
                print(line)
    print(f"\n{'全部通过' if not failures else f'{failures} 个场景失败'}，墙钟 {time.monotonic() - wall:.1f} s")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
