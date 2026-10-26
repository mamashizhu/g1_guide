#!/usr/bin/env python3
"""Gazebo 跟随测试的「领路人」和判定（WSL，配合 tools/follow_gz_sim.sh 和 Windows 端 vision\\watch.py）。

在机器人正前方 1.8 m 放一个 Fuel「Standing person」参观者（雷达和相机都看得见），等雷达人群和
YOLO 数据都到了，发 /nav/goal_key follow；然后按仿真时钟以 0.5 m/s 沿路线移动它（set_pose，
每 0.1 s 一次，朝向始终是行走方向），走完停下，再等机器人停稳。只有这一个人，没有别的移动物体。

  python3 tools/follow_gz_leader.py <输出名> [--route short|walk] [--speed 0.5]
路线（简单展馆，机器人出生点 (0,-4) 朝 +y）：
  short  向前 0.2 m 后左转，沿 y=-2 走到 (-5,-2)，约 6 m，一个转弯
  walk   绕左侧走到展馆北面 (3,3.5)，约 17 m，三个转弯（和 tools/follow_sim.py 的 walk 相同）

判定（和离线 follow_sim 一致）：领路人走动期间机器人与他的间距 ≤ 5 m；领路人停下后 25 s 内
机器人状态 follow_waiting、间距 ≤ 1.9 m、朝向误差 ≤ 20°；全程没有 follow_lost / no_target；
机器人与领路人最近距离 ≥ 0.6 m。记录写到 .navrun/<输出名>/follow_log.csv 和 follow_result.json。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "ws/src/g1_guide"))

from follow_sim import Walker, plan_route  # noqa: E402
from g1_guide.scene_config import load_scene  # noqa: E402

SCENE = ROOT / "ws/src/g1_guide/config/exhibition_simple.json"
NAME = "follow_leader"
START_AHEAD = 1.8
SETTLE_S = 25.0
DONE_DISTANCE = 1.9
DONE_FACE_DEG = 20.0
MAX_GAP = 5.0
MIN_GAP = 0.6
PERSON_SDF = ("<?xml version='1.0'?><sdf version='1.9'><include><name>{name}</name><static>true</static>"
              "<uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing person</uri></include></sdf>")


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def route(kind: str, config: dict) -> list[tuple[float, float]]:
    spawn = config["spawn"]
    start = (float(spawn["x"]), float(spawn["y"]) + START_AHEAD)
    if kind == "short":
        return plan_route(config, [start, (-1.0, -2.0), (-5.0, -2.0)])
    return plan_route(config, [start, (-6.0, -2.0), (-6.0, 3.0), (3.0, 3.5)])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("out")
    parser.add_argument("--route", choices=("short", "walk"), default="short")
    parser.add_argument("--speed", type=float, default=0.5)
    args = parser.parse_args()
    out = ROOT / ".navrun" / args.out
    out.mkdir(parents=True, exist_ok=True)

    import rclpy
    from geometry_msgs.msg import Pose
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from ros_gz_interfaces.msg import Entity
    from ros_gz_interfaces.srv import SetEntityPose
    from std_msgs.msg import String

    config = load_scene(SCENE)
    points = route(args.route, config)
    walker = Walker("leader", points, speed=args.speed)
    print(f"[leader] 路线 {len(points)} 点，{walker.end_t:.0f} s：{[tuple(round(c, 2) for c in p) for p in points]}",
          flush=True)

    def heading(t: float) -> float:
        a, b = walker.at(t), walker.at(t + 0.5)
        if math.hypot(b[0] - a[0], b[1] - a[1]) < 1e-3:
            a, b = walker.at(t - 0.5), walker.at(t)
        return math.atan2(b[1] - a[1], b[0] - a[0]) if math.hypot(b[0] - a[0], b[1] - a[1]) > 1e-3 else math.pi / 2

    x0, y0 = points[0]
    # Model front = yaw - 90 deg (tools/crowd_e2e_sim.sh): walking away from the robot.
    yaw0 = heading(0.0) + math.pi / 2
    subprocess.run(["ros2", "run", "ros_gz_sim", "remove", "-world", "exhibition", "-name", NAME],
                   capture_output=True, timeout=30)
    created = subprocess.run(["ros2", "run", "ros_gz_sim", "create", "-world", "exhibition", "-name", NAME,
                              "-x", f"{x0}", "-y", f"{y0}", "-z", "0", "-Y", f"{yaw0}",
                              "-string", PERSON_SDF.format(name=NAME)], capture_output=True, text=True, timeout=120)
    print(f"[leader] 放置参观者：{'成功' if created.returncode == 0 else created.stderr[-300:]}", flush=True)

    rclpy.init()
    node = Node("follow_leader", parameter_overrides=[Parameter("use_sim_time", value=True)])
    state = {"robot": None, "status": {}, "people": [], "people_at": 0.0, "vision_at": 0.0,
             "persons": 0, "states": {}}

    def on_odom(message):
        p = message.pose.pose
        state["robot"] = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_status(message):
        try:
            status = json.loads(message.data)
        except ValueError:
            return
        state["status"] = status
        name = status.get("state")
        state["states"][name] = state["states"].get(name, 0) + 1

    def on_people(message):
        state["people"] = json.loads(message.data).get("people", [])
        state["people_at"] = time.monotonic()

    def on_vision(message):
        state["vision_at"] = time.monotonic()
        state["persons"] = len(json.loads(message.data).get("persons", []))

    node.create_subscription(Odometry, "/odom_truth", on_odom, qos_profile_sensor_data)
    node.create_subscription(String, "/nav/status", on_status, 10)
    node.create_subscription(String, "/guide/people", on_people, 10)
    node.create_subscription(String, "/guide/vision", on_vision, 10)
    goal = node.create_publisher(String, "/nav/goal_key", 10)
    client = node.create_client(SetEntityPose, "/world/exhibition/set_pose")

    def sim_now() -> float:
        return node.get_clock().now().nanoseconds * 1e-9

    def spin(seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    def place(x: float, y: float, yaw: float):
        request = SetEntityPose.Request()
        request.entity = Entity(name=NAME, type=Entity.MODEL)
        request.pose = Pose()
        request.pose.position.x, request.pose.position.y = x, y
        request.pose.orientation.z, request.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return client.call_async(request)

    result = {"route": args.route, "speed": args.speed, "route_s": round(walker.end_t, 1)}
    try:
        if not client.wait_for_service(timeout_sec=60):
            result["result"] = "no_set_pose_service"
            return 1
        # Wait for both people sources: lidar sees the visitor, YOLO sees a person.
        t0 = time.monotonic()
        while time.monotonic() - t0 < 240:
            spin(0.5)
            near = [p for p in state["people"] if math.hypot(p["x"] - x0, p["y"] - y0) < 0.6]
            if (state["robot"] and near and time.monotonic() - state["people_at"] < 2
                    and time.monotonic() - state["vision_at"] < 2 and state["persons"]):
                break
        else:
            result["result"] = "no_sensor_data"
            result["detail"] = {"robot": state["robot"], "people": state["people"],
                                "vision_age": time.monotonic() - state["vision_at"]}
            return 1
        print(f"[leader] 雷达和 YOLO 都看到参观者（等了 {time.monotonic() - t0:.0f} s），发 follow", flush=True)
        while goal.get_subscription_count() == 0:
            spin(0.2)
        goal.publish(String(data="follow"))
        spin(3.0)
        print(f"[leader] 导航状态：{state['status'].get('state')}，开始走", flush=True)

        log = open(out / "follow_log.csv", "w", newline="")
        writer = csv.writer(log)
        writer.writerow(["t", "rx", "ry", "ryaw", "lx", "ly", "gap", "state", "target_id", "yolo_id", "source"])
        start = sim_now()
        last_place = last_log = -1.0
        gaps, min_gap, done = [], math.inf, None
        pending = None
        while True:
            spin(0.01)
            t = sim_now() - start
            if t - last_place >= 0.1 and (pending is None or pending.done()):
                lx, ly = walker.at(t)
                pending = place(lx, ly, heading(t) + math.pi / 2)
                last_place = t
            if state["robot"] is None:
                continue
            lx, ly = walker.at(t)
            rx, ry, ryaw = state["robot"]
            gap = math.hypot(lx - rx, ly - ry)
            min_gap = min(min_gap, gap)
            if walker.moving(t):
                gaps.append(gap)
            status = state["status"]
            if t - last_log >= 0.5:
                writer.writerow([round(t, 2), round(rx, 3), round(ry, 3), round(ryaw, 3), round(lx, 3), round(ly, 3),
                                 round(gap, 3), status.get("state"), status.get("target_id"),
                                 status.get("yolo_id"), status.get("source")])
                last_log = t
                if round(t) % 5 == 0 and t - round(t) < 0.5:
                    print(f"[leader] t={t:5.1f}s 间距 {gap:4.2f} m  {status.get('state')} "
                          f"src={status.get('source')} yolo={status.get('yolo_id')}", flush=True)
            if status.get("state") in ("follow_lost", "no_target", "cancelled", "invalid_goal"):
                done = status.get("state")
                break
            if t > walker.end_t:
                error = abs(math.degrees(math.atan2(math.sin(math.atan2(ly - ry, lx - rx) - ryaw),
                                                    math.cos(math.atan2(ly - ry, lx - rx) - ryaw))))
                if status.get("state") == "follow_waiting" and gap <= DONE_DISTANCE and error <= DONE_FACE_DEG:
                    done = "ok"
                    result.update(final_gap=round(gap, 2), final_face_deg=round(error, 1),
                                  settle_s=round(t - walker.end_t, 1))
                    break
                if t - walker.end_t > SETTLE_S:
                    done = "not_settled"
                    result.update(final_gap=round(gap, 2), final_face_deg=round(error, 1))
                    break
        log.close()
        ok = (done == "ok" and (max(gaps, default=0) <= MAX_GAP) and min_gap >= MIN_GAP)
        result.update(result="ok" if ok else (done if done != "ok" else "gap_out_of_range"),
                      gap_mean=round(sum(gaps) / len(gaps), 2) if gaps else None,
                      gap_max=round(max(gaps), 2) if gaps else None, gap_min=round(min_gap, 2),
                      states=state["states"], last_status=state["status"])
        return 0 if ok else 1
    finally:
        goal.publish(String(data="cancel"))
        spin(0.5)
        (out / "follow_result.json").write_text(json.dumps(result, ensure_ascii=False, indent=1))
        print(f"[leader] 结果：{json.dumps(result, ensure_ascii=False)}", flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
