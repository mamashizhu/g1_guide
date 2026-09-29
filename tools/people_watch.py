#!/usr/bin/env python3
"""打印 /guide/people（PLAN M5.2 雷达人群感知），可选存成 JSONL。

用法（WSL，已 source ROS）：
  python3 tools/people_watch.py              # 一直打印，Ctrl-C 退出
  python3 tools/people_watch.py 5 out.jsonl  # 打印 5 秒并存盘

每行：3 m 内人数 | 每个人的编号、距离、方位（左正右负）、身高（≥ 表示超出雷达视野，只是下限）
"""

import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def line(payload: dict) -> str:
    near = sum(p.get("group", 1) for p in payload["people"] if p["distance"] <= 3.0)
    people = "  ".join(
        f"#{p['id']} {p['distance']:.1f}m {p['bearing_deg']:+.0f}° "
        f"{'≥' if p['height_clipped'] else ''}{p['height']:.2f}m"
        + (" 儿童" if p["child"] else "") + (f" ×{p['group']}" if p["group"] > 1 else "")
        for p in payload["people"])
    return f"3m内 {near} 人 | {people or '（无）'}"


def main() -> None:
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else float("inf")
    out = open(sys.argv[2], "a", encoding="utf-8") if len(sys.argv) > 2 else None
    rclpy.init()
    node = Node("people_watch")

    def on_people(message):
        payload = json.loads(message.data)
        print(f"{time.strftime('%H:%M:%S')} {line(payload)}", flush=True)
        if out:
            out.write(json.dumps({"wall": time.time(), **payload}, ensure_ascii=False) + "\n")
            out.flush()

    node.create_subscription(String, "/guide/people", on_people, 10)
    end = time.monotonic() + duration
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
