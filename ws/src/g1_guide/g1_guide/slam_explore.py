"""Tour the exhibit stations while slam_toolbox builds the map, then save and check it.

The navigator already knows how to reach every station safely, so exploration
is just a sequence of ``/nav/goal_key`` requests.  The 360° navigation lidar
sees 8 m in every direction, so passing through the six stations covers the
halls without extra spinning in place.
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Mapping, Sequence


def tour_order(stations: Mapping[str, Mapping], start: tuple[float, float],
               requested: str = "") -> list[str]:
    """Station keys to visit: the requested order, else greedy nearest-first."""

    if requested.strip():
        keys = requested.replace(",", " ").split()
        unknown = [key for key in keys if key not in stations]
        if unknown:
            raise ValueError(f"unknown station keys: {unknown}")
        return keys
    remaining = dict(stations)
    here = start
    order = []
    while remaining:
        key = min(remaining, key=lambda k: (math.hypot(float(remaining[k]["x"]) - here[0],
                                                       float(remaining[k]["y"]) - here[1]), k))
        here = (float(remaining[key]["x"]), float(remaining[key]["y"]))
        order.append(key)
        del remaining[key]
    return order


def default_output_dir() -> Path:
    """``<project>/maps`` when running from the source tree (symlink install)."""

    here = Path(__file__).resolve()
    if len(here.parents) > 4 and here.parents[3].name == "ws":
        return here.parents[4] / "maps"
    return Path.cwd() / "maps"


def main(args: Sequence[str] | None = None) -> None:
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from std_msgs.msg import String

    from .scene_config import load_scene
    from .slam_map import check_map, from_occupancy_grid, save_map

    class SlamExplorer(Node):
        def __init__(self):
            super().__init__("slam_explore")
            share = Path(get_package_share_directory("g1_guide"))
            self.declare_parameter("scene_config", str(share / "config/exhibition.json"))
            self.declare_parameter("order", "")
            self.declare_parameter("output_dir", "")
            self.declare_parameter("map_name", "exhibition_map")
            self.declare_parameter("goal_timeout_s", 400.0)
            self.declare_parameter("settle_s", 6.0)
            self.scene_path = self.get_parameter("scene_config").value
            config = load_scene(self.scene_path)
            self.stations = {item["station"]["key"]: item["station"] for item in config["exhibits"]}
            output = self.get_parameter("output_dir").value
            self.output_dir = Path(output) if output else default_output_dir()
            self.goal_timeout = float(self.get_parameter("goal_timeout_s").value)
            self.settle = float(self.get_parameter("settle_s").value)

            self.goal_pub = self.create_publisher(String, "/nav/goal_key", 10)
            self.create_subscription(String, "/nav/status", self.on_status, 10)
            self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
            map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
            self.create_timer(0.5, self.tick)

            self.pose = None
            self.map = None
            self.order: list[str] | None = None
            self.current = None
            self.sent_at = 0.0
            self.arrived: list[str] = []
            self.skipped: list[str] = []
            self.finished_at = None
            self.done = False

        def on_odom(self, message):
            p = message.pose.pose.position
            self.pose = (float(p.x), float(p.y))

        def on_map(self, message):
            self.map = message

        def on_status(self, message):
            try:
                status = json.loads(message.data)
            except ValueError:
                return
            if self.current is None or status.get("goal_key") != self.current:
                return
            if status.get("state") == "arrived":
                self.get_logger().info(f"到达展位 {self.current}")
                self.arrived.append(self.current)
                self.current = None

        def send(self, key: str):
            self.goal_pub.publish(String(data=key))
            self.current = key
            self.sent_at = time.monotonic()
            self.get_logger().info(f"前往展位 {key}（{len(self.arrived) + len(self.skipped) + 1}/{len(self.order)}）")

        def tick(self):
            if self.done:
                return
            if self.order is None:
                # Wait for the navigator, odometry and the first SLAM map.
                if self.pose is None or self.map is None or self.goal_pub.get_subscription_count() == 0:
                    return
                self.order = tour_order(self.stations, self.pose, self.get_parameter("order").value)
                self.get_logger().info(f"开始建图巡航，顺序：{' '.join(self.order)}")
            if self.current is not None:
                if time.monotonic() - self.sent_at > self.goal_timeout:
                    self.get_logger().warning(f"展位 {self.current} 超时，跳过")
                    self.goal_pub.publish(String(data="0"))
                    self.skipped.append(self.current)
                    self.current = None
                return
            visited = len(self.arrived) + len(self.skipped)
            if visited < len(self.order):
                self.send(self.order[visited])
                return
            # Let slam_toolbox publish a map that includes the last scans.
            if self.finished_at is None:
                self.finished_at = time.monotonic()
                self.get_logger().info("巡航结束，等待地图更新后保存")
                return
            if time.monotonic() - self.finished_at < self.settle:
                return
            self.save()

        def save(self):
            self.done = True
            stem = self.output_dir / self.get_parameter("map_name").value
            pgm, yaml = save_map(from_occupancy_grid(self.map), stem)
            report = check_map(yaml, self.scene_path, overlay=stem.with_name(stem.name + "_overlay.png"))
            report.update(arrived=self.arrived, skipped=self.skipped)
            stem.with_name(stem.name + "_report.json").write_text(
                json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
            verdict = "合格" if report["pass"] else "不合格"
            self.get_logger().info(
                f"地图已保存：{pgm}\n墙体偏差 p95={report['p95_m']} m，平均 {report['mean_m']} m，"
                f"{(report['within_tolerance_ratio'] or 0.0):.1%} 的墙格在 0.2 m 内 → {verdict}\n"
                f"叠加图：{report['overlay']}")

    rclpy.init(args=args)
    node = SlamExplorer()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
