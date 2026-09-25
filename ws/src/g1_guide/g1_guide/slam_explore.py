"""Tour the exhibit stations while slam_toolbox builds the map, then save and check it.

With ``save_map`` false the same tour is the navigation acceptance test for
slam_nav.launch.py: it records, against the Gazebo truth pose, how far the robot
really stopped from each station and how far its localized (map-frame) pose
was from the truth along the way.

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


def summarize(values: Sequence[float]) -> dict:
    if not values:
        return {"mean_m": None, "max_m": None, "samples": 0}
    return {"mean_m": round(sum(values) / len(values), 3), "max_m": round(max(values), 3),
            "samples": len(values)}


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
    from rclpy.time import Time
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener

    from .scene_config import load_scene
    from .slam_map import check_map, from_occupancy_grid, save_map, write_map_meta

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
            self.declare_parameter("save_map", True)
            self.declare_parameter("lidar", "")
            self.declare_parameter("global_frame", "odom")
            self.declare_parameter("report_name", "nav_report")
            self.scene_path = self.get_parameter("scene_config").value
            config = load_scene(self.scene_path)
            self.stations = {item["station"]["key"]: item["station"] for item in config["exhibits"]}
            output = self.get_parameter("output_dir").value
            self.output_dir = Path(output) if output else default_output_dir()
            self.goal_timeout = float(self.get_parameter("goal_timeout_s").value)
            self.settle = float(self.get_parameter("settle_s").value)
            self.save_map = bool(self.get_parameter("save_map").value)
            self.global_frame = self.get_parameter("global_frame").value
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            # Gazebo truth: /odom_truth in the 真机雷达版, /odom is exact otherwise.
            self.truth = None
            self.truth_topic_seen = False
            self.create_subscription(Odometry, "/odom_truth", self.on_truth, qos_profile_sensor_data)
            self.localization_errors: list[float] = []
            self.odom_errors: list[float] = []
            self.station_results: dict[str, dict] = {}
            self.create_timer(1.0, self.sample_errors)

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
            if not self.truth_topic_seen:
                self.truth = self.pose

        def on_truth(self, message):
            p = message.pose.pose.position
            self.truth_topic_seen = True
            self.truth = (float(p.x), float(p.y))

        def localized(self):
            """Robot position as the navigator sees it (global frame)."""
            if self.global_frame == "odom":
                return self.pose
            try:
                t = self.tf_buffer.lookup_transform(self.global_frame, "base_link", Time()).transform
            except TransformException:
                return None
            return (float(t.translation.x), float(t.translation.y))

        def sample_errors(self):
            if self.order is None or self.done or self.truth is None:
                return
            estimate = self.localized()
            if estimate is not None:
                self.localization_errors.append(math.dist(estimate, self.truth))
            if self.pose is not None:
                self.odom_errors.append(math.dist(self.pose, self.truth))

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
                station = self.stations[self.current]
                goal = (float(station["x"]), float(station["y"]))
                estimate = self.localized()
                result = {
                    "true_error_m": round(math.dist(self.truth, goal), 3) if self.truth else None,
                    "localization_error_m": round(math.dist(estimate, self.truth), 3)
                    if estimate and self.truth else None,
                    "odom_error_m": round(math.dist(self.pose, self.truth), 3)
                    if self.pose and self.truth else None,
                }
                self.station_results[self.current] = result
                self.get_logger().info(
                    f"到达展位 {self.current}：真实到站误差 {result['true_error_m']} m，"
                    f"定位误差 {result['localization_error_m']} m，里程计误差 {result['odom_error_m']} m")
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
                ready_map = self.map is not None or not self.save_map
                if (self.pose is None or not ready_map or self.localized() is None
                        or self.goal_pub.get_subscription_count() == 0):
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
            if self.save_map:
                self.save()
            else:
                self.report()

        def report(self):
            self.done = True
            errors = [r["true_error_m"] for r in self.station_results.values()
                      if r["true_error_m"] is not None]
            report = {
                "global_frame": self.global_frame,
                "arrived": self.arrived, "skipped": self.skipped,
                "stations": self.station_results,
                "true_arrival_error": summarize(errors),
                "localization_error": summarize(self.localization_errors),
                "odom_error": summarize(self.odom_errors),
                "pass": not self.skipped and bool(errors) and max(errors) <= 0.25,
            }
            self.output_dir.mkdir(parents=True, exist_ok=True)
            path = self.output_dir / (self.get_parameter("report_name").value + ".json")
            path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
            verdict = "合格" if report["pass"] else "不合格"
            self.get_logger().info(
                f"导航验收：{len(self.arrived)}/{len(self.order)} 站到达，"
                f"真实到站误差最大 {report['true_arrival_error']['max_m']} m，"
                f"定位误差平均 {report['localization_error']['mean_m']} m，"
                f"里程计误差最大 {report['odom_error']['max_m']} m → {verdict}\n报告：{path}")

        def save(self):
            self.done = True
            stem = self.output_dir / self.get_parameter("map_name").value
            pgm, yaml = save_map(from_occupancy_grid(self.map), stem)
            write_map_meta(yaml, self.scene_path, self.get_parameter("lidar").value)
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
