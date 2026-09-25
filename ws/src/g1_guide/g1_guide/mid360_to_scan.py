"""Flatten the MID-360 point cloud into the 2D /nav_scan used by navigation and SLAM.

This is the "真机雷达版" data path.  The real G1 has no 2D lidar: its MID-360
sits upside down in the head and looks mostly downward (360° x -7°..+52°).
Every return between ``min_height`` and ``max_height`` above the floor is
projected onto the level ``nav_lidar_link`` plane and the nearest one per
bearing becomes that beam's range.  Low furniture that the old single-plane
scan flew over therefore shows up in the map and in obstacle avoidance, while
floor hits and the robot's own body are dropped.

The output keeps the simple version's contract (topic, frame, 360 beams), so
the navigator and slam_toolbox run unchanged on either lidar model.
"""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def flatten_points(points: np.ndarray, cloud_to_odom: tuple[np.ndarray, np.ndarray],
                   nav_in_odom: tuple[np.ndarray, np.ndarray], floor_top: float,
                   min_height: float = 0.08, max_height: float = 2.0,
                   beams: int = 360, range_max: float = 40.0,
                   self_radius: float = 0.5) -> list[float]:
    """Nearest horizontal range per bearing in the nav frame; ``inf`` if empty.

    ``cloud_to_odom`` and ``nav_in_odom`` are (rotation, translation) pairs.
    Heights are measured in odom, whose z is the world height in this sim.
    """

    ranges = np.full(beams, np.inf)
    if points.size == 0:
        return ranges.tolist()
    rotation, translation = cloud_to_odom
    world = points @ rotation.T + translation
    height = world[:, 2] - floor_top
    keep = np.isfinite(world).all(axis=1) & (height >= min_height) & (height <= max_height)
    nav_rotation, nav_translation = nav_in_odom
    local = (world[keep] - nav_translation) @ nav_rotation
    distance = np.hypot(local[:, 0], local[:, 1])
    keep = (distance > self_radius) & (distance <= range_max)
    distance, local = distance[keep], local[keep]
    if distance.size == 0:
        return ranges.tolist()
    step = 2 * math.pi / beams
    bins = np.rint((np.arctan2(local[:, 1], local[:, 0]) + math.pi) / step).astype(int) % beams
    np.minimum.at(ranges, bins, distance)
    return ranges.tolist()


def main(args: Sequence[str] | None = None) -> None:
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from pathlib import Path
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan, PointCloud2
    from sensor_msgs_py.point_cloud2 import read_points_numpy
    from tf2_ros import Buffer, TransformException, TransformListener

    from .scene_config import load_scene

    def as_pair(transform):
        r, t = transform.rotation, transform.translation
        return (quaternion_matrix(r.x, r.y, r.z, r.w),
                np.array([t.x, t.y, t.z], dtype=float))

    class Mid360ToScan(Node):
        def __init__(self):
            super().__init__("mid360_to_scan")
            share = Path(get_package_share_directory("g1_guide"))
            self.declare_parameter("scene_config", str(share / "config/exhibition.json"))
            self.declare_parameter("min_height", 0.08)
            self.declare_parameter("max_height", 2.0)
            self.declare_parameter("range_max", 40.0)
            self.declare_parameter("self_radius", 0.5)
            self.floor_top = float(load_scene(self.get_parameter("scene_config").value)
                                   ["room"].get("floor_top", 0.0))
            self.params = {name: float(self.get_parameter(name).value)
                           for name in ("min_height", "max_height", "range_max", "self_radius")}
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.pub = self.create_publisher(LaserScan, "/nav_scan", qos_profile_sensor_data)
            self.create_subscription(PointCloud2, "/mid360/points", self.on_cloud,
                                     qos_profile_sensor_data)
            self.pending = None
            self.create_timer(0.02, self.flush)

        def on_cloud(self, message):
            self.pending = message

        def flush(self):
            # A cloud can arrive before TF for its stamp; keep retrying it
            # until a newer cloud replaces it.
            message = self.pending
            if message is None:
                return
            stamp = Time.from_msg(message.header.stamp)
            try:
                cloud = self.tf_buffer.lookup_transform("odom", message.header.frame_id, stamp)
                nav = self.tf_buffer.lookup_transform("odom", "nav_lidar_link", stamp)
            except TransformException:
                return
            self.pending = None
            points = read_points_numpy(message, field_names=("x", "y", "z"), skip_nans=True)
            ranges = flatten_points(np.asarray(points, dtype=float).reshape(-1, 3),
                                    as_pair(cloud.transform), as_pair(nav.transform),
                                    self.floor_top, self.params["min_height"],
                                    self.params["max_height"], 360,
                                    self.params["range_max"], self.params["self_radius"])
            scan = LaserScan()
            scan.header.stamp = message.header.stamp
            scan.header.frame_id = "nav_lidar_link"
            scan.angle_min = -math.pi
            scan.angle_increment = 2 * math.pi / len(ranges)
            scan.angle_max = scan.angle_min + scan.angle_increment * (len(ranges) - 1)
            scan.scan_time = 0.1
            scan.range_min = self.params["self_radius"]
            scan.range_max = self.params["range_max"]
            scan.ranges = [float(r) for r in ranges]
            self.pub.publish(scan)

    rclpy.init(args=args)
    node = Mid360ToScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
