"""ROS adapter for safe, local-only navigation of the Gazebo glide proxy.

Two modes, chosen by parameters (defaults = the original, user-verified mode):

* 布局模式 (``map_yaml`` empty, ``global_frame`` odom): static collision
  geometry is read from the exhibition configuration that generates Gazebo,
  and the robot pose is the /odom pose.
* SLAM 地图模式 (``map_yaml`` set, ``global_frame`` map): static geometry is
  the occupied cells of a saved SLAM map, and the pose is the map -> base_link
  TF that AMCL maintains on top of (drifting) odometry.

In both, laser endpoints are transformed through live TF and added to the grid
before planning.  No biped gait claim is made here.

讲解模式 (PLAN M5.5): /guide/present_mode ``point`` (default) or ``face``.  In
``face`` mode the robot turns to the people on /guide/people after reaching a
station, then reports ``arrived``; goal key ``face`` does only that turn and
ends in ``faced``.  ``arrived`` carries ``present_mode`` and ``people_near``.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import time

# Re-exported: the geometry helpers moved to nav_core with the decision logic.
from .nav_core import (NavigatorCore, StaticGeometry, forward_clearance,  # noqa: F401
                       known_static_hit, nearest_distance, plan_with_escape, wrap_angle)


def yaw_of(quaternion) -> float:
    x, y, z, w = normalized_quaternion(quaternion)
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


def normalized_quaternion(quaternion) -> tuple[float, float, float, float]:
    x, y, z, w = (float(quaternion.x), float(quaternion.y),
                  float(quaternion.z), float(quaternion.w))
    if not all(math.isfinite(v) for v in (x, y, z, w)):
        raise ValueError("non-finite orientation")
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm < 1e-9:
        raise ValueError("zero quaternion")
    return x/norm, y/norm, z/norm, w/norm


def scan_returns_3d(scan, transform) -> list[tuple[float, float, float]]:
    """Project finite nav LiDAR returns through the full 3-D sensor TF.

    The glide proxy tips while moving. A pitched laser sees the floor; losing
    height would hallucinate a wall across the exhibit goal (F04-i evidence).
    """
    tx = float(transform.translation.x)
    ty = float(transform.translation.y)
    tz = float(transform.translation.z)
    qx, qy, qz, qw = normalized_quaternion(transform.rotation)
    if not all(math.isfinite(v) for v in (tx, ty, tz, scan.angle_min, scan.angle_increment)):
        raise ValueError("invalid scan geometry")
    rxx = 1 - 2*(qy*qy + qz*qz)
    rxy = 2*(qx*qy - qz*qw)
    ryx = 2*(qx*qy + qz*qw)
    ryy = 1 - 2*(qx*qx + qz*qz)
    rzx = 2*(qx*qz - qy*qw)
    rzy = 2*(qy*qz + qx*qw)
    points = []
    for index, raw in enumerate(scan.ranges):
        distance = float(raw)
        if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
            continue
        angle = scan.angle_min + index * scan.angle_increment
        cx, sy = math.cos(angle), math.sin(angle)
        points.append((tx + distance*(rxx*cx + rxy*sy),
                       ty + distance*(ryx*cx + ryy*sy),
                       tz + distance*(rzx*cx + rzy*sy)))
    return points


def scan_endpoints(scan, transform) -> list[tuple[float, float]]:
    """Project all finite returns to XY, for diagnostics only."""
    return [(x, y) for x, y, _z in scan_returns_3d(scan, transform)]


def obstacle_endpoints(scan, transform, floor_top: float,
                       floor_margin: float = 0.15) -> list[tuple[float, float]]:
    """Keep returns above the floor surface plus a noise tolerance."""
    if not all(math.isfinite(v) for v in (floor_top, floor_margin)) or floor_margin < 0:
        raise ValueError("invalid floor threshold")
    return [(x, y) for x, y, z in scan_returns_3d(scan, transform)
            if z > floor_top + floor_margin]


def main(args=None) -> None:
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener

    from .nav_control import Pose2D
    from .scene_config import load_scene

    class ExhibitionNavigator(Node):
        def __init__(self):
            super().__init__("exhibition_navigator")
            default_config = str(Path(get_package_share_directory("g1_guide")) / "config/exhibition.json")
            self.declare_parameter("scene_config", default_config)
            self.declare_parameter("map_yaml", "")
            self.declare_parameter("global_frame", "odom")
            # Goal tolerance; the SLAM map mode tightens it to leave room for
            # the localization error (true stop error = tolerance + that).
            self.declare_parameter("arrival_distance", 0.25)
            self.config = load_scene(self.get_parameter("scene_config").value)
            self.global_frame = self.get_parameter("global_frame").value
            map_yaml = self.get_parameter("map_yaml").value
            if map_yaml:
                from .slam_map import check_map_scene, load_map, navigation_scene
                check_map_scene(map_yaml, self.get_parameter("scene_config").value)
                # Stations still come from the scene: they are the task, not the map.
                static = navigation_scene(load_map(map_yaml), self.config)
                self.get_logger().info(
                    f"SLAM 地图模式：{map_yaml}，{len(static['obstacles'])} 个障碍矩形，"
                    f"定位坐标系 {self.global_frame}")
            else:
                static = self.config
            stations = {item["station"]["key"]: item["station"] for item in self.config["exhibits"]}
            self.core = NavigatorCore(static, stations,
                                      float(self.get_parameter("arrival_distance").value))
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
            self.status_pub = self.create_publisher(String, "/nav/status", 10)
            self.create_subscription(String, "/nav/goal_key", self.on_goal, 10)
            self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
            self.create_subscription(LaserScan, "/nav_scan", self.on_scan, qos_profile_sensor_data)
            self.create_subscription(String, "/guide/people", self.on_people, 10)
            # Volatile on purpose: it matches both plain and latched publishers.
            # The voice side republishes the mode before every goal.
            self.create_subscription(String, "/guide/present_mode", self.on_present_mode, 10)
            self.create_timer(0.1, self.tick)
            self.pose = None
            self.odom_at = 0.0
            self.scan_at = 0.0
            self.scan_valid = False
            self.pending_scan = None
            self.last_state = None
            self.last_status_name = None
            self.last_status_at = 0.0
            self.publish_stop()

        def publish_stop(self):
            self.cmd_pub.publish(Twist())

        def publish_status(self, payload: dict):
            state = payload["state"]
            encoded = json.dumps(payload, allow_nan=False)
            now = time.monotonic()
            if encoded != self.last_state and (state != self.last_status_name or now-self.last_status_at >= 0.5):
                self.status_pub.publish(String(data=encoded))
                if state != self.last_status_name:
                    self.get_logger().info(encoded)
                self.last_state = encoded
                self.last_status_name = state
                self.last_status_at = now

        def on_goal(self, message):
            self.publish_stop()
            self.publish_status(self.core.set_goal(message.data))

        def on_people(self, message):
            try:
                people = json.loads(message.data)["people"]
                self.core.set_people([p for p in people if {"x", "y"} <= p.keys()], time.monotonic())
            except (ValueError, KeyError, TypeError, AttributeError):
                return

        def on_present_mode(self, message):
            mode = message.data.strip()
            if self.core.set_present_mode(mode):
                self.get_logger().info(f"讲解模式：{mode}")
            else:
                self.get_logger().warning(f"未知讲解模式 {mode!r}，可选 point / face")

        def on_odom(self, message):
            if message.header.frame_id != "odom":
                self.pose = None
                return
            try:
                p = message.pose.pose.position
                values = (float(p.x), float(p.y), yaw_of(message.pose.pose.orientation))
                if not all(math.isfinite(v) for v in values):
                    raise ValueError("non-finite odometry")
                self.odom_at = time.monotonic()
                if self.global_frame == "odom":
                    self.pose = Pose2D(*values)
            except ValueError:
                self.pose = None

        def on_scan(self, message):
            if message.header.frame_id != "nav_lidar_link" or len(message.ranges) < 90:
                self.scan_valid = False
                return
            self.pending_scan = message

        def project_pending_scan(self):
            message = self.pending_scan
            if message is None:
                return
            try:
                # A scan may arrive before TF for its stamp. Retry in later
                # executor ticks; never project it using the newest robot
                # pose, which makes static walls look like moving obstacles.
                transform = self.tf_buffer.lookup_transform(
                    self.global_frame, message.header.frame_id, Time.from_msg(message.header.stamp))
                points = obstacle_endpoints(
                    message, transform.transform, float(self.config["room"]["floor_top"]))
                if len(points) < 30:
                    self.scan_valid = False
                    self.pending_scan = None
                    return
                self.core.set_scan(points, (float(transform.transform.translation.x),
                                            float(transform.transform.translation.y)),
                                   time.monotonic())
                self.scan_at = time.monotonic()
                self.scan_valid = True
                self.pending_scan = None
            except (TransformException, ValueError):
                return

        def update_map_pose(self):
            """Robot pose in the map frame from the localization TF."""
            try:
                transform = self.tf_buffer.lookup_transform(self.global_frame, "base_link", Time())
            except TransformException:
                self.pose = None
                return
            t = transform.transform
            self.pose = Pose2D(float(t.translation.x), float(t.translation.y), yaw_of(t.rotation))

        def tick(self):
            now = time.monotonic()
            if self.global_frame != "odom":
                self.update_map_pose()
            self.project_pending_scan()
            if self.core.goal_key is None:
                self.publish_stop()
                return
            if self.pose is None or now-self.odom_at > 2.0 or not self.scan_valid or now-self.scan_at > 5.0:
                self.publish_stop()
                self.publish_status(self.core.payload("sensor_stale"))
                return
            command = self.core.step(self.pose, now)
            twist = Twist()
            twist.linear.x = command.linear_x
            twist.angular.z = command.angular_z
            self.cmd_pub.publish(twist)
            if command.status is not None:
                self.publish_status(command.status)

        def shutdown(self):
            if rclpy.ok():
                self.publish_stop()

    rclpy.init(args=args)
    node = ExhibitionNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
