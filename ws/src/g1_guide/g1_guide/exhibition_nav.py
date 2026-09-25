"""ROS adapter for safe, local-only navigation of the Gazebo glide proxy.

Static collision geometry is read from the same exhibition configuration that
generates Gazebo. Laser endpoints are transformed through live TF and added to
the grid before planning. No map/SLAM or biped gait claim is made here.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import time


REPLAN_WALL_INTERVAL_S = 8.0


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


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


def known_static_hit(point: tuple[float, float], config: dict, tolerance: float = 0.25) -> bool:
    """Whether a laser endpoint coincides with configured static geometry."""
    x, y = point
    room = config["room"]
    inner_x = (float(room["width"])-float(room["wall_thickness"]))/2
    inner_y = (float(room["depth"])-float(room["wall_thickness"]))/2
    if abs(abs(x)-inner_x) <= tolerance or abs(abs(y)-inner_y) <= tolerance:
        return True
    for item in (*config["exhibits"], *config["obstacles"]):
        px, py = map(float, item["position"][:2])
        sx, sy = map(float, item["size"][:2])
        dx = max(abs(x-px)-sx/2, 0.0)
        dy = max(abs(y-py)-sy/2, 0.0)
        if math.hypot(dx, dy) <= tolerance:
            return True
    return False


def forward_clearance(pose, points: list[tuple[float, float]], half_angle: float = 0.52) -> float:
    candidates = []
    for x, y in points:
        dx, dy = x-pose.x, y-pose.y
        distance = math.hypot(dx, dy)
        if distance > 1e-6 and abs(wrap_angle(math.atan2(dy, dx)-pose.yaw)) <= half_angle:
            candidates.append(distance)
    return min(candidates, default=math.inf)


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

    from .nav_control import Pose2D, compute_command
    from .nav_grid import OccupancyGrid

    class ExhibitionNavigator(Node):
        def __init__(self):
            super().__init__("exhibition_navigator")
            default_config = str(Path(get_package_share_directory("g1_guide")) / "config/exhibition.json")
            self.declare_parameter("scene_config", default_config)
            self.config = json.loads(Path(self.get_parameter("scene_config").value).read_text(encoding="utf-8"))
            # The follower stops below 0.80 m forward clearance; plan with
            # 0.85 m robot-centre clearance to avoid boundary deadlocks.
            self.grid = OccupancyGrid(self.config, extra_margin=0.30)
            self.stations = {item["station"]["key"]: item["station"] for item in self.config["exhibits"]}
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
            self.status_pub = self.create_publisher(String, "/nav/status", 10)
            self.create_subscription(String, "/nav/goal_key", self.on_goal, 10)
            self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
            self.create_subscription(LaserScan, "/nav_scan", self.on_scan, qos_profile_sensor_data)
            self.create_timer(0.1, self.tick)
            self.pose = None
            self.odom_at = 0.0
            self.scan_at = 0.0
            self.scan_points: list[tuple[float, float]] = []
            self.dynamic_points: list[tuple[float, float]] = []
            self.scan_valid = False
            self.scan_origin = None
            self.pending_scan = None
            self.goal_key = None
            self.path: list[tuple[float, float]] = []
            self.path_index = 0
            self.last_plan_at = 0.0
            self.last_state = None
            self.last_status_name = None
            self.last_status_at = 0.0
            self.replans = 0
            self.min_clearance = math.inf
            self.publish_stop()

        def publish_stop(self):
            self.cmd_pub.publish(Twist())

        def status(self, state: str, **details):
            payload = {"state": state, "goal_key": self.goal_key,
                       "replans": self.replans, "path_points": len(self.path),
                       **details}
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
            key = message.data.strip()
            self.publish_stop()
            self.path = []
            self.path_index = 0
            if key in ("0", "cancel", "q"):
                self.goal_key = None
                self.status("cancelled")
                return
            if key not in self.stations:
                self.goal_key = None
                self.status("invalid_goal", requested=key)
                return
            self.goal_key = key
            self.last_plan_at = 0.0
            self.status("requested")

        def on_odom(self, message):
            if message.header.frame_id != "odom":
                self.pose = None
                return
            try:
                p = message.pose.pose.position
                values = (float(p.x), float(p.y), yaw_of(message.pose.pose.orientation))
                if not all(math.isfinite(v) for v in values):
                    raise ValueError("non-finite odometry")
                self.pose = Pose2D(*values)
                self.odom_at = time.monotonic()
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
                    "odom", message.header.frame_id, Time.from_msg(message.header.stamp))
                points = obstacle_endpoints(
                    message, transform.transform, float(self.config["room"]["floor_top"]))
                if len(points) < 30:
                    self.scan_valid = False
                    self.pending_scan = None
                    return
                self.scan_points = points
                self.dynamic_points = [point for point in points
                                       if not known_static_hit(point, self.config)]
                self.scan_origin = (float(transform.transform.translation.x),
                                    float(transform.transform.translation.y))
                self.scan_at = time.monotonic()
                self.scan_valid = True
                self.pending_scan = None
            except (TransformException, ValueError):
                return

        def tick(self):
            now = time.monotonic()
            self.project_pending_scan()
            if self.goal_key is None:
                self.publish_stop()
                return
            if self.pose is None or now-self.odom_at > 2.0 or not self.scan_valid or now-self.scan_at > 5.0:
                self.publish_stop()
                self.status("sensor_stale")
                return
            station = self.stations[self.goal_key]
            goal = Pose2D(float(station["x"]), float(station["y"]), float(station["yaw"]))
            distance_to_goal = math.hypot(self.pose.x-goal.x, self.pose.y-goal.y)
            # The planner is refreshed as fresh LiDAR obstacles enter its map.
            if not self.path or now-self.last_plan_at > REPLAN_WALL_INTERVAL_S:
                # Planning runs in the same ROS callback thread. Never leave
                # the preceding velocity active while A* is calculating.
                self.publish_stop()
                plan = self.grid.plan((self.pose.x, self.pose.y), (goal.x, goal.y),
                                      dynamic_points=self.dynamic_points)
                self.last_plan_at = time.monotonic()
                if plan is None:
                    self.path = []
                    self.publish_stop()
                    nearest = min((math.hypot(x-self.pose.x, y-self.pose.y)
                                   for x, y in self.scan_points), default=math.inf)
                    goal_nearest = min(self.scan_points,
                                       key=lambda p: math.hypot(p[0]-goal.x, p[1]-goal.y))
                    self.status("blocked", distance_m=round(distance_to_goal, 3),
                                scan_points=len(self.scan_points),
                                dynamic_points=len(self.dynamic_points),
                                nearest_scan_m=round(nearest, 3) if math.isfinite(nearest) else None,
                                nearest_goal_scan_point=[round(v, 3) for v in goal_nearest],
                                nearest_goal_scan_m=round(math.hypot(
                                    goal_nearest[0]-goal.x, goal_nearest[1]-goal.y), 3),
                                scan_origin=[round(v, 3) for v in self.scan_origin]
                                if self.scan_origin is not None else None,
                                start_dynamic_clear=self.grid.is_clear(
                                    self.pose.x, self.pose.y, self.dynamic_points),
                                goal_dynamic_clear=self.grid.is_clear(
                                    goal.x, goal.y, self.dynamic_points),
                                static_path_exists=self.grid.plan(
                                    (self.pose.x, self.pose.y), (goal.x, goal.y)) is not None)
                    return
                self.path = plan
                self.path_index = 0
                self.replans += 1
            while self.path_index < len(self.path)-1 and math.hypot(
                    self.pose.x-self.path[self.path_index][0],
                    self.pose.y-self.path[self.path_index][1]) <= 0.25:
                self.path_index += 1
            waypoint_xy = self.path[self.path_index]
            waypoint = Pose2D(float(waypoint_xy[0]), float(waypoint_xy[1]), goal.yaw)
            clearance = forward_clearance(self.pose, self.scan_points)
            if math.isfinite(clearance):
                self.min_clearance = min(self.min_clearance, clearance)
            final = self.path_index == len(self.path)-1
            velocity = compute_command(self.pose, waypoint, goal, clearance, final=final)
            twist = Twist()
            twist.linear.x = max(-0.35, min(0.35, float(velocity.linear_x)))
            twist.angular.z = max(-0.8, min(0.8, float(velocity.angular_z)))
            if not all(math.isfinite(v) for v in (twist.linear.x, twist.angular.z)):
                self.publish_stop()
                self.status("invalid_command")
                return
            self.cmd_pub.publish(twist)
            self.status(velocity.status, distance_m=round(distance_to_goal, 3),
                        clearance_m=round(clearance, 3) if math.isfinite(clearance) else None,
                        waypoint_index=self.path_index)
            if velocity.status == "blocked":
                self.path = []
            if velocity.status == "arrived":
                self.goal_key = None
                self.publish_stop()

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
