"""ROS node: people around the robot from the MID-360 cloud (PLAN M5.2).

Input  /mid360/points (sensor_msgs/PointCloud2), TF odom <- cloud frame and
       odom <- base_link.
Output /guide/people (std_msgs/String, JSON from ``people_core.snapshot``) at
       up to ``rate_hz``, and /guide/people_markers (MarkerArray) for RViz:
       a cylinder per person, orange for a child, with its id and height.

Only the 真机雷达版 (``lidar:=mid360``) has the 3D cloud this needs.
"""

from __future__ import annotations

import json
from typing import Sequence

import numpy as np


def main(args: Sequence[str] | None = None) -> None:
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from pathlib import Path
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py.point_cloud2 import read_points_numpy
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener
    from visualization_msgs.msg import Marker, MarkerArray

    from .exhibition_nav import yaw_of
    from .mid360_to_scan import quaternion_matrix
    from .nav_core import StaticGeometry
    from .people_core import PeopleParams, PeopleTracker, detect_people, snapshot
    from .scene_config import load_scene

    class PeopleTrackerNode(Node):
        def __init__(self):
            super().__init__("people_tracker")
            share = Path(get_package_share_directory("g1_guide"))
            self.declare_parameter("scene_config", str(share / "config/exhibition.json"))
            self.declare_parameter("global_frame", "odom")
            self.declare_parameter("rate_hz", 5.0)
            config = load_scene(self.get_parameter("scene_config").value)
            self.frame = self.get_parameter("global_frame").value
            self.floor_top = float(config["room"].get("floor_top", 0.0))
            self.params = PeopleParams()
            self.static = StaticGeometry(config, self.params.static_tolerance, include_people=False)
            self.tracker = PeopleTracker()
            self.period = 1.0 / float(self.get_parameter("rate_hz").value)
            self.last_publish = 0.0
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.pub = self.create_publisher(String, "/guide/people", 10)
            self.marker_pub = self.create_publisher(MarkerArray, "/guide/people_markers", 10)
            self.create_subscription(PointCloud2, "/mid360/points", self.on_cloud,
                                     qos_profile_sensor_data)
            self.pending = None
            self.create_timer(0.02, self.flush)

        def on_cloud(self, message):
            self.pending = message

        def flush(self):
            message = self.pending
            if message is None:
                return
            stamp = Time.from_msg(message.header.stamp)
            try:
                cloud = self.tf_buffer.lookup_transform(self.frame, message.header.frame_id, stamp)
                base = self.tf_buffer.lookup_transform(self.frame, "base_link", stamp)
            except TransformException:
                return  # TF for this stamp not in yet; retry until a newer cloud arrives
            self.pending = None
            now = stamp.nanoseconds * 1e-9
            if now - self.last_publish < self.period:
                return
            self.last_publish = now
            r, t = cloud.transform.rotation, cloud.transform.translation
            points = read_points_numpy(message, field_names=("x", "y", "z"), skip_nans=True)
            points = np.asarray(points, dtype=float).reshape(-1, 3)
            world = points @ quaternion_matrix(r.x, r.y, r.z, r.w).T + np.array([t.x, t.y, t.z])
            b = base.transform
            robot = (float(b.translation.x), float(b.translation.y), yaw_of(b.rotation))
            detections = detect_people(world, self.static, robot[:2], float(t.z),
                                       self.floor_top, self.params)
            tracks = self.tracker.update(detections, now)
            payload = snapshot(tracks, robot, now, self.frame)
            self.pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
            self.publish_markers(payload, message.header.stamp)

        def publish_markers(self, payload, stamp):
            markers = MarkerArray()
            clear = Marker()
            clear.action = Marker.DELETEALL
            markers.markers.append(clear)
            for person in payload["people"]:
                body = Marker()
                body.header.frame_id = self.frame
                body.header.stamp = stamp
                body.ns, body.id, body.type = "people", person["id"], Marker.CYLINDER
                body.pose.position.x, body.pose.position.y = person["x"], person["y"]
                body.pose.position.z = person["height"] / 2
                body.pose.orientation.w = 1.0
                body.scale.x = body.scale.y = 0.4 * person["group"] ** 0.5
                body.scale.z = person["height"]
                body.color.a = 0.6
                body.color.r, body.color.g, body.color.b = (
                    (1.0, 0.5, 0.0) if person["child"] else (0.2, 0.6, 1.0))
                label = Marker()
                label.header = body.header
                label.ns, label.id, label.type = "people_label", person["id"], Marker.TEXT_VIEW_FACING
                label.pose.position.x, label.pose.position.y = person["x"], person["y"]
                label.pose.position.z = person["height"] + 0.3
                label.pose.orientation.w = 1.0
                label.scale.z = 0.25
                label.color.a = label.color.r = label.color.g = label.color.b = 1.0
                height = f"≥{person['height']:.2f}" if person["height_clipped"] else f"{person['height']:.2f}"
                label.text = f"#{person['id']} {height}m" + (" 儿童" if person["child"] else "")
                markers.markers.extend([body, label])
            self.marker_pub.publish(markers)

    rclpy.init(args=args)
    node = PeopleTrackerNode()
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
