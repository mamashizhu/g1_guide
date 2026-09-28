"""ROS node: play named arm gestures on the gliding G1 in the exhibition.

Input  /guide/gesture (std_msgs/String): a gesture name such as ``wave``, or
       JSON ``{"name": "point", "exhibit": "E03"}`` /
       ``{"name": "point", "target": [x, y, z]}`` (world frame).  A bare
       ``point`` aims at the nearest exhibit.
Output /g1/arm/<joint>/cmd_pos (std_msgs/Float64, bridged to Gazebo) and
       /guide/gesture_status (JSON: gesture, state, reason).

While /nav/status reports the robot moving, the arms are sent home and every
other gesture is rejected.  With ``auto_point`` the guide points at the exhibit
as soon as navigation reports ``arrived``.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from g1_guide.arm_kinematics import ALL_ARM_JOINTS, ArmKinematics, arm_joint_names
from g1_guide.gestures import GestureEngine, GestureError, GestureLibrary

MOVING_STATES = {"requested", "aligning", "moving", "blocked", "backing_up"}
COMMAND_TOPIC = "/g1/arm/{joint}/cmd_pos"
# Pick the left arm only for targets clearly on the left; straight ahead uses
# the right hand, as most guides do.
LEFT_ARM_MIN_Y = 0.25


def parse_request(text: str) -> dict:
    """Normalise a /guide/gesture payload to a dict with at least ``name``."""
    text = text.strip()
    if text.startswith("{"):
        request = json.loads(text)
        if not isinstance(request, dict) or not isinstance(request.get("name"), str):
            raise GestureError("JSON gesture needs a string 'name'")
        return request
    if not text:
        raise GestureError("empty gesture name")
    return {"name": text}


def world_to_base(target, position, quaternion) -> np.ndarray:
    """Express a world point in base_link given the odom pose (x, y, z, w)."""
    x, y, z, w = quaternion
    rotation = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    return rotation.T @ (np.asarray(target, float) - np.asarray(position, float))


def point_pose(kinematics: ArmKinematics, home: list[float], target_base) -> tuple[list[float], str, float]:
    """14 joint values pointing one arm at ``target_base``; the other stays home."""
    side = "left" if target_base[1] > LEFT_ARM_MIN_Y else "right"
    arm, error = kinematics.solve_point(side, target_base)
    values = list(home)
    offset = 0 if side == "left" else 7
    values[offset:offset + 7] = arm
    return values, side, error


def main(args=None):
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Float64, String

    from g1_guide.scene_config import load_scene

    class GesturePlayer(Node):
        def __init__(self):
            super().__init__("gesture_player")
            share = Path(get_package_share_directory("g1_guide"))
            self.declare_parameter("gestures_file", str(share / "config/gestures.yaml"))
            self.declare_parameter("model_file", str(share / "urdf/g1_glide.urdf"))
            self.declare_parameter("scene_config", str(share / "config/exhibition.json"))
            self.declare_parameter("rate_hz", 50.0)
            self.declare_parameter("max_speed", 1.5)
            self.declare_parameter("auto_point", True)
            param = lambda name: self.get_parameter(name).value

            self.kinematics = ArmKinematics.from_file(Path(param("model_file")))
            self.library = GestureLibrary.from_file(Path(param("gestures_file")))
            self.engine = GestureEngine(self.library, self.kinematics.limits,
                                        max_speed=float(param("max_speed")))
            config = load_scene(Path(param("scene_config")))
            self.exhibits = {item["exhibit_id"]: item for item in config["exhibits"]}
            self.by_station = {item["station"]["key"]: item for item in config["exhibits"]}
            self.auto_point = bool(param("auto_point"))

            self.pose = None
            self.moving = False
            self.last_status = None
            self.publishers_ = [self.create_publisher(Float64, COMMAND_TOPIC.format(joint=j), 10)
                                for j in ALL_ARM_JOINTS]
            self.status_pub = self.create_publisher(String, "/guide/gesture_status", 10)
            self.create_subscription(String, "/guide/gesture", self.on_gesture, 10)
            self.create_subscription(String, "/nav/status", self.on_nav, 10)
            self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
            self.create_timer(1.0 / float(param("rate_hz")), self.tick)
            self.get_logger().info("gestures: " + ", ".join(sorted(self.library.gestures)))

        def now(self) -> float:
            return self.get_clock().now().nanoseconds * 1e-9

        def report(self, state, gesture=None, **details):
            payload = {"state": state, "gesture": gesture, **details}
            self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
            if state == "rejected":
                self.get_logger().warning(json.dumps(payload, ensure_ascii=False))

        def on_odom(self, message):
            p, q = message.pose.pose.position, message.pose.pose.orientation
            self.pose = ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

        def on_nav(self, message):
            try:
                status = json.loads(message.data)
            except json.JSONDecodeError:
                return
            state = status.get("state")
            was_moving, self.moving = self.moving, state in MOVING_STATES
            if self.moving and not was_moving:
                self.play({"name": "home"})
            if state == "arrived" and self.auto_point:
                exhibit = self.by_station.get(str(status.get("goal_key")))
                if exhibit is not None:
                    self.play({"name": "point", "exhibit": exhibit["exhibit_id"]})

        def on_gesture(self, message):
            try:
                request = parse_request(message.data)
            except (GestureError, json.JSONDecodeError) as error:
                self.report("rejected", reason=str(error))
                return
            self.play(request)

        def point_target(self, request):
            if "target" in request:
                target = request["target"]
                if not (isinstance(target, list) and len(target) == 3):
                    raise GestureError("target must be [x, y, z] in the world frame")
                return [float(v) for v in target], None
            if self.pose is None:
                raise GestureError("no /odom yet; cannot locate exhibits")
            if "exhibit" in request:
                exhibit = self.exhibits.get(request["exhibit"])
                if exhibit is None:
                    raise GestureError(f"unknown exhibit {request['exhibit']!r}")
            else:
                here = np.array(self.pose[0][:2])
                exhibit = min(self.exhibits.values(), key=lambda e: np.linalg.norm(
                    np.array(e["position"][:2]) - here))
            return exhibit["position"], exhibit["exhibit_id"]

        def play(self, request):
            name = request["name"]
            if self.moving and name != "home":
                self.report("rejected", name, reason="robot is moving; arms stay home")
                return
            try:
                details = {}
                values = None
                if self.library.gestures.get(name) and self.library.gestures[name].kind == "point":
                    target, exhibit = self.point_target(request)
                    if self.pose is None:
                        raise GestureError("no /odom yet")
                    base = world_to_base(target, *self.pose)
                    values, side, error = point_pose(self.kinematics, self.library.home(), base)
                    details = {"exhibit": exhibit, "arm": side,
                               "ik_error_deg": round(math.degrees(error), 2)}
                self.engine.start(name, self.now(), point_values=values)
            except GestureError as error:
                self.report("rejected", name, reason=str(error))
                return
            self.report("playing", name, **details)

        def tick(self):
            values = self.engine.sample(self.now())
            for publisher, value in zip(self.publishers_, values):
                publisher.publish(Float64(data=float(value)))
            if self.engine.active != self.last_status:
                if self.engine.active is None:
                    self.report("idle", self.last_status)
                self.last_status = self.engine.active

    rclpy.init(args=args)
    node = GesturePlayer()
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
