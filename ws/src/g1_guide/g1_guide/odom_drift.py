"""Simulated leg odometry for the 真机雷达版: Gazebo truth plus accumulating drift.

Gazebo's OdometryPublisher is exact, which hides the reason a real robot needs
localization at all.  This node integrates the truth motion step by step with
the errors real odometry has -- a distance scale error, a turn scale error and
a slow heading bias while moving -- and publishes the result as /odom plus the
odom -> base_link TF.  AMCL (or slam_toolbox) then has to correct it via
map -> odom.  All errors at 0 reproduce the truth exactly.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class DriftModel:
    linear_scale: float = 0.0     # +0.03: odometry reports 3 % more distance
    angular_scale: float = 0.0    # +0.02: odometry reports 2 % more rotation
    yaw_bias: float = 0.0         # rad/s added while the robot is moving


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def step(drifted: tuple[float, float, float], before: tuple[float, float, float],
         after: tuple[float, float, float], dt: float, model: DriftModel,
         moving_speed: float = 0.02) -> tuple[float, float, float]:
    """Advance the drifted pose by the truth motion ``before -> after``."""

    bx, by, byaw = before
    dx, dy = after[0] - bx, after[1] - by
    # Motion in the robot's own frame is what odometry actually measures.
    forward = math.cos(byaw) * dx + math.sin(byaw) * dy
    left = -math.sin(byaw) * dx + math.cos(byaw) * dy
    turn = wrap(after[2] - byaw)
    moving = dt > 0 and math.hypot(dx, dy) / dt > moving_speed
    forward *= 1.0 + model.linear_scale
    left *= 1.0 + model.linear_scale
    turn = turn * (1.0 + model.angular_scale) + (model.yaw_bias * dt if moving else 0.0)
    x, y, yaw = drifted
    return (x + math.cos(yaw) * forward - math.sin(yaw) * left,
            y + math.sin(yaw) * forward + math.cos(yaw) * left,
            wrap(yaw + turn))


def main(args: Sequence[str] | None = None) -> None:
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from tf2_ros import TransformBroadcaster

    def yaw_of(q) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    class OdomDrift(Node):
        def __init__(self):
            super().__init__("odom_drift")
            self.declare_parameter("linear_scale", 0.0)
            self.declare_parameter("angular_scale", 0.0)
            self.declare_parameter("yaw_bias", 0.0)
            self.model = DriftModel(*(float(self.get_parameter(name).value)
                                      for name in ("linear_scale", "angular_scale", "yaw_bias")))
            self.pub = self.create_publisher(Odometry, "/odom", 10)
            self.tf = TransformBroadcaster(self)
            self.create_subscription(Odometry, "/odom_truth", self.on_truth, qos_profile_sensor_data)
            self.last = None
            self.last_t = None
            self.pose = None
            self.get_logger().info(f"里程计漂移模型：{self.model}")

        def on_truth(self, message):
            p = message.pose.pose.position
            truth = (float(p.x), float(p.y), yaw_of(message.pose.pose.orientation))
            t = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            if self.pose is None or t < self.last_t:
                # Start (or a sim reset) aligns odometry with the truth.
                self.pose = truth
            else:
                self.pose = step(self.pose, self.last, truth, t - self.last_t, self.model)
            self.last, self.last_t = truth, t
            x, y, yaw = self.pose
            qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)

            odom = Odometry()
            odom.header.stamp = message.header.stamp
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
            odom.pose.pose.position.z = p.z
            odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
            odom.twist = message.twist
            self.pub.publish(odom)

            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = "base_link"
            transform.transform.translation.x, transform.transform.translation.y = x, y
            transform.transform.translation.z = p.z
            transform.transform.rotation.z, transform.transform.rotation.w = qz, qw
            self.tf.sendTransform(transform)

    rclpy.init(args=args)
    node = OdomDrift()
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
