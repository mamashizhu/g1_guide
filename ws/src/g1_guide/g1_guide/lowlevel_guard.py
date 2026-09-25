"""Wall-clock target protection for simulation; PD runs in ros2_control, not here."""
import json
from pathlib import Path
import time
import signal

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from control_msgs.msg import MultiDOFCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from g1_guide.joint_targets import JointTargetGuard


class GuardNode(Node):
    def __init__(self):
        super().__init__("g1_lowlevel_guard")
        path = self.declare_parameter("limits_file", "").value
        self.guard = JointTargetGuard(json.loads(Path(path).read_text(encoding="utf-8")))
        self.reference = self.create_publisher(MultiDOFCommand, "/g1_pd/reference", 1)
        self.status = self.create_publisher(String, "/g1_lowlevel/status", 1)
        self.create_subscription(JointState, "/joint_states", self.feedback, qos_profile_sensor_data)
        self.create_subscription(JointState, "/g1_lowlevel/target", self.command, 1)
        self.create_timer(0.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def feedback(self, message):
        try:
            self.guard.feedback(message.name, message.position, time.monotonic())
        except ValueError as error:
            self.get_logger().error(f"Invalid feedback: {error}", throttle_duration_sec=2.0)

    def command(self, message):
        try:
            self.guard.command(message.name, message.position, time.monotonic())
        except ValueError as error:
            self.get_logger().warning(f"Rejected target: {error}")

    def tick(self):
        try:
            positions, velocities, state = self.guard.step(time.monotonic())
            # Derivative reference deliberately zero: if this process dies, the
            # upstream PD must hold its last bounded position, not retain a
            # nonzero velocity feed-forward. Rate limiting is applied to q.
            self.reference.publish(MultiDOFCommand(
                dof_names=list(self.guard.names), values=positions,
                values_dot=[0.0] * len(positions)))
        except ValueError:
            state = "waiting_feedback"
        self.status.publish(String(data=json.dumps({
            "state": state, "rejected_count": self.guard.rejected_count,
            "support": "fixed_pelvis", "hardware": False,
        })))


def main(args=None):
    # Do not invalidate the ROS context from a signal thread while spin_once
    # is taking a message. Stop dispatch first, then destroy node and context.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stopping = False

    def stop(signum, frame):
        nonlocal stopping
        stopping = True

    previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    try:
        node = GuardNode()
        while not stopping and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
