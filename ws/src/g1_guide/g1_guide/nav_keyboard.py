"""Read 1..N exhibit keys in one terminal and send local navigation requests."""

from __future__ import annotations

import time

from .g1_teleop import KeyReader


def main(args=None) -> None:
    import rclpy
    from std_msgs.msg import String

    rclpy.init(args=args)
    node = rclpy.create_node("exhibition_nav_keyboard")
    publisher = node.create_publisher(String, "/nav/goal_key", 10)
    node.get_logger().info("Press 1–6 to navigate, 0 or Space to cancel, q to quit.")
    try:
        with KeyReader() as reader:
            while rclpy.ok():
                key = reader.read_key()
                if key in ("q", "Q"):
                    publisher.publish(String(data="cancel"))
                    rclpy.spin_once(node, timeout_sec=0.1)
                    break
                if key in ("0", " "):
                    publisher.publish(String(data="cancel"))
                elif key is not None and key.isdecimal():
                    publisher.publish(String(data=key))
                rclpy.spin_once(node, timeout_sec=0.0)
                time.sleep(0.02)
    finally:
        publisher.publish(String(data="cancel"))
        rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
