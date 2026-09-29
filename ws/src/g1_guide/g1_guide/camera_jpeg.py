"""ROS node: throttle a camera and publish it as JPEG for the vision process.

Raw 640x480 RGB is ~0.9 MB a frame; through rosbridge (Windows side, PLAN M5)
that is too heavy.  Default: /front_camera/image -> /front_camera/image/compressed
(sensor_msgs/CompressedImage) at 5 Hz.  Only started with ``vision:=true``,
because subscribing makes Gazebo render the camera (see config/bridge.yaml).
"""

from __future__ import annotations

from typing import Sequence

import numpy as np


def encode_jpeg(data: bytes, width: int, height: int, step: int, encoding: str,
                quality: int = 80) -> bytes:
    import cv2

    rows = np.frombuffer(data, dtype=np.uint8).reshape(height, step)
    image = rows[:, :width * 3].reshape(height, width, 3)
    if encoding == "rgb8":
        image = image[:, :, ::-1]
    elif encoding != "bgr8":
        raise ValueError(f"unsupported encoding {encoding!r}")
    ok, buffer = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    if not ok:
        raise ValueError("JPEG encoding failed")
    return buffer.tobytes()


def main(args: Sequence[str] | None = None) -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, Image

    class CameraJpeg(Node):
        def __init__(self):
            super().__init__("camera_jpeg")
            self.declare_parameter("input", "/front_camera/image")
            self.declare_parameter("rate_hz", 5.0)
            self.declare_parameter("quality", 80)
            source = self.get_parameter("input").value
            self.period = 1.0 / float(self.get_parameter("rate_hz").value)
            self.quality = int(self.get_parameter("quality").value)
            self.last = -1e9
            self.pub = self.create_publisher(CompressedImage, source + "/compressed", 1)
            self.create_subscription(Image, source, self.on_image, qos_profile_sensor_data)

        def on_image(self, message):
            now = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            if now - self.last < self.period:
                return
            self.last = now
            try:
                data = encode_jpeg(bytes(message.data), message.width, message.height,
                                   message.step, message.encoding, self.quality)
            except ValueError as error:
                self.get_logger().warning(str(error))
                return
            out = CompressedImage()
            out.header = message.header
            out.format = "jpeg"
            out.data = data
            self.pub.publish(out)

    rclpy.init(args=args)
    node = CameraJpeg()
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
