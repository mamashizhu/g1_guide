#!/usr/bin/env python3
"""从仿真头部相机存图（PLAN M5.0 CV 可行性验证用）。

用法（WSL，已 source ROS）：python3 tools/cv_capture.py <输出目录> <张数> [间隔秒] [前缀]
相机话题默认 /front_camera/image（加装的水平 USB 相机），CV_TOPIC=/camera/image 换头部 D435i。
每张存 <前缀>_NN.png，并在 frames.jsonl 里记下当时的机器人位姿（/odom）。
"""

import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def to_bgr(message: Image) -> np.ndarray:
    data = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    image = data[:, :message.width * 3].reshape(message.height, message.width, 3)
    return image[:, :, ::-1] if message.encoding == "rgb8" else image


def main() -> None:
    import cv2

    out = Path(sys.argv[1])
    count = int(sys.argv[2])
    interval = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
    prefix = sys.argv[4] if len(sys.argv) > 4 else "frame"
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node("cv_capture")
    latest = {}
    node.create_subscription(Image, os.environ.get("CV_TOPIC", "/front_camera/image"), lambda m: latest.__setitem__("image", m),
                             qos_profile_sensor_data)
    node.create_subscription(Odometry, "/odom", lambda m: latest.__setitem__("odom", m),
                             qos_profile_sensor_data)
    saved, next_at = 0, 0.0
    deadline = time.monotonic() + count * interval + 60
    with open(out / "frames.jsonl", "a", encoding="utf-8") as log:
        while saved < count and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if "image" not in latest or time.monotonic() < next_at:
                continue
            image = latest.pop("image")
            name = f"{prefix}_{saved:02d}.png"
            cv2.imwrite(str(out / name), to_bgr(image))
            pose = None
            if "odom" in latest:
                p = latest["odom"].pose.pose
                q = p.orientation
                pose = [round(p.position.x, 3), round(p.position.y, 3),
                        round(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), 3)]
            log.write(json.dumps({"file": name, "pose": pose}) + "\n")
            print(name, pose, flush=True)
            saved += 1
            next_at = time.monotonic() + interval
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
