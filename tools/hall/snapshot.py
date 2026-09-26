#!/usr/bin/env python3
"""Photograph exhibits in a running simulation with a temporary camera.

Spawns a static camera model into the running world, bridges its image to ROS,
and saves one PNG per named view.  Use it to check how an exhibit really
renders (texture orientation, proportions) without driving the robot there.

    ros2 launch g1_guide exhibition.launch.py headless:=true &
    python3 tools/hall/snapshot.py --out /tmp/shots            # all views
    python3 tools/hall/snapshot.py --out /tmp/shots E05 E06    # some views

Views are camera poses in world coordinates (x y z roll pitch yaw; positive
pitch looks down).  Run inside WSL with the ROS environment sourced.
"""

from __future__ import annotations

import argparse
import math
import subprocess
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

WORLD = "exhibition"
TOPIC = "/snapshot/image"
VIEWS = {  # exhibition.json hall, each view roughly where a visitor would stand
    "E01": (-3.7, 8.6, 1.6, 0.0, -0.05, 1.5708),
    "E03": (-6.6, 9.1, 1.6, 0.0, -0.05, 3.1416),
    "E05": (3.3, 5.9, 1.55, 0.0, 0.46, 1.5708),
    "E05_top": (3.3, 6.5, 1.9, 0.0, 1.2, 1.5708),
    "E06": (-3.9, -7.3, 1.45, 0.0, 0.2, 3.1416),
}

CAMERA_SDF = f"""<sdf version="1.9"><model name="snapshot_cam"><static>true</static>
<link name="link"><sensor name="snapshot" type="camera">
<always_on>true</always_on><update_rate>5</update_rate><topic>{TOPIC}</topic>
<camera><horizontal_fov>1.0</horizontal_fov><image><width>1280</width><height>960</height></image>
<clip><near>0.05</near><far>30</far></clip></camera></sensor></link></model></sdf>"""


def _set_pose(pose) -> None:
    x, y, z, roll, pitch, yaw = pose
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    q = (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
         cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)
    req = (f'name: "snapshot_cam", position: {{x: {x}, y: {y}, z: {z}}}, '
           f"orientation: {{w: {q[0]}, x: {q[1]}, y: {q[2]}, z: {q[3]}}}")
    subprocess.run(["gz", "service", "-s", f"/world/{WORLD}/set_pose", "--reqtype", "gz.msgs.Pose",
                    "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req", req],
                   check=True, capture_output=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("views", nargs="*", default=list(VIEWS))
    parser.add_argument("--out", type=Path, default=Path("/tmp/g1_snapshots"))
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    subprocess.run(["ros2", "run", "ros_gz_sim", "create", "-world", WORLD, "-name", "snapshot_cam",
                    "-string", CAMERA_SDF, "-z", "30"], check=True, capture_output=True)
    bridge = subprocess.Popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                               f"{TOPIC}@sensor_msgs/msg/Image[gz.msgs.Image"],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rclpy.init()
    node = rclpy.create_node("snapshot")
    latest: dict[str, Image] = {}
    node.create_subscription(Image, TOPIC, lambda msg: latest.update(img=msg), 1)
    bridge_cv = CvBridge()
    try:
        for name in args.views:
            _set_pose(VIEWS[name])
            latest.clear()
            deadline = time.monotonic() + 60
            frames = 0
            while frames < 3 and time.monotonic() < deadline:  # skip frames rendered before the move
                rclpy.spin_once(node, timeout_sec=0.5)
                if "img" in latest:
                    frames += 1
                    image = latest.pop("img")
            if frames < 3:
                print(f"{name}: no image (is the world running?)")
                continue
            path = args.out / f"{name}.png"
            cv2.imwrite(str(path), bridge_cv.imgmsg_to_cv2(image, "bgr8"))
            print(f"{name}: {path}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        bridge.terminate()
        subprocess.run(["gz", "service", "-s", f"/world/{WORLD}/remove", "--reqtype", "gz.msgs.Entity",
                        "--reptype", "gz.msgs.Boolean", "--timeout", "5000",
                        "--req", 'name: "snapshot_cam", type: MODEL'], capture_output=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
