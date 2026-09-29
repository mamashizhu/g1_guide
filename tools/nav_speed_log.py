#!/usr/bin/env python3
"""每秒打印一行：墙钟时间、仿真时间、实时率、机器人速度（仿真时间下）、导航状态。

配合 tools/nav_gz_tour.sh 使用，判断"变慢"是导航停顿还是 Gazebo 实时率下降。
"""
import json
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


class SpeedLog(Node):
    def __init__(self):
        super().__init__("nav_speed_log")
        self.sim = None
        self.pose = None
        self.state = "-"
        self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(String, "/nav/status", self.on_status, 10)
        self.last = None
        self.start = time.monotonic()
        self.create_timer(1.0, self.tick)

    def on_clock(self, m):
        self.sim = m.clock.sec + m.clock.nanosec * 1e-9

    def on_odom(self, m):
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y)

    def on_status(self, m):
        try:
            self.state = json.loads(m.data)["state"]
        except (ValueError, KeyError):
            pass

    def tick(self):
        now = time.monotonic()
        if self.sim is None or self.pose is None:
            return
        if self.last is not None:
            wall0, sim0, pose0 = self.last
            dsim = self.sim - sim0
            rtf = dsim / (now - wall0)
            speed = math.dist(self.pose, pose0) / dsim if dsim > 0 else 0.0
            print(f"{now-self.start:6.0f} sim={self.sim:7.1f} rtf={rtf:4.2f} v={speed:4.2f} "
                  f"({self.pose[0]:6.2f},{self.pose[1]:6.2f}) {self.state}", flush=True)
        self.last = (now, self.sim, self.pose)


rclpy.init()
rclpy.spin(SpeedLog())
