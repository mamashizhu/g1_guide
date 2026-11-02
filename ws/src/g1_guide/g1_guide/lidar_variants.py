"""Launch pieces shared by slam.launch.py and slam_nav.launch.py.

simple (简化雷达版): nothing extra -- the model publishes /nav_scan and /odom.
mid360 (真机雷达版): the model publishes a 3D cloud and truth odometry, so the
2D scan and the (optionally drifting) /odom are produced by nodes.
"""

from __future__ import annotations

import math
from pathlib import Path

LIDAR_VARIANTS = ("simple", "mid360")

# 真机雷达版 odometry error used when drift is switched on: 3 % distance,
# 2 % turn and 0.3 °/s heading creep while moving -- in the range of legged
# robot odometry, large enough to show localization pulling it back.
DRIFT = {"linear_scale": 0.03, "angular_scale": 0.02, "yaw_bias": 0.005}


def lidar_actions(share: Path, lidar: str, scene_config, drift: bool) -> list:
    from launch.actions import SetLaunchConfiguration
    from launch_ros.actions import Node

    if lidar not in LIDAR_VARIANTS:
        raise RuntimeError(f"lidar must be one of {LIDAR_VARIANTS}, got {lidar!r}")
    if lidar == "simple":
        if drift:
            raise RuntimeError("odom_drift needs lidar:=mid360; the simple model publishes exact /odom")
        return []
    model = {"linear_scale": 0.0, "angular_scale": 0.0, "yaw_bias": 0.0}
    if drift:
        model = dict(DRIFT)
    return [
        # g1_sim.launch.py (included later) reads model_file from the context.
        SetLaunchConfiguration("model_file", str(share / "urdf/g1_glide_mid360.urdf")),
        Node(package="g1_bridge", executable="managed_bridge", name="mid360_bridge",
             parameters=[{"config_file": str(share / "config/bridge_mid360.yaml")}],
             output="screen"),
        Node(package="g1_guide", executable="mid360_to_scan", name="mid360_to_scan",
             output="screen",
             parameters=[{"scene_config": scene_config, "use_sim_time": True}]),
        Node(package="g1_guide", executable="odom_drift", name="odom_drift", output="screen",
             parameters=[{**model, "use_sim_time": True}]),
    ]


def default_map_name(scene_config: str, lidar: str) -> str:
    """``<scene stem>_map[_mid360]``: one map per hall and lidar, never mixed up."""

    return Path(scene_config).stem + "_map" + ("_mid360" if lidar == "mid360" else "")


def project_maps_dir(launch_file: str) -> Path:
    """``<project>/maps``: installed launch files are symlinks into the source tree."""

    return Path(launch_file).resolve().parents[4] / "maps"


def shutdown_after(node) -> list:
    """Stop the whole launch once ``node`` (a finished tour) exits."""

    from launch.actions import EmitEvent, RegisterEventHandler
    from launch.event_handlers import OnProcessExit
    from launch.events import Shutdown

    return [node, RegisterEventHandler(OnProcessExit(
        target_action=node, on_exit=[EmitEvent(event=Shutdown(reason="tour finished"))]))]


def with_camera_rate(urdf: str, rate: float) -> str:
    """Set the head RGBD camera's update rate in a generated model's text.

    Every rendered frame costs time and, through the WSL d3d12 leak, makes
    the next ones slower (see CLAUDE.md); 30 Hz RGB + depth dominated the
    server's rendering.  A model without the camera is returned unchanged.
    """
    import re

    rate = float(rate)
    if not (rate > 0.0 and rate < 1000.0):
        raise ValueError(f"camera_rate must be in (0, 1000) Hz, got {rate}")
    return re.sub(r'(<sensor name="d435_rgbd"[^>]*>.*?<update_rate>)[^<]*(</update_rate>)',
                  lambda m: f"{m[1]}{rate:g}{m[2]}", urdf, count=1, flags=re.S)


# 展示用第三人称跟拍相机（网页控制台 demo_web 的「Gazebo 画面」）：挂在骨盆上，
# 在机器人身后上方往前下方看；展馆没有屋顶，高处的相机不会被墙挡住。
# 只在有人订阅 /chase_camera/image 时渲染（桥接 lazy，见 exhibition.launch.py）。
CHASE_CAMERA_POSE = "-1.6 0 2.4 0 0.82 0"   # 相对 pelvis（离地约 0.8 m）：x y z roll pitch yaw
CHASE_CAMERA_TOPIC = "/chase_camera/image"


def with_chase_camera(urdf: str, pose: str = CHASE_CAMERA_POSE, rate: float = 10.0) -> str:
    """Add the third-person chase camera to a generated glide model's text.

    A model without ``pelvis`` (or one that already has the camera) is
    returned unchanged.
    """
    values = pose.split()
    if len(values) != 6 or not all(_is_number(v) for v in values):
        raise ValueError(f"chase camera pose needs 6 numbers 'x y z roll pitch yaw', got {pose!r}")
    rate = float(rate)
    if not (rate > 0.0 and rate < 1000.0):
        raise ValueError(f"chase camera rate must be in (0, 1000) Hz, got {rate}")
    if '<link name="pelvis"' not in urdf or 'name="chase_camera"' in urdf:
        return urdf
    block = (
        '  <gazebo reference="pelvis">\n'
        '    <sensor name="chase_camera" type="camera">\n'
        f'      <pose>{" ".join(values)}</pose>\n'
        '      <always_on>true</always_on>\n'
        '      <visualize>false</visualize>\n'
        f'      <update_rate>{rate:g}</update_rate>\n'
        f'      <topic>{CHASE_CAMERA_TOPIC}</topic>\n'
        '      <camera>\n'
        '        <horizontal_fov>1.45</horizontal_fov>\n'
        '        <image><width>960</width><height>540</height><format>R8G8B8</format></image>\n'
        '        <clip><near>0.1</near><far>60</far></clip>\n'
        '      </camera>\n'
        '    </sensor>\n'
        '  </gazebo>\n'
    )
    end = urdf.rfind("</robot>")
    if end < 0:
        raise ValueError("model text has no </robot>")
    return urdf[:end] + block + urdf[end:]


def _is_number(text: str) -> bool:
    try:
        return math.isfinite(float(text))
    except ValueError:
        return False
