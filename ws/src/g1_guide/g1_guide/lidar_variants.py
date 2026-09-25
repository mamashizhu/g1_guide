"""Launch pieces shared by slam.launch.py and slam_nav.launch.py.

simple (简化雷达版): nothing extra -- the model publishes /nav_scan and /odom.
mid360 (真机雷达版): the model publishes a 3D cloud and truth odometry, so the
2D scan and the (optionally drifting) /odom are produced by nodes.
"""

from __future__ import annotations

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
