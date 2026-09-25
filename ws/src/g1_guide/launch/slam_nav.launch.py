"""SLAM 地图导航 (PLAN M2.2): navigate on a saved SLAM map, localized by AMCL.

Pipeline, the same as on a real robot:
  saved map (map_server) + /nav_scan + odometry --AMCL--> map -> odom
  exhibition_nav plans on the map's occupied cells, in the map frame.

Defaults are the 真机雷达版 with leg-odometry drift on:
  lidar:=mid360 odom_drift:=true map:=<project>/maps/<scene stem>_map_mid360.yaml
tour:=true   visits all stations and writes a report with true arrival and
             localization errors (<output_dir>/nav_report.json).

The original layout-based navigation stays in exhibition_nav.launch.py.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context):
    from g1_guide.lidar_variants import (default_map_name, lidar_actions, project_maps_dir,
                                         shutdown_after)
    from g1_guide.scene_config import load_scene

    share = Path(get_package_share_directory("g1_guide"))
    scene_config = LaunchConfiguration("scene_config")
    lidar = LaunchConfiguration("lidar").perform(context)
    drift = LaunchConfiguration("odom_drift").perform(context).lower() == "true"
    map_yaml = LaunchConfiguration("map").perform(context) or str(
        project_maps_dir(__file__) / (default_map_name(scene_config.perform(context), lidar) + ".yaml"))
    if not Path(map_yaml).is_file():
        raise RuntimeError(f"SLAM map not found: {map_yaml}. Build it first with slam.launch.py explore:=true")
    from g1_guide.slam_map import check_map_scene
    try:
        check_map_scene(map_yaml, scene_config.perform(context))
    except ValueError as exc:
        raise RuntimeError(str(exc)) from exc
    spawn = load_scene(scene_config.perform(context))["spawn"]
    output_dir = LaunchConfiguration("output_dir").perform(context) or str(project_maps_dir(__file__))

    actions = lidar_actions(share, lidar, scene_config, drift)
    actions += [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/exhibition.launch.py")),
            launch_arguments={"scene_config": scene_config,
                              "headless": LaunchConfiguration("headless")}.items(),
        ),
        Node(package="nav2_map_server", executable="map_server", name="map_server", output="screen",
             parameters=[{"yaml_filename": map_yaml, "use_sim_time": True}]),
        Node(package="nav2_amcl", executable="amcl", name="amcl", output="screen",
             parameters=[str(share / "config/amcl.yaml"), {
                 "use_sim_time": True,
                 "initial_pose.x": float(spawn["x"]), "initial_pose.y": float(spawn["y"]),
                 "initial_pose.z": 0.0, "initial_pose.yaw": float(spawn["yaw"])}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_localization", output="screen",
             parameters=[{"use_sim_time": True, "autostart": True,
                          "node_names": ["map_server", "amcl"]}]),
        Node(package="g1_guide", executable="exhibition_nav", name="exhibition_navigator",
             output="screen",
             parameters=[{"scene_config": scene_config, "use_sim_time": True,
                          "map_yaml": map_yaml, "global_frame": "map",
                          # 0.15 m + AMCL error (~0.05-0.1 m) keeps the true
                          # stop error inside the 0.25 m acceptance.
                          "arrival_distance": 0.15}]),
        Node(package="rviz2", executable="rviz2", name="slam_rviz", output="log",
             arguments=["-d", str(share / "config/slam.rviz")],
             parameters=[{"use_sim_time": True}],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ]
    if LaunchConfiguration("tour").perform(context).lower() == "true":
        # The tour ends the launch, so a scripted acceptance run terminates.
        actions += shutdown_after(Node(
            package="g1_guide", executable="slam_explore", name="nav_tour", output="screen",
            parameters=[{"scene_config": scene_config, "use_sim_time": True,
                         "order": LaunchConfiguration("order"),
                         "output_dir": output_dir, "save_map": False,
                         "global_frame": "map",
                         "report_name": LaunchConfiguration("report_name")}]))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory("g1_guide"))
    return LaunchDescription([
        DeclareLaunchArgument("scene_config", default_value=str(share / "config/exhibition_simple.json")),
        DeclareLaunchArgument("lidar", default_value="mid360",
                              description="mid360 = 真机雷达版 (default), simple = 简化雷达版"),
        DeclareLaunchArgument("odom_drift", default_value="true",
                              description="mid360 only: simulate leg-odometry drift"),
        DeclareLaunchArgument("map", default_value="",
                              description="SLAM map yaml; empty = <project>/maps/<scene stem>_map[_mid360].yaml"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("tour", default_value="false"),
        DeclareLaunchArgument("order", default_value=""),
        DeclareLaunchArgument("output_dir", default_value=""),
        DeclareLaunchArgument("report_name", default_value="nav_report"),
        OpaqueFunction(function=_launch),
    ])
