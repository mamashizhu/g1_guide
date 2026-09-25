"""SLAM mapping in the exhibition: Gazebo + navigator + slam_toolbox (online async).

lidar:=simple  (default) 简化雷达版: level single-plane /nav_scan, 8 m.
lidar:=mid360  真机雷达版: MID-360 as a 3D lidar with the real field of view;
               mid360_to_scan flattens its cloud into /nav_scan, so low
               furniture is mapped and avoided.
explore:=true  runs slam_explore, which tours the exhibit stations, saves the map
               and checks it against the scene layout.
rviz:=true     opens RViz with the map, scan and robot model.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context):
    share = Path(get_package_share_directory("g1_guide"))
    slam_share = Path(get_package_share_directory("slam_toolbox"))
    scene_config = LaunchConfiguration("scene_config")
    lidar = LaunchConfiguration("lidar").perform(context)
    if lidar not in ("simple", "mid360"):
        raise RuntimeError(f"lidar must be simple or mid360, got {lidar!r}")
    real = lidar == "mid360"
    map_name = LaunchConfiguration("map_name").perform(context) or (
        "exhibition_map_mid360" if real else "exhibition_map")
    actions = []
    if real:
        # g1_sim.launch.py (included below) reads model_file from the context.
        actions += [
            SetLaunchConfiguration("model_file", str(share / "urdf/g1_glide_mid360.urdf")),
            Node(package="g1_bridge", executable="managed_bridge", name="mid360_bridge",
                 parameters=[{"config_file": str(share / "config/bridge_mid360.yaml")}],
                 output="screen"),
            Node(package="g1_guide", executable="mid360_to_scan", name="mid360_to_scan",
                 output="screen",
                 parameters=[{"scene_config": scene_config, "use_sim_time": True}]),
        ]
    actions += [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/exhibition_nav.launch.py")),
            launch_arguments={"scene_config": scene_config,
                              "headless": LaunchConfiguration("headless")}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(slam_share / "launch/online_async_launch.py")),
            launch_arguments={
                "slam_params_file": str(share / "config" / (
                    "slam_toolbox_mid360.yaml" if real else "slam_toolbox.yaml")),
                "use_sim_time": "true"}.items(),
        ),
        Node(package="rviz2", executable="rviz2", name="slam_rviz", output="log",
             arguments=["-d", str(share / "config/slam.rviz")],
             parameters=[{"use_sim_time": True}],
             condition=IfCondition(LaunchConfiguration("rviz"))),
        Node(package="g1_guide", executable="slam_explore", name="slam_explore", output="screen",
             parameters=[{"scene_config": scene_config, "use_sim_time": True,
                          "order": LaunchConfiguration("order"),
                          "output_dir": LaunchConfiguration("output_dir"),
                          "map_name": map_name}],
             condition=IfCondition(LaunchConfiguration("explore"))),
    ]
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory("g1_guide"))
    return LaunchDescription([
        # Start with the simple 16 x 12 m box hall; pass scene_config for the imported hall.
        DeclareLaunchArgument("scene_config", default_value=str(share / "config/exhibition_simple.json")),
        DeclareLaunchArgument("lidar", default_value="simple",
                              description="simple = 简化雷达版, mid360 = 真机雷达版"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("explore", default_value="false"),
        DeclareLaunchArgument("order", default_value="",
                              description="Station keys to visit, e.g. '4 6 3 1 2 5'; empty = nearest first"),
        DeclareLaunchArgument("output_dir", default_value="",
                              description="Where to save the map; empty = <project>/maps"),
        DeclareLaunchArgument("map_name", default_value="",
                              description="Map file name; empty = exhibition_map[_mid360]"),
        OpaqueFunction(function=_launch),
    ])
