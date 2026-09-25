"""SLAM mapping in the exhibition: Gazebo + navigator + slam_toolbox (online async).

explore:=true  runs slam_explore, which tours the exhibit stations, saves the map
               and checks it against the scene layout.
rviz:=true     opens RViz with the map, scan and robot model.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("g1_guide"))
    slam_share = Path(get_package_share_directory("slam_toolbox"))
    scene_config = LaunchConfiguration("scene_config")
    return LaunchDescription([
        # Start with the simple 16 x 12 m box hall; pass scene_config for the imported hall.
        DeclareLaunchArgument("scene_config", default_value=str(share / "config/exhibition_simple.json")),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("explore", default_value="false"),
        DeclareLaunchArgument("order", default_value="",
                              description="Station keys to visit, e.g. '4 6 3 1 2 5'; empty = nearest first"),
        DeclareLaunchArgument("output_dir", default_value="",
                              description="Where to save the map; empty = <project>/maps"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/exhibition_nav.launch.py")),
            launch_arguments={"scene_config": scene_config,
                              "headless": LaunchConfiguration("headless")}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(slam_share / "launch/online_async_launch.py")),
            launch_arguments={"slam_params_file": str(share / "config/slam_toolbox.yaml"),
                              "use_sim_time": "true"}.items(),
        ),
        Node(package="rviz2", executable="rviz2", name="slam_rviz", output="log",
             arguments=["-d", str(share / "config/slam.rviz")],
             parameters=[{"use_sim_time": True}],
             condition=IfCondition(LaunchConfiguration("rviz"))),
        Node(package="g1_guide", executable="slam_explore", name="slam_explore", output="screen",
             parameters=[{"scene_config": scene_config, "use_sim_time": True,
                          "order": LaunchConfiguration("order"),
                          "output_dir": LaunchConfiguration("output_dir")}],
             condition=IfCondition(LaunchConfiguration("explore"))),
    ])
