"""Exhibition Gazebo plus local closed-loop navigation; no hardware drivers."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("g1_guide"))
    scene_config = LaunchConfiguration("scene_config")
    headless = LaunchConfiguration("headless")
    return LaunchDescription([
        DeclareLaunchArgument("scene_config", default_value=str(share / "config/exhibition.json")),
        DeclareLaunchArgument("headless", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/exhibition.launch.py")),
            launch_arguments={"scene_config": scene_config, "headless": headless}.items(),
        ),
        Node(package="g1_guide", executable="exhibition_nav",
             name="exhibition_navigator", output="screen",
             parameters=[{"scene_config": scene_config, "use_sim_time": True}]),
        Node(package="g1_guide", executable="gesture_player",
             name="gesture_player", output="screen",
             parameters=[{"scene_config": scene_config, "use_sim_time": True}]),
    ])
