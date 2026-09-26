"""Generate the configured museum and include the verified G1 simulation."""
import json
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context):
    from g1_guide.exhibition_world import generate_world
    from g1_guide.scene_config import load_scene

    share = Path(get_package_share_directory("g1_guide"))
    config_path = Path(LaunchConfiguration("scene_config").perform(context))
    config = load_scene(config_path)
    # Each launch owns an isolated generated world; never overwrite user assets.
    output_dir = Path(tempfile.mkdtemp(prefix="g1-exhibition-"))
    world = output_dir / "exhibition.sdf"
    world.write_text(generate_world(config, share / "assets/exhibition"), encoding="utf-8")
    bridge_config = output_dir / "overview_bridge.yaml"
    bridge_config.write_text(json.dumps([{
        "ros_topic_name": "/exhibition/overview", "gz_topic_name": "/exhibition/overview",
        "ros_type_name": "sensor_msgs/msg/Image", "gz_type_name": "gz.msgs.Image",
        "direction": "GZ_TO_ROS", "frame_id": "exhibition_overview",
    }]), encoding="utf-8")
    spawn = config["spawn"]
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch/g1_sim.launch.py")),
        launch_arguments={
            "world": str(world),
            "headless": LaunchConfiguration("headless").perform(context),
            "spawn_x": str(spawn["x"]), "spawn_y": str(spawn["y"]),
            "spawn_z": str(spawn["z"]), "spawn_yaw": str(spawn["yaw"]),
        }.items(),
    ), Node(package="g1_bridge", executable="managed_bridge", name="exhibition_overview_bridge",
            parameters=[{"config_file": str(bridge_config)}], output="screen")]


def generate_launch_description():
    share = Path(get_package_share_directory("g1_guide"))
    return LaunchDescription([
        DeclareLaunchArgument("scene_config", default_value=str(share / "config/exhibition.json")),
        DeclareLaunchArgument("headless", default_value="false"),
        OpaqueFunction(function=_launch),
    ])
