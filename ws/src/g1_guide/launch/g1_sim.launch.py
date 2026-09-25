#!/usr/bin/env python3
"""Launch the classroom world, generated G1 model, and Gazebo/ROS bridges."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _validate_model(context, *args, **kwargs):
    package_share = Path(get_package_share_directory("g1_guide"))
    model_file = Path(LaunchConfiguration("model_file").perform(context))
    if not model_file.is_absolute():
        model_file = package_share / model_file
    if not model_file.is_file():
        raise RuntimeError(
            "Generated G1 model is missing: "
            f"{model_file}. Run scripts/make_glide_urdf.py after installing "
            "the official Unitree g1_description source."
        )
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {
                    "robot_description": model_file.read_text(encoding="utf-8"),
                    "use_sim_time": True,
                }
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory("g1_guide"))
    default_world = str(package_share / "worlds" / "classroom.sdf")
    default_bridge = str(package_share / "config" / "bridge.yaml")
    default_model = str(package_share / "urdf" / "g1_glide.urdf")

    world = LaunchConfiguration("world")
    model_file = LaunchConfiguration("model_file")
    headless = LaunchConfiguration("headless")

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("bridge_config", default_value=default_bridge),
            DeclareLaunchArgument("model_file", default_value=default_model),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "spawn_z", default_value="0.847",
                # Official zero-pose foot collision minimum is -0.791864 m
                # below pelvis. Classroom floor top is 0.05 m; leave ~5 mm.
                description="Pelvis height: default fits the official zero-pose G1 and classroom floor; override for other models/worlds.",
            ),
            OpaqueFunction(function=_validate_model),
            ExecuteProcess(
                cmd=["gz", "sim", "-r", world],
                output="screen",
                condition=UnlessCondition(headless),
            ),
            ExecuteProcess(
                cmd=["gz", "sim", "-r", "-s", world],
                output="screen",
                condition=IfCondition(headless),
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-name", "g1_glide", "-file", model_file,
                    "-z", LaunchConfiguration("spawn_z"),
                    "-x", LaunchConfiguration("spawn_x"),
                    "-y", LaunchConfiguration("spawn_y"),
                    "-Y", LaunchConfiguration("spawn_yaw"),
                ],
                output="screen",
            ),
            Node(
                package="g1_bridge",
                executable="managed_bridge",
                parameters=[{"config_file": LaunchConfiguration("bridge_config")}],
                output="screen",
            ),
        ]
    )
