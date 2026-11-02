#!/usr/bin/env python3
"""Launch the classroom world, generated G1 model, and Gazebo/ROS bridges.

``gui_render:=gpu`` (default) is the original single ``gz sim`` process with
the GPU GUI.  ``gui_render:=software`` runs a GPU server plus a separate
llvmpipe GUI: no WSL d3d12 leak in the GUI (CLAUDE.md), but a low frame rate
in the imported hall.
``camera_rate`` sets the head RGBD camera rate (default 10 Hz, model 30 Hz).
``chase_camera:=true`` (default) adds the third-person camera used by the web
console (/chase_camera/image); it renders only while someone subscribes.
"""

from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _validate_model(context, *args, **kwargs):
    from g1_guide.lidar_variants import with_camera_rate, with_chase_camera

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
    urdf = model_file.read_text(encoding="utf-8")
    spawned = Path(tempfile.mkdtemp(prefix="g1-model-")) / model_file.name
    spawn_urdf = with_camera_rate(urdf, LaunchConfiguration("camera_rate").perform(context))
    if LaunchConfiguration("chase_camera").perform(context).lower() == "true":
        spawn_urdf = with_chase_camera(spawn_urdf, LaunchConfiguration("chase_pose").perform(context))
    spawned.write_text(spawn_urdf, encoding="utf-8")
    return [
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name", "g1_glide", "-file", str(spawned),
                "-z", LaunchConfiguration("spawn_z"),
                "-x", LaunchConfiguration("spawn_x"),
                "-y", LaunchConfiguration("spawn_y"),
                "-Y", LaunchConfiguration("spawn_yaw"),
            ],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {
                    "robot_description": urdf,
                    "use_sim_time": True,
                }
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    from g1_guide.lidar_variants import CHASE_CAMERA_POSE

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
            DeclareLaunchArgument("gui_render", default_value="gpu",
                                  description="gpu = GPU GUI (default), software = llvmpipe GUI (no leak, low fps)"),
            DeclareLaunchArgument("camera_rate", default_value="10",
                                  description="Head RGBD camera Hz; rendered only while subscribed"),
            DeclareLaunchArgument("chase_camera", default_value="true",
                                  description="Third-person camera for the web console (rendered only while viewed)"),
            DeclareLaunchArgument("chase_pose", default_value=CHASE_CAMERA_POSE,
                                  description="Chase camera pose relative to the pelvis: x y z roll pitch yaw"),
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
                condition=IfCondition(PythonExpression(
                    ["'", headless, "' != 'true' and '", LaunchConfiguration("gui_render"), "' != 'software'"])),
            ),
            ExecuteProcess(
                cmd=["gz", "sim", "-r", "-s", world],
                output="screen",
                condition=IfCondition(PythonExpression(
                    ["'", headless, "' == 'true' or '", LaunchConfiguration("gui_render"), "' == 'software'"])),
            ),
            ExecuteProcess(
                cmd=["gz", "sim", "-g"],
                output="screen",
                additional_env={"LIBGL_ALWAYS_SOFTWARE": "1", "GALLIUM_DRIVER": "llvmpipe"},
                condition=IfCondition(PythonExpression(
                    ["'", headless, "' != 'true' and '", LaunchConfiguration("gui_render"), "' == 'software'"])),
            ),
            Node(
                package="g1_bridge",
                executable="managed_bridge",
                parameters=[{"config_file": LaunchConfiguration("bridge_config")}],
                output="screen",
            ),
        ]
    )
