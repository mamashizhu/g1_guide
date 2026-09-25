"""Supported-base G1: real Gazebo effort dynamics, no glide controller."""
import importlib.util
import atexit
import json
from pathlib import Path
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context):
    from g1_guide.lowlevel_config import configuration

    share = Path(get_package_share_directory("g1_guide"))
    workspace = Path(LaunchConfiguration("g1_ws").perform(context)).resolve()
    source = workspace / "src/g1_description/g1_29dof_mode_11.urdf"
    temporary = tempfile.TemporaryDirectory(prefix="g1-lowlevel-")
    # launch exits only after supervising its children; retain the owner until
    # interpreter exit, then clean this exact generated directory.
    atexit.register(temporary.cleanup)
    output = Path(temporary.name)
    limits, config = configuration(source)
    controllers = output / "controllers.yaml"
    controllers.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
    limits_file = output / "limits.json"
    limits_file.write_text(json.dumps(limits), encoding="utf-8")
    spec = importlib.util.spec_from_file_location("controlled_generator", share / "scripts/make_controlled_urdf.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    model = output / "g1_controlled.urdf"
    module.generate(source, model, workspace, controllers, support_height=1.3)
    bridge = output / "clock.yaml"
    bridge.write_text(yaml.safe_dump([{
        "ros_topic_name": "/clock", "gz_topic_name": "/clock",
        "ros_type_name": "rosgraph_msgs/msg/Clock", "gz_type_name": "gz.msgs.Clock",
        "direction": "GZ_TO_ROS",
    }, {
        "ros_topic_name": "/lowlevel/overview", "gz_topic_name": "/lowlevel/overview",
        "ros_type_name": "sensor_msgs/msg/Image", "gz_type_name": "gz.msgs.Image",
        "direction": "GZ_TO_ROS",
    }]), encoding="utf-8")
    gazebo = ["gz", "sim", "-r", str(share / "worlds/lowlevel.sdf")]
    if LaunchConfiguration("headless").perform(context).lower() == "true":
        gazebo.append("-s")
    actions = [
        ExecuteProcess(cmd=gazebo, output="screen"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": model.read_text(encoding="utf-8"), "use_sim_time": True}], output="screen"),
        Node(package="ros_gz_sim", executable="create", arguments=[
            "-world", "lowlevel", "-name", "g1_controlled", "-file", str(model)], output="screen"),
        Node(package="g1_bridge", executable="managed_bridge", name="lowlevel_clock_bridge",
             parameters=[{"config_file": str(bridge)}], output="screen"),
        Node(package="controller_manager", executable="spawner", arguments=[
            "joint_state_broadcaster", "g1_pd", "--param-file", str(controllers),
            "--activate-as-group", "--controller-manager-timeout", "60"], output="screen"),
        Node(package="g1_guide", executable="g1_lowlevel_guard", parameters=[
            {"limits_file": str(limits_file), "use_sim_time": True}], output="screen"),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("g1_ws", default_value="/mnt/d/智能机器人/claude_course/ws"),
        DeclareLaunchArgument("headless", default_value="false"),
        OpaqueFunction(function=_launch),
    ])
