"""Exhibition Gazebo plus local closed-loop navigation; no hardware drivers.

``lidar:=simple`` (default) is the original, user-verified setup.
``lidar:=mid360`` swaps in the real G1's 3D lidar and adds ``people_tracker``
(/guide/people, PLAN M5.2), which needs the 3D cloud.
``vision:=true`` adds ``camera_jpeg`` (front USB camera -> JPEG for the
Windows vision process); off by default so the camera is not rendered.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context):
    from g1_guide.lidar_variants import lidar_actions

    share = Path(get_package_share_directory("g1_guide"))
    scene_config = LaunchConfiguration("scene_config")
    lidar = LaunchConfiguration("lidar").perform(context)
    actions = lidar_actions(share, lidar, scene_config, drift=False)
    actions += [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/exhibition.launch.py")),
            launch_arguments={"scene_config": scene_config,
                              "headless": LaunchConfiguration("headless"),
                              "gui_render": LaunchConfiguration("gui_render"),
                              "camera_rate": LaunchConfiguration("camera_rate")}.items(),
        ),
        Node(package="g1_guide", executable="exhibition_nav",
             name="exhibition_navigator", output="screen",
             parameters=[{"scene_config": scene_config, "use_sim_time": True}]),
        Node(package="g1_guide", executable="gesture_player",
             name="gesture_player", output="screen",
             parameters=[{"scene_config": scene_config, "use_sim_time": True}]),
    ]
    if lidar == "mid360":
        actions.append(Node(package="g1_guide", executable="people_tracker",
                            name="people_tracker", output="screen",
                            parameters=[{"scene_config": scene_config, "use_sim_time": True}]))
    if LaunchConfiguration("vision").perform(context).lower() == "true":
        # Front USB camera as 5 Hz JPEG for the Windows vision process (PLAN M5).
        actions.append(Node(package="g1_guide", executable="camera_jpeg", name="camera_jpeg",
                            output="screen", parameters=[{"use_sim_time": True}]))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory("g1_guide"))
    return LaunchDescription([
        DeclareLaunchArgument("scene_config", default_value=str(share / "config/exhibition.json")),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("gui_render", default_value="gpu",
                              description="gpu = GPU GUI (default), software = llvmpipe GUI (no leak, low fps)"),
        DeclareLaunchArgument("camera_rate", default_value="10",
                              description="Head RGBD camera Hz; rendered only while subscribed"),
        DeclareLaunchArgument("lidar", default_value="simple",
                              description="simple = 简化雷达版 (default), mid360 = 真机雷达版 + 人群感知"),
        DeclareLaunchArgument("vision", default_value="false",
                              description="true = publish the front camera as JPEG for vision/watch.py"),
        OpaqueFunction(function=_launch),
    ])
