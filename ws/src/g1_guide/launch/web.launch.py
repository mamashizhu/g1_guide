"""网页展示控制台：rosbridge + demo_web（http://localhost:8080）。

可以和任何一个仿真 launch 一起用（导航、SLAM、低阶控制）：
  ros2 launch g1_guide web.launch.py
换场景时 scene_config 要和仿真用的一致（网页的 2D 地图按它画）。
完整演示一条命令见 demo.launch.py。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("g1_guide"))
    rosbridge = Path(get_package_share_directory("rosbridge_server")) / "launch/rosbridge_websocket_launch.xml"
    return LaunchDescription([
        DeclareLaunchArgument("scene_config", default_value=str(share / "config/exhibition.json")),
        DeclareLaunchArgument("web_port", default_value="8080", description="网页端口"),
        DeclareLaunchArgument("rosbridge", default_value="true",
                              description="false = 已经单独开了 rosbridge"),
        DeclareLaunchArgument("rosbridge_port", default_value="9090"),
        # scoped: rosbridge's own "port" argument must not leak into this launch.
        GroupAction(scoped=True, condition=IfCondition(LaunchConfiguration("rosbridge")), actions=[
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(str(rosbridge)),
                launch_arguments={"port": LaunchConfiguration("rosbridge_port")}.items()),
        ]),
        Node(package="g1_guide", executable="demo_web", name="demo_web", output="screen",
             parameters=[{"scene_config": LaunchConfiguration("scene_config"),
                          "port": LaunchConfiguration("web_port"),
                          "rosbridge_port": LaunchConfiguration("rosbridge_port"),
                          "use_sim_time": True}]),
    ])
