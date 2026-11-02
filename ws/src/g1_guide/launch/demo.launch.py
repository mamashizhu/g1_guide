"""一条命令开完整演示（WSL 端）：展馆 + 导航 + 真机雷达人群感知 + 前置相机 + 网页控制台。

  ros2 launch g1_guide demo.launch.py
然后 Windows 端双击 demo.cmd（视觉 + 语音 + 打开网页），或浏览器打开 http://localhost:8080。

参数和 exhibition_nav.launch.py 一样，只是默认 lidar:=mid360 vision:=true；
headless:=true 不开 Gazebo 窗口（网页里的跟拍画面照样有，内存泄漏少很多）。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("g1_guide"))
    names = ("scene_config", "headless", "gui_render", "camera_rate", "lidar", "vision", "vision_rate")
    return LaunchDescription([
        DeclareLaunchArgument("scene_config", default_value=str(share / "config/exhibition.json")),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("gui_render", default_value="gpu"),
        DeclareLaunchArgument("camera_rate", default_value="10"),
        DeclareLaunchArgument("lidar", default_value="mid360"),
        DeclareLaunchArgument("vision", default_value="true"),
        DeclareLaunchArgument("vision_rate", default_value="5"),
        DeclareLaunchArgument("web_port", default_value="8080"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/exhibition_nav.launch.py")),
            launch_arguments={n: LaunchConfiguration(n) for n in names}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/web.launch.py")),
            launch_arguments={"scene_config": LaunchConfiguration("scene_config"),
                              "web_port": LaunchConfiguration("web_port")}.items()),
    ])
