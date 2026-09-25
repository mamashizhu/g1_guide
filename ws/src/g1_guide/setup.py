from glob import glob
import os

from setuptools import setup


package_name = "g1_guide"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config"), glob("config/*.json")),
        (os.path.join("share", package_name, "config"), glob("config/*.rviz")),
        (os.path.join("share", package_name, "assets", "exhibition", "posters"),
         glob("assets/exhibition/posters/*.png")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="G1 Course Maintainers",
    maintainer_email="course@example.invalid",
    description="G1 exhibition guide robot: museum simulation, navigation, low-level control, sensors.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "g1_teleop = g1_guide.g1_teleop:main",
            "g1_camera_viewer = g1_guide.g1_camera_viewer:main",
            "g1_lowlevel_guard = g1_guide.lowlevel_guard:main",
            "sensor_dashboard = g1_guide.sensor_dashboard:main",
            "exhibition_nav = g1_guide.exhibition_nav:main",
            "nav_keyboard = g1_guide.nav_keyboard:main",
            "slam_explore = g1_guide.slam_explore:main",
            "slam_map_check = g1_guide.slam_map:main",
            "mid360_to_scan = g1_guide.mid360_to_scan:main",
            "odom_drift = g1_guide.odom_drift:main",
        ],
    },
)
