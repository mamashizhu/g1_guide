"""ROS-independent regression tests for the generated glide model."""

from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
GENERATOR_PATH = PACKAGE_ROOT / "scripts" / "make_glide_urdf.py"
SPEC = importlib.util.spec_from_file_location("make_glide_urdf", GENERATOR_PATH)
if SPEC is None or SPEC.loader is None:  # pragma: no cover - import setup failure
    raise RuntimeError(f"Cannot import generator from {GENERATOR_PATH}")
GENERATOR = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = GENERATOR
SPEC.loader.exec_module(GENERATOR)


SOURCE_URDF = """\
<robot name="fixture">
  <link name="pelvis">
    <visual><geometry><mesh filename="package://g1_description/meshes/body.STL" /></geometry></visual>
  </link>
  <link name="torso_link" />
  <joint name="torso_joint" type="revolute">
    <origin xyz="0 0 0.4" rpy="0 0 0" />
    <parent link="pelvis" />
    <child link="torso_link" />
    <axis xyz="0 0 1" />
  </joint>
  <link name="d435_link" />
  <joint name="d435_joint" type="fixed">
    <origin xyz="0.0576235 0.01753 0.42987" rpy="0 0.8307767239493009 0" />
    <parent link="torso_link" />
    <child link="d435_link" />
  </joint>
  <link name="imu_in_torso" />
  <joint name="imu_in_torso_joint" type="fixed">
    <origin xyz="-0.03959 -0.00224 0.14792" rpy="0 0 0" />
    <parent link="torso_link" />
    <child link="imu_in_torso" />
  </joint>
  <link name="mid360_link" />
  <joint name="mid360_joint" type="fixed">
    <origin xyz="0.0002835 0.00003 0.428434" rpy="3.141592653589793 0.05112069379091391 0" />
    <parent link="torso_link" />
    <child link="mid360_link" />
  </joint>
</robot>
"""


def _write_source(workspace: Path, text: str = SOURCE_URDF) -> Path:
    source = workspace / "src" / "g1_description" / "fixture.urdf"
    source.parent.mkdir(parents=True)
    source.write_text(text, encoding="utf-8")
    return source


def _root_links(root: ET.Element) -> list[str]:
    links = {link.get("name") for link in root.findall("link")}
    children = {
        child.get("link")
        for joint in root.findall("joint")
        for child in joint.findall("child")
    }
    return sorted(link for link in links if link and link not in children)


def _joint_link(joint: ET.Element, tag: str) -> str:
    element = joint.find(tag)
    assert element is not None
    link = element.get("link")
    assert link is not None
    return link


class ModelGenerationTest(unittest.TestCase):
    def test_generate_rewrites_mesh_and_promotes_base_link(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory) / "g1_ws"
            source = _write_source(workspace)
            output = Path(directory) / "generated.urdf"

            GENERATOR.generate(source, output, workspace)
            root = ET.parse(output).getroot()

            self.assertEqual(_root_links(root), ["base_link"])
            base_joint = root.find("./joint[@name='base_link_to_pelvis']")
            self.assertIsNotNone(base_joint)
            assert base_joint is not None
            self.assertEqual(base_joint.get("type"), "fixed")
            self.assertEqual(_joint_link(base_joint, "parent"), "base_link")
            self.assertEqual(_joint_link(base_joint, "child"), "pelvis")
            self.assertEqual(base_joint.find("origin").get("xyz"), "0 0 0")

            mesh = root.find(".//mesh")
            self.assertIsNotNone(mesh)
            assert mesh is not None
            self.assertEqual(
                Path(mesh.get("filename", "")),
                workspace / "src" / "g1_description" / "meshes" / "body.STL",
            )
            self.assertNotIn("package://", mesh.get("filename", ""))

    def test_camera_frames_and_official_pitch_are_preserved(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            directory_path = Path(directory)
            workspace = directory_path / "g1_ws"
            source = _write_source(workspace)
            output = directory_path / "generated.urdf"

            GENERATOR.generate(source, output, workspace)
            root = ET.parse(output).getroot()

            camera_joint = root.find("./joint[@name='d435_joint']")
            self.assertIsNotNone(camera_joint)
            assert camera_joint is not None
            self.assertEqual(
                camera_joint.find("origin").get("rpy"),
                "0 0.8307767239493009 0",
            )
            optical_joint = root.find("./joint[@name='d435_optical_joint']")
            self.assertIsNotNone(optical_joint)
            assert optical_joint is not None
            self.assertEqual(_joint_link(optical_joint, "parent"), "d435_link")
            self.assertEqual(
                _joint_link(optical_joint, "child"), "d435_optical_frame"
            )

            sensor = root.find("./gazebo[@reference='d435_link']/sensor")
            self.assertIsNotNone(sensor)
            assert sensor is not None
            self.assertEqual(sensor.get("type"), "rgbd")
            self.assertEqual(sensor.findtext("gz_frame_id"), "d435_optical_frame")
            self.assertEqual(sensor.findtext("frame_id"), "d435_optical_frame")
            camera = sensor.find("camera")
            self.assertIsNotNone(camera)
            assert camera is not None
            depth_camera = camera.find("depth_camera")
            self.assertIsNotNone(depth_camera)
            assert depth_camera is not None
            self.assertEqual(depth_camera.findtext("clip/near"), "0.10")
            self.assertEqual(depth_camera.findtext("clip/far"), "10.0")
            self.assertIsNone(sensor.find("depth_camera"))

    def test_odometry_uses_connected_odom_and_base_frames(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            directory_path = Path(directory)
            workspace = directory_path / "g1_ws"
            source = _write_source(workspace)
            output = directory_path / "generated.urdf"

            GENERATOR.generate(source, output, workspace)
            root = ET.parse(output).getroot()
            plugin = root.find(
                "./gazebo/plugin[@name='gz::sim::systems::OdometryPublisher']"
            )
            self.assertIsNotNone(plugin)
            assert plugin is not None
            self.assertEqual(plugin.findtext("odom_frame"), "odom")
            self.assertEqual(plugin.findtext("robot_base_frame"), "base_link")
            self.assertEqual(plugin.findtext("dimensions"), "3")
            self.assertEqual(plugin.findtext("publish_tf"), "true")

            # Every frame used by the odometry and camera paths is in the same
            # fixed-joint graph; this catches disconnected frame-only links.
            edges = {
                (_joint_link(joint, "parent"), _joint_link(joint, "child"))
                for joint in root.findall("joint")
            }
            self.assertIn(("base_link", "pelvis"), edges)
            self.assertIn(("d435_link", "d435_optical_frame"), edges)

    def test_lidar_and_imu_xml_semantics_and_fixed_mount_frames(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            directory_path = Path(directory)
            workspace = directory_path / "g1_ws"
            source = _write_source(workspace)
            output = directory_path / "generated.urdf"

            GENERATOR.generate(source, output, workspace)
            root = ET.parse(output).getroot()
            edges = {
                (_joint_link(joint, "parent"), _joint_link(joint, "child"))
                for joint in root.findall("joint")
            }
            self.assertIn(("torso_link", "mid360_link"), edges)
            self.assertIn(("torso_link", "imu_in_torso"), edges)
            self.assertIn(("base_link", "nav_lidar_link"), edges)

            nav_joint = root.find("./joint[@name='nav_lidar_joint']")
            self.assertIsNotNone(nav_joint)
            self.assertEqual(nav_joint.find("origin").get("rpy"), "0 0 0")
            nav_lidar = root.find("./gazebo[@reference='nav_lidar_link']/sensor")
            self.assertIsNotNone(nav_lidar)
            self.assertEqual(nav_lidar.get("type"), "gpu_lidar")
            self.assertEqual(nav_lidar.findtext("topic"), "/nav_scan")
            self.assertEqual(nav_lidar.findtext("gz_frame_id"), "nav_lidar_link")
            self.assertEqual(nav_lidar.findtext("update_rate"), "5")

            lidar_joint = root.find("./joint[@name='mid360_joint']")
            self.assertIsNotNone(lidar_joint)
            assert lidar_joint is not None
            self.assertEqual(lidar_joint.get("type"), "fixed")
            self.assertEqual(
                lidar_joint.find("origin").get("xyz"),
                "0.0002835 0.00003 0.428434",
            )
            lidar = root.find("./gazebo[@reference='mid360_link']/sensor")
            self.assertIsNotNone(lidar)
            assert lidar is not None
            self.assertEqual(lidar.get("type"), "gpu_lidar")
            self.assertEqual(lidar.findtext("topic"), "/scan")
            self.assertEqual(lidar.findtext("gz_frame_id"), "mid360_link")
            self.assertEqual(lidar.findtext("update_rate"), "10")
            self.assertEqual(lidar.findtext("lidar/scan/horizontal/samples"), "360")
            self.assertEqual(lidar.findtext("lidar/scan/horizontal/resolution"), "1")
            self.assertEqual(lidar.findtext("lidar/range/min"), "0.12")
            self.assertEqual(lidar.findtext("lidar/range/max"), "8.0")

            imu_joint = root.find("./joint[@name='imu_in_torso_joint']")
            self.assertIsNotNone(imu_joint)
            assert imu_joint is not None
            self.assertEqual(imu_joint.get("type"), "fixed")
            self.assertEqual(
                imu_joint.find("origin").get("xyz"), "-0.03959 -0.00224 0.14792"
            )
            imu = root.find("./gazebo[@reference='imu_in_torso']/sensor")
            self.assertIsNotNone(imu)
            assert imu is not None
            self.assertEqual(imu.get("type"), "imu")
            self.assertEqual(imu.findtext("topic"), "/imu")
            self.assertEqual(imu.findtext("gz_frame_id"), "imu_in_torso")
            self.assertEqual(imu.findtext("update_rate"), "50")
            self.assertIsNotNone(imu.find("imu"))

    def test_bridge_has_lidar_and_imu_message_types_and_frames(self) -> None:
        bridge_config = PACKAGE_ROOT / "config" / "bridge.yaml"
        entries = yaml.safe_load(bridge_config.read_text(encoding="utf-8"))
        by_ros_topic = {entry["ros_topic_name"]: entry for entry in entries}

        self.assertEqual(
            by_ros_topic["/scan"],
            {
                "ros_topic_name": "/scan",
                "gz_topic_name": "/scan",
                "ros_type_name": "sensor_msgs/msg/LaserScan",
                "gz_type_name": "gz.msgs.LaserScan",
                "direction": "GZ_TO_ROS",
                "frame_id": "mid360_link",
            },
        )
        self.assertEqual(
            by_ros_topic["/nav_scan"],
            {
                "ros_topic_name": "/nav_scan",
                "gz_topic_name": "/nav_scan",
                "ros_type_name": "sensor_msgs/msg/LaserScan",
                "gz_type_name": "gz.msgs.LaserScan",
                "direction": "GZ_TO_ROS",
                "frame_id": "nav_lidar_link",
            },
        )
        self.assertEqual(
            by_ros_topic["/imu"],
            {
                "ros_topic_name": "/imu",
                "gz_topic_name": "/imu",
                "ros_type_name": "sensor_msgs/msg/Imu",
                "gz_type_name": "gz.msgs.IMU",
                "direction": "GZ_TO_ROS",
                "frame_id": "imu_in_torso",
            },
        )
        self.assertEqual(len(by_ros_topic), len(entries))

    def test_base_link_gravity_disabled(self) -> None:
        """VelocityControl is kinematic; gravity must be off so the robot does not fall."""
        with tempfile.TemporaryDirectory() as directory:
            directory_path = Path(directory)
            workspace = directory_path / "g1_ws"
            source = _write_source(workspace)
            output = directory_path / "generated.urdf"

            GENERATOR.generate(source, output, workspace)
            root = ET.parse(output).getroot()

            gravity_tag = root.find("./gazebo[@reference='base_link']/gravity")
            self.assertIsNotNone(
                gravity_tag,
                "Missing <gazebo reference='base_link'><gravity> — robot will fall under physics",
            )
            assert gravity_tag is not None
            self.assertEqual(
                gravity_tag.text,
                "false",
                "<gravity> must be 'false' to prevent VelocityControl model from falling",
            )
            self.assertEqual(GENERATOR._root_links(root), ["base_link"])
            self.assertTrue(
                all(joint.get("type") == "fixed" for joint in root.findall("joint"))
            )
            for gazebo in root.findall("gazebo"):
                if gazebo.get("reference") is None:
                    self.assertIsNone(gazebo.find("gravity"))

    def test_invalid_multiple_root_input_is_rejected(self) -> None:
        invalid = """\
<robot name="invalid">
  <link name="pelvis" />
  <link name="detached" />
</robot>
"""
        with tempfile.TemporaryDirectory() as directory:
            directory_path = Path(directory)
            workspace = directory_path / "g1_ws"
            source = _write_source(workspace, invalid)

            with self.assertRaisesRegex(ValueError, "exactly one root"):
                GENERATOR.generate(source, directory_path / "generated.urdf", workspace)

    def test_invalid_nonroot_base_link_input_is_rejected(self) -> None:
        invalid = """\
<robot name="invalid">
  <link name="pelvis" />
  <link name="base_link" />
  <joint name="bad_base" type="fixed">
    <parent link="pelvis" />
    <child link="base_link" />
  </joint>
</robot>
"""
        with tempfile.TemporaryDirectory() as directory:
            directory_path = Path(directory)
            workspace = directory_path / "g1_ws"
            source = _write_source(workspace, invalid)

            with self.assertRaisesRegex(ValueError, "base_link.*not the root"):
                GENERATOR.generate(source, directory_path / "generated.urdf", workspace)


if __name__ == "__main__":
    unittest.main()
