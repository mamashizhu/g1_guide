"""Standard-library tests for the supported effort-controlled G1 model."""

from __future__ import annotations

import copy
import importlib.util
import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest import mock


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
G1_WS = PACKAGE_ROOT.parents[1]
OFFICIAL_URDF = G1_WS / "src" / "g1_description" / "g1_29dof_mode_11.urdf"
GLIDE_SCRIPT = PACKAGE_ROOT / "scripts" / "make_glide_urdf.py"
MODULE_PATH = PACKAGE_ROOT / "scripts" / "make_controlled_urdf.py"
SPEC = importlib.util.spec_from_file_location("make_controlled_urdf", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
controlled = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(controlled)


class ControlledModelTests(unittest.TestCase):
    def setUp(self) -> None:
        self.controllers = (PACKAGE_ROOT / "config" / "test-controllers.yaml").resolve()
        self.output = (PACKAGE_ROOT / "test" / "captured-output.urdf").resolve()
        self.captured: list[ET.Element] = []
        self.original_is_file = Path.is_file
        self.original_tree_write = ET.ElementTree.write

    def _capture_write(self, tree: ET.ElementTree, *_args, **_kwargs) -> None:
        import io

        xml_stream = io.BytesIO()
        self.original_tree_write(tree, xml_stream, encoding="utf-8", xml_declaration=True)
        xml_bytes = xml_stream.getvalue()
        self.captured.append(ET.fromstring(xml_bytes))

    def _file_exists(self, path: Path) -> bool:
        if path.resolve() == self.controllers:
            return True
        return self.original_is_file(path)

    def generate(self, root: ET.Element | None = None, **kwargs) -> ET.Element:
        source = OFFICIAL_URDF
        with (
            mock.patch.object(
                ET.ElementTree,
                "write",
                lambda tree, *args, **options: self._capture_write(tree, *args, **options),
            ),
            mock.patch.object(Path, "is_file", lambda path: self._file_exists(path)),
            mock.patch.object(
                controlled.ET,
                "parse",
                return_value=ET.ElementTree(root) if root is not None else ET.parse(source),
            ),
        ):
            controlled.generate(source, self.output, G1_WS, self.controllers, **kwargs)
        self.assertEqual(len(self.captured), 1)
        return self.captured.pop()

    def test_official_model_preserves_29_movable_joints_limits_and_inertias(self) -> None:
        root = self.generate()
        source = ET.parse(OFFICIAL_URDF).getroot()
        actual = {joint.get("name"): joint for joint in root.findall("joint") if joint.get("type") == "revolute"}
        official = {joint.get("name"): joint for joint in source.findall("joint") if joint.get("type") == "revolute"}
        self.assertEqual(len(actual), 29)
        self.assertEqual(actual.keys(), official.keys())
        for name, original_joint in official.items():
            self.assertEqual(actual[name].find("limit").attrib, original_joint.find("limit").attrib)
        original_inertias = {
            link.get("name"): ET.tostring(link.find("inertial"), encoding="unicode")
            for link in source.findall("link")
            if link.find("inertial") is not None
        }
        output_inertias = {
            link.get("name"): ET.tostring(link.find("inertial"), encoding="unicode")
            for link in root.findall("link")
            if link.find("inertial") is not None
        }
        self.assertEqual(output_inertias, original_inertias)
        self.assertEqual(len(root.findall(".//collision")), len(source.findall(".//collision")))

    def test_supported_base_has_movable_effort_joints_and_no_static_or_glide_plugins(self) -> None:
        root = self.generate()
        world_joint = root.find("joint[@name='world_to_pelvis_support']")
        self.assertEqual(world_joint.get("type"), "fixed")
        self.assertEqual(world_joint.find("parent").get("link"), "world")
        self.assertEqual(world_joint.find("child").get("link"), "pelvis")
        self.assertEqual(world_joint.find("origin").get("xyz"), "0 0 1.3")
        self.assertIsNone(root.find("static"))

        control = root.find("ros2_control")
        self.assertEqual(control.get("name"), "GazeboSimSystem")
        self.assertEqual(control.findtext("hardware/plugin"), "gz_ros2_control/GazeboSimSystem")
        joints = control.findall("joint")
        self.assertEqual(len(joints), 29)
        source_joints = {joint.get("name"): joint for joint in root.findall("joint[@type='revolute']")}
        for joint in joints:
            effort = float(source_joints[joint.get("name")].find("limit").get("effort"))
            command = joint.find("command_interface[@name='effort']")
            self.assertAlmostEqual(float(command.find("param[@name='min']").text), -effort)
            self.assertAlmostEqual(float(command.find("param[@name='max']").text), effort)
            self.assertEqual(joint.findtext("state_interface[@name='position']/param[@name='initial_value']"), "0")
            self.assertIsNotNone(joint.find("state_interface[@name='velocity']"))
            self.assertIsNotNone(joint.find("state_interface[@name='effort']"))

        plugins = root.findall("gazebo/plugin")
        self.assertEqual(len(plugins), 1)
        plugin = plugins[0]
        self.assertEqual(plugin.get("filename"), "libgz_ros2_control-system.so")
        self.assertEqual(plugin.get("name"), "gz_ros2_control::GazeboSimROS2ControlPlugin")
        self.assertEqual(Path(plugin.findtext("parameters")).resolve(), self.controllers)
        all_plugins = root.findall(".//plugin")
        self.assertFalse(any("VelocityControl" in (item.get("name") or "") for item in all_plugins))
        self.assertFalse(any("Odometry" in (item.get("name") or "") for item in all_plugins))

    def test_support_height_is_configurable(self) -> None:
        root = self.generate(support_height=2.25)
        self.assertEqual(
            root.find("joint[@name='world_to_pelvis_support']/origin").get("xyz"),
            "0 0 2.25",
        )

    def test_meshes_are_absolute_existing_paths(self) -> None:
        root = self.generate()
        meshes = root.findall(".//mesh")
        self.assertGreater(len(meshes), 0)
        for mesh in meshes:
            path = Path(mesh.get("filename"))
            self.assertTrue(path.is_absolute())
            self.assertTrue(self.original_is_file(path), str(path))

    def test_controlled_generation_does_not_modify_f01_glide_script(self) -> None:
        before = GLIDE_SCRIPT.read_bytes()
        self.generate()
        self.assertEqual(GLIDE_SCRIPT.read_bytes(), before)

    def test_rejects_same_input_and_output(self) -> None:
        with self.assertRaisesRegex(ValueError, "different files"):
            controlled.generate(OFFICIAL_URDF, OFFICIAL_URDF, G1_WS, self.controllers)

    def test_rejects_non_absolute_or_missing_controller_parameters(self) -> None:
        with self.assertRaisesRegex(ValueError, "must be absolute"):
            controlled.generate(OFFICIAL_URDF, self.output, G1_WS, Path("controllers.yaml"))
        with self.assertRaisesRegex(FileNotFoundError, "Controller parameters"):
            controlled.generate(OFFICIAL_URDF, self.output, G1_WS, PACKAGE_ROOT / "absent.yaml")

    def test_rejects_non_finite_support_height(self) -> None:
        with self.assertRaisesRegex(ValueError, "finite, non-negative"):
            controlled.generate(OFFICIAL_URDF, self.output, G1_WS, self.controllers, math.inf)

    def test_rejects_invalid_limits_before_writing(self) -> None:
        for attribute, value, message in (
            ("effort", "nan", "finite"),
            ("lower", "3", "lower limit"),
            ("velocity", "0", "velocity limits"),
        ):
            with self.subTest(attribute=attribute):
                root = copy.deepcopy(ET.parse(OFFICIAL_URDF).getroot())
                root.find("joint[@type='revolute']/limit").set(attribute, value)
                with self.assertRaisesRegex(ValueError, message):
                    self.generate(root)
                self.assertEqual(self.captured, [])

    def test_rejects_wrong_root_and_existing_control_definitions(self) -> None:
        bad_root = copy.deepcopy(ET.parse(OFFICIAL_URDF).getroot())
        ET.SubElement(bad_root, "link", {"name": "orphan_root"})
        with self.assertRaisesRegex(ValueError, "root link named pelvis"):
            self.generate(bad_root)

        undefined_parent = copy.deepcopy(ET.parse(OFFICIAL_URDF).getroot())
        undefined_parent.find("joint[@type='revolute']/parent").set("link", "unrelated")
        with self.assertRaisesRegex(ValueError, "undefined parent or child link"):
            self.generate(undefined_parent)

        duplicate_control = copy.deepcopy(ET.parse(OFFICIAL_URDF).getroot())
        ET.SubElement(duplicate_control, "ros2_control")
        with self.assertRaisesRegex(ValueError, "already defines ros2_control"):
            self.generate(duplicate_control)


if __name__ == "__main__":
    unittest.main()
