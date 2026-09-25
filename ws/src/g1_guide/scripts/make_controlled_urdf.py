#!/usr/bin/env python3
"""Generate a supported, effort-controlled 29-DOF G1 URDF for Gazebo."""

from __future__ import annotations

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable
from urllib.parse import unquote, urlsplit


SOURCE_RELATIVE_PATH = Path("src") / "g1_description" / "g1_29dof_mode_11.urdf"
EXPECTED_REVOLUTE_JOINTS = 29


def _default_g1_ws() -> Path:
    configured = os.environ.get("G1_WS")
    if configured:
        return Path(configured).expanduser()
    # scripts/ -> g1_guide/ -> src/ -> ws/ (symlink-install resolves to source)
    return Path(__file__).resolve().parents[3]


def _link_names(root: ET.Element) -> set[str]:
    names = [link.get("name", "") for link in root.findall("link")]
    if not all(names) or len(names) != len(set(names)):
        raise ValueError("Source URDF links must have unique, non-empty names")
    return set(names)


def _single_pelvis_root(root: ET.Element) -> None:
    links = _link_names(root)
    child_links = [child.get("link", "") for child in root.findall("joint/child")]
    parent_links = [parent.get("link", "") for parent in root.findall("joint/parent")]
    if any(name not in links for name in child_links + parent_links):
        raise ValueError("Source URDF joint refers to an undefined parent or child link")
    if len(child_links) != len(set(child_links)):
        raise ValueError("Source URDF link is the child of more than one joint")
    roots = sorted(links - set(child_links))
    if roots != ["pelvis"]:
        raise ValueError(
            "Source URDF must have exactly one root link named pelvis; found "
            + (", ".join(roots) if roots else "none")
        )


def _reject_existing_control_definitions(root: ET.Element) -> None:
    if root.findall(".//ros2_control"):
        raise ValueError("Source URDF already defines ros2_control; refusing duplicates")
    if root.findall(".//transmission"):
        raise ValueError("Source URDF already defines transmissions; refusing duplicate control setup")
    for plugin in root.findall(".//plugin"):
        filename = (plugin.get("filename") or "").lower()
        name = (plugin.get("name") or "").lower()
        if "gz_ros2_control" in filename or "gz_ros2_control" in name:
            raise ValueError("Source URDF already defines a gz_ros2_control plugin")


def _joint_limits(root: ET.Element) -> list[tuple[str, str]]:
    joints = root.findall("joint")
    names = [joint.get("name", "") for joint in joints]
    if not all(names) or len(names) != len(set(names)):
        raise ValueError("Source URDF joints must have unique, non-empty names")

    revolute = [joint for joint in joints if joint.get("type") == "revolute"]
    if len(revolute) != EXPECTED_REVOLUTE_JOINTS:
        raise ValueError(
            f"Expected {EXPECTED_REVOLUTE_JOINTS} revolute joints; found {len(revolute)}"
        )

    validated: list[tuple[str, str]] = []
    for joint in revolute:
        name = joint.get("name", "<unnamed>")
        limit = joint.find("limit")
        if limit is None:
            raise ValueError(f"Revolute joint {name} has no limit element")
        raw = {key: limit.get(key) for key in ("lower", "upper", "effort", "velocity")}
        if any(value is None for value in raw.values()):
            raise ValueError(f"Revolute joint {name} needs lower, upper, effort, and velocity limits")
        try:
            lower, upper, effort, velocity = (float(raw[key]) for key in ("lower", "upper", "effort", "velocity"))
        except (TypeError, ValueError) as exc:
            raise ValueError(f"Revolute joint {name} has a non-numeric limit") from exc
        if not all(math.isfinite(value) for value in (lower, upper, effort, velocity)):
            raise ValueError(f"Revolute joint {name} limits must all be finite")
        if lower >= upper:
            raise ValueError(f"Revolute joint {name} lower limit must be less than upper limit")
        if effort <= 0 or velocity <= 0:
            raise ValueError(f"Revolute joint {name} effort and velocity limits must be positive")
        validated.append((name, raw["effort"] or ""))
    return validated


def _mesh_path(filename: str, source_path: Path, g1_ws: Path) -> Path:
    if filename.startswith("package://"):
        package_and_file = filename[len("package://") :]
        package_name, separator, relative_file = package_and_file.partition("/")
        if not separator or not package_name or not relative_file:
            raise ValueError(f"Invalid package mesh URI: {filename}")
        return (g1_ws / "src" / package_name / relative_file).resolve()
    if filename.startswith("file://"):
        parsed = urlsplit(filename)
        if parsed.netloc not in ("", "localhost"):
            raise ValueError(f"Only local file mesh URIs are supported: {filename}")
        path_text = unquote(parsed.path)
        # file:///C:/... on Windows has a leading slash before the drive letter.
        if os.name == "nt" and len(path_text) >= 3 and path_text[0] == "/" and path_text[2] == ":":
            path_text = path_text[1:]
        return Path(path_text).expanduser().resolve()
    mesh = Path(filename).expanduser()
    return (mesh if mesh.is_absolute() else source_path.parent / mesh).resolve()


def _rewrite_meshes(root: ET.Element, source_path: Path, g1_ws: Path) -> None:
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if not filename:
            raise ValueError("Every mesh element must have a filename")
        resolved = _mesh_path(filename, source_path, g1_ws)
        if not resolved.is_file():
            raise FileNotFoundError(f"Mesh file does not exist: {resolved} (from {filename})")
        # Keep a plain absolute filesystem path: Gazebo Jetty handles UTF-8 paths
        # more consistently than percent-encoded file URIs.
        mesh.set("filename", str(resolved))


def _element(tag: str, attributes: dict[str, str] | None = None) -> ET.Element:
    return ET.Element(tag, attributes or {})


def _add_support(root: ET.Element, support_height: float) -> None:
    if "world" in _link_names(root):
        raise ValueError("Source URDF already contains a world link")
    if any(joint.get("name") == "world_to_pelvis_support" for joint in root.findall("joint")):
        raise ValueError("Source URDF already contains world_to_pelvis_support")
    root.insert(0, _element("link", {"name": "world"}))
    joint = _element("joint", {"name": "world_to_pelvis_support", "type": "fixed"})
    joint.append(_element("origin", {"xyz": f"0 0 {support_height:.17g}", "rpy": "0 0 0"}))
    joint.append(_element("parent", {"link": "world"}))
    joint.append(_element("child", {"link": "pelvis"}))
    root.append(joint)


def _add_ros2_control(root: ET.Element, limits: list[tuple[str, str]]) -> None:
    control = _element("ros2_control", {"name": "GazeboSimSystem", "type": "system"})
    hardware = _element("hardware")
    plugin = _element("plugin")
    plugin.text = "gz_ros2_control/GazeboSimSystem"
    hardware.append(plugin)
    control.append(hardware)
    for name, effort in limits:
        joint = _element("joint", {"name": name})
        command = _element("command_interface", {"name": "effort"})
        for key, value in (("min", f"-{float(effort):.17g}"), ("max", f"{float(effort):.17g}")):
            param = _element("param", {"name": key})
            param.text = value
            command.append(param)
        joint.append(command)
        position = _element("state_interface", {"name": "position"})
        initial = _element("param", {"name": "initial_value"})
        initial.text = "0"
        position.append(initial)
        joint.extend((position, _element("state_interface", {"name": "velocity"}), _element("state_interface", {"name": "effort"})))
        control.append(joint)
    root.append(control)


def _add_gazebo_plugin(root: ET.Element, controllers_path: Path) -> None:
    gazebo = _element("gazebo")
    plugin = _element(
        "plugin",
        {
            "filename": "libgz_ros2_control-system.so",
            "name": "gz_ros2_control::GazeboSimROS2ControlPlugin",
        },
    )
    parameters = _element("parameters")
    parameters.text = str(controllers_path)
    plugin.append(parameters)
    gazebo.append(plugin)
    root.append(gazebo)


def generate(
    source_path: Path,
    output_path: Path,
    g1_ws: Path,
    controllers_path: Path,
    support_height: float = 1.3,
) -> None:
    source_path = Path(source_path).expanduser().resolve()
    output_path = Path(output_path).expanduser().resolve()
    g1_ws = Path(g1_ws).expanduser().resolve()
    controllers_path = Path(controllers_path).expanduser()

    if source_path == output_path:
        raise ValueError("Input source and output path must refer to different files")
    if not math.isfinite(support_height) or support_height < 0:
        raise ValueError("support_height must be a finite, non-negative number")
    if not controllers_path.is_absolute():
        raise ValueError(f"controllers_path must be absolute: {controllers_path}")
    controllers_path = controllers_path.resolve()
    if not controllers_path.is_file():
        raise FileNotFoundError(f"Controller parameters file does not exist: {controllers_path}")
    if not source_path.is_file():
        raise FileNotFoundError(f"Official G1 source URDF does not exist: {source_path}")

    try:
        root = ET.parse(source_path).getroot()
    except ET.ParseError as exc:
        raise ValueError(f"Source URDF is not valid XML: {source_path}: {exc}") from exc
    if root.tag != "robot":
        raise ValueError(f"Source XML root must be <robot>: {source_path}")

    _single_pelvis_root(root)
    _reject_existing_control_definitions(root)
    limits = _joint_limits(root)
    _rewrite_meshes(root, source_path, g1_ws)
    _add_support(root, support_height)
    _add_ros2_control(root, limits)
    _add_gazebo_plugin(root, controllers_path)
    ET.indent(root, space="  ")

    try:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        ET.ElementTree(root).write(output_path, encoding="utf-8", xml_declaration=True)
    except OSError as exc:
        raise OSError(f"Could not write generated URDF to {output_path}: {exc}") from exc


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--g1-ws", type=Path, default=_default_g1_ws(), help="Workspace containing src/g1_description")
    parser.add_argument("--source", type=Path, help="Official source URDF (defaults to --g1-ws/src/g1_description/...)")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "urdf" / "g1_controlled.urdf",
        help="Generated model destination",
    )
    parser.add_argument("--controllers", type=Path, required=True, help="Absolute controller parameters YAML path")
    parser.add_argument("--support-height", type=float, default=1.3, help="Fixed world-to-pelvis support height in metres")
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    g1_ws = args.g1_ws.expanduser().resolve()
    source = (args.source or (g1_ws / SOURCE_RELATIVE_PATH)).expanduser().resolve()
    try:
        generate(source, args.output, g1_ws, args.controllers, args.support_height)
    except (FileNotFoundError, ValueError, OSError) as exc:
        print(f"g1_guide: controlled URDF generation failed: {exc}", file=sys.stderr)
        return 2
    print(f"Generated controlled model: {args.output.expanduser().resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
