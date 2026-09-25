#!/usr/bin/env python3
"""Generate a fixed-base G1 URDF with RGB-D, lidar, and IMU sensors for Gazebo Jetty.

The script deliberately refuses to fabricate a robot model. It reads the official
Unitree source model from ``$G1_WS/src/g1_description/g1_29dof_mode_11.urdf``
and writes the generated result only after the source has been found.
"""

from __future__ import annotations

import argparse
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable


PACKAGE_NAME = "g1_guide"
SOURCE_RELATIVE_PATH = Path("src") / "g1_description" / "g1_29dof_mode_11.urdf"


def _default_g1_ws() -> Path:
    configured = os.environ.get("G1_WS")
    if configured:
        return Path(configured).expanduser()
    return Path("/mnt/d/智能机器人/claude_course/ws")


def _file_uri(path: Path) -> str:
    """Return a Gazebo-readable absolute path for a local mesh.

    ``Path.as_uri()`` percent-encodes the Chinese workspace directory.  Gazebo
    Jetty accepts the POSIX path directly but does not decode that URI in all
    mesh-loading paths, so preserve the UTF-8 path spelling here.
    """

    return str(path.resolve())


def _package_mesh_path(filename: str, source_path: Path, g1_ws: Path) -> Path | None:
    if filename.startswith("package://"):
        package_and_file = filename[len("package://") :]
        package_name, separator, relative_file = package_and_file.partition("/")
        if not separator:
            return None
        package_root = g1_ws / "src" / package_name
        return package_root / relative_file
    if filename.startswith("file://"):
        local_name = filename[len("file://") :]
        return Path(local_name)
    return source_path.parent / filename


def _rewrite_meshes(root: ET.Element, source_path: Path, g1_ws: Path) -> None:
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if not filename:
            continue
        resolved = _package_mesh_path(filename, source_path, g1_ws)
        if resolved is None:
            raise ValueError(f"Cannot resolve mesh URI: {filename}")
        mesh.set("filename", _file_uri(resolved))


MOVABLE_ONLY_CHILDREN = {
    "axis",
    "axis2",
    "calibration",
    "dynamics",
    "limit",
    "mimic",
    "safety_controller",
}


def _freeze_joints(root: ET.Element) -> None:
    for joint in root.findall("joint"):
        joint.set("type", "fixed")
        for child in list(joint):
            if child.tag in MOVABLE_ONLY_CHILDREN:
                joint.remove(child)

    # Unitree descriptions can contain controller/transmission declarations that
    # have no meaning for the kinematic glide model and can make Gazebo reject it.
    for element in list(root):
        if element.tag in {"transmission", "ros2_control"}:
            root.remove(element)


def _link_names(root: ET.Element) -> set[str]:
    return {link.get("name", "") for link in root.findall("link")}


def _root_links(root: ET.Element) -> list[str]:
    links = _link_names(root)
    child_links = {
        child.get("link", "")
        for joint in root.findall("joint")
        for child in joint.findall("child")
    }
    candidates = sorted(link for link in links if link and link not in child_links)
    if not candidates:
        raise ValueError("The source URDF does not contain a root/base link")
    if len(candidates) != 1:
        raise ValueError(
            "The source URDF must contain exactly one root link; found "
            + ", ".join(candidates)
        )
    return candidates


def _base_link(root: ET.Element) -> str:
    """Return the generated model's control frame.

    ``generate`` calls ``_ensure_base_link`` first, so a valid generated model
    always has ``base_link`` as its URDF root.  Keeping this check here makes
    camera fallback generation fail loudly rather than silently attaching a
    sensor to a disconnected or ambiguous tree when the helper is reused.
    """

    roots = _root_links(root)
    if roots != ["base_link"]:
        raise ValueError(
            "The generated model root must be base_link; found " + ", ".join(roots)
        )
    return "base_link"


def _ensure_base_link(root: ET.Element) -> None:
    """Make ``base_link`` the root without changing the official model frames.

    Unitree's source uses ``pelvis`` as its root.  The glide controller and
    odometry publisher use ``base_link`` as the model control frame.  Add a
    coincident, non-physical frame and a fixed joint to the original pelvis.
    This preserves every source link pose, including the official camera mount,
    while making odom -> base_link -> pelvis a connected tree.  The odometry
    plugin is configured for 3D so this co-location does not project the
    pelvis's world height onto the ground plane.
    """

    roots = _root_links(root)
    if roots == ["base_link"]:
        return

    links = _link_names(root)
    if "base_link" in links:
        raise ValueError(
            "The source contains base_link, but it is not the root; refusing "
            "to guess its intended transform"
        )

    source_root = roots[0]
    root.insert(0, _element("link", {"name": "base_link"}))

    base_joint = _element("joint", {"name": "base_link_to_pelvis", "type": "fixed"})
    base_joint.append(_element("origin", {"xyz": "0 0 0", "rpy": "0 0 0"}))
    base_joint.append(_element("parent", {"link": "base_link"}))
    base_joint.append(_element("child", {"link": source_root}))
    root.append(base_joint)


def _element(tag: str, attributes: dict[str, str] | None = None) -> ET.Element:
    return ET.Element(tag, attributes or {})


def _add_camera(root: ET.Element) -> None:
    existing_links = _link_names(root)
    base = _base_link(root)

    # The official mode_11 model already contains the physical d435_link and
    # its mount joint.  Reuse it when present; older descriptions need the
    # small synthetic mount below.
    if "d435_link" not in existing_links:
        camera_link = _element("link", {"name": "d435_link"})
        inertial = _element("inertial")
        inertial.append(_element("origin", {"xyz": "0 0 0", "rpy": "0 0 0"}))
        inertial.append(_element("mass", {"value": "0.08"}))
        inertia = _element(
            "inertia",
            {
                "ixx": "0.0001",
                "ixy": "0",
                "ixz": "0",
                "iyy": "0.0001",
                "iyz": "0",
                "izz": "0.0001",
            },
        )
        inertial.append(inertia)
        camera_link.append(inertial)
        root.append(camera_link)

        mount_joint = _element("joint", {"name": "d435_mount_joint", "type": "fixed"})
        mount_joint.append(_element("parent", {"link": base}))
        mount_joint.append(_element("child", {"link": "d435_link"}))
        mount_joint.append(
            _element("origin", {"xyz": "0.20 0.0 0.45", "rpy": "0 0 0"})
        )
        root.append(mount_joint)

    if "d435_optical_frame" in existing_links:
        raise ValueError("The source URDF already contains d435_optical_frame")

    optical_link = _element("link", {"name": "d435_optical_frame"})
    root.append(optical_link)

    optical_joint = _element(
        "joint", {"name": "d435_optical_joint", "type": "fixed"}
    )
    optical_joint.append(_element("parent", {"link": "d435_link"}))
    optical_joint.append(_element("child", {"link": "d435_optical_frame"}))
    optical_joint.append(
        _element(
            "origin",
            {"xyz": "0 0 0", "rpy": "-1.57079632679 0 -1.57079632679"},
        )
    )
    root.append(optical_joint)

    gazebo = _element("gazebo", {"reference": "d435_link"})
    sensor = _element("sensor", {"name": "d435_rgbd", "type": "rgbd"})
    sensor.append(_element("always_on", {},))
    sensor.find("always_on").text = "true"
    sensor.append(_element("visualize", {},))
    sensor.find("visualize").text = "true"
    sensor.append(_element("update_rate", {},))
    sensor.find("update_rate").text = "30"
    sensor.append(_element("topic", {},))
    sensor.find("topic").text = "/camera"
    sensor.append(_element("gz_frame_id", {},))
    sensor.find("gz_frame_id").text = "d435_optical_frame"
    sensor.append(_element("frame_id", {},))
    sensor.find("frame_id").text = "d435_optical_frame"

    camera = _element("camera")
    camera.append(_element("horizontal_fov", {},))
    camera.find("horizontal_fov").text = "1.0471975512"
    image = _element("image")
    for tag, text in (("width", "640"), ("height", "480"), ("format", "R8G8B8")):
        child = _element(tag)
        child.text = text
        image.append(child)
    camera.append(image)
    clip = _element("clip")
    near = _element("near")
    near.text = "0.10"
    far = _element("far")
    far.text = "10.0"
    clip.extend([near, far])
    camera.append(clip)
    sensor.append(camera)

    depth_camera = _element("depth_camera")
    depth_clip = _element("clip")
    depth_near = _element("near")
    depth_near.text = "0.10"
    depth_far = _element("far")
    depth_far.text = "10.0"
    depth_clip.extend([depth_near, depth_far])
    depth_camera.append(depth_clip)
    # SDFormat declares depth_camera under camera (not directly under sensor).
    # Keeping this nesting also avoids the Gazebo warning that the sibling form
    # is an undefined sensor child while preserving the RGB-D sensor contract.
    camera.append(depth_camera)
    gazebo.append(sensor)
    root.append(gazebo)


def _ensure_sensor_mount(
    root: ET.Element,
    *,
    link_name: str,
    joint_name: str,
    parent_link: str,
    xyz: str,
    rpy: str = "0 0 0",
) -> None:
    """Keep an official sensor mount or add a fixed fallback mount."""

    links = _link_names(root)
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    joint = joints.get(joint_name)
    if link_name in links or joint is not None:
        if link_name not in links or joint is None:
            raise ValueError(
                f"Sensor mount {joint_name!r} must define both link {link_name!r} "
                "and its fixed joint"
            )
        if joint.get("type") != "fixed":
            raise ValueError(f"Sensor mount {joint_name!r} must be fixed")
        parent = joint.find("parent")
        child = joint.find("child")
        if (
            parent is None
            or parent.get("link") != parent_link
            or child is None
            or child.get("link") != link_name
        ):
            raise ValueError(
                f"Sensor mount {joint_name!r} must connect "
                f"{parent_link!r} to {link_name!r}"
            )
        return

    if parent_link not in links:
        raise ValueError(f"Sensor mount parent link {parent_link!r} is missing")
    root.append(_element("link", {"name": link_name}))
    joint = _element("joint", {"name": joint_name, "type": "fixed"})
    joint.append(_element("origin", {"xyz": xyz, "rpy": rpy}))
    joint.append(_element("parent", {"link": parent_link}))
    joint.append(_element("child", {"link": link_name}))
    root.append(joint)


def _add_lidar_and_imu(root: ET.Element) -> None:
    """Add low-rate Gazebo sensors on the official fixed G1 sensor frames."""

    torso = "torso_link" if "torso_link" in _link_names(root) else _base_link(root)
    _ensure_sensor_mount(
        root,
        link_name="mid360_link",
        joint_name="mid360_joint",
        parent_link=torso,
        xyz="0 0 0.42",
        # Keep a horizontal scan plane in the fallback frame. The official G1
        # URDF has a calibrated MID360 mount and is left untouched above.
    )
    _ensure_sensor_mount(
        root,
        link_name="imu_in_torso",
        joint_name="imu_in_torso_joint",
        parent_link=torso,
        xyz="0 0 0.15",
    )
    # Keep the official, inverted MID360 link and its original /scan intact.
    # Navigation uses an additional, level simulation sensor so its scan
    # plane has an unambiguous upright ROS TF and no mount-roll ambiguity.
    _ensure_sensor_mount(
        root,
        link_name="nav_lidar_link",
        joint_name="nav_lidar_joint",
        parent_link="base_link",
        xyz="0 0 0.47",
        rpy="0 0 0",
    )

    lidar_gazebo = _element("gazebo", {"reference": "mid360_link"})
    lidar = _element("sensor", {"name": "g1_2d_lidar", "type": "gpu_lidar"})
    for tag, value in (
        ("always_on", "true"),
        ("visualize", "false"),
        ("update_rate", "10"),
        ("topic", "/scan"),
        ("gz_frame_id", "mid360_link"),
    ):
        child = _element(tag)
        child.text = value
        lidar.append(child)
    lidar_spec = _element("lidar")
    scan = _element("scan")
    horizontal = _element("horizontal")
    for tag, value in (
        ("samples", "360"),
        ("resolution", "1"),
        ("min_angle", "-3.141592653589793"),
        ("max_angle", "3.141592653589793"),
    ):
        child = _element(tag)
        child.text = value
        horizontal.append(child)
    scan.append(horizontal)
    lidar_spec.append(scan)
    distance = _element("range")
    for tag, value in (
        ("min", "0.12"),
        ("max", "8.0"),
        ("resolution", "0.01"),
    ):
        child = _element(tag)
        child.text = value
        distance.append(child)
    lidar_spec.append(distance)
    lidar.append(lidar_spec)
    lidar_gazebo.append(lidar)
    root.append(lidar_gazebo)

    nav_gazebo = _element("gazebo", {"reference": "nav_lidar_link"})
    nav_lidar = _element("sensor", {"name": "g1_nav_lidar", "type": "gpu_lidar"})
    for tag, value in (
        ("always_on", "true"), ("visualize", "false"), ("update_rate", "5"),
        ("topic", "/nav_scan"), ("gz_frame_id", "nav_lidar_link"),
    ):
        child = _element(tag)
        child.text = value
        nav_lidar.append(child)
    nav_lidar.append(ET.fromstring(ET.tostring(lidar_spec)))
    nav_gazebo.append(nav_lidar)
    root.append(nav_gazebo)

    imu_gazebo = _element("gazebo", {"reference": "imu_in_torso"})
    imu = _element("sensor", {"name": "g1_imu", "type": "imu"})
    for tag, value in (
        ("always_on", "true"),
        ("visualize", "false"),
        ("update_rate", "50"),
        ("topic", "/imu"),
        ("gz_frame_id", "imu_in_torso"),
    ):
        child = _element(tag)
        child.text = value
        imu.append(child)
    imu.append(_element("imu"))
    imu_gazebo.append(imu)
    root.append(imu_gazebo)


def _add_motion_plugins(root: ET.Element) -> None:
    # SDFormat lumps every fixed joint into the root, so base_link becomes the
    # single physical link. SDF has no model-level <gravity>; it is per link.
    gravity_off = _element("gazebo", {"reference": "base_link"})
    no_gravity = _element("gravity")
    no_gravity.text = "false"
    gravity_off.append(no_gravity)
    root.append(gravity_off)

    gazebo = _element("gazebo")

    velocity = _element(
        "plugin",
        {
            "name": "gz::sim::systems::VelocityControl",
            "filename": "gz-sim-velocity-control-system",
        },
    )
    for tag, text in (
        ("topic", "/cmd_vel"),
        ("initial_linear_velocity", "0 0 0"),
        ("initial_angular_velocity", "0 0 0"),
    ):
        child = _element(tag)
        child.text = text
        velocity.append(child)
    gazebo.append(velocity)

    odometry = _element(
        "plugin",
        {
            "name": "gz::sim::systems::OdometryPublisher",
            "filename": "gz-sim-odometry-publisher-system",
        },
    )
    for tag, text in (
        ("odom_topic", "/odom"),
        ("odom_frame", "odom"),
        ("robot_base_frame", "base_link"),
        ("dimensions", "3"),
        ("tf_topic", "/tf"),
        ("publish_tf", "true"),
    ):
        child = _element(tag)
        child.text = text
        odometry.append(child)
    gazebo.append(odometry)
    root.append(gazebo)


def _indent(root: ET.Element) -> None:
    ET.indent(root, space="  ")


def generate(source_path: Path, output_path: Path, g1_ws: Path) -> None:
    if not source_path.is_file():
        raise FileNotFoundError(
            "Official G1 source URDF was not found at "
            f"{source_path}. Set G1_WS to the Linux workspace containing "
            "src/g1_description/g1_29dof_mode_11.urdf; the generator will not "
            "create a fabricated robot model."
        )

    try:
        root = ET.parse(source_path).getroot()
    except ET.ParseError as exc:
        raise ValueError(f"Source URDF is not valid XML: {source_path}") from exc

    _rewrite_meshes(root, source_path, g1_ws)
    _freeze_joints(root)
    _ensure_base_link(root)
    _add_camera(root)
    _add_lidar_and_imu(root)
    _add_motion_plugins(root)
    _indent(root)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(output_path, encoding="utf-8", xml_declaration=True)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--g1-ws",
        type=Path,
        default=_default_g1_ws(),
        help="ROS workspace containing src/g1_description (default: $G1_WS)",
    )
    parser.add_argument(
        "--source",
        type=Path,
        help="Override the source URDF path; normally derived from --g1-ws.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "urdf" / "g1_glide.urdf",
        help="Generated URDF destination.",
    )
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    g1_ws = args.g1_ws.expanduser().resolve()
    source = (args.source or (g1_ws / SOURCE_RELATIVE_PATH)).expanduser().resolve()
    output = args.output.expanduser().resolve()
    try:
        generate(source, output, g1_ws)
    except (FileNotFoundError, ValueError, OSError) as exc:
        print(f"g1_guide: generation failed: {exc}", file=sys.stderr)
        return 2
    print(f"Generated {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
