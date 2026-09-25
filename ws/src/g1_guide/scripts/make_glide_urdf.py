#!/usr/bin/env python3
"""Generate a fixed-base G1 URDF with RGB-D, lidar, and IMU sensors for Gazebo Jetty.

Legs and waist are welded so the body glides as one rigid body; the 14 arm
joints stay revolute and are held by per-joint position controllers so the
guide can gesture (``gesture_player``).

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
    # scripts/ -> g1_guide/ -> src/ -> ws/ (symlink-install resolves to source)
    return Path(__file__).resolve().parents[3]


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


ARM_JOINT_SUFFIXES = (
    "shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
    "wrist_roll", "wrist_pitch", "wrist_yaw",
)
ARM_JOINTS = tuple(
    f"{side}_{suffix}_joint" for side in ("left", "right") for suffix in ARM_JOINT_SUFFIXES
)
# gesture_player publishes std_msgs/Float64 here; config/bridge.yaml forwards it.
ARM_COMMAND_TOPIC = "/g1/arm/{joint}/cmd_pos"
GESTURES_FILE = Path(__file__).resolve().parents[1] / "config" / "gestures.yaml"


def _arm_joints_present(root: ET.Element) -> list[str]:
    """Return the arm joints to keep movable; all 14 or none (test fixtures)."""

    revolute = {
        joint.get("name") for joint in root.findall("joint")
        if joint.get("type") in ("revolute", "continuous")
    }
    present = [name for name in ARM_JOINTS if name in revolute]
    if present and len(present) != len(ARM_JOINTS):
        missing = sorted(set(ARM_JOINTS) - set(present))
        raise ValueError("Source URDF has only part of the arm chain; missing " + ", ".join(missing))
    return present


def _freeze_joints(root: ET.Element, keep: Iterable[str] = ()) -> None:
    keep = set(keep)
    for joint in root.findall("joint"):
        if joint.get("name") in keep:
            continue
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


# Livox MID-360 datasheet: 360° x (-7°..+52°), 0.1-40 m (10 % reflectivity), 10 Hz.
# The real sensor scans a non-repetitive pattern; 360 x 30 rays at 10 Hz keeps
# the same field of view at a point rate the local GPU simulates in real time.
MID360_VERTICAL = (-0.12217304763960307, 0.9075712110370514, 30)
MID360_RANGE = ("0.1", "40.0")
LIDAR_MODELS = ("simple", "mid360")


def _add_lidar_and_imu(root: ET.Element, lidar_model: str = "simple") -> None:
    """Add low-rate Gazebo sensors on the official fixed G1 sensor frames.

    ``simple``: the MID-360 mount carries a single-plane 2D scan (/scan) and a
    level simulation-only lidar publishes /nav_scan.
    ``mid360``: the MID-360 mount carries a 3D lidar with the real field of
    view (/mid360/points); /nav_scan is produced from that cloud by
    ``mid360_to_scan``, so nav_lidar_link stays as a frame without a sensor.
    """

    if lidar_model not in LIDAR_MODELS:
        raise ValueError(f"lidar model must be one of {LIDAR_MODELS}")
    real = lidar_model == "mid360"

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
    lidar = _element("sensor", {"name": "g1_mid360" if real else "g1_2d_lidar", "type": "gpu_lidar"})
    for tag, value in (
        ("always_on", "true"),
        ("visualize", "false"),
        ("update_rate", "10"),
        # gz publishes a 3D lidar's cloud on <topic>/points.
        ("topic", "/mid360" if real else "/scan"),
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
    if real:
        vertical = _element("vertical")
        low, high, rows = MID360_VERTICAL
        for tag, value in (("samples", str(rows)), ("resolution", "1"),
                           ("min_angle", repr(low)), ("max_angle", repr(high))):
            child = _element(tag)
            child.text = value
            vertical.append(child)
        scan.append(vertical)
    lidar_spec.append(scan)
    distance = _element("range")
    for tag, value in (
        ("min", MID360_RANGE[0] if real else "0.12"),
        ("max", MID360_RANGE[1] if real else "8.0"),
        ("resolution", "0.01"),
    ):
        child = _element(tag)
        child.text = value
        distance.append(child)
    lidar_spec.append(distance)
    lidar.append(lidar_spec)
    lidar_gazebo.append(lidar)
    root.append(lidar_gazebo)

    if not real:
        _add_nav_lidar(root, lidar_spec)
    _add_imu(root)


def _add_nav_lidar(root: ET.Element, lidar_spec: ET.Element) -> None:
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


def _add_imu(root: ET.Element) -> None:
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


def _add_motion_plugins(root: ET.Element, truth_odom: bool = False) -> None:
    """VelocityControl plus odometry.

    ``truth_odom`` (真机雷达版): Gazebo's exact pose goes to /odom_truth with
    no TF, and the odom_drift node publishes /odom and the odom -> base_link
    TF, the way a real robot's leg odometry would (with drift if enabled).
    """
    # SDFormat lumps every fixed joint into the root, so base_link carries the
    # whole body except the arm links (those get their own gravity-off tags). SDF has no model-level <gravity>; it is per link.
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
        ("odom_topic", "/odom_truth" if truth_odom else "/odom"),
        ("odom_frame", "odom_truth" if truth_odom else "odom"),
        ("robot_base_frame", "base_link_truth" if truth_odom else "base_link"),
        ("dimensions", "3"),
        ("tf_topic", "/tf"),
        ("publish_tf", "false" if truth_odom else "true"),
    ):
        child = _element(tag)
        child.text = text
        odometry.append(child)
    gazebo.append(odometry)
    root.append(gazebo)


def _home_pose(joints: list[str]) -> dict[str, float]:
    """Initial arm angles (rad) = the ``home`` pose from config/gestures.yaml."""

    import math
    import yaml

    data = yaml.safe_load(GESTURES_FILE.read_text(encoding="utf-8"))
    home = data["poses"]["home"]
    mirror = (1, -1, -1, 1, -1, 1, -1)
    result = {}
    for side in ("left", "right"):
        values = home[side]
        if isinstance(values, str):
            values = home[values]
        signs = mirror if side == "right" else (1,) * 7
        for suffix, sign, degrees in zip(ARM_JOINT_SUFFIXES, signs, values):
            result[f"{side}_{suffix}_joint"] = sign * math.radians(float(degrees))
    return {name: result[name] for name in joints}


def _add_arm_control(root: ET.Element, joints: list[str]) -> None:
    """Hold each arm joint with a stiff position controller.

    Velocity-command mode makes the joint track its setpoint kinematically, so
    no gains need tuning and the arm links (gravity off, like base_link) do not
    sag or swing while the body glides.
    """

    if not joints:
        return
    parents = {j.get("name"): j.find("child").get("link") for j in root.findall("joint")}
    for joint in joints:
        link = _element("gazebo", {"reference": parents[joint]})
        gravity = _element("gravity")
        gravity.text = "false"
        link.append(gravity)
        root.append(link)

    gazebo = _element("gazebo")
    home = _home_pose(joints)
    for joint in joints:
        plugin = _element("plugin", {
            "name": "gz::sim::systems::JointPositionController",
            "filename": "gz-sim-joint-position-controller-system",
        })
        for tag, text in (
            ("joint_name", joint),
            ("topic", ARM_COMMAND_TOPIC.format(joint=joint)),
            ("use_velocity_commands", "true"),
            ("initial_position", f"{home[joint]:.6f}"),
        ):
            child = _element(tag)
            child.text = text
            plugin.append(child)
        gazebo.append(plugin)

    states = _element("plugin", {
        "name": "gz::sim::systems::JointStatePublisher",
        "filename": "gz-sim-joint-state-publisher-system",
    })
    topic = _element("topic")
    topic.text = "/joint_states"
    states.append(topic)
    for joint in joints:
        name = _element("joint_name")
        name.text = joint
        states.append(name)
    gazebo.append(states)
    root.append(gazebo)


def _indent(root: ET.Element) -> None:
    ET.indent(root, space="  ")


def generate(source_path: Path, output_path: Path, g1_ws: Path,
             lidar_model: str = "simple") -> None:
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
    arm_joints = _arm_joints_present(root)
    _freeze_joints(root, keep=arm_joints)
    _ensure_base_link(root)
    _add_camera(root)
    _add_lidar_and_imu(root, lidar_model)
    _add_motion_plugins(root, truth_odom=lidar_model == "mid360")
    _add_arm_control(root, arm_joints)
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
        default=None,
        help="Generated URDF destination (default: urdf/g1_glide.urdf, "
             "or urdf/g1_glide_mid360.urdf with --lidar mid360).",
    )
    parser.add_argument(
        "--lidar",
        choices=LIDAR_MODELS,
        default="simple",
        help="simple: 2D scan planes (default); mid360: 3D lidar with the real MID-360 field of view.",
    )
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    g1_ws = args.g1_ws.expanduser().resolve()
    source = (args.source or (g1_ws / SOURCE_RELATIVE_PATH)).expanduser().resolve()
    urdf_dir = Path(__file__).resolve().parents[1] / "urdf"
    default_name = "g1_glide_mid360.urdf" if args.lidar == "mid360" else "g1_glide.urdf"
    output = (args.output or urdf_dir / default_name).expanduser().resolve()
    try:
        generate(source, output, g1_ws, args.lidar)
    except (FileNotFoundError, ValueError, OSError) as exc:
        print(f"g1_guide: generation failed: {exc}", file=sys.stderr)
        return 2
    print(f"Generated {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
