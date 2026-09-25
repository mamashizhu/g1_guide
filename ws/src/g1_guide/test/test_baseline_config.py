"""Static wiring checks; these do not replace the real simulation probe."""
from pathlib import Path
import ast
import math
import xml.etree.ElementTree as ET

import yaml
import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def test_clock_bridge_is_single_gazebo_to_ros():
    bridges = yaml.safe_load((ROOT / "config/bridge.yaml").read_text())
    clock = [entry for entry in bridges if entry["ros_topic_name"] == "/clock"]
    assert clock == [{
        "ros_topic_name": "/clock", "gz_topic_name": "/clock",
        "ros_type_name": "rosgraph_msgs/msg/Clock", "gz_type_name": "gz.msgs.Clock",
        "direction": "GZ_TO_ROS",
    }]
    topics = [entry["ros_topic_name"] for entry in bridges]
    assert len(topics) == len(set(topics))


def test_probe_dependencies_declared():
    root = ET.parse(ROOT / "package.xml").getroot()
    names = {dep.text for dep in root.findall("exec_depend")}
    assert {"rosgraph_msgs", "tf2_ros", "sensor_msgs", "nav_msgs"} <= names


def test_managed_bridge_is_wired_without_global_workarounds():
    tree = ast.parse((ROOT / "launch/g1_sim.launch.py").read_text())
    nodes = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Name) and n.func.id == "Node"]
    bridge = next(n for n in nodes if any(k.arg == "executable"
                  and isinstance(k.value, ast.Constant) and k.value.value == "managed_bridge"
                  for k in n.keywords))
    assert next(k.value.value for k in bridge.keywords if k.arg == "package") == "g1_bridge"
    assert not any(k.arg == "additional_env" for k in bridge.keywords)
    assert not any(isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                   and n.func.id == "SetEnvironmentVariable" for n in ast.walk(tree))


def test_spawn_node_uses_configured_height():
    tree = ast.parse((ROOT / "launch/g1_sim.launch.py").read_text())
    nodes = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
             and isinstance(node.func, ast.Name) and node.func.id == "Node"]
    creators = [node for node in nodes if any(k.arg == "executable"
                and isinstance(k.value, ast.Constant) and k.value.value == "create"
                for k in node.keywords)]
    assert len(creators) == 1
    arguments = next(k.value for k in creators[0].keywords if k.arg == "arguments")
    index = next(i for i, value in enumerate(arguments.elts)
                 if isinstance(value, ast.Constant) and value.value == "-z")
    height = arguments.elts[index + 1]
    assert isinstance(height, ast.Call) and height.func.id == "LaunchConfiguration"
    assert height.args[0].value == "spawn_z"


def test_default_spawn_clears_official_zero_pose_feet():
    """Check collision geometry, not a guessed pelvis/foot height."""
    model = ET.parse(ROOT.parent / "g1_description/g1_29dof_mode_11.urdf").getroot()
    transforms = {"pelvis": np.eye(4)}
    pending = list(model.findall("joint"))
    while pending:
        progressed = False
        for joint in list(pending):
            parent = joint.find("parent").get("link")
            if parent not in transforms:
                continue
            origin = joint.find("origin")
            r, p, y = map(float, origin.get("rpy", "0 0 0").split())
            cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
            matrix = np.eye(4)
            matrix[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                             [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                             [-sp, cp*sr, cp*cr]]
            matrix[:3, 3] = list(map(float, origin.get("xyz", "0 0 0").split()))
            transforms[joint.find("child").get("link")] = transforms[parent] @ matrix
            pending.remove(joint)
            progressed = True
        assert progressed, "Disconnected source URDF"
    foot_bottoms = []
    for side in ("left", "right"):
        name = f"{side}_ankle_roll_link"
        for collision in model.find(f"./link[@name='{name}']").findall("collision"):
            sphere = collision.find("geometry/sphere")
            assert sphere is not None
            xyz = list(map(float, collision.find("origin").get("xyz").split()))
            center = transforms[name] @ np.array([*xyz, 1.0])
            foot_bottoms.append(center[2] - float(sphere.get("radius")))
    tree = ast.parse((ROOT / "launch/g1_sim.launch.py").read_text())
    declarations = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Name) and node.func.id == "DeclareLaunchArgument"
                    and node.args and isinstance(node.args[0], ast.Constant)
                    and node.args[0].value == "spawn_z"]
    assert len(declarations) == 1
    height = float(next(k.value.value for k in declarations[0].keywords if k.arg == "default_value"))
    world = ET.parse(ROOT / "worlds/classroom.sdf").getroot()
    thickness = float(world.findtext(".//model[@name='floor']/link/collision/geometry/box/size").split()[2])
    clearance = height + min(foot_bottoms) - thickness / 2
    assert 0.001 < clearance < 0.01, f"Zero-pose foot clearance {clearance} m"
