"""Deterministic configuration for the supported-base Gazebo PD experiment."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET


def configuration(source: Path):
    limits, gains = {}, {}
    for joint in ET.parse(source).getroot().findall("joint"):
        if joint.get("type") != "revolute":
            continue
        name = joint.attrib["name"]
        bound = joint.find("limit")
        lower, upper, effort = (float(bound.attrib[k]) for k in ("lower", "upper", "effort"))
        if name in limits or not all(map(math.isfinite, (lower, upper, effort))) or not lower < upper or effort <= 0:
            raise ValueError(f"Invalid limits: {name}")
        limits[name] = [lower, upper]
        if "wrist_pitch" in name or "wrist_yaw" in name:
            p, d, cap = 10.0, 0.15, 2.0
        elif "wrist_roll" in name:
            p, d, cap = 10.0, 0.05, 4.0
        elif "shoulder" in name:
            p, d, cap = 60.0, 2.0, 12.0
        elif "elbow" in name:
            p, d, cap = 40.0, 1.5, 10.0
        elif "ankle_roll" in name:
            # Small foot-roll inertia: d=1 is numerically unstable at 1ms.
            p, d, cap = 5.0, 0.05, 2.0
        elif "ankle" in name:
            p, d, cap = 25.0, 1.0, 15.0
        elif "waist" in name:
            p, d, cap = 300.0, 8.0, 30.0
        else:
            p, d, cap = 100.0, 4.0, 50.0
        cap = min(cap, effort)
        gains[name] = dict(p=p, i=0.0, d=d, u_clamp_min=-cap, u_clamp_max=cap,
                           i_clamp_min=0.0, i_clamp_max=0.0, feedforward_gain=0.0,
                           antiwindup_strategy="none", activate_state_publisher=False)
    if len(limits) != 29:
        raise ValueError("This experiment requires the official 29-DoF G1 model")
    controllers = {
        "controller_manager": {"ros__parameters": {
            "update_rate": 1000, "use_sim_time": True,
            "joint_state_broadcaster": {"type": "joint_state_broadcaster/JointStateBroadcaster"},
            "g1_pd": {"type": "pid_controller/PidController"},
        }},
        "joint_state_broadcaster": {"ros__parameters": {
            "use_sim_time": True, "joints": list(limits),
            "interfaces": ["position", "velocity", "effort"], "update_rate": 100,
        }},
        "g1_pd": {"ros__parameters": {
            # Below the manager rate: the manager's sub-rate scheduler skips
            # repeated /clock stamps instead of forcing a zero-dt PID update.
            "update_rate": 500, "use_sim_time": True,
            "dof_names": list(limits), "command_interface": "effort",
            "reference_and_state_interfaces": ["position", "velocity"],
            "use_external_measured_states": False, "set_current_state_as_first_setpoint": True,
            "gains": gains,
        }},
    }
    return limits, controllers
