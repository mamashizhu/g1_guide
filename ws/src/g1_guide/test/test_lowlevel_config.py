from pathlib import Path
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from g1_guide.lowlevel_config import configuration


def test_official_pd_configuration():
    source = Path(__file__).resolve().parents[2] / "g1_description/g1_29dof_mode_11.urdf"
    limits, config = configuration(source)
    params = config["g1_pd"]["ros__parameters"]
    official = {j.attrib["name"]: float(j.find("limit").attrib["effort"])
                for j in ET.parse(source).getroot().findall("joint") if j.get("type") == "revolute"}
    assert len(limits) == 29
    assert params["command_interface"] == "effort"
    assert params["reference_and_state_interfaces"] == ["position", "velocity"]
    assert params["use_external_measured_states"] is False
    assert params["update_rate"] == 500
    assert config["controller_manager"]["ros__parameters"]["update_rate"] > params["update_rate"]
    assert params["dof_names"] == list(limits)
    for name, gain in params["gains"].items():
        assert gain["i"] == 0
        assert gain["feedforward_gain"] == 0
        assert gain["p"] > 0 and gain["d"] > 0
        assert gain["u_clamp_min"] == -gain["u_clamp_max"]
        assert 0 < gain["u_clamp_max"] <= official[name]
        assert limits[name][0] <= 0 <= limits[name][1]


def test_test_world_is_dynamic_and_fixed_step():
    world = Path(__file__).resolve().parents[1] / "worlds/lowlevel.sdf"
    root = ET.parse(world).getroot()
    assert root.findtext("world/physics/max_step_size") == "0.001"
    assert root.findtext("world/gravity") == "0 0 -9.81"
    assert all("VelocityControl" not in p.attrib["name"] for p in root.findall(".//plugin"))
