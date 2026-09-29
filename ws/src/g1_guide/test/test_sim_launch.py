"""Sim launch helpers: head camera rate and lazy sensor bridges (WSL render leak)."""

from pathlib import Path

import pytest
import yaml

from g1_guide.lidar_variants import with_camera_rate

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("model", ["g1_glide.urdf", "g1_glide_mid360.urdf"])
def test_camera_rate_changes_only_the_head_camera(model):
    urdf = (ROOT / "urdf" / model).read_text(encoding="utf-8")
    patched = with_camera_rate(urdf, 10)
    camera = patched[patched.index('<sensor name="d435_rgbd"'):]
    assert camera[:camera.index("</sensor>")].count("<update_rate>10</update_rate>") == 1
    # Everything else (lidars, IMU) keeps its rate.
    assert patched.replace("<update_rate>10</update_rate>", "<update_rate>30</update_rate>", 1) == urdf


def test_camera_rate_validation_and_models_without_camera():
    assert with_camera_rate("<robot/>", 5) == "<robot/>"
    for bad in (0, -1, float("nan"), 5000):
        with pytest.raises(ValueError):
            with_camera_rate("<robot/>", bad)


def test_heavy_sensors_are_bridged_lazily_and_navigation_inputs_are_not():
    entries = {e["ros_topic_name"]: e for e in
               yaml.safe_load((ROOT / "config/bridge.yaml").read_text(encoding="utf-8"))}
    for topic in ("/camera/image", "/camera/depth_image", "/camera/camera_info",
                  "/camera/points", "/scan"):
        assert entries[topic].get("lazy") is True, topic
    for topic in ("/nav_scan", "/odom", "/clock", "/tf", "/cmd_vel", "/joint_states"):
        assert not entries[topic].get("lazy", False), topic
