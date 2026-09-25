"""Pure-Python tests for the parameterised exhibition world generator."""

from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from g1_guide.exhibition_world import generate_world, validate_layout


ROOT = Path(__file__).resolve().parents[1]
CONFIG = json.loads((ROOT / "config/exhibition.json").read_text(encoding="utf-8"))


def _assets() -> Path:
    # The素材 agent supplies these deterministic local files.  Reusing them
    # keeps this test pure and avoids depending on a host temporary directory.
    return ROOT / "assets" / "exhibition"


def test_config_layout_is_valid_and_generation_is_parseable():
    validate_layout(deepcopy(CONFIG))
    sdf = generate_world(deepcopy(CONFIG), _assets())
    root = ET.fromstring(sdf)
    assert root.attrib["version"] == "1.9"
    world = root.find("world")
    assert world is not None and world.attrib["name"] == "exhibition"
    assert len(world.findall("model[@name='station_1']")) == 1
    assert len(world.findall("model[@name='station_6']")) == 1


def test_sensor_world_loads_gpu_rendering_and_imu_system():
    world = ET.fromstring(generate_world(deepcopy(CONFIG), _assets())).find("world")
    assert world is not None
    plugins = {element.get("name"): element for element in world.findall("plugin")}
    assert plugins["gz::sim::systems::Sensors"].findtext("render_engine") == "ogre2"
    assert plugins["gz::sim::systems::Imu"].get("filename") == "gz-sim-imu-system"


def test_overview_pose_is_on_movable_model_and_sensor_is_local_zero():
    sdf = generate_world(deepcopy(CONFIG), _assets())
    world = ET.fromstring(sdf).find("world")
    camera_model = world.find("model[@name='exhibition_overview']")
    assert camera_model is not None
    assert camera_model.findtext("pose") == " ".join(f"{float(v):.9g}" for v in CONFIG["overview_camera"]["pose"])
    assert camera_model.findtext("link/pose") == "0 0 0 0 0 0"
    sensor = camera_model.find("link/sensor")
    assert sensor is not None
    assert sensor.findtext("pose") == "0 0 0 0 0 0"
    assert sensor.findtext("topic") == "/exhibition/overview"
    assert sensor.findtext("update_rate") == "2"
    assert sensor.findtext("camera/horizontal_fov") == "1.35"
    assert sensor.findtext("camera/image/width") == "960"
    assert sensor.findtext("camera/image/height") == "720"
    assert sensor.findtext("camera/image/format") == "R8G8B8"


def test_posters_have_absolute_pbr_plane_and_backboard_collision():
    sdf = generate_world(deepcopy(CONFIG), _assets())
    world = ET.fromstring(sdf).find("world")
    for exhibit in CONFIG["exhibits"][:4]:
        model = world.find(f"model[@name='{exhibit['model_name']}']")
        assert model is not None
        assert model.find("link/collision/geometry/box") is not None
        plane = model.find("link/visual[@name='poster_visual']/geometry/plane")
        assert plane is not None
        assert plane.findtext("normal") == "0 -1 0"
        visual = model.find("link/visual[@name='poster_visual']")
        uri = visual.findtext("material/pbr/metal/albedo_map")
        assert visual.findtext("material/diffuse") == "1 1 1 1"
        assert visual.findtext("material/pbr/metal/metalness") == "0"
        assert uri.startswith("file:///")
        assert uri.endswith(f"/posters/{exhibit['asset']}")


def test_book_and_globe_are_non_box_models_with_matching_collision_geometry():
    sdf = generate_world(deepcopy(CONFIG), _assets())
    world = ET.fromstring(sdf).find("world")
    book = world.find("model[@name='exhibit_book']")
    globe = world.find("model[@name='exhibit_globe']")
    assert book is not None and book.find("link/visual[@name='book_pages_left_visual']") is not None
    assert globe is not None and globe.find("link/visual[@name='globe_sphere_visual']/geometry/sphere") is not None
    for model in (book, globe):
        visuals = {node.attrib["name"].removesuffix("_visual"): node.find("geometry")
                   for node in model.findall("link/visual")}
        collisions = {node.attrib["name"].removesuffix("_collision"): node.find("geometry")
                      for node in model.findall("link/collision")}
        assert set(visuals) == set(collisions)
        for name in visuals:
            assert ET.tostring(visuals[name]) == ET.tostring(collisions[name])


@pytest.mark.parametrize("mutator", [
    lambda c: c["room"].update(width=float("nan")),
    lambda c: c["exhibits"][0].update(size=[-1.0, 0.1, 1.6]),
    lambda c: c["exhibits"][1].update(exhibit_id=c["exhibits"][0]["exhibit_id"]),
    lambda c: c["exhibits"][1]["station"].update(key=c["exhibits"][0]["station"]["key"]),
    lambda c: c["spawn"].update(x=8.0),
    lambda c: c["exhibits"][0]["station"].update(x=-7.5),
    lambda c: c["exhibits"][0]["station"].update(x=-4.0, y=5.6),
    lambda c: c.update(schema_version=True),
    lambda c: c["exhibits"][0].update(model_name="g1_glide"),
    lambda c: c["exhibits"][0].update(model_name="bad\"name"),
    lambda c: c["exhibits"][0].update(exhibit_id="../outside"),
    lambda c: c["exhibits"][0].update(model_name="station_6"),
    lambda c: c["obstacles"][0].update(name=c["exhibits"][0]["model_name"]),
    lambda c: c["obstacles"][0].update(name="floor"),
    lambda c: c["obstacles"][0].update(position=[10, 0, .35]),
    lambda c: c["exhibits"][4].update(position=[-4, 0, .6]),
    lambda c: c["exhibits"][4].update(size=[.5, 1.2, .9]),
    lambda c: c["exhibits"][0]["station"].update(key="front"),
    lambda c: c["exhibits"][0]["station"].update(key="7"),
    lambda c: c["exhibits"][5].update(position=c["exhibits"][4]["position"]),
    lambda c: c["obstacles"][0].update(position=[-4, 0, .35]),
    lambda c: c["obstacles"][1].update(position=c["obstacles"][0]["position"]),
])
def test_layout_rejects_invalid_or_unsafe_mutations(mutator):
    config = deepcopy(CONFIG)
    mutator(config)
    with pytest.raises(ValueError):
        validate_layout(config)


def test_cli_arguments_are_not_needed_for_library_api():
    # Keep the test suite independent of ROS and of the optional asset package.
    # The command-line wrapper is intentionally a thin caller of this API.
    assert callable(generate_world)


def test_every_sdf_pose_has_six_finite_values_and_walls_rest_on_floor():
    world = ET.fromstring(generate_world(deepcopy(CONFIG), _assets())).find("world")
    for pose in world.iter("pose"):
        values = [float(x) for x in pose.text.split()]
        assert len(values) == 6
        assert all(math.isfinite(x) for x in values)
    for name in ("north", "south", "east", "west"):
        wall = world.find(f"model[@name='wall_{name}']")
        z = float(wall.findtext("pose").split()[2])
        height = float(wall.findtext("link/collision/geometry/box/size").split()[2])
        assert z - height / 2 == pytest.approx(CONFIG["room"]["floor_top"])


@pytest.mark.parametrize("size", [[1.4, 1.2, .9], [1.2, 1.0, 1.1]])
def test_solid_collision_extents_match_validated_footprint_and_floor(size):
    config = deepcopy(CONFIG)
    for exhibit in config["exhibits"][4:]:
        exhibit["size"] = size
        exhibit["position"][2] = config["room"]["floor_top"] + size[2] / 2
    world = ET.fromstring(generate_world(config, _assets())).find("world")
    for exhibit in config["exhibits"][4:]:
        model = world.find(f"model[@name='{exhibit['model_name']}']")
        pedestal = model.find("link/collision[@name='pedestal_collision']")
        assert list(map(float, pedestal.findtext("geometry/box/size").split())) == size
        for collision in model.findall("link/collision"):
            x, y, z, roll, pitch, yaw = map(float, collision.findtext("pose", "0 0 0 0 0 0").split())
            cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
            rotation = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                        [-sp, cp*sr, cp*cr]]
            geom = collision.find("geometry")
            if geom.find("box") is not None:
                half = [float(v)/2 for v in geom.findtext("box/size").split()]
                extent = [sum(abs(a)*b for a, b in zip(row, half)) for row in rotation]
            elif geom.find("sphere") is not None:
                extent = [float(geom.findtext("sphere/radius"))] * 3
            else:
                radius, half = float(geom.findtext("cylinder/radius")), float(geom.findtext("cylinder/length"))/2
                extent = [radius*math.hypot(row[0], row[1]) + half*abs(row[2]) for row in rotation]
            assert abs(x) + extent[0] <= size[0]/2 + 1e-8
            assert abs(y) + extent[1] <= size[1]/2 + 1e-8
            assert exhibit["position"][2] + z - extent[2] >= config["room"]["floor_top"] - 1e-8


def test_missing_poster_fails_explicitly(tmp_path):
    with pytest.raises(FileNotFoundError, match="poster asset"):
        generate_world(deepcopy(CONFIG), tmp_path)


def test_additional_exhibit_uses_next_numeric_station_key():
    config = deepcopy(CONFIG)
    extra = deepcopy(config["exhibits"][0])
    extra.update(exhibit_id="E07", model_name="exhibit_extra", position=[0, 5.78, 1.8], size=[.6, .1, 1.6])
    extra["station"].update(key="7", x=0)
    config["exhibits"].append(extra)
    world = ET.fromstring(generate_world(config, _assets())).find("world")
    assert world.find("model[@name='station_7']") is not None
