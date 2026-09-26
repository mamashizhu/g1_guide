"""Pure-Python tests for the parameterised exhibition world generator."""

from __future__ import annotations

from copy import deepcopy
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from g1_guide.exhibition_world import generate_world, validate_layout
from g1_guide.scene_config import load_scene


ROOT = Path(__file__).resolve().parents[1]
CONFIG = load_scene(ROOT / "config/exhibition.json")


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


def test_book_and_vase_are_non_box_models_with_matching_collision_geometry():
    sdf = generate_world(deepcopy(CONFIG), _assets())
    world = ET.fromstring(sdf).find("world")
    book = world.find("model[@name='exhibit_painting_manual']")
    vase = world.find("model[@name='exhibit_blue_white_vase']")
    assert book is not None and book.find("link/visual[@name='cradle_board_visual']") is not None
    assert vase is not None and vase.find("link/visual[@name='body_visual']/geometry/sphere") is not None
    assert vase.find("link/visual[@name='neck_visual']/geometry/cylinder") is not None
    # Printed pages are visual-only planes on top of solid page blocks.
    prints = book.findall("link/visual[@name]")
    prints = {node.attrib["name"]: node for node in prints if node.attrib["name"].endswith("_print")}
    assert set(prints) == {"page_left_print", "page_right_print", "volume_cover_print"}
    for node in prints.values():
        assert node.find("geometry/plane") is not None
        assert node.findtext("material/pbr/metal/albedo_map").startswith("file:///")
    for model in (book, vase):
        assert all(c.find("geometry/plane") is None for c in model.findall("link/collision"))
        visuals = {node.attrib["name"].removesuffix("_visual"): node.find("geometry")
                   for node in model.findall("link/visual")
                   if not node.attrib["name"].endswith("_print")}
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
    lambda c: c["spawn"].update(y=11.0),
    lambda c: c["exhibits"][0]["station"].update(x=-7.5),
    lambda c: c["exhibits"][0]["station"].update(y=10.6),
    lambda c: c.update(schema_version=True),
    lambda c: c["exhibits"][0].update(model_name="g1_glide"),
    lambda c: c["exhibits"][0].update(model_name="bad\"name"),
    lambda c: c["exhibits"][0].update(exhibit_id="../outside"),
    lambda c: c["exhibits"][0].update(model_name="station_6"),
    lambda c: c["people"][0].update(name=c["exhibits"][0]["model_name"]),
    lambda c: c["people"][0].update(name="floor"),
    lambda c: c["people"][0].update(position=[20, 0]),
    lambda c: c["exhibits"][4].update(position=[-4, 0, .6]),
    lambda c: c["exhibits"][4].update(size=[.5, 1.2, .9]),
    lambda c: c["exhibits"][0]["station"].update(key="front"),
    lambda c: c["exhibits"][0]["station"].update(key="7"),
    lambda c: c["exhibits"][5].update(position=c["exhibits"][4]["position"]),
    lambda c: c["people"][0].update(position=c["exhibits"][4]["position"][:2]),
    lambda c: c["people"][1].update(position=c["people"][0]["position"]),
    lambda c: c["people"][0].update(kind="ghost"),
    lambda c: c["people"][3].update(waypoints=[[0.0, 0.0]]),
    lambda c: c["people"][3].update(speed=9.0),
    # A space in an actor skin path hangs Gazebo rather than erroring.
    lambda c: c["people"][3].update(
        skin_uri="https://fuel.gazebosim.org/1.0/o/models/male%20visitor/1/files/m/w.dae"),
    lambda c: c["people"][3].update(
        skin_uri="https://fuel.gazebosim.org/1.0/o/models/male visitor/1/files/m/w.dae"),
    lambda c: c["hall"].update(name="floor"),
    lambda c: c["hall"].update(pose=[0, 0, 0]),
    lambda c: c["exhibits"][0].update(yaw=float("nan")),
    lambda c: c["nav_obstacles"][0].update(size=[0.0, 0.5]),
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


def test_every_sdf_pose_has_six_finite_values_and_hall_replaces_generated_walls():
    world = ET.fromstring(generate_world(deepcopy(CONFIG), _assets())).find("world")
    for pose in world.iter("pose"):
        values = [float(x) for x in pose.text.split()]
        assert len(values) == 6
        assert all(math.isfinite(x) for x in values)
    # The imported mesh carries the walls, so none are generated alongside it.
    for name in ("north", "south", "east", "west"):
        assert world.find(f"model[@name='wall_{name}']") is None
    hall = next(node for node in world.findall("include")
                if node.findtext("name") == CONFIG["hall"]["name"])
    assert hall.findtext("uri") == CONFIG["hall"]["model_uri"]
    assert hall.findtext("static") == "true"
    # The mesh has no floor of its own, so this world still supplies one.
    floor = world.find("model[@name='floor']")
    z = float(floor.findtext("pose").split()[2])
    thickness = float(floor.findtext("link/collision/geometry/box/size").split()[2])
    assert z + thickness / 2 == pytest.approx(CONFIG["room"]["floor_top"])


def test_box_room_fallback_still_builds_walls_resting_on_the_floor():
    config = deepcopy(CONFIG)
    config.pop("hall")
    world = ET.fromstring(generate_world(config, _assets())).find("world")
    for name in ("north", "south", "east", "west"):
        wall = world.find(f"model[@name='wall_{name}']")
        z = float(wall.findtext("pose").split()[2])
        height = float(wall.findtext("link/collision/geometry/box/size").split()[2])
        assert z - height / 2 == pytest.approx(config["room"]["floor_top"])


def test_static_visitors_are_collidable_includes_and_actors_loop_their_waypoints():
    world = ET.fromstring(generate_world(deepcopy(CONFIG), _assets())).find("world")
    people = {person["name"]: person for person in CONFIG["people"]}
    assert sum(1 for p in people.values() if p["kind"] == "static") >= 2
    assert sum(1 for p in people.values() if p["kind"] == "actor") >= 1

    for name, person in people.items():
        if person["kind"] == "static":
            node = next(n for n in world.findall("include") if n.findtext("name") == name)
            assert node.findtext("uri") == person["model_uri"]
            # Static so the lidar gets a stable return instead of a ragdoll.
            assert node.findtext("static") == "true"
            x, y, z, _roll, _pitch, yaw = map(float, node.findtext("pose").split())
            assert (x, y, z) == pytest.approx((*person["position"], 0.0))
            assert yaw == pytest.approx(person.get("yaw", 0.0))
            continue

        actor = world.find(f"actor[@name='{name}']")
        assert actor is not None
        assert actor.findtext("skin/filename") == person["skin_uri"]
        assert actor.findtext("script/loop") == "true"
        waypoints = actor.findall("script/trajectory/waypoint")
        # Closed loop: the first waypoint is repeated at the end.
        assert len(waypoints) == len(person["waypoints"]) + 1
        positions = [tuple(map(float, w.findtext("pose").split()))[:2] for w in waypoints]
        assert positions[:-1] == [tuple(p) for p in person["waypoints"]]
        assert positions[-1] == positions[0]
        times = [float(w.findtext("time")) for w in waypoints]
        assert times == sorted(times)
        assert times[0] == 0.0
        expected = sum(math.dist(positions[i], positions[i + 1])
                       for i in range(len(positions) - 1)) / person["speed"]
        assert times[-1] == pytest.approx(expected)


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
    extra.update(exhibit_id="E07", model_name="exhibit_extra",
                 position=[0.3, 10.95, 1.75], size=[.6, .1, 1.6], yaw=0.0)
    extra["station"].update(key="7", x=0.3, y=9.1)
    config["exhibits"].append(extra)
    world = ET.fromstring(generate_world(config, _assets())).find("world")
    assert world.find("model[@name='station_7']") is not None
