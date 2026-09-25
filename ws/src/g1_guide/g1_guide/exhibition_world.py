"""Generate and validate the parameterised exhibition Gazebo world.

The scene is deliberately generated as SDF text rather than assembled from
Gazebo services.  This keeps the world reproducible and lets the launch file
write an isolated world for every run.
"""

from __future__ import annotations

from pathlib import Path
import math
import re
from typing import Any, Mapping, Sequence
from xml.sax.saxutils import escape


_EPSILON = 1.0e-9
_FLOOR_THICKNESS = 0.10
_STATION_MARKER_RADIUS = 0.45
_POSTER_KINDS = {"poster"}
_SOLID_KINDS = {"book", "globe"}
_IDENTIFIER = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
_RESERVED_MODEL_NAMES = {"floor", "wall_north", "wall_south", "wall_east",
                         "wall_west", "exhibition_overview", "g1_glide"}


def _fail(path: str, message: str) -> None:
    raise ValueError(f"{path}: {message}")


def _number(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        _fail(path, "must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        _fail(path, "must be a finite number")
    return result


def _vector(value: Any, length: int, path: str) -> list[float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        _fail(path, f"must contain {length} numbers")
    if len(value) != length:
        _fail(path, f"must contain {length} numbers")
    return [_number(item, f"{path}[{index}]") for index, item in enumerate(value)]


def _dimensions(value: Any, path: str) -> list[float]:
    result = _vector(value, 3, path)
    if any(item <= 0.0 for item in result):
        _fail(path, "dimensions must be positive")
    return result


def _text(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value.strip():
        _fail(path, "must be a non-empty string")
    return value


def _identifier(value: Any, path: str) -> str:
    result = _text(value, path)
    if _IDENTIFIER.fullmatch(result) is None:
        _fail(path, "must be an SDF identifier (letters, digits, and underscores only)")
    return result


def _mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        _fail(path, "must be an object")
    return value


def _rect(position: Sequence[float], size: Sequence[float]) -> tuple[float, float, float, float]:
    half_x, half_y = size[0] / 2.0, size[1] / 2.0
    return (position[0] - half_x, position[0] + half_x,
            position[1] - half_y, position[1] + half_y)


def _circle_intersects_rect(x: float, y: float, radius: float,
                            rect: tuple[float, float, float, float]) -> bool:
    left, right, bottom, top = rect
    nearest_x = min(max(x, left), right)
    nearest_y = min(max(y, bottom), top)
    distance_x = x - nearest_x
    distance_y = y - nearest_y
    return distance_x * distance_x + distance_y * distance_y <= radius * radius + _EPSILON


def validate_layout(config: dict) -> None:
    """Validate the exhibition schema and all 2-D navigation clearances.

    A valid layout returns ``None``.  Invalid or unsafe layouts raise
    ``ValueError`` with the offending configuration path in the message.
    The clearance test treats touching a wall or object as unsafe.
    """

    root = _mapping(config, "config")
    if type(root.get("schema_version")) is not int or root["schema_version"] != 1:
        _fail("schema_version", "must be integer 1")
    room = _mapping(root.get("room"), "room")
    width = _number(room.get("width"), "room.width")
    depth = _number(room.get("depth"), "room.depth")
    wall_height = _number(room.get("wall_height"), "room.wall_height")
    wall_thickness = _number(room.get("wall_thickness"), "room.wall_thickness")
    floor_top = _number(room.get("floor_top"), "room.floor_top")
    for name, value in (("width", width), ("depth", depth),
                        ("wall_height", wall_height), ("wall_thickness", wall_thickness)):
        if value <= 0.0:
            _fail(f"room.{name}", "must be positive")
    if wall_thickness >= min(width, depth):
        _fail("room.wall_thickness", "must be smaller than the room dimensions")

    world_name = _text(root.get("world_name"), "world_name")
    if world_name != "exhibition":
        _fail("world_name", "must be 'exhibition'")

    spawn = _mapping(root.get("spawn"), "spawn")
    spawn_position = [_number(spawn.get(name), f"spawn.{name}")
                     for name in ("x", "y", "z", "yaw")]
    camera = _mapping(root.get("overview_camera"), "overview_camera")
    _vector(camera.get("pose"), 6, "overview_camera.pose")

    exhibits_value = root.get("exhibits")
    if not isinstance(exhibits_value, Sequence) or isinstance(exhibits_value, (str, bytes)):
        _fail("exhibits", "must be a list")
    if len(exhibits_value) < 6:
        _fail("exhibits", "must contain at least six exhibits")
    exhibits: list[tuple[Mapping[str, Any], list[float], list[float]]] = []
    exhibit_ids: set[str] = set()
    model_names: set[str] = set(_RESERVED_MODEL_NAMES)
    station_keys: set[str] = set()
    posters = 0
    solids = 0
    for index, raw_exhibit in enumerate(exhibits_value):
        path = f"exhibits[{index}]"
        exhibit = _mapping(raw_exhibit, path)
        exhibit_id = _identifier(exhibit.get("exhibit_id"), f"{path}.exhibit_id")
        if exhibit_id in exhibit_ids:
            _fail(f"{path}.exhibit_id", f"duplicate exhibit ID {exhibit_id!r}")
        exhibit_ids.add(exhibit_id)
        model_name = _identifier(exhibit.get("model_name"), f"{path}.model_name")
        if model_name in model_names:
            _fail(f"{path}.model_name", f"duplicate model name {model_name!r}")
        model_names.add(model_name)
        kind = _text(exhibit.get("kind"), f"{path}.kind")
        if kind not in _POSTER_KINDS | _SOLID_KINDS:
            _fail(f"{path}.kind", "must be poster, book, or globe")
        if kind in _POSTER_KINDS:
            posters += 1
            asset = _text(exhibit.get("asset"), f"{path}.asset")
            asset_path = Path(asset)
            if asset_path.is_absolute() or asset_path.name != asset:
                _fail(f"{path}.asset", "must be a filename inside the posters directory")
        else:
            solids += 1
        position = _vector(exhibit.get("position"), 3, f"{path}.position")
        size = _dimensions(exhibit.get("size"), f"{path}.size")
        if kind in _SOLID_KINDS and (size[0] < 1.2 or size[1] < 1.0):
            _fail(f"{path}.size", "solid exhibit footprint must be at least 1.2 m x 1.0 m")
        if position[2] - size[2] / 2.0 < floor_top - _EPSILON:
            _fail(path, "exhibit extends below the floor top")
        if kind in _SOLID_KINDS and abs(position[2] - size[2] / 2.0 - floor_top) > _EPSILON:
            _fail(path, "solid exhibit pedestal must rest on the floor")
        station = _mapping(exhibit.get("station"), f"{path}.station")
        key = _text(station.get("key"), f"{path}.station.key")
        if key in station_keys:
            _fail(f"{path}.station.key", f"duplicate station key {key!r}")
        station_keys.add(key)
        station_x = _number(station.get("x"), f"{path}.station.x")
        station_y = _number(station.get("y"), f"{path}.station.y")
        _number(station.get("yaw"), f"{path}.station.yaw")
        exhibits.append((exhibit, position, size))

    if posters < 4:
        _fail("exhibits", "must contain at least four poster exhibits")
    if solids < 2:
        _fail("exhibits", "must contain at least two solid exhibits")
    if station_keys != {str(index) for index in range(1, len(exhibits) + 1)}:
        _fail("station.key", "must be consecutive numeric strings 1..N (N >= 6)")

    obstacles_value = root.get("obstacles")
    if not isinstance(obstacles_value, Sequence) or isinstance(obstacles_value, (str, bytes)):
        _fail("obstacles", "must be a list")
    obstacles: list[tuple[Mapping[str, Any], list[float], list[float]]] = []
    obstacle_names: set[str] = set()
    for index, raw_obstacle in enumerate(obstacles_value):
        path = f"obstacles[{index}]"
        obstacle = _mapping(raw_obstacle, path)
        name = _identifier(obstacle.get("name"), f"{path}.name")
        if name in obstacle_names:
            _fail(f"{path}.name", f"duplicate obstacle name {name!r}")
        if name in model_names:
            _fail(f"{path}.name", f"duplicate/reserved model name {name!r}")
        obstacle_names.add(name)
        model_names.add(name)
        position = _vector(obstacle.get("position"), 3, f"{path}.position")
        size = _dimensions(obstacle.get("size"), f"{path}.size")
        if position[2] - size[2] / 2.0 < floor_top - _EPSILON:
            _fail(path, "obstacle extends below the floor top")
        color = _vector(obstacle.get("color"), 4, f"{path}.color")
        if any(component < 0.0 or component > 1.0 for component in color):
            _fail(f"{path}.color", "components must be in [0, 1]")
        obstacles.append((obstacle, position, size))

    # Use the inner face of the wall as the navigable room boundary.  The
    # station/spawn circle must fit fully inside this boundary.
    inner_x = width / 2.0 - wall_thickness / 2.0
    inner_y = depth / 2.0 - wall_thickness / 2.0
    radius = _number(root.get("robot_clearance_radius"), "robot_clearance_radius")
    if radius <= 0.0:
        _fail("robot_clearance_radius", "must be positive")
    if inner_x <= radius or inner_y <= radius:
        _fail("robot_clearance_radius", "does not fit inside the room")
    boundary = (-inner_x, inner_x, -inner_y, inner_y)

    for object_name, rectangle in (
        [(f"exhibits[{index}]", _rect(position, size))
         for index, (_exhibit, position, size) in enumerate(exhibits)]
        + [(f"obstacles[{index}]", _rect(position, size))
           for index, (_obstacle, position, size) in enumerate(obstacles)]
    ):
        left, right, bottom, top = rectangle
        if left < boundary[0] or right > boundary[1] \
                or bottom < boundary[2] or top > boundary[3]:
            _fail(object_name, "model footprint must be inside the room boundary")

    for exhibit in exhibits:
        station_model_name = f"station_{exhibit[0]['station']['key']}"
        _identifier(station_model_name, "station.model_name")
        if station_model_name in model_names:
            _fail("station.key", f"duplicate/reserved model name {station_model_name!r}")
        model_names.add(station_model_name)

    safety_points: list[tuple[str, float, float]] = [("spawn", spawn_position[0], spawn_position[1])]
    safety_points.extend(
        (f"exhibits[{index}].station", _number(exhibit["station"]["x"], "station.x"),
         _number(exhibit["station"]["y"], "station.y"))
        for index, (exhibit, _position, _size) in enumerate(exhibits)
    )
    object_rectangles = [
        (f"exhibits[{index}]", _rect(position, size))
        for index, (_exhibit, position, size) in enumerate(exhibits)
    ] + [
        (f"obstacles[{index}]", _rect(position, size))
        for index, (_obstacle, position, size) in enumerate(obstacles)
    ]
    for index, (name, rect) in enumerate(object_rectangles):
        for other_name, other in object_rectangles[index + 1:]:
            if min(rect[1], other[1]) > max(rect[0], other[0]) + _EPSILON \
                    and min(rect[3], other[3]) > max(rect[2], other[2]) + _EPSILON:
                _fail(name, f"footprint overlaps {other_name}")
    for point_name, x, y in safety_points:
        if x - radius <= boundary[0] or x + radius >= boundary[1] \
                or y - radius <= boundary[2] or y + radius >= boundary[3]:
            _fail(point_name, "clearance circle crosses the room boundary")
        for object_name, rectangle in object_rectangles:
            if _circle_intersects_rect(x, y, radius, rectangle):
                _fail(point_name, f"clearance circle intersects {object_name}")


def _fmt(value: float) -> str:
    return f"{float(value):.9g}"


def _pose(values: Sequence[float]) -> str:
    return " ".join(_fmt(item) for item in values)


def _material(color: Sequence[float], *, ambient: bool = True) -> str:
    ambient_xml = f"<ambient>{_pose(color)}</ambient>" if ambient else ""
    return f"<material>{ambient_xml}<diffuse>{_pose(color)}</diffuse></material>"


def _geometry_box(size: Sequence[float]) -> str:
    return f"<geometry><box><size>{_pose(size)}</size></box></geometry>"


def _geometry_cylinder(radius: float, length: float) -> str:
    return (f"<geometry><cylinder><radius>{_fmt(radius)}</radius>"
            f"<length>{_fmt(length)}</length></cylinder></geometry>")


def _geometry_sphere(radius: float) -> str:
    return f"<geometry><sphere><radius>{_fmt(radius)}</radius></sphere></geometry>"


def _paired_primitive(name: str, geometry: str, pose: Sequence[float],
                      material: str) -> str:
    local_pose = f"<pose>{_pose(pose)}</pose>" if any(abs(item) > _EPSILON for item in pose) else ""
    return (f"<collision name=\"{escape(name)}_collision\">{local_pose}{geometry}</collision>"
            f"<visual name=\"{escape(name)}_visual\">{local_pose}{geometry}{material}</visual>")


def _model(name: str, pose: Sequence[float], body: str, *, static: bool = True) -> str:
    if len(pose) == 3:
        pose = (*pose, 0.0, 0.0, 0.0)
    if len(pose) != 6:
        raise ValueError("SDF model pose requires xyz or xyz/rpy")
    return (f"<model name=\"{escape(name)}\"><static>{str(static).lower()}</static>"
            f"<pose>{_pose(pose)}</pose>{body}</model>")


def _box_model(name: str, position: Sequence[float], size: Sequence[float],
               material: str, *, collision: bool = True) -> str:
    geometry = _geometry_box(size)
    collision_xml = f"<collision name=\"collision\">{geometry}</collision>" if collision else ""
    return _model(
        name, position,
        f"<link name=\"link\">{collision_xml}"
        f"<visual name=\"visual\">{geometry}{material}</visual></link>",
    )


def _book_model(exhibit: Mapping[str, Any], position: Sequence[float]) -> str:
    # Every visible primitive has an identical collision primitive.  The
    # split covers and pages make this unmistakably an open book, while the
    # low pedestal keeps the exhibit stable without hiding the book geometry.
    top = float(exhibit["size"][2]) / 2.0
    components = [
        ("pedestal", _geometry_box(exhibit["size"]), (0.0, 0.0, 0.0),
         _material((0.25, 0.10, 0.035, 1.0))),
        ("book_cover_left", _geometry_box((0.56, 0.78, 0.07)), (-0.28, 0.0, top + 0.085, 0.0, -0.18, 0.0),
         _material((0.50, 0.08, 0.05, 1.0))),
        ("book_cover_right", _geometry_box((0.56, 0.78, 0.07)), (0.28, 0.0, top + 0.085, 0.0, 0.18, 0.0),
         _material((0.50, 0.08, 0.05, 1.0))),
        ("book_pages_left", _geometry_box((0.51, 0.70, 0.12)), (-0.25, 0.0, top + 0.19, 0.0, -0.18, 0.0),
         _material((0.92, 0.84, 0.60, 1.0))),
        ("book_pages_right", _geometry_box((0.51, 0.70, 0.12)), (0.25, 0.0, top + 0.19, 0.0, 0.18, 0.0),
         _material((0.92, 0.84, 0.60, 1.0))),
        ("book_spine", _geometry_box((0.10, 0.76, 0.09)), (0.0, 0.0, top + 0.14),
         _material((0.35, 0.04, 0.03, 1.0))),
    ]
    pieces = []
    for name, geometry, local_pose, material in components:
        if len(local_pose) == 3:
            local_pose = (*local_pose, 0.0, 0.0, 0.0)
        pieces.append(_paired_primitive(name, geometry, local_pose, material))
    return _model(str(exhibit["model_name"]), (*position[:2], position[2], 0.0, 0.0, 0.0),
                  f"<link name=\"link\">{''.join(pieces)}</link>")


def _globe_model(exhibit: Mapping[str, Any], position: Sequence[float]) -> str:
    top = float(exhibit["size"][2]) / 2.0
    components = [
        ("pedestal", _geometry_box(exhibit["size"]), (0.0, 0.0, 0.0),
         _material((0.12, 0.14, 0.18, 1.0))),
        ("globe_stem", _geometry_cylinder(0.045, 0.40), (0.0, 0.0, top + 0.20),
         _material((0.55, 0.58, 0.63, 1.0))),
        ("globe_sphere", _geometry_sphere(0.28), (0.0, 0.0, top + 0.50),
         _material((0.04, 0.24, 0.62, 1.0))),
        ("globe_axis", _geometry_cylinder(0.018, 0.72), (0.0, 0.0, top + 0.50, 0.35, 0.0, 0.0),
         _material((0.78, 0.80, 0.84, 1.0))),
        ("equator", _geometry_cylinder(0.283, 0.006), (0.0, 0.0, top + 0.50),
         _material((0.85, 0.85, 0.55, 1.0))),
    ]
    # Stylised land patches, not geographic data. All remain inside the
    # pedestal footprint and carry identical collision and visible geometry.
    for index, (longitude, latitude, radius) in enumerate([
        (-115, 30, .075), (-100, 12, .07), (-85, -8, .065), (-78, -25, .055),
        (-40, 38, .065), (-28, 20, .075), (70, 0, .08), (90, -18, .06),
    ]):
        lon, lat = math.radians(longitude), math.radians(latitude)
        r = 0.25
        components.append((f"land_{index}", _geometry_sphere(radius),
                           (r * math.cos(lat) * math.cos(lon),
                            r * math.cos(lat) * math.sin(lon), top + .50 + r * math.sin(lat)),
                           _material((.13, .55, .22, 1))))
    pieces = []
    for name, geometry, local_pose, material in components:
        if len(local_pose) == 3:
            local_pose = (*local_pose, 0.0, 0.0, 0.0)
        pieces.append(_paired_primitive(name, geometry, local_pose, material))
    return _model(str(exhibit["model_name"]), (*position[:2], position[2], 0.0, 0.0, 0.0),
                  f"<link name=\"link\">{''.join(pieces)}</link>")


def _poster_model(exhibit: Mapping[str, Any], position: Sequence[float],
                  size: Sequence[float], asset_uri: str) -> str:
    board_geometry = _geometry_box(size)
    plane_size = (size[0], size[2])
    plane_y = -size[1] / 2.0 - 0.002
    plane_pose = f"<pose>0 {_fmt(plane_y)} 0 0 0 0</pose>"
    poster_visual = (
        "<visual name=\"poster_visual\">"
        f"{plane_pose}<geometry><plane><normal>0 -1 0</normal>"
        f"<size>{_pose(plane_size)}</size></plane></geometry>"
        "<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><pbr><metal>"
        "<metalness>0</metalness><roughness>1</roughness>"
        f"<albedo_map>{escape(asset_uri)}</albedo_map>"
        "</metal></pbr></material></visual>"
    )
    backboard_visual = (
        f"<visual name=\"backboard_visual\">{board_geometry}"
        f"{_material((0.18, 0.20, 0.24, 1.0))}</visual>"
    )
    return _model(
        str(exhibit["model_name"]), (*position[:2], position[2], 0.0, 0.0, 0.0),
        f"<link name=\"link\"><collision name=\"collision\">{board_geometry}</collision>"
        f"{backboard_visual}{poster_visual}</link>",
    )


def generate_world(config: dict, asset_root: Path) -> str:
    """Return a complete SDF world for ``config``.

    ``asset_root`` is the directory containing the ``posters`` directory.
    Poster paths are resolved to absolute ``file://`` URIs and are required to
    exist, so a generated world cannot silently render missing artwork.
    """

    validate_layout(config)
    root = _mapping(config, "config")
    room = _mapping(root["room"], "room")
    width = float(room["width"])
    depth = float(room["depth"])
    wall_height = float(room["wall_height"])
    wall_thickness = float(room["wall_thickness"])
    floor_top = float(room["floor_top"])
    floor_center = floor_top - _FLOOR_THICKNESS / 2.0
    asset_root = Path(asset_root).expanduser().resolve()
    posters_root = asset_root / "posters"

    lines = [
        '<?xml version="1.0"?>',
        '<sdf version="1.9">',
        f'  <world name="{escape(str(root["world_name"]))}">',
        '    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>',
        '    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>',
        '    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>',
        '    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">',
        '      <render_engine>ogre2</render_engine>',
        '    </plugin>',
        '    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>',
        '    <scene><ambient>0.30 0.30 0.34 1</ambient>',
        '      <background>0.055 0.07 0.10 1</background><shadows>true</shadows></scene>',
        '    <light name="exhibition_sun" type="directional">',
        '      <cast_shadows>true</cast_shadows><pose>0 0 8 0.35 0.2 -0.5</pose>',
        '      <diffuse>0.92 0.92 0.92 1</diffuse><specular>0.25 0.25 0.25 1</specular>',
        '      <direction>-0.25 0.1 -1</direction>',
        '    </light>',
    ]
    floor_size = (width, depth, _FLOOR_THICKNESS)
    lines.append("    " + _box_model("floor", (0.0, 0.0, floor_center), floor_size,
                                      _material((0.32, 0.34, 0.38, 1.0))))
    wall_z = floor_top + wall_height / 2.0
    walls = (
        ("wall_north", (0.0, depth / 2.0, wall_z), (width, wall_thickness, wall_height)),
        ("wall_south", (0.0, -depth / 2.0, wall_z), (width, wall_thickness, wall_height)),
        ("wall_east", (width / 2.0, 0.0, wall_z), (wall_thickness, depth, wall_height)),
        ("wall_west", (-width / 2.0, 0.0, wall_z), (wall_thickness, depth, wall_height)),
    )
    for name, position, size in walls:
        lines.append("    " + _box_model(name, position, size,
                                          _material((0.64, 0.67, 0.72, 1.0), ambient=False)))

    for exhibit in root["exhibits"]:
        position = exhibit["position"]
        size = exhibit["size"]
        kind = exhibit["kind"]
        if kind == "poster":
            asset_path = posters_root / str(exhibit["asset"])
            if not asset_path.is_file():
                raise FileNotFoundError(f"poster asset does not exist: {asset_path}")
            asset_uri = asset_path.resolve().as_uri()
            model = _poster_model(exhibit, position, size, asset_uri)
        elif kind == "book":
            model = _book_model(exhibit, position)
        else:
            model = _globe_model(exhibit, position)
        lines.append("    " + model)

    for obstacle in root["obstacles"]:
        color = obstacle.get("color", [0.45, 0.45, 0.48, 1.0])
        lines.append("    " + _box_model(str(obstacle["name"]), obstacle["position"],
                                          obstacle["size"], _material(color)))

    # Station markers are intentionally visual-only: navigation targets must
    # not become extra collision obstacles.
    station_color = (0.10, 0.85, 0.82, 0.75)
    for exhibit in root["exhibits"]:
        station = exhibit["station"]
        marker_name = f"station_{station['key']}"
        marker_pose = (float(station["x"]), float(station["y"]), floor_top + 0.006)
        marker_geometry = _geometry_cylinder(_STATION_MARKER_RADIUS, 0.012)
        marker_body = (f"<link name=\"link\"><visual name=\"marker\">"
                       f"{marker_geometry}{_material(station_color)}</visual></link>")
        lines.append("    " + _model(marker_name, marker_pose, marker_body))

    overview_pose = _vector(root["overview_camera"]["pose"], 6, "overview_camera.pose")
    camera_model = (
        '<model name="exhibition_overview"><static>true</static>'
        f"<pose>{_pose(overview_pose)}</pose>"
        '<link name="link"><pose>0 0 0 0 0 0</pose>'
        '<sensor name="overview_camera" type="camera"><pose>0 0 0 0 0 0</pose>'
        '<always_on>true</always_on><visualize>true</visualize><update_rate>2</update_rate>'
        '<topic>/exhibition/overview</topic><camera><horizontal_fov>1.35</horizontal_fov>'
        '<image><width>960</width><height>720</height><format>R8G8B8</format></image>'
        '<clip><near>0.1</near><far>100</far></clip></camera></sensor></link></model>'
    )
    lines.append("    " + camera_model)
    lines.extend(["  </world>", "</sdf>", ""])
    return "\n".join(lines)


__all__ = ["generate_world", "validate_layout"]
