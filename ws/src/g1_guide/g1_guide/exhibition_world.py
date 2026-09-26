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

from .scene_config import world_footprint


_EPSILON = 1.0e-9
_FLOOR_THICKNESS = 0.10
_STATION_MARKER_RADIUS = 0.45
_POSTER_KINDS = {"poster"}
_SOLID_KINDS = {"book", "vase"}
_PERSON_KINDS = {"static", "actor"}
_PLINTH_COLOR = (0.86, 0.84, 0.80, 1.0)  # off-white museum plinth
_SUPPORTED_SCHEMAS = (1, 2)
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


def _rect(position: Sequence[float], size: Sequence[float],
          yaw: float = 0.0) -> tuple[float, float, float, float]:
    """World-aligned bounds of a box centred on ``position`` and turned by ``yaw``."""
    extent = world_footprint({"size": size, "yaw": yaw})
    if extent is None:
        raise ValueError("cannot compute a footprint for this size/yaw")
    half_x, half_y = extent[0] / 2.0, extent[1] / 2.0
    return (position[0] - half_x, position[0] + half_x,
            position[1] - half_y, position[1] + half_y)


def _angle(value: Any, path: str) -> float:
    result = _number(value, path)
    if abs(result) > 4.0 * math.pi:
        _fail(path, "must be an angle in radians")
    return result


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
    if type(root.get("schema_version")) is not int \
            or root["schema_version"] not in _SUPPORTED_SCHEMAS:
        _fail("schema_version", f"must be one of {_SUPPORTED_SCHEMAS}")
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

    model_names: set[str] = set(_RESERVED_MODEL_NAMES)
    hall = root.get("hall")
    if hall is not None:
        hall = _mapping(hall, "hall")
        hall_name = _identifier(hall.get("name"), "hall.name")
        if hall_name in model_names:
            _fail("hall.name", f"duplicate/reserved model name {hall_name!r}")
        model_names.add(hall_name)
        _text(hall.get("model_uri"), "hall.model_uri")
        _vector(hall.get("pose"), 6, "hall.pose")

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
    exhibits: list[tuple[Mapping[str, Any], list[float], list[float], float]] = []
    exhibit_ids: set[str] = set()
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
            _fail(f"{path}.kind", "must be poster, book, or vase")
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
        yaw = _angle(exhibit.get("yaw", 0.0), f"{path}.yaw")
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
        station_yaw = _number(station.get("yaw"), f"{path}.station.yaw")
        # A station that does not look at its exhibit would stop the robot
        # with the artwork behind it, so the tour geometry is checked here.
        facing = math.atan2(position[1] - station_y, position[0] - station_x)
        if abs((facing - station_yaw + math.pi) % (2.0 * math.pi) - math.pi) > math.radians(12.0):
            _fail(f"{path}.station.yaw", "does not face the exhibit")
        exhibits.append((exhibit, position, size, yaw))

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

    # Visitors.  Static ones are rigid models the lidar sees, so they get a
    # footprint and join the clearance checks; walking actors are animated
    # skins without collision and are only checked for a sane path.
    people_value = root.get("people", ())
    if not isinstance(people_value, Sequence) or isinstance(people_value, (str, bytes)):
        _fail("people", "must be a list")
    people: list[tuple[Mapping[str, Any], list[float], list[float]]] = []
    for index, raw_person in enumerate(people_value):
        path = f"people[{index}]"
        person = _mapping(raw_person, path)
        name = _identifier(person.get("name"), f"{path}.name")
        if name in model_names:
            _fail(f"{path}.name", f"duplicate/reserved model name {name!r}")
        model_names.add(name)
        kind = _text(person.get("kind"), f"{path}.kind")
        if kind not in _PERSON_KINDS:
            _fail(f"{path}.kind", "must be static or actor")
        if kind == "static":
            _text(person.get("model_uri"), f"{path}.model_uri")
            position = _vector(person.get("position"), 2, f"{path}.position")
            _angle(person.get("yaw", 0.0), f"{path}.yaw")
            size = _dimensions([*_vector(person.get("size"), 2, f"{path}.size"), 1.0],
                               f"{path}.size")
            people.append((person, position, size))
        else:
            skin = _text(person.get("skin_uri"), f"{path}.skin_uri")
            # A Fuel path containing a space hangs Gazebo's mesh resolver
            # instead of failing, which shows up as a world that never steps.
            if " " in skin or "%20" in skin:
                _fail(f"{path}.skin_uri", "must not contain spaces; pick a model "
                                          "whose Fuel path has none")
            speed = _number(person.get("speed", 1.0), f"{path}.speed")
            if not 0.1 <= speed <= 3.0:
                _fail(f"{path}.speed", "must be between 0.1 and 3.0 m/s")
            waypoints = person.get("waypoints")
            if not isinstance(waypoints, Sequence) or isinstance(waypoints, (str, bytes)) \
                    or len(waypoints) < 2:
                _fail(f"{path}.waypoints", "must contain at least two points")
            previous = None
            for step, raw_point in enumerate(waypoints):
                point = _vector(raw_point, 2, f"{path}.waypoints[{step}]")
                if previous is not None and math.dist(previous, point) < 0.5:
                    _fail(f"{path}.waypoints[{step}]",
                          "consecutive waypoints must be at least 0.5 m apart")
                previous = point

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

    # Baked hall geometry: rectangles the planner must respect but that no
    # model here draws, because the imported mesh already shows them.
    nav_value = root.get("nav_obstacles", ())
    if not isinstance(nav_value, Sequence) or isinstance(nav_value, (str, bytes)):
        _fail("nav_obstacles", "must be a list")
    hall_rectangles = []
    for index, raw_nav in enumerate(nav_value):
        path = f"nav_obstacles[{index}]"
        nav = _mapping(raw_nav, path)
        _text(nav.get("name"), f"{path}.name")
        position = _vector(nav.get("position"), 2, f"{path}.position")
        size = _vector(nav.get("size"), 2, f"{path}.size")
        if any(item <= 0.0 for item in size):
            _fail(f"{path}.size", "dimensions must be positive")
        hall_rectangles.append((path, _rect(position, size)))

    drawn_rectangles = [
        (f"exhibits[{index}]", _rect(position, size, yaw))
        for index, (_exhibit, position, size, yaw) in enumerate(exhibits)
    ] + [
        (f"obstacles[{index}]", _rect(position, size))
        for index, (_obstacle, position, size) in enumerate(obstacles)
    ] + [
        (f"people[{index}]", _rect(position, size))
        for index, (_person, position, size) in enumerate(people)
    ]
    for object_name, rectangle in drawn_rectangles:
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
        for index, (exhibit, _position, _size, _yaw) in enumerate(exhibits)
    )
    # Posters sit flush against hall walls on purpose, so the hall rectangles
    # take part in the standing-room checks but not in the overlap test.
    object_rectangles = drawn_rectangles
    for index, (name, rect) in enumerate(object_rectangles):
        for other_name, other in object_rectangles[index + 1:]:
            if min(rect[1], other[1]) > max(rect[0], other[0]) + _EPSILON \
                    and min(rect[3], other[3]) > max(rect[2], other[2]) + _EPSILON:
                _fail(name, f"footprint overlaps {other_name}")
    for point_name, x, y in safety_points:
        if x - radius <= boundary[0] or x + radius >= boundary[1] \
                or y - radius <= boundary[2] or y + radius >= boundary[3]:
            _fail(point_name, "clearance circle crosses the room boundary")
        for object_name, rectangle in object_rectangles + hall_rectangles:
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


def _textured_plane(name: str, size: Sequence[float], pose: Sequence[float], uri: str) -> str:
    """A visual-only printed surface.  Planes never get collision: a plane
    collision in Gazebo is infinite, so the solid block underneath collides."""
    return (f"<visual name=\"{escape(name)}_print\"><pose>{_pose(pose)}</pose>"
            f"<geometry><plane><normal>0 0 1</normal><size>{_pose(size)}</size></plane></geometry>"
            "<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><pbr><metal>"
            "<metalness>0</metalness><roughness>1</roughness>"
            f"<albedo_map>{escape(uri)}</albedo_map></metal></pbr></material></visual>")


def _book_model(exhibit: Mapping[str, Any], position: Sequence[float], textures: Path) -> str:
    """An open thread-bound album on a tilted wooden cradle, plus closed volumes.

    Modelled on how museums show woodblock books: pages nearly flat, paper a
    few centimetres thick, printed pages from a real scan.  The viewer stands
    on the -y side, so the cradle rises towards +y.  Page and cover textures
    are ``<exhibit_id>_left/_right/_cover.png`` in the posters directory.
    """
    exhibit_id = str(exhibit["exhibit_id"])
    uris = {}
    for part in ("left", "right", "cover"):
        path = textures / f"{exhibit_id}_{part}.png"
        if not path.is_file():
            raise FileNotFoundError(f"book texture does not exist: {path}")
        uris[part] = path.resolve().as_uri()

    top = float(exhibit["size"][2]) / 2.0
    tilt = math.radians(15.0)
    # Everything stays inside the smallest allowed plinth (1.2 m x 1.0 m).
    cradle_x, cradle_z = -0.14, top + 0.1144  # board centre; its front edge rests 3 cm above the plinth
    page = (0.27, 0.445)                      # width:height 0.607, the woodblock page ratio
    wood = _material((0.36, 0.22, 0.12, 1.0))
    paper = _material((0.86, 0.80, 0.66, 1.0))
    cover = _material((0.76, 0.66, 0.50, 1.0))

    def on_cradle(x: float, y: float, z: float) -> tuple[float, ...]:
        """Pose of a point given in the tilted cradle frame."""
        return (cradle_x + x, y * math.cos(tilt) - z * math.sin(tilt),
                cradle_z + y * math.sin(tilt) + z * math.cos(tilt), tilt, 0.0, 0.0)

    components = [
        ("pedestal", _geometry_box(exhibit["size"]), (0.0, 0.0, 0.0), _material(_PLINTH_COLOR)),
        ("cradle_board", _geometry_box((0.66, 0.54, 0.03)), on_cradle(0.0, 0.0, 0.0), wood),
        ("cradle_lip", _geometry_box((0.66, 0.02, 0.03)), on_cradle(0.0, -0.26, 0.03), wood),
        ("cradle_back_leg", _geometry_box((0.60, 0.05, 0.157)),
         (cradle_x, 0.2164, top + 0.0784, 0.0, 0.0, 0.0), wood),
        ("cradle_front_leg", _geometry_box((0.60, 0.05, 0.043)),
         (cradle_x, -0.2086, top + 0.0215, 0.0, 0.0, 0.0), wood),
        ("open_cover", _geometry_box((0.58, 0.47, 0.004)), on_cradle(0.0, 0.005, 0.017), cover),
        ("pages_left", _geometry_box((page[0], page[1], 0.02)), on_cradle(-0.1355, 0.005, 0.029), paper),
        ("pages_right", _geometry_box((page[0], page[1], 0.02)), on_cradle(0.1355, 0.005, 0.029), paper),
    ]
    # Four closed volumes, slightly askew like a set laid out beside the open one.
    for index, (dx, dy, yaw) in enumerate(((0.0, 0.0, 0.20), (0.006, -0.004, 0.17),
                                            (-0.004, 0.005, 0.22), (0.003, 0.002, 0.19))):
        components.append((f"volume_{index}", _geometry_box((page[0], page[1], 0.012)),
                           (0.40 + dx, 0.05 + dy, top + 0.006 + 0.012 * index, 0.0, 0.0, yaw),
                           paper if index % 2 else cover))
    pieces = [_paired_primitive(name, geometry, pose, material)
              for name, geometry, pose, material in components]
    pieces.append(_textured_plane("page_left", page, on_cradle(-0.1355, 0.005, 0.0395), uris["left"]))
    pieces.append(_textured_plane("page_right", page, on_cradle(0.1355, 0.005, 0.0395), uris["right"]))
    pieces.append(_textured_plane("volume_cover", page,
                                  (0.403, 0.052, top + 0.048 + 0.0005, 0.0, 0.0, 0.19), uris["cover"]))
    return _model(str(exhibit["model_name"]), (*position[:2], position[2], 0.0, 0.0, 0.0),
                  f"<link name=\"link\">{''.join(pieces)}</link>")


def _vase_model(exhibit: Mapping[str, Any], position: Sequence[float]) -> str:
    """A blue-and-white porcelain vase (yuhuchun shape) on a wooden stand.

    Built from spheres and cylinders like the rest of the hall, so it needs no
    mesh files.  Blue bands and flower dots stand in for painted decoration;
    everything stays inside the pedestal footprint with matching collision.
    """
    top = float(exhibit["size"][2]) / 2.0
    porcelain = _material((0.94, 0.95, 0.96, 1.0))
    cobalt = _material((0.10, 0.20, 0.58, 1.0))
    components = [
        ("pedestal", _geometry_box(exhibit["size"]), (0.0, 0.0, 0.0), _material(_PLINTH_COLOR)),
        ("stand", _geometry_cylinder(0.16, 0.04), (0.0, 0.0, top + 0.02),
         _material((0.22, 0.12, 0.06, 1.0))),
        ("foot", _geometry_cylinder(0.10, 0.05), (0.0, 0.0, top + 0.065), porcelain),
        ("body", _geometry_sphere(0.20), (0.0, 0.0, top + 0.27), porcelain),
        ("shoulder", _geometry_sphere(0.09), (0.0, 0.0, top + 0.46), porcelain),
        ("neck", _geometry_cylinder(0.05, 0.26), (0.0, 0.0, top + 0.56), porcelain),
        ("mouth", _geometry_cylinder(0.085, 0.025), (0.0, 0.0, top + 0.70), porcelain),
        ("band_equator", _geometry_cylinder(0.203, 0.03), (0.0, 0.0, top + 0.27), cobalt),
        ("band_lower", _geometry_cylinder(0.156, 0.015), (0.0, 0.0, top + 0.14), cobalt),
        ("band_upper", _geometry_cylinder(0.163, 0.015), (0.0, 0.0, top + 0.39), cobalt),
        ("band_neck", _geometry_cylinder(0.053, 0.02), (0.0, 0.0, top + 0.62), cobalt),
        ("band_rim", _geometry_cylinder(0.088, 0.008), (0.0, 0.0, top + 0.708), cobalt),
    ]
    # Two staggered rings of flower dots above and below the equator band.
    for ring, (dz, offset) in enumerate(((0.07, 0.0), (-0.07, math.pi / 8))):
        for index in range(8):
            angle = offset + index * math.pi / 4
            components.append((f"flower_{ring}_{index}", _geometry_sphere(0.028),
                               (0.18 * math.cos(angle), 0.18 * math.sin(angle), top + 0.27 + dz),
                               cobalt))
    pieces = []
    for name, geometry, local_pose, material in components:
        if len(local_pose) == 3:
            local_pose = (*local_pose, 0.0, 0.0, 0.0)
        pieces.append(_paired_primitive(name, geometry, local_pose, material))
    return _model(str(exhibit["model_name"]), (*position[:2], position[2], 0.0, 0.0, 0.0),
                  f"<link name=\"link\">{''.join(pieces)}</link>")


def _hall_model(hall: Mapping[str, Any]) -> str:
    """Include the imported hall mesh; it supplies the walls this world skips."""
    return (f"<include><name>{escape(str(hall['name']))}</name>"
            f"<uri>{escape(str(hall['model_uri']))}</uri>"
            f"<pose>{_pose(hall['pose'])}</pose><static>true</static></include>")


def _static_person_model(person: Mapping[str, Any]) -> str:
    """A rigid visitor: collision included, so the lidar and planner see them."""
    x, y = person["position"]
    yaw = float(person.get("yaw", 0.0))
    return (f"<include><name>{escape(str(person['name']))}</name>"
            f"<uri>{escape(str(person['model_uri']))}</uri>"
            f"<pose>{_pose((x, y, 0.0, 0.0, 0.0, yaw))}</pose>"
            "<static>true</static></include>")


def _actor_person(person: Mapping[str, Any]) -> str:
    """A walking visitor: a skinned animation looping through its waypoints.

    Actors carry no collision in Gazebo, so these are seen by the cameras but
    not by the lidar; the static visitors are what the planner avoids.
    """
    skin = escape(str(person["skin_uri"]))
    speed = float(person.get("speed", 1.0))
    points = [(float(x), float(y)) for x, y in person["waypoints"]]
    closed = points + [points[0]]
    waypoints = []
    time = 0.0
    for index, (x, y) in enumerate(closed):
        nxt = closed[(index + 1) % len(closed)]
        heading = math.atan2(nxt[1] - y, nxt[0] - x)
        waypoints.append(f"<waypoint><time>{_fmt(time)}</time>"
                         f"<pose>{_pose((x, y, 0.0, 0.0, 0.0, heading))}</pose></waypoint>")
        time += math.dist((x, y), nxt) / speed
    return (f"<actor name=\"{escape(str(person['name']))}\">"
            f"<skin><filename>{skin}</filename><scale>1.0</scale></skin>"
            f"<animation name=\"walk\"><filename>{skin}</filename>"
            "<scale>1.0</scale><interpolate_x>true</interpolate_x></animation>"
            "<script><loop>true</loop><delay_start>0.0</delay_start>"
            "<auto_start>true</auto_start>"
            f"<trajectory id=\"0\" type=\"walk\">{''.join(waypoints)}</trajectory>"
            "</script></actor>")


def _poster_model(exhibit: Mapping[str, Any], position: Sequence[float],
                  size: Sequence[float], asset_uri: str, yaw: float = 0.0) -> str:
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
        str(exhibit["model_name"]), (*position[:2], position[2], 0.0, 0.0, yaw),
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
    hall = root.get("hall")
    if hall is None:
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
    else:
        lines.append("    " + _hall_model(hall))

    for exhibit in root["exhibits"]:
        position = exhibit["position"]
        size = exhibit["size"]
        kind = exhibit["kind"]
        if kind == "poster":
            asset_path = posters_root / str(exhibit["asset"])
            if not asset_path.is_file():
                raise FileNotFoundError(f"poster asset does not exist: {asset_path}")
            asset_uri = asset_path.resolve().as_uri()
            model = _poster_model(exhibit, position, size, asset_uri,
                                  float(exhibit.get("yaw", 0.0)))
        elif kind == "book":
            model = _book_model(exhibit, position, posters_root)
        else:
            model = _vase_model(exhibit, position)
        lines.append("    " + model)

    for obstacle in root["obstacles"]:
        color = obstacle.get("color", [0.45, 0.45, 0.48, 1.0])
        lines.append("    " + _box_model(str(obstacle["name"]), obstacle["position"],
                                          obstacle["size"], _material(color)))

    for person in root.get("people", ()):
        lines.append("    " + (_static_person_model(person) if person["kind"] == "static"
                               else _actor_person(person)))

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
