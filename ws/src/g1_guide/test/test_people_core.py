import math

import numpy as np

from g1_guide.nav_core import StaticGeometry
from g1_guide.people_core import (PeopleTracker, count_within, detect_people, face_yaw,
                                  height_ceiling, snapshot)

LIDAR_Z = 1.27
ROBOT = (0.0, 0.0)
CONFIG = {
    "room": {"width": 12.0, "depth": 12.0, "wall_thickness": 0.02, "floor_top": 0.0},
    "exhibits": [{"position": [4.0, 0.0], "size": [1.0, 1.0], "yaw": 0.0}],
    "people": [{"kind": "static", "position": [0.0, 2.0], "size": [0.5, 0.4], "yaw": 0.0}],
}


def person(x, y, height, radius=0.2):
    """What the head lidar sees of a standing cylinder: rows up to its ceiling."""
    top = min(height, height_ceiling(LIDAR_Z, math.hypot(x, y)))
    points = []
    for z in np.arange(0.2, top + 1e-9, 0.07):
        for angle in np.linspace(0, 2 * math.pi, 12, endpoint=False):
            points.append((x + radius * math.cos(angle), y + radius * math.sin(angle), z))
    return np.array(points)


def floor_and_walls():
    xs = np.linspace(-5.9, 5.9, 60)
    wall = [(x, 5.99, z) for x in xs for z in (0.5, 1.0, 1.4)]
    floor = [(x, y, 0.01) for x in xs[::4] for y in xs[::4]]
    exhibit = [(4.0 + dx, dy, 0.8) for dx in (-0.5, 0.5) for dy in np.linspace(-0.5, 0.5, 8)]
    return np.array(wall + floor + exhibit)


def static(include_people=False):
    return StaticGeometry(CONFIG, tolerance=0.15, include_people=include_people)


def test_two_people_apart_and_background_removed():
    cloud = np.vstack([floor_and_walls(), person(2.0, 1.0, 1.7), person(-1.5, -2.0, 1.7)])
    found = detect_people(cloud, static(), ROBOT, LIDAR_Z)
    assert len(found) == 2
    for x, y in ((2.0, 1.0), (-1.5, -2.0)):
        assert min(math.hypot(d.x - x, d.y - y) for d in found) < 0.1


def test_child_measured_in_full_adult_clipped():
    child, = detect_people(person(1.5, 0.0, 1.1), static(), ROBOT, LIDAR_Z)
    adult, = detect_people(person(2.0, 0.0, 1.7), static(), ROBOT, LIDAR_Z)
    assert abs(child.height - 1.1) < 0.08 and not child.height_clipped
    assert adult.height_clipped and adult.height > 1.3


def test_static_visitor_kept_for_people_but_not_for_navigation():
    cloud = person(0.0, 2.0, 1.7)
    assert len(detect_people(cloud, static(include_people=False), ROBOT, LIDAR_Z)) == 1
    assert detect_people(cloud, static(include_people=True), ROBOT, LIDAR_Z) == []


def test_self_returns_and_low_objects_ignored():
    arms = np.array([(0.3 * math.cos(a), 0.3 * math.sin(a), 1.0) for a in np.linspace(0, 6, 20)])
    pointing_hand = person(0.55, 0.0, 1.0, radius=0.05)
    door_frame = np.array([(3.0 + dx, -3.0, z) for dx in (0.0, 0.1) for z in np.arange(0.2, 2.31, 0.1)])
    bag = person(1.5, -1.0, 0.5)
    cloud = np.vstack([arms, pointing_hand, bag])
    assert detect_people(cloud, static(), ROBOT, LIDAR_Z) == []
    assert detect_people(door_frame, static(), ROBOT, 3.5) == []


def test_side_by_side_counts_as_group():
    cloud = np.vstack([person(2.0, -0.3, 1.7), person(2.0, 0.3, 1.7)])
    found = detect_people(cloud, static(), ROBOT, LIDAR_Z)
    assert sum(d.group for d in found) == 2


def test_tracker_keeps_id_while_walking_away_then_drops():
    tracker = PeopleTracker()
    ids = set()
    for step, x in enumerate(np.arange(1.5, 4.01, 0.1)):
        tracks = tracker.update(detect_people(person(x, 0.5, 1.1), static(), ROBOT, LIDAR_Z), step * 0.1)
        ids.update(t.track_id for t in tracks)
    assert ids == {1}
    payload = snapshot(tracker.tracks, (0.0, 0.0, 0.0), 2.5)
    assert payload["people"][0]["distance"] > 3.9 and count_within(payload) == 0
    assert tracker.update([], 2.6) and tracker.update([], 4.0) == []


def test_child_flag_needs_single_unclipped_short_person():
    tracker = PeopleTracker()
    tracker.update(detect_people(person(1.5, 0.0, 1.1), static(), ROBOT, LIDAR_Z), 0.0)
    child = snapshot(tracker.tracks, (0.0, 0.0, 0.0), 0.0)["people"][0]
    assert child["child"] and not child["height_clipped"]
    tracker = PeopleTracker()
    tracker.update(detect_people(person(2.0, 0.0, 1.7), static(), ROBOT, LIDAR_Z), 0.0)
    assert not snapshot(tracker.tracks, (0.0, 0.0, 0.0), 0.0)["people"][0]["child"]


def test_face_yaw_points_at_group_centre():
    tracker = PeopleTracker()
    cloud = np.vstack([person(1.0, 2.0, 1.7), person(2.0, 1.0, 1.7), person(-5.0, 0.0, 1.7)])
    tracker.update(detect_people(cloud, static(), ROBOT, LIDAR_Z), 0.0)
    payload = snapshot(tracker.tracks, (0.0, 0.0, math.pi), 0.0)
    assert count_within(payload, 3.0) == 2
    assert abs(math.degrees(face_yaw(payload, 3.0)) - 45.0) < 5.0
    bearing = {round(p["x"]): p["bearing_deg"] for p in payload["people"]}
    assert abs(abs(bearing[-5]) - 0.0) < 5.0


def test_nobody_gives_no_heading():
    assert face_yaw({"robot": [0, 0, 0], "people": []}) is None
