"""Path shortcutting, early replanning and obstacle memory (smoother, loop-free travel)."""

import math

from g1_guide.nav_control import TURN_IN_PLACE_ERROR, Pose2D, compute_command
from g1_guide.nav_core import MEMORY_S, NavigatorCore
from g1_guide.nav_grid import OccupancyGrid, _DynamicPointIndex

ROOM = {"room": {"width": 16, "depth": 12, "wall_thickness": .15},
        "robot_clearance_radius": 0.55, "obstacles": [], "exhibits": []}


def test_shortcut_keeps_only_real_corners():
    config = dict(ROOM, obstacles=[{"position": [0.0, 0.0], "size": [1.0, 4.0]}])
    grid = OccupancyGrid(config, extra_margin=0.30)
    path = grid.plan((-3.0, 0.0), (3.0, 0.0))
    short = grid.shortcut(path)
    assert short[0] == path[0] and short[-1] == path[-1]
    assert len(short) <= 5 < len(path)
    for a, b in zip(short, short[1:]):
        assert grid._line_clear(a, b, _DynamicPointIndex((), grid.clearance))


def test_shortcut_respects_dynamic_points():
    grid = OccupancyGrid(ROOM, extra_margin=0.30)
    person = [(0.0, 0.0), (0.1, 0.0)]
    path = grid.plan((-3.0, 0.0), (3.0, 0.0), dynamic_points=person)
    short = grid.shortcut(path, person)
    assert len(short) >= 3
    assert not grid.path_blocked(short, person, (-3.0, 0.0))


def test_path_blocked_sees_new_obstacle_ahead_but_not_next_to_robot():
    grid = OccupancyGrid(ROOM, extra_margin=0.30)
    path = [(-3.0, 0.0), (3.0, 0.0)]
    assert not grid.path_blocked(path, [], (-3.0, 0.0))
    assert grid.path_blocked(path, [(0.0, 0.2)], (-3.0, 0.0))
    # Beside the robot: the forward gate's business, not a reason to replan.
    assert not grid.path_blocked(path, [(-3.0, 0.5)], (-3.0, 0.0))
    # Beyond the horizon.
    assert not grid.path_blocked(path, [(2.9, 0.2)], (-3.0, 0.0), horizon=3.0)


def test_small_heading_error_drives_while_turning():
    pose = Pose2D(0.0, 0.0, 0.0)
    target = Pose2D(3.0, 1.5, 0.0)  # ~27° off
    command = compute_command(pose, target, Pose2D(9.0, 4.5, 0.0), 5.0)
    assert command.status == "moving"
    assert 0.0 < command.linear_x < 0.6 and command.angular_z > 0.0
    sharp = Pose2D(math.cos(TURN_IN_PLACE_ERROR + 0.1), math.sin(TURN_IN_PLACE_ERROR + 0.1), 0.0)
    assert compute_command(pose, sharp, Pose2D(9.0, 9.0, 0.0), 5.0).status == "aligning"


def ring(cx, cy, radius, skip=None):
    """Scan endpoints on a circle around (cx, cy), optionally with a gap."""
    points = []
    for degree in range(360):
        if skip and skip[0] <= degree < skip[1]:
            continue
        a = math.radians(degree + 0.5)
        points.append((cx + radius*math.cos(a), cy + radius*math.sin(a)))
    return points


def test_memory_keeps_obstacle_out_of_view_and_clears_when_seen_free():
    core = NavigatorCore(ROOM, {})
    person = (2.0, 0.0)
    core.set_scan([person, *ring(0, 0, 5.0, skip=(0, 1))], (0.0, 0.0), now=0.0)
    assert person in core.dynamic_points
    # Hidden behind something nearer on the same bearing: still remembered.
    core.set_scan([(1.0, 0.01), *ring(0, 0, 5.0, skip=(0, 1))], (0.0, 0.0), now=1.0)
    assert person in core.dynamic_points
    # A beam passes through its spot to the far wall: forgotten.
    core.set_scan(ring(0, 0, 5.0), (0.0, 0.0), now=2.0)
    assert person not in core.dynamic_points


def test_memory_expires_and_ignores_own_body():
    core = NavigatorCore(ROOM, {})
    far_point = (0.0, 7.9)   # beyond the open range: cannot be cleared by a missing return
    core.set_scan([far_point, (0.3, 0.0)], (0.0, 0.0), now=0.0)
    assert far_point in core.dynamic_points and (0.3, 0.0) not in core.dynamic_points
    core.set_scan([], (0.0, 0.0), now=MEMORY_S + 0.1)
    assert core.dynamic_points == []


def test_without_time_there_is_no_memory():
    core = NavigatorCore(ROOM, {})
    core.set_scan([(2.0, 0.0)], (0.0, 0.0))
    core.set_scan([], (0.0, 0.0))
    assert core.dynamic_points == []
