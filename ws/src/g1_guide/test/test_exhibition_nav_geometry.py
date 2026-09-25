"""Pure geometry tests for the live scan projection used by Gazebo navigation."""

import math
from types import SimpleNamespace as NS

import pytest

from g1_guide.exhibition_nav import (
    forward_clearance, known_static_hit, obstacle_endpoints,
    scan_endpoints, scan_returns_3d, wrap_angle, yaw_of,
)


def _quat(yaw):
    return NS(x=0.0, y=0.0, z=math.sin(yaw/2), w=math.cos(yaw/2))


def test_project_uses_sensor_yaw_and_translation():
    scan = NS(ranges=[1.0, math.inf, 2.0], angle_min=0.0,
              angle_increment=math.pi/2, range_min=0.1, range_max=3.0)
    transform = NS(translation=NS(x=2.0, y=3.0, z=1.3), rotation=_quat(math.pi/2))
    points = scan_endpoints(scan, transform)
    assert points[0] == pytest.approx((2.0, 4.0))
    assert points[1] == pytest.approx((2.0, 1.0))


def test_level_nav_lidar_uses_standard_scan_angle():
    scan = NS(ranges=[2.0], angle_min=math.pi/2,
              angle_increment=0.01, range_min=0.1, range_max=3.0)
    level = NS(translation=NS(x=0.0, y=-4.0, z=1.3),
               rotation=NS(x=0.0, y=0.0, z=0.0, w=1.0))
    assert scan_endpoints(scan, level)[0] == pytest.approx((0.0, -2.0))


def test_pitched_lidar_floor_returns_are_not_obstacles():
    pitch = 0.40
    scan = NS(ranges=[3.1, 3.1], angle_min=0.0,
              angle_increment=math.pi, range_min=0.1, range_max=8.0)
    transform = NS(translation=NS(x=0.0, y=0.0, z=1.3),
                   rotation=NS(x=0.0, y=math.sin(pitch/2), z=0.0,
                               w=math.cos(pitch/2)))
    returns = scan_returns_3d(scan, transform)
    assert returns[0][2] == pytest.approx(1.3-3.1*math.sin(pitch))
    assert returns[0][2] < 0.20
    assert returns[1][2] > 1.3
    assert len(obstacle_endpoints(scan, transform, floor_top=0.05)) == 1


def test_known_static_returns_are_removed_only_from_replanning():
    config = {"room": {"width": 16, "depth": 12, "wall_thickness": .15},
              "exhibits": [{"position": [-4, 0, .5], "size": [1.4, 1.2, .9]}],
              "obstacles": []}
    assert known_static_hit((-4.1, -.62), config)
    assert known_static_hit((-4.0, -5.96), config)
    assert not known_static_hit((-4.0, -1.9), config)


def test_forward_clearance_ignores_behind_and_side():
    pose = NS(x=0.0, y=0.0, yaw=0.0)
    assert forward_clearance(pose, [(-0.1, 0), (0, 0.2), (1.4, 0.1)]) == pytest.approx(math.hypot(1.4, 0.1))
    assert math.isinf(forward_clearance(pose, [(-1, 0)]))


def test_bad_orientation_fails_closed():
    with pytest.raises(ValueError):
        yaw_of(NS(x=0, y=0, z=0, w=0))
    assert wrap_angle(3*math.pi) == pytest.approx(math.pi)
