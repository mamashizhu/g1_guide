"""跟随：follow_core.PersonFollower and NavigatorCore's ``follow`` goal.

Scenario tests (walking leaders, crossings, occlusion) live in
tools/follow_sim.py; these pin the individual rules.
"""

import math
from pathlib import Path

from g1_guide.follow_core import (FOLLOW_DISTANCE, HOLD_BAND, LOST_S, MATCH_BEARING, PersonFollower,
                                  VisionPerson, scan_range, vision_persons)
from g1_guide.nav_control import Pose2D, compute_command
from g1_guide.nav_core import NavigatorCore, wrap_angle
from g1_guide.scene_config import load_scene

CONFIG = load_scene(Path(__file__).resolve().parents[1] / "config/exhibition_simple.json")
STATIONS = {e["station"]["key"]: e["station"] for e in CONFIG["exhibits"]}
ROBOT = Pose2D(0.0, -4.0, math.pi / 2)    # spawn of the simple hall, facing +y


def seen(x, y, track=None, pose=ROBOT):
    """A YOLO person at world (x, y) as the robot at ``pose`` would see it."""
    return VisionPerson(math.atan2(y - pose.y, x - pose.x), (pose.x, pose.y), track, 0.8)


def person(pid, x, y, group=1):
    return {"id": pid, "x": x, "y": y, "group": group}


# ---- PersonFollower ----
def test_vision_persons_turns_robot_bearings_into_world():
    pose = Pose2D(1.0, 2.0, math.pi / 2)
    [v] = vision_persons([{"bearing_deg": 30.0, "track_id": 7, "conf": 0.9}], pose)
    assert v.origin == (1.0, 2.0) and v.track_id == 7
    assert abs(v.bearing - math.radians(120)) < 1e-9


def test_start_picks_person_straight_ahead_not_nearest():
    f = PersonFollower()
    people = [person(1, 0.0, -1.5), person(2, -1.0, -2.8)]   # 2.5 m ahead / 1.6 m off to the left
    assert f.start(ROBOT, 0.0, people, None)
    assert f.lidar_id == 1


def test_start_with_camera_takes_camera_choice_and_yolo_id():
    f = PersonFollower()
    people = [person(1, 0.0, -1.5), person(2, 0.6, -1.6)]
    assert f.start(ROBOT, 0.0, people, [seen(0.6, -1.6, track=5)])
    assert (f.lidar_id, f.yolo_id) == (2, 5)


def test_start_fails_with_nobody_in_front():
    f = PersonFollower()
    assert not f.start(ROBOT, 0.0, [person(1, 0.0, -6.0)], None)     # behind the robot
    assert not f.start(ROBOT, 0.0, [person(1, 0.0, 0.0)], None)      # 4 m: too far to pick


def test_start_by_lidar_id():
    f = PersonFollower()
    assert f.start(ROBOT, 0.0, [person(1, 0.0, -2.0), person(9, 2.0, -2.0)], None, person_id=9)
    assert f.lidar_id == 9
    assert not PersonFollower().start(ROBOT, 0.0, [person(1, 0.0, -2.0)], None, person_id=9)


def test_tracks_its_lidar_id_and_estimates_velocity():
    f = PersonFollower()
    f.start(ROBOT, 0.0, [person(1, 0.0, -2.0)], None)
    for i in range(1, 11):
        f.observe_people([person(1, 0.05 * i, -2.0), person(2, 3.0, 0.0)], 0.1 * i)
    assert f.lidar_id == 1 and abs(f.x - 0.5) < 1e-9
    assert abs(f.vx - 0.5) < 0.05 and abs(f.vy) < 0.05


def test_new_lidar_id_near_prediction_is_reacquired():
    f = PersonFollower()
    f.start(ROBOT, 0.0, [person(1, 0.0, -2.0)], None)
    f.observe_people([person(4, 0.2, -2.0)], 0.2)           # id 1 dropped, id 4 right there
    assert f.lidar_id == 4 and f.reacquired == 1
    f.observe_people([person(5, 2.5, -2.0)], 0.4)           # far off: not the target
    assert f.lidar_id == 4


def test_yolo_bearing_corrects_a_swapped_lidar_id():
    f = PersonFollower()
    f.start(ROBOT, 0.0, [person(1, 0.0, -2.0), person(2, 0.7, -2.0)], [seen(0.0, -2.0, track=3)])
    # The lidar tracker hands id 1 to the other person; the camera still sees
    # YOLO id 3 where the target is.
    f.observe_vision([seen(0.05, -2.0, track=3), seen(0.6, -2.0, track=4)], 0.2)
    f.observe_people([person(1, 0.65, -2.0), person(2, 0.05, -2.0)], 0.2)
    assert f.lidar_id == 2 and f.swaps == 1


def test_merge_then_split_follows_pre_merge_motion():
    f = PersonFollower()
    f.start(ROBOT, 0.0, [person(1, 0.0, -2.0)], None)
    for i in range(1, 11):                                   # walking +x at 0.5 m/s
        f.observe_people([person(1, 0.05 * i, -2.0)], 0.1 * i)
    # Someone crossing (walking -y) merges with the target for 0.6 s ...
    for i in range(1, 4):
        f.observe_people([person(1, 0.5 + 0.1 * i, -2.0 + 0.1, group=2)], 1.0 + 0.2 * i)
    # ... and the tracker keeps id 1 on the crosser when they part.
    f.observe_people([person(1, 0.85, -2.6), person(7, 1.35, -2.0)], 1.8)
    assert f.lidar_id == 7 and f.swaps == 1


def test_split_waits_while_only_one_is_visible():
    f = PersonFollower()
    f.start(ROBOT, 0.0, [person(1, 0.0, -2.0)], None)
    for i in range(1, 11):
        f.observe_people([person(1, 0.05 * i, -2.0)], 0.1 * i)
    f.observe_people([person(1, 0.6, -2.0, group=2)], 1.2)
    f.observe_people([person(1, 0.6, -2.3)], 1.4)            # only the crosser: unsure
    assert f.source == "predicted" and f.lidar_id == 1
    f.observe_people([person(1, 0.6, -2.8), person(8, 0.9, -2.0)], 1.6)  # now 0.85 m apart
    assert f.lidar_id == 8


def test_simple_lidar_uses_yolo_bearing_and_laser_range():
    scan = [(0.0, -2.0 + 0.02 * k) for k in range(-3, 4)] + [(3.0, 0.0)]
    assert abs(scan_range(scan, (0.0, -4.0), math.pi / 2) - 1.94) < 1e-9
    f = PersonFollower()
    assert f.start(ROBOT, 0.0, None, [seen(0.0, -2.0, track=1)], scan)
    assert f.source == "vision" and abs(f.y - (-4.0 + 1.94 + 0.15)) < 1e-9
    f.observe_vision([seen(0.0, -1.8, track=1)], 0.2, [(0.0, -1.95)])
    assert abs(f.y - (-1.95 + 0.15)) < 1e-9


def test_target_beyond_lidar_people_range_is_kept_by_camera():
    f = PersonFollower()
    f.start(ROBOT, 0.0, [person(1, 0.0, -2.0)], [seen(0.0, -2.0, track=2)])
    # Lidar people now come without the target (out of their range).
    f.observe_people([], 1.0)
    f.observe_vision([seen(0.0, 3.0, track=2)], 1.0, [(0.0, 2.85)])
    assert f.source == "vision" and abs(f.y - 3.0) < 1e-9


def test_hidden_yolo_id_is_not_replaced_by_a_neighbour_at_once():
    f = PersonFollower()
    f.start(ROBOT, 0.0, [person(1, 0.0, -2.0)], [seen(0.0, -2.0, track=2)])
    f.observe_vision([seen(0.05, -2.0, track=9)], 0.5)        # id 2 missing for 0.5 s
    assert f.yolo_id == 2
    f.observe_vision([seen(0.05, -2.0, track=9)], 1.6)        # missing > YOLO_KEEP_S
    assert f.yolo_id == 9


def test_trail_marks_walked_floor():
    f = PersonFollower()
    f.start(ROBOT, 0.0, [person(1, 0.0, -2.0)], None)
    f.observe_people([person(1, 1.0, -2.0)], 0.5)
    assert f.near_trail(0.1, -2.1, 1.0) and f.near_trail(1.0, -2.0, 1.0)
    assert not f.near_trail(3.0, -2.0, 1.0)
    assert not f.near_trail(0.0, -2.0, 30.0)                  # forgotten after TRAIL_S


# ---- NavigatorCore follow goal ----
def nav():
    c = NavigatorCore(CONFIG, STATIONS)
    c.set_scan([], None)
    return c


def test_follow_goal_keys():
    c = nav()
    assert c.set_goal("follow") == {**c.payload("requested", follow_id=None)}
    assert c.goal_key == "follow" and c.follow_request is None
    assert c.set_goal("follow:12")["follow_id"] == 12
    assert c.set_goal("follow:x")["state"] == "invalid_goal" and c.goal_key is None


def test_no_target_after_waiting():
    c = nav()
    c.set_goal("follow")
    c.set_people([], 0.0)
    assert c.step(ROBOT, 0.0).status["state"] == "follow_starting"
    status = c.step(ROBOT, 3.5).status
    assert status["state"] == "no_target" and c.goal_key is None


def test_holds_and_faces_a_near_target():
    c = nav()
    c.set_goal("follow")
    target = (0.8, -4.0 + FOLLOW_DISTANCE)                     # off to the right, inside the band
    c.set_people([person(1, *target)], 0.0)
    command = c.step(ROBOT, 0.0)
    assert command.status["state"] == "follow_waiting"
    assert command.linear_x == 0.0 and command.angular_z < 0.0  # turns right, towards them


def test_walks_towards_a_far_target_and_stops_short():
    c = nav()
    c.set_goal("follow")
    pose, t = ROBOT, 0.0
    target = (2.0, -1.8)                                       # open floor, 3 m away
    while t < 15.0:
        c.set_scan([], None)
        c.set_people([person(1, *target)], t)
        command = c.step(pose, t)
        if command.status and command.status["state"] == "follow_waiting" and command.linear_x == 0.0:
            break
        yaw = pose.yaw + command.angular_z * 0.1
        pose = Pose2D(pose.x + command.linear_x * 0.1 * math.cos(pose.yaw),
                      pose.y + command.linear_x * 0.1 * math.sin(pose.yaw), wrap_angle(yaw))
        t += 0.1
    distance = math.hypot(target[0] - pose.x, target[1] - pose.y)
    assert FOLLOW_DISTANCE - 0.1 <= distance <= FOLLOW_DISTANCE + HOLD_BAND + 0.05


def test_lost_after_unseen_long_enough():
    c = nav()
    c.set_goal("follow")
    c.set_people([person(1, 0.0, -2.5)], 0.0)
    c.step(ROBOT, 0.0)
    c.set_people([], 1.0)
    assert c.step(ROBOT, 2.0).status["state"] == "follow_searching"
    status = c.step(ROBOT, LOST_S + 0.5).status
    assert status["state"] == "follow_lost" and c.goal_key is None


def test_cancel_stops_following():
    c = nav()
    c.set_goal("follow")
    assert c.set_goal("cancel")["state"] == "cancelled" and c.goal_key is None


def test_compute_command_speed_cap():
    pose, far = Pose2D(0, 0, 0), Pose2D(5, 0, 0)
    assert compute_command(pose, far, far, 10.0).linear_x == 0.6
    assert compute_command(pose, far, far, 10.0, max_linear=0.8).linear_x == 0.8
    assert compute_command(pose, far, far, 10.0, max_linear=0.0).status == "invalid"


def test_match_bearing_is_tight_enough_to_tell_neighbours_apart():
    # Two people 0.7 m apart, 3 m away, must not both match one YOLO bearing.
    assert 2 * math.atan2(0.35, 3.0) > MATCH_BEARING


# ---- review fixes ----
def test_stale_camera_does_not_drop_lidar_updates():
    """Target crossing close in front, out of the camera: an old YOLO bearing
    must not freeze the lidar track (it used to, until "searching")."""
    f = PersonFollower()
    f.start(ROBOT, 0.0, [person(1, 0.0, -2.5)], [seen(0.0, -2.5, track=3)])
    for i in range(1, 6):                                     # walks 0.5 m to the right in 0.5 s
        f.observe_people([person(1, 0.1 * i, -2.5)], 0.1 * i)
    assert abs(f.x - 0.5) < 1e-9 and f.seen_at == 0.5 and f.swaps == 0


def test_swap_needs_someone_where_the_camera_sees_the_target():
    f = PersonFollower()
    f.start(ROBOT, 0.0, [person(1, 0.0, -2.0)], [seen(0.0, -2.0, track=3)])
    f.observe_vision([seen(0.0, -2.0, track=3)], 0.1)
    f.observe_people([person(1, 1.0, -2.0)], 0.2)            # off the camera bearing, nobody else
    assert f.lidar_id == 1 and abs(f.x - 1.0) < 1e-9


def test_requested_id_waits_for_lidar_people():
    assert not PersonFollower().start(ROBOT, 0.0, None, [seen(0.0, -2.0, track=1)],
                                      [(0.0, -2.1)], person_id=4)


def test_visible_obstacle_beside_the_trail_stays_in_the_plan():
    c = nav()
    c.set_goal("follow")
    c.set_scan([], (0.0, -4.0), 0.0)
    c.set_people([person(1, 0.0, -2.5)], 0.0)
    c.step(ROBOT, 0.0)
    for i in range(1, 11):                                    # target walks +x past a box at (1.0, -2.2)
        t = 0.2 * i
        c.set_people([person(1, 0.2 * i, -2.5)], t)
        c.set_scan([(1.0, -2.2), (2.5, -3.0)], (0.0, -4.0), t)
    obstacles = c._follow_obstacles(2.0)
    assert (1.0, -2.2) in obstacles                           # seen now: kept beside the trail
    # No scan since t=2.0: still remembered, but no longer seen now.
    assert any(abs(x - 1.0) < 0.06 and abs(y + 2.2) < 0.06 for x, y, _ in c.memory.values())
    assert (1.0, -2.2) not in c._follow_obstacles(3.0)        # stale + on the trail: target's own
    assert (2.5, -3.0) in c._follow_obstacles(3.0)             # stale but off the trail: kept
