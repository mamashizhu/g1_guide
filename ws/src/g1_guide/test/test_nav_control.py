"""Pure-Python boundary and safety tests for planar navigation control."""

import math
import unittest

from g1_guide.nav_control import (
    ARRIVAL_DISTANCE,
    ARRIVAL_YAW_ERROR,
    MAX_ANGULAR_SPEED,
    MAX_LINEAR_SPEED,
    OBSTACLE_STOP_DISTANCE,
    Pose2D,
    Velocity,
    compute_command,
)


class ComputeCommandTest(unittest.TestCase):
    def setUp(self):
        self.pose = Pose2D(0.0, 0.0, 0.0)
        self.waypoint = Pose2D(2.0, 0.0, 0.0)
        self.goal = Pose2D(3.0, 0.0, 0.0)

    def test_public_types_and_moving_command(self):
        command = compute_command(self.pose, self.waypoint, self.goal, 2.0)
        self.assertIsInstance(command, Velocity)
        self.assertEqual(command.status, "moving")
        self.assertGreater(command.linear_x, 0.0)
        self.assertEqual(command.angular_z, 0.0)

    def test_linear_and_angular_speeds_are_bounded(self):
        command = compute_command(
            Pose2D(0.0, 0.0, -math.pi),
            Pose2D(0.0, 100.0, 0.0),
            self.goal,
            10.0,
        )
        self.assertEqual(command.status, "aligning")
        self.assertEqual(command.linear_x, 0.0)
        self.assertLessEqual(abs(command.angular_z), MAX_ANGULAR_SPEED)

        fast = compute_command(self.pose, Pose2D(100.0, 0.0, 0.0), self.goal, 10.0)
        self.assertEqual(fast.linear_x, MAX_LINEAR_SPEED)
        self.assertLessEqual(abs(fast.angular_z), MAX_ANGULAR_SPEED)

    def test_turns_in_place_before_forward_motion(self):
        command = compute_command(
            self.pose, Pose2D(0.0, 4.0, 0.0), self.goal, 2.0
        )
        self.assertEqual(command.status, "aligning")
        self.assertEqual(command.linear_x, 0.0)
        self.assertGreater(command.angular_z, 0.0)

    def test_heading_error_wraps_across_pi(self):
        command = compute_command(
            Pose2D(0.0, 0.0, -math.pi + 0.1),
            Pose2D(-2.0, 0.0, 0.0),
            self.goal,
            2.0,
        )
        self.assertEqual(command.status, "moving")
        self.assertAlmostEqual(command.angular_z, -0.15, places=7)

    def test_non_final_short_waypoint_stops_for_path_index_advance(self):
        command = compute_command(
            self.pose, Pose2D(0.1, 0.0, 0.0), Pose2D(0.0, 2.0, 0.0), 2.0
        )
        self.assertEqual(command.status, "aligning")
        self.assertEqual(command.linear_x, 0.0)
        self.assertEqual(command.angular_z, 0.0)

    def test_non_final_coincident_waypoint_does_not_target_remote_goal(self):
        command = compute_command(
            self.pose, self.pose, Pose2D(0.0, 2.0, 0.0), 2.0
        )
        self.assertEqual(command.status, "aligning")
        self.assertEqual(command.linear_x, 0.0)
        self.assertEqual(command.angular_z, 0.0)

    def test_final_short_waypoint_may_track_goal(self):
        command = compute_command(
            self.pose, Pose2D(0.1, 0.0, 0.0), Pose2D(0.0, 2.0, 0.0), 2.0,
            final=True
        )
        self.assertEqual(command.status, "aligning")
        self.assertEqual(command.linear_x, 0.0)
        self.assertGreater(command.angular_z, 0.0)

    def test_clearance_below_threshold_stops_all_motion(self):
        for clearance in (0.0, OBSTACLE_STOP_DISTANCE - 1e-9):
            with self.subTest(clearance=clearance):
                command = compute_command(
                    self.pose, Pose2D(0.0, 1.0, 0.0), self.goal, clearance
                )
                self.assertEqual(command, Velocity(0.0, 0.0, "blocked"))

    def test_clearance_at_threshold_allows_motion(self):
        command = compute_command(
            self.pose, self.waypoint, self.goal, OBSTACLE_STOP_DISTANCE
        )
        self.assertEqual(command.status, "moving")

    def test_positive_infinite_clearance_means_no_finite_obstacle_return(self):
        command = compute_command(
            self.pose, self.waypoint, self.goal, math.inf
        )
        self.assertEqual(command.status, "moving")
        self.assertGreater(command.linear_x, 0.0)
        self.assertLessEqual(command.linear_x, MAX_LINEAR_SPEED)

    def test_cruise_speed_not_capped_by_short_waypoint_spacing(self):
        pose = Pose2D(0.0, 0.0, 0.0)
        command = compute_command(pose, Pose2D(0.4, 0.0, 0.0), Pose2D(5.0, 0.0, 0.0), 5.0)
        self.assertEqual(command.status, "moving")
        self.assertEqual(command.linear_x, MAX_LINEAR_SPEED)

    def test_invalid_negative_clearance_stops(self):
        command = compute_command(self.pose, self.waypoint, self.goal, -0.01)
        self.assertEqual(command, Velocity(0.0, 0.0, "invalid"))

    def test_arrival_tolerances_are_inclusive(self):
        command = compute_command(
            Pose2D(0.0, 0.0, 0.0),
            self.waypoint,
            Pose2D(ARRIVAL_DISTANCE, 0.0, ARRIVAL_YAW_ERROR),
            2.0,
            final=True,
        )
        self.assertEqual(command, Velocity(0.0, 0.0, "arrived"))

    def test_arrival_requires_final_flag_and_position_and_yaw(self):
        goal = Pose2D(0.0, 0.0, 0.0)
        intermediate = compute_command(self.pose, self.waypoint, goal, 2.0)
        self.assertNotEqual(intermediate.status, "arrived")

        too_far = compute_command(
            self.pose, self.waypoint, Pose2D(ARRIVAL_DISTANCE + 1e-6, 0.0, 0.0),
            2.0, final=True
        )
        self.assertNotEqual(too_far.status, "arrived")

        yaw_outside = compute_command(
            self.pose, self.waypoint,
            Pose2D(0.0, 0.0, ARRIVAL_YAW_ERROR + 1e-6),
            2.0, final=True
        )
        self.assertEqual(yaw_outside.status, "aligning")
        self.assertEqual(yaw_outside.linear_x, 0.0)

    def test_final_goal_alignment_has_no_linear_speed(self):
        command = compute_command(
            self.pose, self.waypoint, Pose2D(0.1, 0.0, 0.5), 2.0, final=True
        )
        self.assertEqual(command.status, "aligning")
        self.assertEqual(command.linear_x, 0.0)
        self.assertGreater(command.angular_z, 0.0)

    def test_blocked_takes_precedence_over_alignment_but_arrived_is_terminal(self):
        aligning = compute_command(
            self.pose, self.waypoint, Pose2D(0.0, 0.0, 0.5), 0.79, final=True
        )
        self.assertEqual(aligning, Velocity(0.0, 0.0, "blocked"))

        arrived = compute_command(
            self.pose, self.waypoint, self.pose, 0.0, final=True
        )
        self.assertEqual(arrived, Velocity(0.0, 0.0, "arrived"))

    def test_non_finite_values_fail_closed(self):
        for value in (math.nan, math.inf, -math.inf):
            cases = (
                (Pose2D(value, 0.0, 0.0), self.waypoint, self.goal, 2.0),
                (self.pose, Pose2D(0.0, value, 0.0), self.goal, 2.0),
                (self.pose, self.waypoint, Pose2D(0.0, 0.0, value), 2.0),
            )
            for args in cases:
                with self.subTest(value=value, args=args):
                    self.assertEqual(
                        compute_command(*args), Velocity(0.0, 0.0, "invalid")
                    )

        for clearance in (math.nan, -math.inf):
            with self.subTest(clearance=clearance):
                self.assertEqual(
                    compute_command(self.pose, self.waypoint, self.goal, clearance),
                    Velocity(0.0, 0.0, "invalid"),
                )

    def test_malformed_inputs_fail_closed(self):
        self.assertEqual(
            compute_command(None, self.waypoint, self.goal, 2.0),
            Velocity(0.0, 0.0, "invalid"),
        )
        self.assertEqual(
            compute_command(self.pose, self.waypoint, self.goal, "clear"),
            Velocity(0.0, 0.0, "invalid"),
        )
        self.assertEqual(
            compute_command(self.pose, self.waypoint, self.goal, 2.0, final=1),
            Velocity(0.0, 0.0, "invalid"),
        )

    def test_finite_but_overflowing_angle_arithmetic_fails_closed(self):
        command = compute_command(
            Pose2D(0.0, 0.0, -1e308),
            self.waypoint,
            Pose2D(0.0, 0.0, 1e308),
            2.0,
            final=True,
        )
        self.assertEqual(command, Velocity(0.0, 0.0, "invalid"))


if __name__ == "__main__":
    unittest.main()
