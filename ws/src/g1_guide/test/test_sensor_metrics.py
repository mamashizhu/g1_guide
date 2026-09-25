"""Unit tests for ROS-independent sensor summaries and freshness state."""

from types import SimpleNamespace as NS
import unittest

from g1_guide.sensor_metrics import (
    StreamTracker,
    dashboard_joint_line,
    dashboard_requires_display,
    dashboard_vector_rows,
    summarize_imu,
    summarize_joint_state,
    summarize_laser_scan,
    summarize_odometry,
)


def vector(x=0.0, y=0.0, z=0.0, w=None):
    values = {"x": x, "y": y, "z": z}
    if w is not None:
        values["w"] = w
    return NS(**values)


class StreamTrackerTests(unittest.TestCase):
    def test_startup_is_missing_and_valid_message_ages_to_stale(self):
        tracker = StreamTracker(0.5)
        self.assertEqual(tracker.snapshot(10.0).status, "missing")
        tracker.update(10.0, 123.0)
        current = tracker.snapshot(10.49)
        self.assertEqual(current.status, "fresh")
        self.assertEqual(current.header_stamp_sec, 123.0)
        stale = tracker.snapshot(10.51)
        self.assertEqual(stale.status, "stale")
        self.assertAlmostEqual(stale.age_sec, 0.51)

    def test_invalid_message_is_visible_until_next_valid_message(self):
        tracker = StreamTracker(1.0)
        tracker.mark_invalid(4.0, "NaN in scan", 20.0)
        self.assertEqual(tracker.snapshot(4.1).status, "invalid")
        tracker.update(4.2, 20.2)
        self.assertEqual(tracker.snapshot(4.3).status, "fresh")

    def test_bad_header_stamp_and_backwards_monotonic_time_are_invalid(self):
        tracker = StreamTracker(1.0)
        tracker.update(float("nan"), 1.0)
        self.assertEqual(tracker.snapshot(5.1).status, "invalid")
        tracker = StreamTracker(1.0)
        tracker.update(5.0, float("nan"))
        self.assertEqual(tracker.snapshot(5.1).status, "invalid")
        tracker.update(8.0, 9.0)
        self.assertEqual(tracker.snapshot(7.9).status, "invalid")

    def test_stale_threshold_must_be_positive_and_finite(self):
        for value in (0.0, -1.0, float("nan"), float("inf")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                StreamTracker(value)


class SensorSummaryTests(unittest.TestCase):
    def test_headless_display_policy_allows_both_snapshot_modes(self):
        self.assertFalse(dashboard_requires_display("core.png", None))
        self.assertFalse(dashboard_requires_display(None, "joints.png"))
        self.assertTrue(dashboard_requires_display(None, None))

    def test_dashboard_text_rows_are_ascii_and_bounded_for_columns(self):
        rows = []
        for label, values in (
            ("orientation xyzw:", (0.0, 0.0, 0.0, 1.0)),
            ("gyro xyz (rad/s):", (0.01, -0.02, 0.03)),
            ("acceleration xyz (m/s^2):", (0.0, 0.0, 9.81)),
        ):
            rows.extend(dashboard_vector_rows(label, values))
        rows.append(dashboard_joint_line("an_extremely_long_joint_name_that_would_clip", 0.25))
        self.assertTrue(all(text.isascii() for text in rows))
        self.assertTrue(all(len(text) <= 40 for text in rows), rows)
        self.assertIn("...", rows[-1])

    def test_laser_scan_counts_valid_returns_and_projects_occupancy_points(self):
        scan = NS(angle_min=0.0, angle_increment=1.0, range_min=0.1,
                  range_max=5.0, ranges=[1.0, float("inf"), float("nan"), 6.0])
        result = summarize_laser_scan(scan)
        self.assertTrue(result.valid)
        self.assertEqual(result.values["usable_count"], 2)
        self.assertEqual(result.values["hit_count"], 1)
        self.assertEqual(result.values["invalid_count"], 2)
        self.assertAlmostEqual(result.values["nearest_m"], 1.0)
        self.assertAlmostEqual(result.values["points_xy"][0][0], 1.0)

    def test_laser_scan_with_no_usable_measurements_is_invalid(self):
        scan = NS(angle_min=0.0, angle_increment=0.1, range_min=0.1,
                  range_max=2.0, ranges=[float("nan"), -1.0])
        self.assertFalse(summarize_laser_scan(scan).valid)

    def test_imu_valid_values_and_zero_quaternion_validation(self):
        imu = NS(orientation=vector(w=1.0), angular_velocity=vector(1.0, 2.0, 3.0),
                 linear_acceleration=vector(z=9.81), orientation_covariance=[0.0])
        result = summarize_imu(imu)
        self.assertTrue(result.valid)
        self.assertEqual(result.values["angular_velocity_xyz"], (1.0, 2.0, 3.0))
        imu.orientation = vector(w=0.0)
        self.assertFalse(summarize_imu(imu).valid)
        imu.orientation_covariance = [-1.0]
        unavailable = summarize_imu(imu)
        self.assertTrue(unavailable.valid)
        self.assertIsNone(unavailable.values["orientation_xyzw"])

    def test_odometry_requires_finite_pose_and_twist(self):
        odom = NS(
            pose=NS(pose=NS(position=vector(1.0, 2.0, 3.0), orientation=vector(w=1.0))),
            twist=NS(twist=NS(linear=vector(0.1), angular=vector(z=0.2))),
        )
        result = summarize_odometry(odom)
        self.assertTrue(result.valid)
        self.assertEqual(result.values["position_xyz"], (1.0, 2.0, 3.0))
        odom.twist.twist.linear.x = float("inf")
        self.assertFalse(summarize_odometry(odom).valid)

    def test_joint_state_displays_real_names_positions_and_rejects_mismatch(self):
        message = NS(name=["hip", "knee"], position=[0.2, -0.4], velocity=[], effort=[])
        result = summarize_joint_state(message, display_limit=1)
        self.assertTrue(result.valid)
        self.assertEqual(result.values["count"], 2)
        self.assertEqual(result.values["joints"], (("hip", 0.2),))
        self.assertEqual(result.values["remaining_count"], 1)
        message.position = [0.2]
        self.assertFalse(summarize_joint_state(message).valid)


if __name__ == "__main__":
    unittest.main()
