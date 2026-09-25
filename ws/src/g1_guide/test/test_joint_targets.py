import math
import unittest
from collections import OrderedDict

from g1_guide.joint_targets import JointTargetGuard


def limits(count=2):
    return OrderedDict(("joint_{}".format(i), (-1.0, 1.0)) for i in range(count))


class JointTargetGuardTests(unittest.TestCase):
    def setUp(self):
        self.guard = JointTargetGuard(limits(), feedback_timeout=0.8)
        self.guard.feedback(["joint_0", "joint_1"], [0.0, 0.0], 0.0)

    def test_names_order_idle_and_first_command_rate_limit(self):
        self.assertEqual(self.guard.names, ("joint_0", "joint_1"))
        self.assertEqual(self.guard.step(0.0), ([0.0, 0.0], [0.0, 0.0], "hold_idle"))
        self.guard.command(["joint_1", "joint_0"], [0.5, -0.5], 0.0)
        positions, velocities, status = self.guard.step(0.05)
        self.assertEqual(positions, [-0.015, 0.015])
        self.assertEqual(velocities, [-0.3, 0.3])
        self.assertEqual(status, "tracking")

    def test_reordered_full_feedback_and_29_joint_partial_commands(self):
        guard = JointTargetGuard(limits(29))
        names = list(reversed(guard.names))
        guard.feedback(names, [0.1] * 29, 10.0)
        guard.command(["joint_28", "joint_3"], [0.8, -0.8], 10.0)
        positions, _, _ = guard.step(10.05)
        self.assertEqual(len(positions), 29)
        self.assertAlmostEqual(positions[28], 0.115)
        self.assertAlmostEqual(positions[3], 0.085)
        self.assertTrue(all(positions[i] == 0.1 for i in range(29) if i not in (3, 28)))

    def test_command_validation_is_atomic_and_counted(self):
        self.guard.command(["joint_0"], [0.4], 0.1)
        before_target = list(self.guard._target)
        before_command_time = self.guard._last_command_time
        bad_commands = [
            ((["joint_0", "joint_1"], [0.8, 2.0], 0.2), ValueError),
            ((["joint_0", "joint_0"], [0.1, 0.2], 0.2), ValueError),
            ((["missing"], [0.2], 0.2), ValueError),
            ((["joint_1"], [math.nan], 0.2), ValueError),
            ((["joint_1"], [0.2], 0.9), ValueError),  # feedback stale
        ]
        for args, error in bad_commands:
            with self.assertRaises(error):
                self.guard.command(*args)
            self.assertEqual(self.guard._target, before_target)
            self.assertEqual(self.guard._last_command_time, before_command_time)
        self.assertEqual(self.guard.rejected_count, len(bad_commands))
        with self.assertRaisesRegex(ValueError, "backwards"):
            self.guard.command(["joint_1"], [0.2], 0.8)
        self.assertEqual(self.guard.rejected_count, len(bad_commands) + 1)

    def test_no_feedback_and_bad_feedback_shapes(self):
        guard = JointTargetGuard(limits())
        with self.assertRaises(ValueError):
            guard.command(["joint_0"], [0.0], 0.0)
        self.assertEqual(guard.rejected_count, 1)
        with self.assertRaises(ValueError):
            guard.step(0.0)
        with self.assertRaises(ValueError):
            guard.feedback(["joint_0"], [0.0], 0.0)

    def test_feedback_outside_limits_enters_fault_and_recovers_safely(self):
        with self.assertRaises(ValueError):
            self.guard.feedback(["joint_0", "joint_1"], [1.01, 0.0], 0.1)
        self.assertEqual(self.guard.status, "fault")
        self.assertEqual(self.guard.step(0.1), ([0.0, 0.0], [0.0, 0.0], "fault"))
        with self.assertRaises(ValueError):
            self.guard.command(["joint_0"], [0.2], 0.1)
        self.assertEqual(self.guard.rejected_count, 1)
        self.guard.feedback(["joint_1", "joint_0"], [0.1, -0.1], 0.2)
        self.assertEqual(self.guard.step(0.2), ([-0.1, 0.1], [0.0, 0.0], "hold_idle"))

    def test_command_timeout_freezes_last_published_reference(self):
        self.guard.command(["joint_0"], [0.8], 0.0)
        published, _, _ = self.guard.step(0.05)
        # Feedback changes under gravity; timeout must not chase this measurement.
        self.guard.feedback(["joint_0", "joint_1"], [0.2, 0.3], 0.1)
        positions, velocities, status = self.guard.step(0.61)
        self.assertEqual(positions, published)
        self.assertEqual(velocities, [0.0, 0.0])
        self.assertEqual(status, "hold_timeout")
        self.assertEqual(self.guard.step(0.62), (published, [0.0, 0.0], "hold_timeout"))

    def test_long_scheduler_pause_caps_reference_change_dt(self):
        self.guard.command(["joint_0"], [1.0], 0.0)
        first, _, _ = self.guard.step(0.01)
        second, velocities, status = self.guard.step(0.4)
        self.assertAlmostEqual(second[0] - first[0], 0.015)
        self.assertAlmostEqual(velocities[0], 0.3)
        self.assertEqual(status, "tracking")

    def test_stale_feedback_freezes_without_following_measurement(self):
        self.guard.command(["joint_0"], [0.8], 0.0)
        published, _, _ = self.guard.step(0.05)
        positions, velocities, status = self.guard.step(0.81)
        self.assertEqual(positions, published)
        self.assertEqual(velocities, [0.0, 0.0])
        self.assertEqual(status, "stale_feedback")

    def test_clock_rollback_raises_and_freezes_in_fault(self):
        self.guard.feedback(["joint_0", "joint_1"], [0.0, 0.0], 1.0)
        self.guard.command(["joint_0"], [0.8], 1.0)
        self.guard.step(1.05)
        with self.assertRaisesRegex(ValueError, "backwards"):
            self.guard.step(1.04)
        self.assertEqual(self.guard.status, "fault")
        self.assertEqual(self.guard.step(1.05), ([0.015, 0.0], [0.0, 0.0], "fault"))
        # A feedback stamp behind the last observed wall time is also rejected.
        with self.assertRaises(ValueError):
            self.guard.feedback(["joint_0", "joint_1"], [0.0, 0.0], 1.04)

    def test_old_command_time_is_rejected_and_fault_freezes_reference(self):
        self.guard.command(["joint_0"], [0.8], 0.1)
        published, _, _ = self.guard.step(0.15)
        before_count = self.guard.rejected_count
        with self.assertRaisesRegex(ValueError, "backwards"):
            self.guard.command(["joint_1"], [0.5], 0.14)
        self.assertEqual(self.guard.rejected_count, before_count + 1)
        self.assertEqual(self.guard.status, "fault")
        self.assertEqual(self.guard.step(0.15), (published, [0.0, 0.0], "fault"))

    def test_reaches_target_and_holds(self):
        self.guard.command(["joint_0"], [0.01], 0.0)
        positions, velocities, status = self.guard.step(0.05)
        self.assertEqual(positions, [0.01, 0.0])
        self.assertAlmostEqual(velocities[0], 0.2)
        self.assertEqual(status, "holding")


if __name__ == "__main__":
    unittest.main()
