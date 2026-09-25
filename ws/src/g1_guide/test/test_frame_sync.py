"""ROS-independent tests for RGB-D pairing and freshness behavior."""

import unittest

from g1_guide.frame_sync import FrameSynchronizer


class FrameSynchronizerTest(unittest.TestCase):
    def test_pairing_within_tolerance_and_rejecting_mismatch(self):
        sync = FrameSynchronizer(tolerance_sec=0.05, stale_after_sec=1.0)
        sync.add_rgb("rgb-1", stamp=10.0, received_monotonic=1.0)
        result = sync.add_depth("depth-1", stamp=10.04, received_monotonic=1.01)
        self.assertTrue(result.is_valid)
        self.assertEqual(result.pair.rgb.payload, "rgb-1")
        self.assertAlmostEqual(result.pair.stamp_delta_sec, 0.04)

        mismatch = FrameSynchronizer(tolerance_sec=0.05, stale_after_sec=1.0)
        mismatch.add_rgb("rgb", stamp=10.0, received_monotonic=1.0)
        result = mismatch.add_depth("depth", stamp=10.06, received_monotonic=1.01)
        self.assertEqual(result.status, "waiting")
        self.assertIsNone(result.pair)

    def test_dropped_frame_does_not_create_a_pair(self):
        sync = FrameSynchronizer(tolerance_sec=0.02, stale_after_sec=0.5)
        sync.add_rgb("rgb-only", stamp=4.0, received_monotonic=2.0)
        result = sync.snapshot(2.1)
        self.assertEqual(result.status, "waiting")
        self.assertIsNone(result.pair)

        sync.add_depth("wrong-update", stamp=4.2, received_monotonic=2.1)
        self.assertEqual(sync.snapshot(2.2).status, "waiting")
        expired = sync.snapshot(2.7)
        self.assertEqual(expired.status, "stale")
        self.assertIsNone(expired.pair)

    def test_out_of_order_samples_pair_by_nearest_header_time(self):
        sync = FrameSynchronizer(tolerance_sec=0.05, stale_after_sec=1.0)
        sync.add_rgb("rgb-2", stamp=2.0, received_monotonic=3.0)
        sync.add_rgb("rgb-1", stamp=1.0, received_monotonic=3.01)
        sync.add_depth("depth-1", stamp=1.02, received_monotonic=3.02)
        result = sync.add_depth("depth-2", stamp=2.01, received_monotonic=3.03)
        self.assertTrue(result.is_valid)
        self.assertEqual(result.pair.rgb.payload, "rgb-2")
        self.assertEqual(result.pair.depth.payload, "depth-2")

    def test_monotonic_age_makes_paused_simulation_stale(self):
        sync = FrameSynchronizer(tolerance_sec=0.05, stale_after_sec=0.5)
        sync.add_rgb("rgb", stamp=7.0, received_monotonic=100.0)
        sync.add_depth("depth-old", stamp=7.0, received_monotonic=100.0)
        self.assertEqual(sync.snapshot(100.49).status, "valid")
        stale = sync.snapshot(100.51)
        self.assertEqual(stale.status, "stale")
        self.assertIsNone(stale.pair)

        # A fresh RGB half must not refresh an old depth half.
        sync = FrameSynchronizer(tolerance_sec=0.05, stale_after_sec=0.5)
        sync.add_rgb("rgb-old", stamp=8.0, received_monotonic=200.0)
        sync.add_depth("depth-old", stamp=8.0, received_monotonic=200.0)
        sync.add_rgb("rgb-duplicate", stamp=8.0, received_monotonic=200.49)
        sync.add_depth("depth-duplicate", stamp=8.0, received_monotonic=200.49)
        self.assertEqual(sync.snapshot(200.51).status, "stale")

        # Both halves must be fresh; max(receipt times) would incorrectly
        # accept this pair because only RGB was refreshed.
        sync = FrameSynchronizer(tolerance_sec=0.05, stale_after_sec=0.5)
        sync.add_depth("depth-old", stamp=9.0, received_monotonic=300.0)
        sync.add_rgb("rgb-new", stamp=9.0, received_monotonic=300.4)
        self.assertEqual(sync.snapshot(300.51).status, "stale")

    def test_old_or_duplicate_pairs_do_not_replace_newer_pair(self):
        sync = FrameSynchronizer(tolerance_sec=0.05, stale_after_sec=1.0)
        sync.add_rgb("rgb-new", stamp=2.0, received_monotonic=10.0)
        self.assertTrue(sync.add_depth("depth-new", stamp=2.0, received_monotonic=10.0).is_valid)
        sync.add_rgb("rgb-old", stamp=1.0, received_monotonic=10.1)
        result = sync.add_depth("depth-old", stamp=1.0, received_monotonic=10.1)
        self.assertTrue(result.is_valid)
        self.assertEqual(result.pair.rgb.payload, "rgb-new")

    def test_reset_rejects_delayed_pre_reset_stream_until_both_restart(self):
        sync = FrameSynchronizer(
            tolerance_sec=0.05,
            stale_after_sec=1.0,
            clock_reset_threshold_sec=1.0,
        )
        sync.add_rgb("old-rgb", stamp=20.0, received_monotonic=5.0)
        self.assertTrue(sync.add_depth("old-depth", stamp=20.0, received_monotonic=5.0).is_valid)
        self.assertEqual(sync.add_rgb("new-rgb", stamp=0.1, received_monotonic=5.1).status, "clock_reset")
        # This delayed old depth must not complete an old-epoch pair.
        self.assertEqual(sync.add_depth("delayed-old-depth", stamp=20.0, received_monotonic=5.11).status, "clock_reset")
        result = sync.add_depth("new-depth", stamp=0.12, received_monotonic=5.12)
        self.assertTrue(result.is_valid)
        self.assertEqual(result.pair.rgb.payload, "new-rgb")
        self.assertEqual(result.pair.depth.payload, "new-depth")

    def test_header_clock_reset_discards_old_epoch(self):
        sync = FrameSynchronizer(
            tolerance_sec=0.05,
            stale_after_sec=1.0,
            clock_reset_threshold_sec=1.0,
        )
        sync.add_rgb("old-rgb", stamp=20.0, received_monotonic=5.0)
        self.assertTrue(
            sync.add_depth("old-depth", stamp=20.0, received_monotonic=5.0).is_valid
        )
        reset = sync.add_rgb("new-rgb", stamp=0.1, received_monotonic=5.1)
        self.assertEqual(reset.status, "clock_reset")
        self.assertIsNone(reset.pair)
        result = sync.add_depth("new-depth", stamp=0.12, received_monotonic=5.11)
        self.assertTrue(result.is_valid)
        self.assertEqual(result.pair.rgb.payload, "new-rgb")

    def test_reset_guard_follows_trigger_stream_until_other_stream_catches_up(self):
        sync = FrameSynchronizer(
            tolerance_sec=0.05,
            stale_after_sec=1.0,
            clock_reset_threshold_sec=1.0,
        )
        sync.add_rgb("old-rgb", stamp=20.0, received_monotonic=7.0)
        self.assertTrue(sync.add_depth("old-depth", stamp=20.0, received_monotonic=7.0).is_valid)

        self.assertEqual(
            sync.add_rgb("new-rgb-0.1", stamp=0.1, received_monotonic=7.1).status,
            "clock_reset",
        )
        # RGB is the reset-triggering stream and advances the guard reference.
        for stamp in (0.5, 1.0, 1.5):
            self.assertEqual(
                sync.add_rgb(f"new-rgb-{stamp}", stamp=stamp, received_monotonic=7.1 + stamp).status,
                "clock_reset",
            )

        result = sync.add_depth("new-depth-1.5", stamp=1.5, received_monotonic=8.7)
        self.assertTrue(result.is_valid)
        self.assertEqual(result.pair.rgb.payload, "new-rgb-1.5")
        self.assertEqual(result.pair.depth.payload, "new-depth-1.5")

    def test_reset_guard_recovers_without_intermediate_driver_frames(self):
        sync = FrameSynchronizer(
            tolerance_sec=0.05,
            stale_after_sec=1.0,
            clock_reset_threshold_sec=1.0,
        )
        sync.add_rgb("old-rgb", stamp=100.0, received_monotonic=9.0)
        self.assertTrue(sync.add_depth("old-depth", stamp=100.0, received_monotonic=9.0).is_valid)

        self.assertEqual(
            sync.add_rgb("late-old-rgb", stamp=98.0, received_monotonic=9.1).status,
            "clock_reset",
        )
        sync.add_rgb("new-rgb", stamp=101.0, received_monotonic=9.2)
        result = sync.add_depth("new-depth", stamp=101.0, received_monotonic=9.21)
        self.assertTrue(result.is_valid)
        self.assertEqual(result.pair.rgb.payload, "new-rgb")
        self.assertEqual(result.pair.depth.payload, "new-depth")

    def test_reset_candidate_20_to_0_1_to_1_5_recovers_then_stales(self):
        sync = FrameSynchronizer(
            tolerance_sec=0.05,
            stale_after_sec=0.5,
            clock_reset_threshold_sec=1.0,
        )
        sync.add_rgb("old-rgb", stamp=20.0, received_monotonic=10.0)
        self.assertTrue(sync.add_depth("old-depth", stamp=20.0, received_monotonic=10.0).is_valid)

        self.assertEqual(
            sync.add_rgb("reset-rgb", stamp=0.1, received_monotonic=10.1).status,
            "clock_reset",
        )
        sync.add_rgb("candidate-rgb", stamp=1.5, received_monotonic=10.2)
        recovered = sync.add_depth("candidate-depth", stamp=1.5, received_monotonic=10.21)
        self.assertTrue(recovered.is_valid)
        self.assertEqual(recovered.pair.rgb.payload, "candidate-rgb")
        self.assertEqual(recovered.pair.depth.payload, "candidate-depth")

        stale = sync.snapshot(10.72)
        self.assertEqual(stale.status, "stale")
        self.assertIsNone(stale.pair)

    def test_reset_confirmation_discards_old_pending_before_timestamp_reuse(self):
        sync = FrameSynchronizer(
            tolerance_sec=0.05,
            stale_after_sec=1.0,
            clock_reset_threshold_sec=1.0,
        )
        sync.add_rgb("old-rgb", stamp=20.0, received_monotonic=11.0)
        self.assertTrue(sync.add_depth("old-depth", stamp=20.0, received_monotonic=11.0).is_valid)

        sync.add_rgb("new-rgb", stamp=0.1, received_monotonic=11.1)
        sync.add_depth("delayed-old-depth", stamp=20.0, received_monotonic=11.11)
        confirmed = sync.add_depth("new-depth", stamp=0.12, received_monotonic=11.12)
        self.assertTrue(confirmed.is_valid)
        self.assertEqual(confirmed.pair.depth.payload, "new-depth")

        # Reusing the old header value must not revive the delayed old depth.
        result = sync.add_rgb("new-rgb-20", stamp=20.0, received_monotonic=11.2)
        self.assertTrue(result.is_valid)
        self.assertEqual(result.pair.rgb.payload, "new-rgb")
        self.assertEqual(result.pair.depth.payload, "new-depth")

    def test_constructor_rejects_non_finite_timing_thresholds(self):
        for argument in ("tolerance_sec", "stale_after_sec", "clock_reset_threshold_sec"):
            with self.subTest(argument=argument):
                with self.assertRaises(ValueError):
                    FrameSynchronizer(**{argument: float("nan")})
                with self.assertRaises(ValueError):
                    FrameSynchronizer(**{argument: float("inf")})
                with self.assertRaises(ValueError):
                    FrameSynchronizer(**{argument: float("-inf")})

    def test_invalid_input_is_explicit_and_clears_previous_output(self):
        sync = FrameSynchronizer(stale_after_sec=1.0)
        sync.add_rgb("rgb", stamp=1.0, received_monotonic=6.0)
        self.assertTrue(sync.add_depth("depth", stamp=1.0, received_monotonic=6.0).is_valid)
        invalid = sync.mark_invalid("depth conversion failed")
        self.assertEqual(invalid.status, "invalid")
        self.assertIsNone(invalid.pair)

if __name__ == "__main__":
    unittest.main()
