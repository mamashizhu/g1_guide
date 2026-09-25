"""Static, ROS-independent checks for the teleop safety logic."""

import unittest

from g1_guide.g1_teleop import (
    DEFAULT_SPEED,
    MAX_SPEED,
    MIN_SPEED,
    TeleopState,
)


class TeleopStateTest(unittest.TestCase):
    def test_direction_and_watchdog(self):
        state = TeleopState()
        state.handle_key("UP", now=1.0)
        self.assertAlmostEqual(state.command_at(1.1).linear_x, DEFAULT_SPEED)
        self.assertEqual(state.command_at(1.30).linear_x, 0.0)

    def test_stop_and_speed_limits(self):
        state = TeleopState()
        state.handle_key("+", now=0.0)
        for _ in range(100):
            state.handle_key("+", now=0.0)
        self.assertEqual(state.speed, MAX_SPEED)
        for _ in range(100):
            state.handle_key("-", now=0.0)
        self.assertEqual(state.speed, MIN_SPEED)
        state.handle_key("LEFT", now=2.0)
        state.handle_key(" ", now=2.0)
        self.assertEqual(state.command_at(2.0).angular_z, 0.0)

    def test_quit_requests_shutdown_and_stops(self):
        state = TeleopState()
        state.handle_key("UP", now=3.0)
        self.assertTrue(state.handle_key("q", now=3.1))
        self.assertEqual(state.command_at(3.1).linear_x, 0.0)


if __name__ == "__main__":
    unittest.main()
