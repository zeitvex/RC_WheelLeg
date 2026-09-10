#!/usr/bin/env python3
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path


SOURCE_DIR = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SOURCE_DIR))

from deadzone_velocity_limiter import limit_deadzone_axis


class DeadzoneVelocityLimiterTest(unittest.TestCase):
    def test_start_skips_linear_deadzone(self) -> None:
        output = limit_deadzone_axis(0.0, 0.5, 0.02, 1.0, 2.0, 0.22, 0.05)
        self.assertAlmostEqual(output, 0.22)

    def test_small_nonzero_target_is_promoted(self) -> None:
        output = limit_deadzone_axis(0.0, 0.1, 0.02, 1.0, 2.0, 0.22, 0.05)
        self.assertAlmostEqual(output, 0.22)

    def test_tiny_target_remains_zero(self) -> None:
        output = limit_deadzone_axis(0.0, 0.04, 0.02, 1.0, 2.0, 0.22, 0.05)
        self.assertEqual(output, 0.0)

    def test_target_at_epsilon_remains_zero(self) -> None:
        output = limit_deadzone_axis(0.0, 0.05, 0.02, 1.0, 2.0, 0.22, 0.05)
        self.assertEqual(output, 0.0)

    def test_stop_does_not_hold_minimum_speed(self) -> None:
        output = limit_deadzone_axis(0.22, 0.0, 0.02, 1.0, 2.0, 0.22, 0.05)
        self.assertEqual(output, 0.0)

    def test_reverse_brakes_through_zero_first(self) -> None:
        output = limit_deadzone_axis(0.30, -0.5, 0.02, 1.0, 2.0, 0.22, 0.05)
        self.assertGreaterEqual(output, 0.0)
        self.assertLess(output, 0.30)

    def test_deadzone_disabled_preserves_legacy_ramp(self) -> None:
        output = limit_deadzone_axis(0.0, 0.5, 0.02, 1.0, 1.0, 0.0, 0.0)
        self.assertTrue(math.isclose(output, 0.02))


if __name__ == "__main__":
    unittest.main()
