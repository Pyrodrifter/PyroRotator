"""Deterministic safety checks for the streamed slew-cap model."""

import unittest


def smooth_cap(requested, maximum, applied, accel, dt):
    requested = max(1.0, min(requested, maximum))
    if applied <= 0 or applied > maximum:
        applied = maximum
    if requested >= applied or dt <= 0:
        return requested
    return max(requested, applied - accel * dt)


class MotionModelTests(unittest.TestCase):
    def test_cap_reduction_is_acceleration_bounded(self):
        cap = 800.0
        dt = 0.1
        for _ in range(30):
            previous = cap
            cap = smooth_cap(40.0, 800.0, cap, 400.0, dt)
            self.assertLessEqual(previous - cap, 400.0 * dt + 1e-9)
        self.assertEqual(cap, 40.0)

    def test_slew_keeps_full_cap(self):
        cap = 12.0
        for _ in range(20):
            cap = smooth_cap(800.0, 800.0, cap, 400.0, 0.05)
            self.assertEqual(cap, 800.0)

    def test_requested_cap_is_clamped(self):
        self.assertEqual(smooth_cap(-10.0, 800.0, 800.0, 400.0, 0.1), 760.0)
        self.assertEqual(smooth_cap(900.0, 800.0, 100.0, 400.0, 0.1), 800.0)


if __name__ == "__main__":
    unittest.main()
