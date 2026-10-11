from copy import deepcopy
import unittest

from arm_tracker.config import DEFAULTS
from arm_tracker.controller import Stage1Controller


class ControllerTests(unittest.TestCase):
    def test_deadband_does_not_move(self):
        controller = Stage1Controller(deepcopy(DEFAULTS))
        for _ in range(100):
            result = controller.update((0.51, 0.52), 0.05)
        self.assertEqual(result.angles, (90, 90, 90))

    def test_horizontal_and_coupled_vertical_move(self):
        controller = Stage1Controller(deepcopy(DEFAULTS))
        for _ in range(20):
            result = controller.update((0.8, 0.8), 0.05)
        self.assertGreater(result.angles[0], 90)
        self.assertGreater(result.angles[1], 90)
        self.assertLess(result.angles[2], 90)
        self.assertLessEqual(abs((result.angles[1] - 90) + (result.angles[2] - 90)), 1)

    def test_limits_keep_pair_coupled_and_allow_retreat(self):
        controller = Stage1Controller(deepcopy(DEFAULTS))
        controller.sync_angles((120, 120, 60))
        result = controller.update((0.9, 0.9), 0.05)
        self.assertEqual(result.angles, (120, 120, 60))
        self.assertTrue(result.limited)
        for _ in range(3):
            result = controller.update((0.1, 0.1), 0.05)
        self.assertLess(result.angles[0], 120)
        self.assertLess(result.angles[1], 120)
        self.assertGreater(result.angles[2], 60)

    def test_update_clamp_avoids_large_step(self):
        config = deepcopy(DEFAULTS)
        config["tracking"]["base_gain_deg_per_second"] = 180
        controller = Stage1Controller(config)
        result = controller.update((1.0, 0.5), 2.0)
        self.assertLessEqual(result.angles[0], 92)

    def test_idealized_image_errors_decrease_without_limit_violation(self):
        controller = Stage1Controller(deepcopy(DEFAULTS))
        point = [0.8, 0.75]
        previous = controller.angles.copy()
        for _ in range(100):
            controller.update(tuple(point), 0.05)
            base_delta = controller.angles[0] - previous[0]
            shoulder_delta = controller.angles[1] - previous[1]
            elbow_delta = controller.angles[2] - previous[2]
            point[0] -= 0.02 * base_delta
            point[1] -= 0.01 * (shoulder_delta - elbow_delta)
            previous = controller.angles.copy()
            for angle in controller.angles:
                self.assertGreaterEqual(angle, 60)
                self.assertLessEqual(angle, 120)
        self.assertLess(abs(point[0] - 0.5), 0.1)
        self.assertLess(abs(point[1] - 0.5), 0.1)

