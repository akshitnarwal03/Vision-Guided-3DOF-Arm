from copy import deepcopy
import unittest

from arm_tracker.config import DEFAULTS, validate


class ConfigTests(unittest.TestCase):
    def test_stage_two_cannot_be_turned_on_accidentally(self):
        config = deepcopy(DEFAULTS)
        config["tof"]["enabled"] = True
        with self.assertRaises(ValueError):
            validate(config)

    def test_home_must_be_inside_joint_limits(self):
        config = deepcopy(DEFAULTS)
        config["servos"]["base"]["minimum"] = 90
        with self.assertRaises(ValueError):
            validate(config)
