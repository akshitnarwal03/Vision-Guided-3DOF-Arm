import unittest

from arm_tracker.filtering import TargetFilter
from arm_tracker.vision import HandObservation, TargetSelector


def hand(x, score):
    return HandObservation((x, 0.5), score, ())


class FilteringTests(unittest.TestCase):
    def test_single_jump_rejected_then_confirmed(self):
        target_filter = TargetFilter(alpha=0.5, jump_threshold=0.2, confirm_frames=2)
        self.assertEqual(target_filter.update((0.2, 0.5)), (0.2, 0.5))
        self.assertEqual(target_filter.update((0.8, 0.5)), (0.2, 0.5))
        self.assertGreater(target_filter.update((0.81, 0.5))[0], 0.4)

    def test_selector_keeps_previous_hand_and_waits_before_switching(self):
        selector = TargetSelector(lost_frames=3, max_match_distance=0.2)
        self.assertEqual(selector.choose([hand(0.2, 0.9), hand(0.8, 0.5)]).center[0], 0.2)
        self.assertEqual(selector.choose([hand(0.22, 0.4), hand(0.8, 0.99)]).center[0], 0.22)
        for _ in range(3):
            self.assertIsNone(selector.choose([hand(0.8, 0.99)]))
        self.assertEqual(selector.choose([hand(0.8, 0.99)]).center[0], 0.8)


