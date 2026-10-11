import unittest

from arm_tracker.serial_link import (format_limits, format_set, parse_limits,
                                     parse_state)


class ProtocolTests(unittest.TestCase):
    def test_protocol_round_trip_shapes(self):
        self.assertEqual(format_set((90, 91, 89)), "SET B:90 S:91 E:89")
        self.assertEqual(format_limits(((60, 120), (70, 110), (65, 115))),
                         "LIMITS B:60:120 S:70:110 E:65:115")
        state = parse_state("STATE B:90 S:91 E:89 MODE:MOVING TGT:95,96,88")
        self.assertEqual(state.angles, (90, 91, 89))
        self.assertEqual(state.targets, (95, 96, 88))
        self.assertEqual(parse_limits("LIMITS B:60:120 S:70:110 E:65:115")[2],
                         (65, 115))

    def test_malformed_state_rejected(self):
        for line in ("STATE B:90 S:x E:90 MODE:IDLE TGT:90,90,90",
                     "STATE B:90 S:90 E:90 MODE:IDLE"):
            with self.assertRaises(ValueError):
                parse_state(line)

