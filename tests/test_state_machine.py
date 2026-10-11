import unittest

from arm_tracker.state_machine import State, TrackerState


class StateMachineTests(unittest.TestCase):
    def test_loss_hold_and_reacquisition(self):
        machine = TrackerState(lost_frames=3, reacquire_frames=2)
        machine.home()
        machine.home_complete()
        machine.toggle_tracking()
        self.assertEqual(machine.state, State.TRACKING)
        self.assertFalse(machine.observe(False))
        self.assertFalse(machine.observe(False))
        self.assertTrue(machine.observe(False))
        self.assertEqual(machine.state, State.HAND_LOST)
        self.assertFalse(machine.observe(True))
        self.assertEqual(machine.state, State.HAND_LOST)
        self.assertFalse(machine.observe(True))
        self.assertEqual(machine.state, State.TRACKING)

    def test_fault_needs_explicit_reset(self):
        machine = TrackerState(3, 2)
        machine.fault("test")
        machine.toggle_tracking()
        self.assertEqual(machine.state, State.FAULT)
        machine.reset()
        self.assertEqual(machine.state, State.IDLE)

