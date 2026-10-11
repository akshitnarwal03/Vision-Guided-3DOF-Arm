from __future__ import annotations

from enum import Enum


class State(str, Enum):
    STARTUP = "STARTUP"
    HOMING = "HOMING"
    IDLE = "IDLE"
    TRACKING = "TRACKING"
    HAND_LOST = "HAND_LOST"
    LIMITED = "LIMITED"
    FAULT = "FAULT"


class TrackerState:
    def __init__(self, lost_frames: int, reacquire_frames: int):
        self.state = State.STARTUP
        self.lost_frames = lost_frames
        self.reacquire_frames = reacquire_frames
        self.missed = 0
        self.seen = 0
        self.fault_reason = ""

    def home(self) -> None:
        if self.state != State.FAULT:
            self.state = State.HOMING
            self.missed = self.seen = 0

    def home_complete(self) -> None:
        if self.state == State.HOMING:
            self.state = State.IDLE

    def toggle_tracking(self) -> None:
        if self.state == State.IDLE:
            self.state = State.TRACKING
            self.missed = self.seen = 0
        elif self.state in (State.TRACKING, State.HAND_LOST, State.LIMITED):
            self.state = State.IDLE
            self.missed = self.seen = 0

    def observe(self, present: bool) -> bool:
        """Return True when loss first requires an immediate firmware HOLD."""
        if self.state not in (State.TRACKING, State.LIMITED, State.HAND_LOST):
            return False
        if present:
            self.missed = 0
            if self.state == State.HAND_LOST:
                self.seen += 1
                if self.seen >= self.reacquire_frames:
                    self.state = State.TRACKING
                    self.seen = 0
            return False
        self.seen = 0
        self.missed += 1
        if self.state != State.HAND_LOST and self.missed >= self.lost_frames:
            self.state = State.HAND_LOST
            return True
        return False

    def set_limited(self, limited: bool) -> None:
        if self.state in (State.TRACKING, State.LIMITED):
            self.state = State.LIMITED if limited else State.TRACKING

    def fault(self, reason: str) -> None:
        self.state = State.FAULT
        self.fault_reason = reason

    def reset(self) -> None:
        if self.state == State.FAULT:
            self.fault_reason = ""
            self.state = State.IDLE
            self.missed = self.seen = 0
