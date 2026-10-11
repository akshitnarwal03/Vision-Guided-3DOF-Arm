from __future__ import annotations

from math import hypot


class TargetFilter:
    def __init__(self, alpha: float, jump_threshold: float, confirm_frames: int):
        self.alpha = alpha
        self.jump_threshold = jump_threshold
        self.confirm_frames = confirm_frames
        self.value: tuple[float, float] | None = None
        self.pending: tuple[float, float] | None = None
        self.pending_count = 0

    def reset(self) -> None:
        self.value = None
        self.pending = None
        self.pending_count = 0

    def update(self, sample: tuple[float, float]) -> tuple[float, float]:
        if self.value is None:
            self.value = sample
            return sample
        distance = hypot(sample[0] - self.value[0], sample[1] - self.value[1])
        if distance > self.jump_threshold:
            if self.pending is None or hypot(sample[0] - self.pending[0],
                                             sample[1] - self.pending[1]) > self.jump_threshold / 2:
                self.pending = sample
                self.pending_count = 1
            else:
                self.pending_count += 1
            if self.pending_count < self.confirm_frames:
                return self.value
        self.pending = None
        self.pending_count = 0
        self.value = (
            self.alpha * sample[0] + (1 - self.alpha) * self.value[0],
            self.alpha * sample[1] + (1 - self.alpha) * self.value[1],
        )
        return self.value

