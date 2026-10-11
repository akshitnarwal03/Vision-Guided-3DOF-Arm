"""Stage 2 extension point. Stage 1 never reads a distance sensor."""

from dataclasses import dataclass


@dataclass(frozen=True)
class DistanceSample:
    millimeters: int | None = None
    valid: bool = False


class DisabledToF:
    def read(self) -> DistanceSample:
        return DistanceSample()
