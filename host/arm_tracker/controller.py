from __future__ import annotations

from dataclasses import dataclass


JOINTS = ("base", "shoulder", "elbow")


@dataclass(frozen=True)
class ControlResult:
    angles: tuple[int, int, int]
    limited: tuple[str, ...]
    error: tuple[float, float]


class Stage1Controller:
    def __init__(self, config: dict):
        self.config = config
        self.angles = [float(config["servos"][name]["home"]) for name in JOINTS]

    def sync_angles(self, angles: tuple[int, int, int]) -> None:
        self.angles = [float(angle) for angle in angles]

    def output(self) -> tuple[int, int, int]:
        return tuple(round(value) for value in self.angles)

    def update(self, point: tuple[float, float], dt: float) -> ControlResult:
        settings = self.config["tracking"]
        servos = self.config["servos"]
        dt = max(0.0, min(dt, 0.1))
        ex = point[0] - settings["target_x_normalized"]
        ey = point[1] - settings["target_y_normalized"]
        if abs(ex) <= settings["deadband_x_normalized"]:
            ex = 0.0
        if abs(ey) <= settings["deadband_y_normalized"]:
            ey = 0.0
        max_step = settings["max_angle_change_per_update"]
        base_step = max(-max_step, min(max_step,
            servos["base"]["direction"] * settings["base_gain_deg_per_second"] * ex * dt))
        vertical_step = max(-max_step, min(max_step,
            settings["vertical_gain_deg_per_second"] * ey * dt))
        shoulder_step = servos["shoulder"]["direction"] * vertical_step
        elbow_step = -servos["elbow"]["direction"] * vertical_step * settings["shoulder_elbow_ratio"]
        # Scale both planar commands by the same fraction at a joint limit.
        fraction = 1.0
        for index, step in ((1, shoulder_step), (2, elbow_step)):
            servo = servos[JOINTS[index]]
            if step > 0:
                fraction = min(fraction, max(0.0,
                    (servo["maximum"] - self.angles[index]) / step))
            elif step < 0:
                fraction = min(fraction, max(0.0,
                    (servo["minimum"] - self.angles[index]) / step))
        steps = (base_step, shoulder_step * fraction, elbow_step * fraction)
        limited = []
        for index, step in enumerate(steps):
            servo = servos[JOINTS[index]]
            old = self.angles[index]
            self.angles[index] = max(servo["minimum"],
                min(servo["maximum"], old + step))
            if abs((old + step) - self.angles[index]) > 1e-7 or (
                index in (1, 2) and fraction < 1.0 and abs(vertical_step) > 0
            ):
                limited.append(JOINTS[index])
        return ControlResult(self.output(), tuple(limited), (ex, ey))

