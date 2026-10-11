from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

DEFAULTS: dict[str, Any] = {
    "serial": {"port": "auto", "baud_rate": 115200, "command_rate_hz": 20,
               "timeout_ms": 750},
    "camera": {"index": 0, "width": 640, "height": 480, "mirror_preview": False},
    "vision": {"model_path": "models/hand_landmarker.task",
               "detection_confidence": 0.6, "tracking_confidence": 0.6,
               "max_hands": 2, "lost_frame_count": 5, "reacquire_frame_count": 3,
               "jump_threshold_normalized": 0.25, "jump_confirm_frames": 2},
    "tracking": {"target_x_normalized": 0.5, "target_y_normalized": 0.5,
                 "deadband_x_normalized": 0.04, "deadband_y_normalized": 0.05,
                 "base_gain_deg_per_second": 24.0,
                 "vertical_gain_deg_per_second": 20.0,
                 "shoulder_elbow_ratio": 1.0,
                 "max_angle_change_per_update": 1.5,
                 "ema_alpha": 0.25},
    "servos": {
        name: {"pin": pin, "home": 90, "minimum": 60, "maximum": 120,
               "direction": 1}
        for name, pin in (("base", 9), ("shoulder", 10), ("elbow", 11))
    },
    "tof": {"enabled": False},
}


def _merge(base: dict, override: dict) -> dict:
    for key, value in override.items():
        if key not in base:
            raise ValueError(f"Unknown configuration key: {key}")
        if isinstance(base[key], dict):
            if not isinstance(value, dict):
                raise ValueError(f"Configuration section {key} must be a mapping")
            _merge(base[key], value)
        else:
            base[key] = value
    return base


def _between(value: Any, low: float, high: float, label: str) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not low <= value <= high:
        raise ValueError(f"{label} must be between {low} and {high}")


def validate(config: dict) -> dict:
    serial = config["serial"]
    _between(serial["command_rate_hz"], 1, 40, "serial.command_rate_hz")
    _between(serial["timeout_ms"], 300, 2000, "serial.timeout_ms")
    if 1000 / serial["command_rate_hz"] * 3 >= serial["timeout_ms"]:
        raise ValueError("Command interval must be well below firmware timeout")
    if serial["baud_rate"] != 115200:
        raise ValueError("Firmware uses 115200 baud")
    vision = config["vision"]
    for name in ("detection_confidence", "tracking_confidence",
                 "jump_threshold_normalized"):
        _between(vision[name], 0, 1, f"vision.{name}")
    for name in ("lost_frame_count", "reacquire_frame_count", "jump_confirm_frames"):
        _between(vision[name], 1, 100, f"vision.{name}")
    _between(vision["max_hands"], 1, 2, "vision.max_hands")
    tracking = config["tracking"]
    for name in ("target_x_normalized", "target_y_normalized", "ema_alpha"):
        _between(tracking[name], 0, 1, f"tracking.{name}")
    if tracking["ema_alpha"] == 0:
        raise ValueError("EMA alpha must be greater than zero")
    for name in ("deadband_x_normalized", "deadband_y_normalized"):
        _between(tracking[name], 0, 0.5, f"tracking.{name}")
    for name in ("base_gain_deg_per_second", "vertical_gain_deg_per_second",
                 "shoulder_elbow_ratio", "max_angle_change_per_update"):
        _between(tracking[name], 0.001, 180, f"tracking.{name}")
    pins = []
    for name, servo in config["servos"].items():
        _between(servo["pin"], 2, 13, f"servos.{name}.pin")
        pins.append(servo["pin"])
        if servo["direction"] not in (-1, 1):
            raise ValueError(f"servos.{name}.direction must be -1 or 1")
        if not 0 <= servo["minimum"] < servo["home"] < servo["maximum"] <= 180:
            raise ValueError(f"servos.{name} limits must contain home angle")
    if len(set(pins)) != 3:
        raise ValueError("Servo pins must be distinct")
    if config["tof"]["enabled"]:
        raise ValueError("Stage 2 ToF is not implemented; set tof.enabled to false")
    return config


def load_config(path: str | Path) -> dict:
    import yaml

    path = Path(path)
    with path.open("r", encoding="utf-8") as stream:
        supplied = yaml.safe_load(stream) or {}
    if not isinstance(supplied, dict):
        raise ValueError("Configuration root must be a mapping")
    config = validate(_merge(deepcopy(DEFAULTS), supplied))
    config["vision"]["model_path"] = str(
        (path.parent / config["vision"]["model_path"]).resolve()
    )
    return config


def save_config(path: str | Path, config: dict) -> None:
    import yaml

    validate(config)
    with Path(path).open("w", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream, sort_keys=False)

