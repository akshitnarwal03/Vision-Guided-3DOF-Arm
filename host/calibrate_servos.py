from __future__ import annotations

import argparse
import copy
from pathlib import Path

from arm_tracker.config import load_config, save_config
from arm_tracker.serial_link import SerialLink, format_limits


NAMES = ("base", "shoulder", "elbow")


def show(link: SerialLink, config: dict) -> None:
    status = link.status()
    firmware_limits = link.limits()
    print(f"Firmware: {status.mode}, angles {status.angles}, targets {status.targets}")
    for index, name in enumerate(NAMES):
        item = config["servos"][name]
        print(f"{name:8} firmware {firmware_limits[index]}  draft "
              f"({item['minimum']}, {item['maximum']})  direction {item['direction']}")


def run(config_path: Path, step: int) -> None:
    config = load_config(config_path)
    draft = copy.deepcopy(config)
    # load_config resolves the model path; retain a portable relative path when possible.
    model = Path(config["vision"]["model_path"])
    try:
        draft["vision"]["model_path"] = str(model.relative_to(config_path.parent.resolve()))
    except ValueError:
        draft["vision"]["model_path"] = str(model)
    link = SerialLink(config["serial"])
    print("Support the arm, clear the workspace, then calibrate one joint at a time.")
    print("Commands: show | home | step JOINT +/- | direction JOINT +/-1 | "
          "limits JOINT MIN MAX | save | hold | detach | attach | quit")
    try:
        show(link, draft)
        while True:
            parts = input("calibrate> ").strip().lower().split()
            if not parts:
                continue
            try:
                command = parts[0]
                if command == "quit":
                    break
                elif command == "show":
                    show(link, draft)
                elif command == "home":
                    link.command("HOME", "OK HOME")
                    print("Homing started. Use show to confirm MODE:IDLE.")
                elif command == "hold":
                    link.hold()
                elif command == "detach":
                    if input("Servo torque will release. Is the arm supported? type YES: ") == "YES":
                        link.command("DETACH", "OK DETACH")
                elif command == "attach":
                    if input("A moved joint may jump without position feedback. "
                             "Support and align the arm to its last pose, then type YES: ") == "YES":
                        link.command("ATTACH", "OK ATTACH")
                elif command == "step" and len(parts) == 3 and parts[1] in NAMES:
                    joint = NAMES.index(parts[1])
                    sign = 1 if parts[2] == "+" else -1 if parts[2] == "-" else None
                    if sign is None:
                        raise ValueError("Use + or -")
                    status = link.status()
                    if status.mode == "DETACHED":
                        raise ValueError("Attach servos first")
                    angles = list(status.targets)
                    angles[joint] += sign * step
                    low, high = link.limits()[joint]
                    if not low <= angles[joint] <= high:
                        raise ValueError(f"Firmware limit is {low}..{high}")
                    link.set_angles(tuple(angles))
                    print(f"Target angles: {tuple(angles)}")
                elif command == "direction" and len(parts) == 3 and parts[1] in NAMES:
                    direction = int(parts[2])
                    if direction not in (-1, 1):
                        raise ValueError("Direction must be -1 or 1")
                    draft["servos"][parts[1]]["direction"] = direction
                    print("Draft direction updated; use save to persist")
                elif command == "limits" and len(parts) == 4 and parts[1] in NAMES:
                    low, high = int(parts[2]), int(parts[3])
                    if not 0 <= low < 90 < high <= 180:
                        raise ValueError("Limits must be within 0..180 and contain 90")
                    draft["servos"][parts[1]]["minimum"] = low
                    draft["servos"][parts[1]]["maximum"] = high
                    print("Draft limits updated; use save to persist to EEPROM and YAML")
                elif command == "save":
                    limits = tuple((draft["servos"][name]["minimum"],
                                    draft["servos"][name]["maximum"]) for name in NAMES)
                    print(f"Proposed firmware limits: {limits}")
                    print("Confirm only after checking clearance, cables, and each joint by eye.")
                    if input("Type YES to write EEPROM and config.yaml: ") == "YES":
                        link.hold()
                        link.command(format_limits(limits), "OK LIMITS")
                        if link.limits() != limits:
                            raise RuntimeError("Firmware limit verification failed")
                        save_config(config_path, draft)
                        print("Saved firmware limits and config.yaml")
                else:
                    print("Unknown command; use show, home, step, direction, limits, save, "
                          "hold, detach, attach, quit")
            except (ValueError, RuntimeError, TimeoutError) as error:
                print(f"Error: {error}")
    finally:
        try:
            link.hold()
        finally:
            link.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Manual MG995 servo calibration")
    parser.add_argument("--config", default=str(Path(__file__).with_name("config.yaml")))
    parser.add_argument("--step", type=int, default=5)
    args = parser.parse_args()
    if not 1 <= args.step <= 10:
        parser.error("--step must be 1..10 degrees")
    run(Path(args.config), args.step)
