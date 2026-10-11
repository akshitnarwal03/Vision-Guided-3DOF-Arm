from __future__ import annotations

import argparse
import logging
import time
from pathlib import Path

import cv2

from arm_tracker.config import load_config
from arm_tracker.controller import Stage1Controller
from arm_tracker.diagnostics import draw_preview
from arm_tracker.filtering import TargetFilter
from arm_tracker.serial_link import SerialLink
from arm_tracker.state_machine import State, TrackerState
from arm_tracker.vision import HandDetector, TargetSelector


LOG = logging.getLogger("arm_tracker")


def verify_firmware(config: dict, link: SerialLink) -> None:
    expected = tuple((config["servos"][name]["minimum"],
                      config["servos"][name]["maximum"])
                     for name in ("base", "shoulder", "elbow"))
    actual = link.limits()
    if expected != actual:
        raise RuntimeError(f"Firmware limits {actual} differ from config {expected}; run calibration")
    status = link.status()
    if status.mode == "DETACHED":
        raise RuntimeError("Servos are detached; use calibration mode with the arm supported")


def run(config: dict, vision_only: bool) -> None:
    camera = None
    detector = None
    link = None
    state = TrackerState(config["vision"]["lost_frame_count"],
                         config["vision"]["reacquire_frame_count"])
    controller = Stage1Controller(config)
    selector = TargetSelector(config["vision"]["lost_frame_count"],
                              config["vision"]["jump_threshold_normalized"])
    target_filter = TargetFilter(config["tracking"]["ema_alpha"],
                                 config["vision"]["jump_threshold_normalized"],
                                 config["vision"]["jump_confirm_frames"])
    try:
        if not vision_only:
            link = SerialLink(config["serial"])
            verify_firmware(config, link)
            LOG.info("Connected to %s", link.port)
            link.command("HOME", "OK HOME")
            state.home()
        else:
            state.state = State.IDLE
        camera = cv2.VideoCapture(config["camera"]["index"])
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, config["camera"]["width"])
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, config["camera"]["height"])
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not camera.isOpened():
            raise RuntimeError("Camera could not be opened")
        detector = HandDetector(config["vision"])
        last_frame_at = time.monotonic()
        last_command_at = 0.0
        show_details = True
        limited = ()
        last_state = state.state
        while True:
            now = time.monotonic()
            dt = now - last_frame_at
            last_frame_at = now
            if not camera.grab():
                raise RuntimeError("Camera frame grab failed")
            ok, frame = camera.retrieve()
            if not ok:
                raise RuntimeError("Camera frame retrieval failed")
            # Mirroring changes control sign too, so use the same frame for both.
            if config["camera"]["mirror_preview"]:
                frame = cv2.flip(frame, 1)
            if link:
                firmware = link.status()
                if firmware.mode in ("TIMEOUT", "DETACHED"):
                    raise RuntimeError(f"Firmware entered {firmware.mode}")
                if state.state == State.HOMING and firmware.mode in ("IDLE", "HOLD"):
                    state.home_complete()
                    controller.sync_angles(firmware.angles)
                    LOG.info("Home complete")
            else:
                firmware = None
            hands = detector.detect(frame, int(time.monotonic() * 1000))
            selected = selector.choose(hands)
            raw = selected.center if selected else None
            filtered = target_filter.update(raw) if raw else None
            if selector.missing >= config["vision"]["lost_frame_count"]:
                target_filter.reset()
            if state.observe(filtered is not None) and link:
                link.hold()
                controller.sync_angles(link.status().angles)
                LOG.warning("Hand lost; motion held")
            if state.state in (State.TRACKING, State.LIMITED) and filtered:
                result = controller.update(filtered, dt)
                limited = result.limited
                state.set_limited(bool(limited))
                if link and now - last_command_at >= 1 / config["serial"]["command_rate_hz"]:
                    link.set_angles(result.angles)
                    last_command_at = now
            else:
                limited = ()
            if state.state != last_state:
                LOG.info("State: %s -> %s", last_state.value, state.state.value)
                last_state = state.state
            angles = firmware.angles if firmware else controller.output()
            preview = draw_preview(frame, hands, raw, filtered, config, state.state.value,
                                   angles, 1 / dt if dt > 0 else 0,
                                   "connected" if link else "vision only", limited,
                                   show_details)
            cv2.imshow("Robotic arm tracker", preview)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("d"):
                show_details = not show_details
            elif key == ord("x"):
                if link:
                    link.hold()
                state.fault("Operator stop")
            elif key == ord("r") and state.state == State.FAULT:
                if link:
                    link.hold()
                    verify_firmware(config, link)
                    controller.sync_angles(link.status().angles)
                state.reset()
            elif key == ord("h") and state.state != State.FAULT:
                if link:
                    link.command("HOME", "OK HOME")
                    state.home()
                else:
                    state.state = State.IDLE
            elif key == ord("t") and state.state != State.FAULT:
                previous = state.state
                state.toggle_tracking()
                if link and state.state == State.IDLE and previous != State.IDLE:
                    link.hold()
                    controller.sync_angles(link.status().angles)
                if state.state == State.TRACKING and firmware:
                    controller.sync_angles(firmware.angles)
    except (Exception, KeyboardInterrupt) as exc:
        LOG.error("Stopped: %s", exc)
        if link:
            try:
                link.hold()
            except Exception as hold_error:
                LOG.error("HOLD failed; firmware watchdog must stop motion: %s", hold_error)
        if not isinstance(exc, KeyboardInterrupt):
            raise
    finally:
        if link:
            try:
                link.hold()
            except Exception:
                pass
            link.close()
        if detector:
            detector.close()
        if camera:
            camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Stage 1 hand tracking")
    parser.add_argument("--config", default=str(Path(__file__).with_name("config.yaml")))
    parser.add_argument("--vision-only", action="store_true",
                        help="Preview hand tracking without connecting to Arduino or moving servos")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")
    run(load_config(args.config), args.vision_only)
