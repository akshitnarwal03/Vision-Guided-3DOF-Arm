from __future__ import annotations

import re
import time
from dataclasses import dataclass

STATE_PATTERN = re.compile(
    r"^STATE B:(\d+) S:(\d+) E:(\d+) MODE:([A-Z]+) TGT:(\d+),(\d+),(\d+)$"
)
LIMITS_PATTERN = re.compile(
    r"^LIMITS B:(\d+):(\d+) S:(\d+):(\d+) E:(\d+):(\d+)$"
)


@dataclass(frozen=True)
class FirmwareState:
    angles: tuple[int, int, int]
    mode: str
    targets: tuple[int, int, int]


def parse_state(line: str) -> FirmwareState:
    match = STATE_PATTERN.fullmatch(line)
    if not match:
        raise ValueError(f"Malformed firmware state: {line!r}")
    values = match.groups()
    return FirmwareState(tuple(map(int, values[:3])), values[3],
                         tuple(map(int, values[4:])))


def parse_limits(line: str) -> tuple[tuple[int, int], ...]:
    match = LIMITS_PATTERN.fullmatch(line)
    if not match:
        raise ValueError(f"Malformed firmware limits: {line!r}")
    values = tuple(map(int, match.groups()))
    return ((values[0], values[1]), (values[2], values[3]),
            (values[4], values[5]))


def format_set(angles: tuple[int, int, int]) -> str:
    return f"SET B:{angles[0]} S:{angles[1]} E:{angles[2]}"


def format_limits(limits: tuple[tuple[int, int], ...]) -> str:
    return (f"LIMITS B:{limits[0][0]}:{limits[0][1]} "
            f"S:{limits[1][0]}:{limits[1][1]} E:{limits[2][0]}:{limits[2][1]}")


def choose_port(configured: str) -> str:
    from serial.tools import list_ports

    if configured != "auto":
        return configured
    ports = list(list_ports.comports())
    candidates = [port.device for port in ports if port.vid is not None]
    if len(candidates) != 1:
        raise RuntimeError(
            f"Expected one USB serial device, found {candidates}; set serial.port in config.yaml"
        )
    return candidates[0]


class SerialLink:
    def __init__(self, config: dict):
        import serial

        port = choose_port(str(config["port"]))
        self.device = serial.Serial(port, config["baud_rate"], timeout=0.15,
                                    write_timeout=0.25)
        self.port = port
        try:
            # Uno/Nano USB serial opening normally resets the board.
            time.sleep(2.0)
            self.device.reset_input_buffer()
            if self.command("PING", "OK PONG") != "OK PONG":
                raise RuntimeError("Firmware handshake failed")
        except Exception:
            self.device.close()
            raise

    def command(self, command: str, expected_prefix: str) -> str:
        self.device.write((command + "\n").encode("ascii"))
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            line = self.device.readline().decode("ascii", errors="replace").strip()
            if not line or line.startswith("READY "):
                continue
            if line.startswith("ERR "):
                raise RuntimeError(f"Firmware rejected {command!r}: {line}")
            if line.startswith(expected_prefix):
                return line
            raise RuntimeError(f"Unexpected firmware response: {line!r}")
        raise TimeoutError(f"No response to {command!r}")

    def status(self) -> FirmwareState:
        return parse_state(self.command("STATUS", "STATE "))

    def limits(self) -> tuple[tuple[int, int], ...]:
        return parse_limits(self.command("LIMITS?", "LIMITS "))

    def set_angles(self, angles: tuple[int, int, int]) -> None:
        self.command(format_set(angles), "OK SET ")

    def hold(self) -> None:
        self.command("HOLD", "OK HOLD")

    def close(self) -> None:
        self.device.close()

