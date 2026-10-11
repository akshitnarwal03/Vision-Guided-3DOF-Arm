from __future__ import annotations

import cv2


def draw_preview(frame, hands, raw, filtered, config, state, angles,
                 fps: float, serial_status: str, limited: tuple[str, ...],
                 show_details: bool = True):
    height, width = frame.shape[:2]
    settings = config["tracking"]
    tx = int(settings["target_x_normalized"] * width)
    ty = int(settings["target_y_normalized"] * height)
    dx = int(settings["deadband_x_normalized"] * width)
    dy = int(settings["deadband_y_normalized"] * height)
    cv2.rectangle(frame, (tx - dx, ty - dy), (tx + dx, ty + dy), (0, 255, 255), 1)
    cv2.drawMarker(frame, (tx, ty), (0, 255, 255), cv2.MARKER_CROSS, 16, 1)
    for hand in hands:
        for x, y in hand.landmarks:
            cv2.circle(frame, (int(x * width), int(y * height)), 2, (80, 220, 80), -1)
    if raw:
        cv2.circle(frame, (int(raw[0] * width), int(raw[1] * height)), 6,
                   (0, 100, 255), 2)
    if filtered:
        cv2.circle(frame, (int(filtered[0] * width), int(filtered[1] * height)),
                   7, (255, 0, 255), 2)
    lines = [f"{state}   serial:{serial_status}   {fps:.1f} fps",
             f"B:{angles[0]} S:{angles[1]} E:{angles[2]}"]
    if limited:
        lines.append("LIMIT: " + ", ".join(limited))
    if show_details:
        lines.append(f"hands:{len(hands)}  score:{max((h.score for h in hands), default=0):.2f}")
        lines.append("h home | t track | x stop | r reset | d overlay | q quit")
        lines.append("orange raw | purple filtered | yellow target/deadband")
    for index, line in enumerate(lines):
        y = 23 + index * 22
        cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 1, cv2.LINE_AA)
    return frame

