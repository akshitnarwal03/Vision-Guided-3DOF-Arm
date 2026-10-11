from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from pathlib import Path


PALM_LANDMARKS = (0, 5, 9, 13, 17)


@dataclass(frozen=True)
class HandObservation:
    center: tuple[float, float]
    score: float
    landmarks: tuple[tuple[float, float], ...]


def palm_center(landmarks) -> tuple[float, float]:
    return (
        sum(landmarks[i].x for i in PALM_LANDMARKS) / len(PALM_LANDMARKS),
        sum(landmarks[i].y for i in PALM_LANDMARKS) / len(PALM_LANDMARKS),
    )


class TargetSelector:
    def __init__(self, lost_frames: int, max_match_distance: float):
        self.lost_frames = lost_frames
        self.max_match_distance = max_match_distance
        self.previous: tuple[float, float] | None = None
        self.missing = 0

    def reset(self) -> None:
        self.previous = None
        self.missing = 0

    def choose(self, hands: list[HandObservation]) -> HandObservation | None:
        if not hands:
            self.missing += 1
            if self.missing >= self.lost_frames:
                self.previous = None
            return None
        if self.previous is None:
            chosen = max(hands, key=lambda hand: hand.score)
        else:
            chosen = min(hands, key=lambda hand: hypot(
                hand.center[0] - self.previous[0], hand.center[1] - self.previous[1]))
            if hypot(chosen.center[0] - self.previous[0],
                     chosen.center[1] - self.previous[1]) > self.max_match_distance:
                self.missing += 1
                if self.missing >= self.lost_frames:
                    self.previous = None
                return None
        self.previous = chosen.center
        self.missing = 0
        return chosen


class HandDetector:
    def __init__(self, config: dict):
        import mediapipe as mp

        model_path = Path(config["model_path"])
        if not model_path.is_file():
            raise FileNotFoundError(f"Hand Landmarker model missing: {model_path}")
        self.mp = mp
        options = mp.tasks.vision.HandLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=str(model_path)),
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            num_hands=config["max_hands"],
            min_hand_detection_confidence=config["detection_confidence"],
            min_hand_presence_confidence=config["detection_confidence"],
            min_tracking_confidence=config["tracking_confidence"],
        )
        self.detector = mp.tasks.vision.HandLandmarker.create_from_options(options)
        self.last_timestamp_ms = -1

    def detect(self, bgr_frame, timestamp_ms: int) -> list[HandObservation]:
        import cv2

        timestamp_ms = max(timestamp_ms, self.last_timestamp_ms + 1)
        self.last_timestamp_ms = timestamp_ms
        rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        image = self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb)
        result = self.detector.detect_for_video(image, timestamp_ms)
        hands = []
        for index, landmarks in enumerate(result.hand_landmarks):
            score = max((category.score for category in result.handedness[index]),
                        default=0.0)
            hands.append(HandObservation(
                center=palm_center(landmarks),
                score=score,
                landmarks=tuple((point.x, point.y) for point in landmarks),
            ))
        return hands

    def close(self) -> None:
        self.detector.close()

