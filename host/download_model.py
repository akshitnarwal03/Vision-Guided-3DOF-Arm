"""Download Google's official MediaPipe Hand Landmarker model bundle."""

from pathlib import Path
from urllib.request import urlretrieve


URL = ("https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
       "hand_landmarker/float16/latest/hand_landmarker.task")
destination = Path(__file__).parent / "models" / "hand_landmarker.task"
destination.parent.mkdir(parents=True, exist_ok=True)
if destination.exists():
    print(f"Already present: {destination}")
else:
    urlretrieve(URL, destination)
    print(f"Downloaded: {destination}")

