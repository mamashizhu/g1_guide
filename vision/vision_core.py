"""What the front camera sees: people and whether they face the robot (PLAN M5.9).

Pure image in, dict out -- no ROS, so the same code runs on the Windows side
in simulation and on the robot's Orin NX with its USB camera.

"Facing the robot" = YuNet finds a frontal face: YuNet misses profiles, and
its landmarks catch the rest (the nose must sit between the eyes).  How far
that person is comes from the lidar (``voice/crowd.facing_people``); the face-width
estimate (adult face about 0.16 m) is only a fallback -- simulated visitors'
faces are small and read about twice as far as they are.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

MODELS = Path(__file__).resolve().parent / "models"
FACE_WIDTH_M = 0.16
FACING_NOSE_OFFSET = 0.35   # |nose - eye midpoint| / eye distance
FACING_EYE_RATIO = 0.25     # eye distance / face width; smaller = turned away


def focal_px(width: int, hfov: float) -> float:
    return width / 2 / math.tan(hfov / 2)


def bearing_deg(x: float, width: int, hfov: float) -> float:
    """Horizontal angle of image column ``x``; left of centre is positive (ROS)."""
    return math.degrees(math.atan2(width / 2 - x, focal_px(width, hfov)))


def face_facing(face: np.ndarray) -> bool:
    """YuNet row: x, y, w, h, right eye, left eye, nose, mouth corners, score."""
    w = float(face[2])
    eye_a, eye_b, nose = face[4:6], face[6:8], face[8:10]
    eye_distance = float(np.hypot(*(eye_b - eye_a)))
    if w <= 0 or eye_distance < FACING_EYE_RATIO * w:
        return False
    offset = abs(float(nose[0] - (eye_a[0] + eye_b[0]) / 2)) / eye_distance
    return offset < FACING_NOSE_OFFSET


class VisionCore:
    def __init__(self, hfov: float = math.radians(70), person_conf: float = 0.4,
                 face_conf: float = 0.6):
        import cv2
        from ultralytics import YOLO

        self.hfov = hfov
        self.person_conf = person_conf
        self.yolo = YOLO(str(MODELS / "yolo11n.pt"))
        self.face = cv2.FaceDetectorYN.create(str(MODELS / "face_detection_yunet_2023mar.onnx"),
                                              "", (640, 480), face_conf)

    def analyze(self, image: np.ndarray) -> dict:
        h, w = image.shape[:2]
        result = self.yolo(image, classes=[0], conf=self.person_conf, verbose=False)[0]
        persons = []
        for box in result.boxes:
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
            persons.append({"box": [round(x1), round(y1), round(x2), round(y2)],
                            "conf": round(float(box.conf), 2),
                            "bearing_deg": round(bearing_deg((x1 + x2) / 2, w, self.hfov), 1)})
        self.face.setInputSize((w, h))
        _, raw = self.face.detect(image)
        faces = []
        f = focal_px(w, self.hfov)
        for row in [] if raw is None else raw:
            x, y, fw, fh = (float(v) for v in row[:4])
            distance = f * FACE_WIDTH_M / fw if fw > 0 else math.inf
            frontal = face_facing(row)
            faces.append({"box": [round(x), round(y), round(x + fw), round(y + fh)],
                          "conf": round(float(row[14]), 2),
                          "distance": round(distance, 2),
                          "bearing_deg": round(bearing_deg(x + fw / 2, w, self.hfov), 1),
                          "facing": bool(frontal)})
        return {"persons": persons, "faces": faces,
                "facing": sum(1 for face in faces if face["facing"])}

