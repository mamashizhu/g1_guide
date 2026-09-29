r"""PLAN M5.0：仿真相机画面里，COCO 预训练 YOLO 能否检出人、YuNet 能否检出人脸。

用法：vision\.venv\Scripts\python.exe vision\feasibility.py [图片目录]（默认 .navrun\cv）
结果图存到 <目录>\detected\，每张图打印人框和人脸框。
"""
import sys
from pathlib import Path

import cv2
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parents[1]
MODELS = Path(__file__).resolve().parent / "models"


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    folder = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / ".navrun/cv"
    out = folder / "detected"
    out.mkdir(exist_ok=True)
    yolo = YOLO(str(MODELS / "yolo11n.pt"))
    face = cv2.FaceDetectorYN.create(str(MODELS / "face_detection_yunet_2023mar.onnx"), "", (640, 480), 0.6)
    totals = {"images": 0, "with_person": 0, "persons": 0, "faces": 0}
    for path in sorted(folder.glob("*.png")):
        image = cv2.imread(str(path))
        h, w = image.shape[:2]
        result = yolo(image, classes=[0], conf=0.35, verbose=False)[0]
        persons = [(b.xyxy[0].tolist(), float(b.conf)) for b in result.boxes]
        face.setInputSize((w, h))
        _, faces = face.detect(image)
        faces = [] if faces is None else faces
        for (x1, y1, x2, y2), conf in persons:
            cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 200, 0), 2)
            cv2.putText(image, f"person {conf:.2f}", (int(x1), int(y1) - 4), 0, 0.5, (0, 200, 0), 1)
        for f in faces:
            x, y, fw, fh = map(int, f[:4])
            cv2.rectangle(image, (x, y), (x + fw, y + fh), (0, 0, 255), 2)
            cv2.putText(image, f"face {f[14]:.2f} {fw}px", (x, y - 4), 0, 0.5, (0, 0, 255), 1)
        cv2.imwrite(str(out / path.name), image)
        totals["images"] += 1
        totals["with_person"] += bool(persons)
        totals["persons"] += len(persons)
        totals["faces"] += len(faces)
        print(f"{path.name}: 人 {[round(c, 2) for _, c in persons]}  "
              f"人脸 {[(int(f[2]), round(float(f[14]), 2)) for f in faces]}")
    print(totals)


if __name__ == "__main__":
    main()
