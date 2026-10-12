r"""Windows 端视觉进程（PLAN M5）：相机 → YOLO 人 + YuNet 人脸 + MediaPipe 手势 → /guide/vision。

仿真：先在 WSL 用 vision:=true 启动（camera_jpeg 发 /front_camera/image/compressed），再运行
  vision\.venv\Scripts\python.exe vision\watch.py [--port 9090] [--show]
笔记本摄像头（测手势、人脸，不需要仿真）：
  D:\g1_guide\vision\.venv\Scripts\python.exe D:\g1_guide\vision\watch.py --camera 0 --show --no-ros
退出：在画面窗口按 q 或 Esc、点窗口的关闭按钮，或在终端按 Ctrl+C。

/guide/vision（std_msgs/String，JSON）：
  {"stamp": 秒, "persons": [{box, conf, bearing_deg}], "faces": [{box, conf, distance,
   bearing_deg, facing}], "facing": 正脸数, "hand": 当前手势或 null, "gesture": 本帧触发的手势事件或 null}
bearing_deg 左正右负，和 /guide/people 同一约定。只处理最新一帧，处理不过来就丢帧。
真机上换成 UVC 摄像头的话题，代码不变。
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import sys
import threading
import time

import cv2
import numpy as np

from gestures import GestureTracker, HandDetector, classify
from vision_core import VisionCore


class RosCamera:
    """Latest JPEG frame from rosbridge; older frames are dropped."""

    def __init__(self, ros, topic: str):
        import roslibpy

        self.latest = None
        self.ready = threading.Event()
        roslibpy.Topic(ros, topic, "sensor_msgs/CompressedImage", queue_length=1).subscribe(self._on)

    def _on(self, message):
        self.latest = message
        self.ready.set()

    def read(self):
        if not self.ready.wait(timeout=1.0):
            return None, None
        self.ready.clear()
        message = self.latest
        image = cv2.imdecode(np.frombuffer(base64.b64decode(message["data"]), np.uint8), cv2.IMREAD_COLOR)
        stamp = message["header"]["stamp"]
        return image, stamp["sec"] + stamp.get("nanosec", 0) * 1e-9


class LocalCamera:
    def __init__(self, index: int):
        self.cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not self.cap.isOpened():
            raise SystemExit(f"打不开摄像头 {index}")

    def read(self):
        ok, image = self.cap.read()
        return (image if ok else None), time.time()


def draw(image, result, hand):
    for p in result["persons"]:
        cv2.rectangle(image, tuple(p["box"][:2]), tuple(p["box"][2:]), (0, 200, 0), 2)
    for f in result["faces"]:
        cv2.rectangle(image, tuple(f["box"][:2]), tuple(f["box"][2:]),
                      (0, 0, 255) if f["facing"] else (0, 200, 255), 2)
    if hand is not None:
        h, w = image.shape[:2]
        for x, y, _ in hand:
            cv2.circle(image, (int(x * w), int(y * h)), 3, (255, 128, 0), -1)
    label = f"hand: {result['hand']}" + (f"  EVENT {result['gesture']}" if result["gesture"] else "")
    cv2.putText(image, label, (10, 25), 0, 0.7, (255, 255, 255), 2)
    cv2.putText(image, "q / Esc: quit", (10, image.shape[0] - 10), 0, 0.5, (255, 255, 255), 1)
    cv2.imshow("vision", image)
    key = cv2.waitKey(1) & 0xFF
    # False = quit: q / Esc, or the window's close button (imshow would reopen it).
    return key not in (ord("q"), 27) and cv2.getWindowProperty("vision", cv2.WND_PROP_VISIBLE) >= 1


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default=os.environ.get("G1_ROSBRIDGE_HOST", "localhost"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("G1_ROSBRIDGE_PORT", "9090")))
    parser.add_argument("--topic", default="/front_camera/image/compressed")
    parser.add_argument("--camera", type=int, help="用本机摄像头（编号）代替仿真相机")
    parser.add_argument("--hfov", type=float, default=70.0, help="相机水平视野（度）")
    parser.add_argument("--no-ros", action="store_true", help="不连 rosbridge，只打印（配合 --camera）")
    parser.add_argument("--show", action="store_true", help="开窗口显示检测结果")
    args = parser.parse_args()

    import math

    core = VisionCore(hfov=math.radians(args.hfov))
    hands = HandDetector()
    tracker = GestureTracker()
    ros = pub = None
    if not args.no_ros:
        import roslibpy

        ros = roslibpy.Ros(host=args.host, port=args.port)
        ros.run(timeout=10)
        pub = roslibpy.Topic(ros, "/guide/vision", "std_msgs/String")
        pub.advertise()
        print(f"[vision] 已连接 ws://{args.host}:{args.port}", flush=True)
    camera = LocalCamera(args.camera) if args.camera is not None else RosCamera(ros, args.topic)
    print(f"[vision] 图像来源：{'摄像头 %d' % args.camera if args.camera is not None else args.topic}", flush=True)
    try:
        loop(args, camera, core, hands, tracker, ros, pub)
    except KeyboardInterrupt:
        pass
    finally:
        if isinstance(camera, LocalCamera):
            camera.cap.release()
        cv2.destroyAllWindows()
        if ros is not None:
            ros.terminate()
        print("[vision] 已退出", flush=True)


def loop(args, camera, core, hands, tracker, ros, pub) -> None:
    frames, started, last_line = 0, time.monotonic(), ""
    while ros is None or ros.is_connected:
        image, stamp = camera.read()
        if image is None:
            continue
        result = core.analyze(image)
        landmarks = hands.detect(image)
        result["hand"] = classify(landmarks) if landmarks is not None else None
        result["gesture"] = tracker.update(landmarks, time.monotonic())
        result["stamp"] = stamp
        if pub is not None:
            import roslibpy

            pub.publish(roslibpy.Message({"data": json.dumps(result)}))
        frames += 1
        line = (f"人 {len(result['persons'])}，人脸 {len(result['faces'])}，正对 {result['facing']}，"
                f"手 {result['hand']}")
        if line != last_line:
            print(f"[vision] {time.strftime('%H:%M:%S')} {line}", flush=True)
            last_line = line
        if result["gesture"]:
            print(f"[vision] 手势事件：{result['gesture']}", flush=True)
        if args.show and not draw(image, result, landmarks):
            return
        if frames % 50 == 0:
            print(f"[vision] {frames / (time.monotonic() - started):.1f} 帧/秒", flush=True)
    print("[vision] rosbridge 断开", flush=True)


if __name__ == "__main__":
    main()
