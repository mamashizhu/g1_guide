"""网页展示控制台的服务端（README 5.10 / 6.12）：http://localhost:8080

一个 ROS 节点 + 一个 HTTP 服务：

* ``/``：``share/g1_guide/web`` 里的静态页面。页面自己连 rosbridge（默认 9090）
  收发导航、人群、视觉、语音等 JSON 话题，这里只管图像和静态资料。
* ``/stream/<名>``：相机画面的 MJPEG 流（``<img src>`` 直接显示）。**只有网页正在看
  某路画面时才订阅它**，最后一个观看者断开 3 s 后取消订阅，Gazebo 就不再渲染这个
  相机（桥接是 lazy 的，见 config/bridge.yaml 和 CLAUDE.md 的渲染泄漏说明）。
* ``/api/scene``：展馆配置（展品、站位、墙体矩形），网页画 2D 地图用。
* ``/api/media``、``/media/<路径>``：项目里的演示视频（步态训练 demo.mp4 等），支持拖动进度。

不依赖额外的包：http.server + cv2（camera_jpeg 已在用）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
import os
from pathlib import Path
import threading
import time
from typing import Sequence
from urllib.parse import unquote, urlparse

import numpy as np


# 名字 → (话题, 类型, 最高帧率)。depth 是 32FC1 米，转成伪彩色。
STREAMS = {
    "chase": ("/chase_camera/image", "rgb", 8.0),
    "overview": ("/exhibition/overview", "rgb", 2.0),
    "front": ("/front_camera/image", "rgb", 8.0),
    "head": ("/camera/image", "rgb", 5.0),
    "depth": ("/camera/depth_image", "depth", 5.0),
    "lowlevel": ("/lowlevel/overview", "rgb", 5.0),
}
UNSUBSCRIBE_AFTER_S = 3.0
DEPTH_MAX_M = 6.0
MEDIA_SUFFIXES = {".mp4", ".webm", ".png", ".jpg", ".jpeg", ".gif", ".json", ".jsonl", ".tsv"}
# 项目里展示用的资料（相对项目根目录的 glob）。
MEDIA_GLOBS = ("mujoco/policies/*/demo/*.mp4", "mujoco/policies/*/summary*.tsv",
               "mujoco/policies/*/eval_*.jsonl", "docs/media/*")


def image_to_bgr(data: bytes, width: int, height: int, step: int, encoding: str) -> np.ndarray:
    """sensor_msgs/Image (rgb8 / bgr8 / rgba8 / bgra8 / mono8) → BGR uint8."""
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(encoding)
    if channels is None:
        raise ValueError(f"unsupported encoding {encoding!r}")
    rows = np.frombuffer(data, dtype=np.uint8).reshape(height, step)
    image = rows[:, :width * channels].reshape(height, width, channels)
    if encoding == "mono8":
        return np.repeat(image, 3, axis=2)
    if encoding in ("rgb8", "rgba8"):
        return np.ascontiguousarray(image[:, :, 2::-1])
    return np.ascontiguousarray(image[:, :, :3])


def depth_to_bgr(data: bytes, width: int, height: int, step: int, encoding: str,
                 max_m: float = DEPTH_MAX_M) -> np.ndarray:
    """32FC1 (m) or 16UC1 (mm) depth → colour map; invalid pixels black."""
    import cv2

    if encoding == "32FC1":
        rows = np.frombuffer(data, dtype=np.float32).reshape(height, step // 4)
        depth = rows[:, :width].astype(np.float32)
    elif encoding == "16UC1":
        rows = np.frombuffer(data, dtype=np.uint16).reshape(height, step // 2)
        depth = rows[:, :width].astype(np.float32) / 1000.0
    else:
        raise ValueError(f"unsupported depth encoding {encoding!r}")
    valid = np.isfinite(depth) & (depth > 0.0)
    scaled = np.zeros(depth.shape, dtype=np.uint8)
    scaled[valid] = (255.0 * (1.0 - np.clip(depth[valid] / max_m, 0.0, 1.0))).astype(np.uint8)
    colour = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
    colour[~valid] = 0
    return colour


def encode_jpeg(image: np.ndarray, quality: int = 75) -> bytes:
    import cv2

    ok, buffer = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    if not ok:
        raise ValueError("JPEG encoding failed")
    return buffer.tobytes()


def placeholder_jpeg(text: str, width: int = 640, height: int = 360) -> bytes:
    import cv2

    image = np.full((height, width, 3), 32, dtype=np.uint8)
    cv2.putText(image, "waiting for", (24, height // 2 - 14), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (180, 180, 180), 2)
    cv2.putText(image, text, (24, height // 2 + 22), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (180, 180, 180), 2)
    return encode_jpeg(image)


def safe_join(root: Path, relative: str) -> Path | None:
    """``root/relative`` if it stays inside ``root`` (no ``..`` escapes).

    Paths are normalised without following symlinks: the installed web files
    are symlinks into the source tree (--symlink-install).
    """
    root = Path(os.path.normpath(os.path.abspath(root)))
    target = Path(os.path.normpath(root / unquote(relative).lstrip("/")))
    if target != root and root not in target.parents:
        return None
    return target


def parse_range(header: str | None, size: int) -> tuple[int, int] | None:
    """First range of a ``bytes=a-b`` header as inclusive (start, end); None = whole file."""
    if not header or not header.startswith("bytes=") or size <= 0:
        return None
    first = header[6:].split(",")[0].strip()
    start_text, _, end_text = first.partition("-")
    try:
        if start_text == "":
            length = int(end_text)
            start, end = max(0, size - length), size - 1
        else:
            start = int(start_text)
            end = int(end_text) if end_text else size - 1
    except ValueError:
        return None
    if start > end or start >= size:
        return None
    return start, min(end, size - 1)


WEB_VIDEO_SUFFIX = "_web.mp4"


def web_video_path(path: Path) -> Path:
    return path.with_name(path.stem + WEB_VIDEO_SUFFIX)


def make_web_video(path: Path) -> Path | None:
    """H.264 copy of an OpenCV ``mp4v`` video (browsers cannot play mp4v).

    Returns the playable file: ``path`` itself when it already is H.264, the
    ``<stem>_web.mp4`` copy otherwise (made once, redone when the source is
    newer), or None when it cannot be converted.
    """
    import cv2

    capture = cv2.VideoCapture(str(path))
    try:
        if not capture.isOpened():
            return None
        fourcc = int(capture.get(cv2.CAP_PROP_FOURCC)).to_bytes(4, "little").decode("ascii", "replace").lower()
        if fourcc in ("avc1", "h264"):
            return path
        target = web_video_path(path)
        if target.is_file() and target.stat().st_mtime >= path.stat().st_mtime:
            return target
        fps = capture.get(cv2.CAP_PROP_FPS) or 30.0
        size = (int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)), int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        partial = target.with_name(target.stem + ".part.mp4")
        writer = cv2.VideoWriter(str(partial), cv2.VideoWriter_fourcc(*"avc1"), fps, size)
        if not writer.isOpened():
            return None
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            writer.write(frame)
        writer.release()
        partial.replace(target)
        return target
    finally:
        capture.release()


def list_media(project: Path) -> list[dict]:
    items = []
    for pattern in MEDIA_GLOBS:
        for path in sorted(project.glob(pattern)):
            if path.name.endswith((WEB_VIDEO_SUFFIX, ".part.mp4")):
                continue
            if path.suffix.lower() == ".mp4" and web_video_path(path).is_file():
                path = web_video_path(path)   # the browser-playable copy
            if path.is_file() and path.suffix.lower() in MEDIA_SUFFIXES:
                parts = path.relative_to(project).parts
                group = parts[parts.index("policies") + 1] if "policies" in parts[:-1] else parts[-2]
                items.append({"path": "/".join(parts), "name": path.name, "group": group,
                              "size": path.stat().st_size, "kind": path.suffix.lower().lstrip(".")})
    return items


@dataclass
class Stream:
    """Latest JPEG of one camera, shared by every browser watching it."""

    name: str
    topic: str
    kind: str
    max_fps: float
    clients: int = 0
    last_client_at: float = 0.0
    frame: bytes | None = None
    seq: int = 0
    frame_at: float = 0.0
    last_encoded: float = 0.0
    frames: int = 0
    cond: threading.Condition = field(default_factory=threading.Condition)

    def join(self) -> None:
        with self.cond:
            self.clients += 1
            self.last_client_at = time.monotonic()

    def leave(self) -> None:
        with self.cond:
            self.clients = max(0, self.clients - 1)
            self.last_client_at = time.monotonic()

    def wanted(self, now: float) -> bool:
        with self.cond:
            return self.clients > 0 or now - self.last_client_at < UNSUBSCRIBE_AFTER_S

    def due(self, now: float) -> bool:
        return now - self.last_encoded >= 1.0 / self.max_fps

    def publish(self, jpeg: bytes, now: float) -> None:
        with self.cond:
            self.frame = jpeg
            self.seq += 1
            self.frame_at = now
            self.frames += 1
            self.cond.notify_all()

    def wait(self, after_seq: int, timeout: float) -> tuple[bytes | None, int]:
        with self.cond:
            self.cond.wait_for(lambda: self.seq != after_seq, timeout=timeout)
            return self.frame, self.seq

    def info(self, now: float) -> dict:
        with self.cond:
            return {"name": self.name, "topic": self.topic, "clients": self.clients,
                    "live": self.frame is not None and now - self.frame_at < 3.0,
                    "age_s": None if self.frame is None else round(now - self.frame_at, 1)}


def make_handler(web_root: Path, project: Path, streams: dict[str, Stream], scene: dict,
                 settings: dict):
    placeholder = {name: placeholder_jpeg(s.topic) for name, s in streams.items()}

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        server_version = "g1-guide-web"

        def log_message(self, format, *args):  # noqa: A002 - quiet access log
            return

        def _send_bytes(self, body: bytes, content_type: str, status=HTTPStatus.OK,
                        headers: dict | None = None) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-cache")
            for key, value in (headers or {}).items():
                self.send_header(key, value)
            self.end_headers()
            if self.command != "HEAD":
                self.wfile.write(body)

        def _json(self, payload) -> None:
            self._send_bytes(json.dumps(payload, ensure_ascii=False).encode("utf-8"),
                             "application/json; charset=utf-8")

        def do_HEAD(self):  # noqa: N802
            self.do_GET()

        def do_GET(self):  # noqa: N802
            path = urlparse(self.path).path
            try:
                if path.startswith("/stream/"):
                    return self._stream(path[len("/stream/"):])
                if path == "/api/scene":
                    return self._json(scene)
                if path == "/api/config":
                    return self._json(settings)
                if path == "/api/streams":
                    now = time.monotonic()
                    return self._json([s.info(now) for s in streams.values()])
                if path == "/api/media":
                    return self._json(list_media(project))
                if path.startswith("/media/"):
                    target = safe_join(project, path[len("/media/"):])
                    if target is None or target.suffix.lower() not in MEDIA_SUFFIXES:
                        return self.send_error(HTTPStatus.FORBIDDEN)
                    return self._file(target)
                target = safe_join(web_root, "index.html" if path == "/" else path)
                if target is None:
                    return self.send_error(HTTPStatus.FORBIDDEN)
                return self._file(target)
            except (BrokenPipeError, ConnectionResetError):
                return

        def _file(self, target: Path) -> None:
            if not target.is_file():
                return self.send_error(HTTPStatus.NOT_FOUND)
            size = target.stat().st_size
            content_type = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
            if content_type.startswith("text/") or content_type.endswith(("javascript", "json")):
                content_type += "; charset=utf-8"
            span = parse_range(self.headers.get("Range"), size)
            if span is None:
                self._send_bytes(target.read_bytes(), content_type,
                                 headers={"Accept-Ranges": "bytes"})
                return
            start, end = span
            with target.open("rb") as handle:
                handle.seek(start)
                body = handle.read(end - start + 1)
            self._send_bytes(body, content_type, HTTPStatus.PARTIAL_CONTENT,
                             {"Accept-Ranges": "bytes", "Content-Range": f"bytes {start}-{end}/{size}"})

        def _stream(self, name: str) -> None:
            stream = streams.get(name)
            if stream is None:
                return self.send_error(HTTPStatus.NOT_FOUND)
            boundary = "g1frame"
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={boundary}")
            self.send_header("Cache-Control", "no-cache, no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            self.close_connection = True
            stream.join()
            try:
                seq = -1
                frame = placeholder[name]
                while True:
                    self.wfile.write(f"--{boundary}\r\nContent-Type: image/jpeg\r\n"
                                     f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                    new, new_seq = stream.wait(seq, timeout=5.0)
                    if new_seq != seq and new is not None:
                        frame, seq = new, new_seq
                    elif stream.frame is None:
                        frame = placeholder[name]   # keep-alive: detects closed tabs
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                pass
            finally:
                stream.leave()

    return Handler


def project_root(web_root: Path) -> Path:
    """The project folder: share/web files are symlinks into ws/src/g1_guide/web."""
    index = (web_root / "index.html").resolve()
    return index.parents[4]


def main(args: Sequence[str] | None = None) -> None:
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    from .scene_config import load_scene

    share = Path(get_package_share_directory("g1_guide"))

    class DemoWeb(Node):
        def __init__(self):
            super().__init__("demo_web")
            self.declare_parameter("port", 8080)
            self.declare_parameter("rosbridge_port", 9090)
            self.declare_parameter("scene_config", str(share / "config/exhibition.json"))
            self.declare_parameter("jpeg_quality", 75)
            self.quality = int(self.get_parameter("jpeg_quality").value)
            self.streams = {name: Stream(name, topic, kind, fps)
                            for name, (topic, kind, fps) in STREAMS.items()}
            self.subscriptions_by_name = {}
            scene_path = self.get_parameter("scene_config").value
            scene = load_scene(scene_path)
            scene["scene_file"] = Path(scene_path).name
            web_root = share / "web"
            project = project_root(web_root)
            settings = {"rosbridge_port": int(self.get_parameter("rosbridge_port").value),
                        "streams": {n: s.topic for n, s in self.streams.items()}}
            handler = make_handler(web_root, project, self.streams, scene, settings)
            port = int(self.get_parameter("port").value)
            self.server = ThreadingHTTPServer(("0.0.0.0", port), handler)
            self.server.daemon_threads = True
            threading.Thread(target=self.server.serve_forever, daemon=True).start()
            self.create_timer(0.5, self.update_subscriptions)
            threading.Thread(target=self.prepare_videos, args=(project,), daemon=True).start()
            self.get_logger().info(f"网页控制台：http://localhost:{port}  （页面资料 {web_root}，项目 {project}）")

        def prepare_videos(self, project: Path):
            """Step-training demo videos are OpenCV mp4v: make H.264 copies once."""
            for item in list_media(project):
                path = project / item["path"]
                if item["kind"] == "mp4" and not path.name.endswith(WEB_VIDEO_SUFFIX):
                    self.get_logger().info(f"转码成网页能播放的 H.264：{item['path']}")
                    if make_web_video(path) is None:
                        self.get_logger().warning(f"转码失败，网页里播放不了：{item['path']}")

        def update_subscriptions(self):
            now = time.monotonic()
            for name, stream in self.streams.items():
                wanted = stream.wanted(now)
                active = name in self.subscriptions_by_name
                if wanted and not active:
                    self.subscriptions_by_name[name] = self.create_subscription(
                        Image, stream.topic, lambda msg, s=stream: self.on_image(s, msg),
                        qos_profile_sensor_data)
                    self.get_logger().info(f"开始订阅 {stream.topic}（网页在看 {name}）")
                elif active and not wanted:
                    self.destroy_subscription(self.subscriptions_by_name.pop(name))
                    with stream.cond:
                        stream.frame = None
                    self.get_logger().info(f"取消订阅 {stream.topic}（没人看了，Gazebo 停止渲染）")

        def on_image(self, stream: Stream, message):
            now = time.monotonic()
            if not stream.due(now):
                return
            stream.last_encoded = now
            try:
                args = (bytes(message.data), message.width, message.height, message.step,
                        message.encoding)
                image = depth_to_bgr(*args) if stream.kind == "depth" else image_to_bgr(*args)
                stream.publish(encode_jpeg(image, self.quality), now)
            except ValueError as error:
                self.get_logger().warning(f"{stream.topic}: {error}", throttle_duration_sec=10.0)

    rclpy.init(args=args)
    node = DemoWeb()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
