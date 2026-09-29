"""Link to the simulated robot through rosbridge (WSL side: rosbridge_websocket on port 9090).

Navigation reuses the existing interface of exhibition_nav:
  publish  /nav/goal_key  std_msgs/String  "1".."6" or "cancel"
  subscribe /nav/status   std_msgs/String  JSON {"state": ..., "goal_key": ...}
Arm gestures go to gesture_player:
  publish  /guide/gesture std_msgs/String  JSON {"name": "point", "exhibit": "E03"}
People and presentation mode (PLAN M5.2 / M5.5):
  subscribe /guide/people       std_msgs/String  JSON from people_tracker (lidar:=mid360 only)
  publish  /guide/present_mode  std_msgs/String  "point" | "face", resent before every goal
  publish  /nav/goal_key "face": turn in place to the people here
Front camera (PLAN M5.9): subscribe /guide/vision  std_msgs/String  JSON from vision/watch.py
"""

from __future__ import annotations

import json
import threading
import time
from typing import Callable

from crowd import facing_people
from exhibits import Exhibit

# States worth telling the dialog about when the robot enters them.
EVENT_STATES = {"arrived", "blocked", "invalid_goal", "sensor_stale"}
# Names defined in ws/src/g1_guide/config/gestures.yaml.
GESTURES = ("wave", "point", "present", "explain_loop", "emphasize", "invite", "home")
PRESENT_MODES = {"point": "指向展品", "face": "面向观众"}
PEOPLE_STALE_S = 1.5  # older /guide/people means no lidar people data (e.g. lidar:=simple)
VISION_STALE_S = 2.0  # older /guide/vision means the vision process is not running


class RobotLink:
    def __init__(self, exhibits: list[Exhibit], host: str = "localhost", port: int = 9090,
                 on_event: Callable[[str, Exhibit | None], None] | None = None,
                 log: Callable[[str], None] = print):
        self.by_id = {e.exhibit_id: e for e in exhibits}
        self.by_key = {e.station_key: e for e in exhibits}
        self.host, self.port = host, port
        self.on_event = on_event
        self.log = log
        self.status: dict = {"state": "unknown"}
        self._reported: set[str] = set()
        self._ros = None
        self._goal_pub = None
        self._gesture_pub = None
        self._mode_pub = None
        self.present_mode = "point"
        self.people: dict = {}
        self.people_at = -1e9
        self.vision: dict = {}
        self.vision_at = -1e9
        self.facing_at = -1e9   # last frame with a frontal face of someone within 3 m
        self.on_gesture: Callable[[str], None] | None = None   # PLAN M5.1 gesture events
        self._lock = threading.Lock()

    @property
    def connected(self) -> bool:
        return self._ros is not None and self._ros.is_connected

    def connect(self, timeout: float = 3.0) -> bool:
        import roslibpy

        try:
            self._ros = roslibpy.Ros(host=self.host, port=self.port)
            self._ros.run(timeout=timeout)
        except Exception as exc:
            self.log(f"[robot] 连不上 rosbridge ws://{self.host}:{self.port}（{exc}），仅语音模式")
            self._ros = None
            return False
        self._goal_pub = roslibpy.Topic(self._ros, "/nav/goal_key", "std_msgs/String")
        self._goal_pub.advertise()
        self._gesture_pub = roslibpy.Topic(self._ros, "/guide/gesture", "std_msgs/String")
        self._gesture_pub.advertise()
        self._mode_pub = roslibpy.Topic(self._ros, "/guide/present_mode", "std_msgs/String")
        self._mode_pub.advertise()
        roslibpy.Topic(self._ros, "/nav/status", "std_msgs/String").subscribe(self._on_status)
        roslibpy.Topic(self._ros, "/guide/people", "std_msgs/String",
                       throttle_rate=200).subscribe(self._on_people)
        roslibpy.Topic(self._ros, "/guide/vision", "std_msgs/String").subscribe(self._on_vision)
        self.log(f"[robot] 已连接 rosbridge ws://{self.host}:{self.port}")
        return True

    def close(self) -> None:
        if self._ros is not None:
            self._ros.terminate()

    # ---- commands (return a short Chinese result string for the LLM) ----
    def goto(self, exhibit_id: str) -> str:
        exhibit = self.by_id.get(exhibit_id)
        if exhibit is None:
            return f"失败：没有展品 {exhibit_id}，可选 {', '.join(self.by_id)}"
        if not self.connected:
            return "失败：机器人未连接，无法移动"
        self._send_mode()
        self._publish(exhibit.station_key)
        return f"已出发前往 {exhibit.exhibit_id} {exhibit.title}，到达后会收到系统事件"

    def stop(self) -> str:
        if not self.connected:
            return "失败：机器人未连接"
        self._publish("cancel")
        return "已停止"

    def gesture(self, name: str, exhibit_id: str = "") -> str:
        if name not in GESTURES:
            return f"失败：没有手势 {name}，可选 {', '.join(GESTURES)}"
        if not self.connected:
            return "失败：机器人未连接"
        import roslibpy

        request = {"name": name}
        if exhibit_id in self.by_id:
            request["exhibit"] = exhibit_id
        self._gesture_pub.publish(roslibpy.Message({"data": json.dumps(request)}))
        return f"已发出手势 {name}（移动中只会执行 home）"

    def set_present_mode(self, mode: str) -> None:
        if mode in PRESENT_MODES:
            self.present_mode = mode
            self._send_mode()

    def face(self) -> None:
        """Turn in place to the people around (讲解模式「面向观众」)."""
        if self.connected:
            self._send_mode()
            self._publish("face")

    def people_near(self, radius: float = 3.0) -> list[dict] | None:
        """People within ``radius``; None when there is no fresh lidar people data."""
        with self._lock:
            if time.monotonic() - self.people_at > PEOPLE_STALE_S:
                return None
            return [p for p in self.people.get("people", ()) if p.get("distance", 1e9) <= radius]

    @property
    def vision_live(self) -> bool:
        return time.monotonic() - self.vision_at <= VISION_STALE_S

    def facing_recently(self, window: float = 2.0) -> bool:
        """Someone within 3 m looked at the robot in the last ``window`` seconds."""
        return time.monotonic() - self.facing_at <= window

    def describe_status(self) -> str:
        if not self.connected:
            return "机器人未连接"
        with self._lock:
            status = dict(self.status)
        exhibit = self.by_key.get(str(status.get("goal_key")))
        target = f"，目标 {exhibit.exhibit_id} {exhibit.title}" if exhibit else ""
        return f"状态 {status.get('state')}{target}"

    # ---- internals ----
    def _send_mode(self) -> None:
        if self._mode_pub is not None and self.connected:
            import roslibpy

            self._mode_pub.publish(roslibpy.Message({"data": self.present_mode}))

    def _on_people(self, message: dict) -> None:
        try:
            payload = json.loads(message["data"])
        except (KeyError, json.JSONDecodeError):
            return
        with self._lock:
            self.people = payload
            self.people_at = time.monotonic()

    def on_vision(self, vision: dict) -> None:
        """A /guide/vision result (also called directly by tests)."""
        facing = facing_people(vision, self.people_near())
        now = time.monotonic()
        with self._lock:
            self.vision = {**vision, "facing_near": facing}
            self.vision_at = now
            if facing:
                self.facing_at = now
        if vision.get("gesture") and self.on_gesture:
            self.on_gesture(str(vision["gesture"]))

    def _on_vision(self, message: dict) -> None:
        try:
            self.on_vision(json.loads(message["data"]))
        except (KeyError, TypeError, json.JSONDecodeError):
            return

    def _publish(self, data: str) -> None:
        import roslibpy

        self._goal_pub.publish(roslibpy.Message({"data": data}))

    def _on_status(self, message: dict) -> None:
        try:
            status = json.loads(message["data"])
        except (KeyError, json.JSONDecodeError):
            return
        with self._lock:
            self.status = status
        state = status.get("state")
        if state == "requested":
            self._reported.clear()  # new goal: each event may fire once again
        elif state in EVENT_STATES and state not in self._reported:
            self._reported.add(state)  # "blocked" repeats while A* retries; report it once per goal
            if self.on_event:
                self.on_event(state, self.by_key.get(str(status.get("goal_key"))))
