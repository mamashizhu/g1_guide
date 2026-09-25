"""Link to the simulated robot through rosbridge (WSL side: rosbridge_websocket on port 9090).

Navigation reuses the existing interface of exhibition_nav:
  publish  /nav/goal_key  std_msgs/String  "1".."6" or "cancel"
  subscribe /nav/status   std_msgs/String  JSON {"state": ..., "goal_key": ...}
Arm gestures go to gesture_player:
  publish  /guide/gesture std_msgs/String  JSON {"name": "point", "exhibit": "E03"}
"""

from __future__ import annotations

import json
import threading
from typing import Callable

from exhibits import Exhibit

# States worth telling the dialog about when the robot enters them.
EVENT_STATES = {"arrived", "blocked", "invalid_goal", "sensor_stale"}
# Names defined in ws/src/g1_guide/config/gestures.yaml.
GESTURES = ("wave", "point", "present", "explain_loop", "emphasize", "invite", "home")


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
        roslibpy.Topic(self._ros, "/nav/status", "std_msgs/String").subscribe(self._on_status)
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

    def describe_status(self) -> str:
        if not self.connected:
            return "机器人未连接"
        with self._lock:
            status = dict(self.status)
        exhibit = self.by_key.get(str(status.get("goal_key")))
        target = f"，目标 {exhibit.exhibit_id} {exhibit.title}" if exhibit else ""
        return f"状态 {status.get('state')}{target}"

    # ---- internals ----
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
