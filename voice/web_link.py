"""语音端 ↔ 网页展示控制台（ws/src/g1_guide/web，README 6.12），经同一个 rosbridge。

  publish   /guide/voice      std_msgs/String  JSON 事件：
              {"kind": "user" | "event" | "reply_start" | "reply_chunk" | "reply_end" | "log", "text", "t"}
              {"kind": "state", ...}       每 0.3 s 一次：语音状态、音量、风格、唤醒、大模型等
              {"kind": "history", "items"} 网页刚打开时补发最近的对话
  subscribe /guide/voice_cmd  std_msgs/String  JSON 指令：
              {"cmd": "text", "text"}  {"cmd": "interrupt"}  {"cmd": "style", "audience"?, "humor"?}
              {"cmd": "present_mode", "mode"}  {"cmd": "wake_word", "on"}  {"cmd": "wake_mode", "mode"}
              {"cmd": "crowd", "on"}  {"cmd": "mic", "on"}  {"cmd": "barge_in", "on"}  {"cmd": "history"}

终端版（app.py）和窗口版（gui.py）都会自动带上：GuideEngine 连上 rosbridge 后挂接。
"""

from __future__ import annotations

from collections import deque
import json
import threading
import time
from typing import Callable

STATE_PERIOD_S = 0.3
HISTORY = 200
CHAT_KINDS = {"user", "event", "reply_start", "reply_chunk", "reply_end", "log"}


def llm_status(engine) -> str:
    backend = engine.backend
    if backend.primary is None:
        return "离线规则"
    if backend.online:
        return f"{engine.settings.provider} 在线"
    return "暂时离线（规则兜底）"


def state_payload(engine) -> dict:
    listener = engine.listener
    return {
        "kind": "state",
        "t": time.time(),
        "state": engine.state,
        "level": round(float(listener.level), 4) if listener is not None else 0.0,
        "listen": engine.listen,
        "mic": engine.mic_enabled,
        "barge_in": engine.barge_in,
        "wake_word": engine.wake_word,
        "wake_mode": engine.wake_mode,
        "awake": engine.awake,
        "audience": engine.style.audience,
        "humor": engine.style.humor,
        "present_mode": engine.present_mode,
        "crowd": engine.crowd_enabled,
        "llm": llm_status(engine),
        "vision_live": engine.robot.vision_live,
        "exhibits": [{"id": e.exhibit_id, "title": e.title, "key": e.station_key}
                     for e in engine.exhibits],
    }


def apply_command(engine, command: dict, publish_history: Callable[[], None] | None = None) -> bool:
    """Run one /guide/voice_cmd request on the engine; False if it is not understood."""
    cmd = command.get("cmd")
    if cmd == "text":
        engine.submit_text(str(command.get("text", "")))
    elif cmd == "interrupt":
        engine.interrupt()
    elif cmd == "style":
        audience = command.get("audience")
        humor = command.get("humor")
        engine.set_style(audience=audience if audience else None,
                         humor=None if humor is None else max(0, min(10, int(humor))))
    elif cmd == "present_mode":
        engine.set_present_mode(str(command.get("mode")))
    elif cmd == "wake_mode":
        engine.set_wake_mode(str(command.get("mode")))
    elif cmd == "wake_word":
        on = bool(command.get("on"))
        engine.wake_word = on
        if on:
            engine._awake_until = 0.0   # like the GUI: start asleep so the wake word is needed now
        engine.log(f"[唤醒词] {'开' if on else '关'}（网页）")
    elif cmd == "crowd":
        engine.crowd_enabled = bool(command.get("on"))
        engine.log(f"[按雷达人数讲解] {'开' if engine.crowd_enabled else '关'}（网页）")
    elif cmd == "mic":
        engine.mic_enabled = bool(command.get("on"))
        engine.log(f"[麦克风] {'开' if engine.mic_enabled else '关'}（网页）")
    elif cmd == "barge_in":
        engine.barge_in = bool(command.get("on"))
        engine.log(f"[插话打断] {'开' if engine.barge_in else '关'}（网页）")
    elif cmd == "history":
        if publish_history is not None:
            publish_history()
    else:
        return False
    return True


class WebLink:
    """Mirror the engine's ui callback to /guide/voice and take orders from /guide/voice_cmd."""

    def __init__(self, engine, ros):
        import roslibpy

        self.engine = engine
        self.history: deque[dict] = deque(maxlen=HISTORY)
        self._lock = threading.Lock()
        self._running = True
        self._pub = roslibpy.Topic(ros, "/guide/voice", "std_msgs/String", queue_size=50)
        self._pub.advertise()
        roslibpy.Topic(ros, "/guide/voice_cmd", "std_msgs/String").subscribe(self._on_command)
        target = engine.ui

        def ui(kind: str, text: str = "") -> None:
            target(kind, text)
            self.forward(kind, text)

        engine.ui = ui
        threading.Thread(target=self._state_loop, daemon=True).start()

    def forward(self, kind: str, text: str = "") -> None:
        if kind not in CHAT_KINDS:
            return
        item = {"kind": kind, "text": text, "t": time.time()}
        with self._lock:
            self.history.append(item)
        self._send(item)

    def publish_history(self) -> None:
        with self._lock:
            items = list(self.history)
        self._send({"kind": "history", "items": items})

    def close(self) -> None:
        self._running = False

    def _send(self, payload: dict) -> None:
        import roslibpy

        try:
            self._pub.publish(roslibpy.Message({"data": json.dumps(payload, ensure_ascii=False)}))
        except Exception:   # rosbridge gone: the voice guide keeps running without the page
            pass

    def _state_loop(self) -> None:
        while self._running:
            try:
                self._send(state_payload(self.engine))
            except Exception:
                pass
            time.sleep(STATE_PERIOD_S)

    def _on_command(self, message: dict) -> None:
        try:
            command = json.loads(message["data"])
        except (KeyError, TypeError, json.JSONDecodeError):
            return
        if not isinstance(command, dict):
            return
        try:
            if not apply_command(self.engine, command, self.publish_history):
                self.engine.log(f"[网页] 不认识的指令 {command.get('cmd')!r}")
        except Exception as exc:
            self.engine.log(f"[网页] 指令出错 {type(exc).__name__}: {exc}")
