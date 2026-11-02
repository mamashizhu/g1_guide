"""语音端 ↔ 网页控制台的指令（web_link.apply_command / state_payload）。"""

from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from dialog import Style  # noqa: E402
from web_link import apply_command, state_payload  # noqa: E402


class FakeEngine:
    def __init__(self):
        self.style = Style()
        self.texts, self.logs = [], []
        self.interrupted = 0
        self.present_mode, self.wake_mode = "point", "direct"
        self.wake_word, self.crowd_enabled, self.mic_enabled, self.barge_in = False, True, True, False
        self._awake_until = 99.0
        self.listen, self.listener, self.state, self.awake = True, None, "listening", True
        self.backend = SimpleNamespace(primary=None, online=False)
        self.settings = SimpleNamespace(provider="deepseek")
        self.robot = SimpleNamespace(vision_live=False)
        self.exhibits = [SimpleNamespace(exhibit_id="E01", title="清明上河图", station_key="1")]

    def submit_text(self, text): self.texts.append(text)
    def interrupt(self): self.interrupted += 1
    def set_style(self, audience=None, humor=None): self.style.update(audience, humor)
    def set_present_mode(self, mode): self.present_mode = mode
    def set_wake_mode(self, mode): self.wake_mode = mode
    def log(self, text): self.logs.append(text)


def test_text_interrupt_and_style():
    engine = FakeEngine()
    assert apply_command(engine, {"cmd": "text", "text": "带我去看青花瓷"})
    assert apply_command(engine, {"cmd": "interrupt"})
    assert apply_command(engine, {"cmd": "style", "audience": "child"})
    assert apply_command(engine, {"cmd": "style", "humor": 42})
    assert engine.texts == ["带我去看青花瓷"]
    assert engine.interrupted == 1
    assert (engine.style.audience, engine.style.humor) == ("child", 10)


def test_toggles():
    engine = FakeEngine()
    apply_command(engine, {"cmd": "wake_word", "on": True})
    assert engine.wake_word and engine._awake_until == 0.0
    apply_command(engine, {"cmd": "crowd", "on": False})
    apply_command(engine, {"cmd": "mic", "on": False})
    apply_command(engine, {"cmd": "barge_in", "on": True})
    apply_command(engine, {"cmd": "present_mode", "mode": "face"})
    apply_command(engine, {"cmd": "wake_mode", "mode": "vision"})
    assert (engine.crowd_enabled, engine.mic_enabled, engine.barge_in) == (False, False, True)
    assert (engine.present_mode, engine.wake_mode) == ("face", "vision")


def test_history_and_unknown():
    engine = FakeEngine()
    asked = []
    assert apply_command(engine, {"cmd": "history"}, lambda: asked.append(1))
    assert asked == [1]
    assert not apply_command(engine, {"cmd": "fly"})


def test_state_payload():
    state = state_payload(FakeEngine())
    assert state["kind"] == "state"
    assert state["llm"] == "离线规则"
    assert state["level"] == 0.0
    assert state["exhibits"] == [{"id": "E01", "title": "清明上河图", "key": "1"}]
