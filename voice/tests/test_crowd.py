"""PLAN M5.5: arrival, audience and goodbye driven by lidar people (/guide/people).

A fake robot stands in for rosbridge; each test is one scripted situation.
"""

import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import crowd  # noqa: E402
from config import Settings  # noqa: E402
from dialog import GuideDialog, Style  # noqa: E402
from engine import GuideEngine  # noqa: E402
from exhibits import load_exhibits  # noqa: E402
from llm import ToolCall  # noqa: E402

E03 = load_exhibits()[2]
ADULT = {"distance": 1.8, "child": False, "group": 1}
CHILD = {"distance": 1.4, "child": True, "group": 1}


class FakeRobot:
    connected = True

    def __init__(self, people=None):
        self.people = people          # None = no lidar people data
        self.status = {"state": "arrived"}
        self.calls = []

    def people_near(self, radius=3.0):
        if self.people is None:
            return None
        return [p for p in self.people if p["distance"] <= radius]

    def face(self):
        self.calls.append("face")

    def gesture(self, name, exhibit_id=""):
        self.calls.append(name)
        return "ok"

    def set_present_mode(self, mode):
        self.calls.append(f"mode:{mode}")

    def close(self): ...


def make(people=None, mode="point", wake=False):
    settings = Settings("deepseek", "m", "Realtek", "offline", "v", "localhost", 9090)
    log = []
    engine = GuideEngine(settings, use_robot=False, speak=False, listen=False, offline=True,
                         wake_word=wake, ui=lambda kind, text="": log.append((kind, text)))
    engine.robot = engine.dialog.robot = FakeRobot(people)
    engine.set_present_mode(mode)
    return engine, log


def events(engine):
    return [text for kind, text in list(engine.inputs.queue) if kind == "event"]


def spoken(log):
    return [text for kind, text in log if kind == "reply_chunk"]


def test_without_people_data_arrival_is_unchanged():
    engine, log = make(people=None)
    engine._on_robot_event("arrived", E03)
    assert len(events(engine)) == 1 and "已到达" in events(engine)[0]
    assert crowd.FOLLOW_PROMPT not in spoken(log)


def test_nobody_asks_then_skips_the_explanation():
    engine, log = make(people=[{"distance": 5.0, "child": False, "group": 1}])
    engine._arrive(E03, wait_s=0.3, poll_s=0.05)
    assert spoken(log) == [crowd.FOLLOW_PROMPT] and events(engine) == []
    assert engine.presenting is None


def test_someone_comes_while_waiting_then_face_and_present():
    engine, log = make(people=[], mode="face")
    threading.Timer(0.15, lambda: setattr(engine.robot, "people", [ADULT])).start()
    engine._arrive(E03, wait_s=2.0, poll_s=0.05)
    assert spoken(log) == [crowd.FOLLOW_PROMPT]
    assert "face" in engine.robot.calls and engine.presenting is E03
    assert "面向观众" in events(engine)[0] and "不要用 point" in events(engine)[0]


def test_new_goal_while_waiting_stops_waiting():
    engine, _ = make(people=[])
    threading.Timer(0.1, lambda: engine.robot.status.update(state="moving")).start()
    started = time.monotonic()
    engine._arrive(E03, wait_s=5.0, poll_s=0.05)
    assert time.monotonic() - started < 1.0 and events(engine) == []


def test_single_child_gets_child_mode():
    engine, _ = make(people=[CHILD])
    engine._arrive(E03)
    assert engine.style.audience == "child" and engine.presenting is E03
    assert "请开始讲解" in events(engine)[0]


def test_child_with_adult_or_two_children_stay_normal():
    for people in ([CHILD, ADULT], [CHILD, dict(CHILD, distance=2.0)], [dict(CHILD, group=2)]):
        engine, _ = make(people=people)
        engine.style.update("child")
        engine._arrive(E03)
        assert engine.style.audience == "normal", people


def test_manual_choice_is_not_overridden():
    engine, log = make(people=[CHILD])
    engine.set_style(audience="expert")          # operator in the GUI
    engine._arrive(E03)
    assert engine.style.audience == "expert"
    engine, _ = make(people=[CHILD])
    engine.dialog.execute(ToolCall("1", "set_style", {"audience": "normal"}))   # visitor asked
    engine._arrive(E03)
    assert engine.style.audience == "normal"


def test_everyone_leaves_then_wave_and_goodbye_once():
    engine, log = make(people=[ADULT], wake=True)
    engine._arrive(E03)
    engine.robot.people = []
    engine._crowd_tick(100.0)
    engine._crowd_tick(100.0 + crowd.GONE_S - 0.1)
    assert "wave" not in engine.robot.calls
    engine._crowd_tick(100.0 + crowd.GONE_S)
    engine._crowd_tick(100.0 + crowd.GONE_S + 1)
    assert engine.robot.calls.count("wave") == 1 and spoken(log).count(crowd.GOODBYE) == 1
    assert engine.presenting is None and engine._sleep_after_turn


def test_brief_dropout_does_not_say_goodbye():
    engine, _ = make(people=[ADULT])
    engine._arrive(E03)
    engine.robot.people = []
    engine._crowd_tick(0.0)
    engine.robot.people = [ADULT]
    engine._crowd_tick(3.0)
    engine.robot.people = []
    engine._crowd_tick(4.0)
    engine._crowd_tick(8.0)
    assert "wave" not in engine.robot.calls


def test_leaving_for_next_exhibit_is_not_a_goodbye():
    engine, _ = make(people=[ADULT])
    engine._arrive(E03)
    engine.robot.status["state"] = "moving"
    engine.robot.people = []
    engine._crowd_tick(0.0)
    engine._crowd_tick(10.0)
    assert engine.presenting is None and "wave" not in engine.robot.calls


def test_only_an_audience_request_locks_the_mode():
    asked = []
    dialog = GuideDialog(None, FakeRobot(), log=lambda _: None, style=Style(),
                         on_audience_request=asked.append)
    dialog.execute(ToolCall("1", "set_style", {"humor": 8}))
    assert asked == []
    dialog.execute(ToolCall("2", "set_style", {"audience": "normal"}))
    assert asked == ["normal"]
