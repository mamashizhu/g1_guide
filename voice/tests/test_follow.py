"""跟随：「跟我来」→ /nav/goal_key follow[:id]，跟丢/找不到人的系统事件。"""

import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from dialog import NAV_TOOLS, event_message  # noqa: E402
from exhibits import load_exhibits  # noqa: E402
from fallback import RuleBackend  # noqa: E402
from robot import EVENT_STATES, RobotLink  # noqa: E402

EXHIBITS = load_exhibits()
FRONT = {"box": [300, 100, 330, 130], "conf": 0.9, "distance": 2.0, "bearing_deg": 2.0, "facing": True}


def ask(text):
    out = []
    calls = RuleBackend(EXHIBITS).send(text, out.append)
    return "".join(out), [c.name for c in calls]


@pytest.mark.parametrize("text", ["跟我来", "你跟着我走吧", "小吉跟我走", "跟上我"])
def test_follow_intents(text):
    reply, calls = ask(text)
    assert calls == ["follow_visitor"] and "跟着你" in reply


@pytest.mark.parametrize("text", ["别跟了", "别跟着我了", "停下"])
def test_stop_following(text):
    assert ask(text)[1] == ["stop_robot"]


def test_follow_is_a_nav_tool_and_its_endings_are_events():
    assert "follow_visitor" in NAV_TOOLS
    assert {"follow_lost", "no_target"} <= EVENT_STATES
    assert "跟丢" in event_message("follow_lost", None)
    assert "没找到要跟随" in event_message("no_target", None)
    assert "跟丢" in ask(event_message("follow_lost", None))[0]


class FakeLink(RobotLink):
    """RobotLink that records what it would publish."""

    def __init__(self):
        super().__init__(EXHIBITS, on_event=None, log=lambda _: None)
        self.sent = []

    @property
    def connected(self):
        return True

    def _publish(self, data):
        self.sent.append(data)


def test_follow_targets_the_visitor_facing_the_robot():
    robot = FakeLink()
    robot.people = {"people": [{"id": 7, "distance": 2.0, "bearing_deg": 0.0}]}
    robot.people_at = time.monotonic()
    robot.on_vision({"persons": [], "faces": [FRONT]})
    robot.follow()
    assert robot.sent == ["follow:7"]


def test_follow_without_a_face_takes_whoever_is_ahead():
    robot = FakeLink()
    robot.on_vision({"persons": [], "faces": []})
    robot.follow()
    assert robot.sent == ["follow"]


def test_follow_needs_the_robot():
    assert RobotLink(EXHIBITS, log=lambda _: None).follow().startswith("失败")
