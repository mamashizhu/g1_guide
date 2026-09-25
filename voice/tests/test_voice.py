import json
import sys
from pathlib import Path
from types import SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from dialog import GuideDialog, Style, build_system_prompt, build_tools, event_message  # noqa: E402

TAG = Style().tag()
from exhibits import load_exhibits  # noqa: E402
from llm import OpenAICompatBackend, ToolCall, _trim  # noqa: E402
from robot import RobotLink  # noqa: E402
from tts import SentenceBuffer, clean_for_speech  # noqa: E402


# ---------------- exhibits / prompt ----------------

def test_exhibits_match_scene_and_have_knowledge():
    exhibits = load_exhibits()
    assert [e.exhibit_id for e in exhibits] == [f"E0{i}" for i in range(1, 7)]
    assert [e.station_key for e in exhibits] == [str(i) for i in range(1, 7)]
    for e in exhibits:
        assert e.title in e.knowledge
        assert e.knowledge.count("- 问：") >= 6


def test_prompt_and_tool_enum_cover_all_exhibits():
    exhibits = load_exhibits()
    prompt = build_system_prompt(exhibits)
    goto = build_tools(exhibits)[0]
    assert goto.parameters["properties"]["exhibit_id"]["enum"] == [e.exhibit_id for e in exhibits]
    assert all(e.title in prompt for e in exhibits)


# ---------------- speech text ----------------

def test_sentence_buffer_releases_complete_sentences():
    buf = SentenceBuffer()
    assert buf.feed("好的，请跟") == []
    assert buf.feed("我来。地球仪在") == ["好的，请跟我来。"]
    assert buf.flush() == ["地球仪在"]


def test_sentence_buffer_breaks_long_clause_at_comma():
    buf = SentenceBuffer(soft_limit=10)
    out = buf.feed("这是一个非常非常长的句子，没有句号还在继续说")
    assert out == ["这是一个非常非常长的句子，"]


def test_clean_for_speech_drops_markdown_and_emoji():
    assert clean_for_speech("**地球仪** 🌍 很有趣") == "地球仪  很有趣"
    assert clean_for_speech("前往 E06 地球仪") == "前往 六号展品 地球仪"


# ---------------- dialog orchestration ----------------

class FakeRobot:
    def __init__(self):
        self.calls = []

    def goto(self, exhibit_id):
        self.calls.append(("goto", exhibit_id))
        return f"已出发前往 {exhibit_id}"

    def stop(self):
        self.calls.append(("stop",))
        return "已停止"

    def describe_status(self):
        return "状态 moving"


class ScriptedBackend:
    """Each send() pops (text, calls) from the script."""

    def __init__(self, script):
        self.script = list(script)
        self.sent = []
        self.results = []

    def send(self, text, on_text):
        self.sent.append(text)
        reply, calls = self.script.pop(0)
        for ch in reply:
            on_text(ch)
        return calls

    def add_tool_results(self, results):
        self.results.extend(results)


def test_goto_tool_drives_robot_without_extra_round_trip():
    robot = FakeRobot()
    backend = ScriptedBackend([("好的，请跟我来。", [ToolCall("1", "goto_exhibit", {"exhibit_id": "E06"})])])
    reply = GuideDialog(backend, robot, log=lambda _: None).turn("带我去看地球仪", lambda _: None)
    assert reply == "好的，请跟我来。"
    assert robot.calls == [("goto", "E06")]
    assert backend.sent == [f"{TAG} 带我去看地球仪"]
    assert backend.results[0][1] == "已出发前往 E06"


def test_gesture_with_partial_speech_lets_model_continue():
    robot = FakeRobot()
    robot.gesture = lambda name, exhibit_id="": f"已发出手势 {name}"
    backend = ScriptedBackend([("我们到了。", [ToolCall("1", "gesture", {"name": "point"})]),
                               ("这就是植物细胞。", [])])
    reply = GuideDialog(backend, robot, log=lambda _: None).turn("[系统事件] 已到达", lambda _: None)
    assert reply == "我们到了。这就是植物细胞。"
    assert backend.sent == [f"{TAG} [系统事件] 已到达", None]


def test_silent_tool_call_gets_follow_up_for_speech():
    robot = FakeRobot()
    backend = ScriptedBackend([("", [ToolCall("1", "stop_robot", {})]), ("好的，我停下了。", [])])
    reply = GuideDialog(backend, robot, log=lambda _: None).turn("停", lambda _: None)
    assert reply == "好的，我停下了。"
    assert backend.sent == [f"{TAG} 停", None]


def test_unknown_tool_is_reported_not_raised():
    dialog = GuideDialog(ScriptedBackend([]), FakeRobot(), log=lambda _: None)
    assert "未知工具" in dialog.execute(ToolCall("1", "fly", {}))


# ---------------- robot link ----------------

def test_robot_reports_each_event_once_per_goal():
    events = []
    link = RobotLink(load_exhibits(), on_event=lambda s, e: events.append((s, e and e.exhibit_id)))
    for state in ["requested", "moving", "blocked", "moving", "blocked", "arrived", "arrived"]:
        link._on_status({"data": json.dumps({"state": state, "goal_key": "6"})})
    link._on_status({"data": json.dumps({"state": "requested", "goal_key": "1"})})
    link._on_status({"data": json.dumps({"state": "arrived", "goal_key": "1"})})
    assert events == [("blocked", "E06"), ("arrived", "E06"), ("arrived", "E01")]


def test_robot_goto_validates_and_needs_connection():
    link = RobotLink(load_exhibits())
    assert "没有展品" in link.goto("E99")
    assert "未连接" in link.goto("E01")
    assert "没有手势" in link.gesture("dance")
    assert "未连接" in link.gesture("wave")


def test_gesture_tool_matches_gesture_config():
    import re
    from robot import GESTURES
    config = (Path(__file__).resolve().parents[2] / "ws/src/g1_guide/config/gestures.yaml").read_text(encoding="utf-8")
    section = config[config.index("\ngestures:"):]
    defined = set(re.findall(r"^  ([a-z_]+):\s*$", section, re.M))
    assert set(GESTURES) <= defined


def test_event_message_names_exhibit():
    e = load_exhibits()[5]
    assert "E06" in event_message("arrived", e) and "[系统事件]" in event_message("arrived", e)


# ---------------- OpenAI-compatible backend message format ----------------

def _chunk(content=None, tool_calls=None):
    return NS(choices=[NS(delta=NS(content=content, tool_calls=tool_calls))])


def _tc(index, id=None, name=None, args=None):
    return NS(index=index, id=id, function=NS(name=name, arguments=args))


def test_openai_backend_assembles_streamed_tool_call():
    backend = OpenAICompatBackend.__new__(OpenAICompatBackend)
    backend.model, backend.system, backend.tools, backend.messages = "m", "sys", [], []
    stream = [_chunk("好的"), _chunk("，走吧。"),
              _chunk(tool_calls=[_tc(0, "call_1", "goto_exhibit", '{"exhibit_')]),
              _chunk(tool_calls=[_tc(0, None, None, 'id": "E03"}')])]
    backend.client = NS(chat=NS(completions=NS(create=lambda **kw: iter(stream))))
    pieces = []
    calls = backend.send("去看细胞", pieces.append)
    assert "".join(pieces) == "好的，走吧。"
    assert calls == [ToolCall("call_1", "goto_exhibit", {"exhibit_id": "E03"})]
    backend.add_tool_results([(calls[0], "已出发")])
    assert [m["role"] for m in backend.messages] == ["user", "assistant", "tool"]
    assert backend.messages[1]["tool_calls"][0]["id"] == "call_1"


def test_trim_cuts_at_user_turn():
    msgs = [{"role": "user"}, {"role": "assistant"}, {"role": "tool"}] * 20
    trimmed = _trim(msgs, lambda m: m["role"] == "user")
    assert trimmed[0]["role"] == "user" and len(trimmed) <= 30
