"""Wake word, sleep/wake session, audience modes and the humour dial."""

import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from config import Settings  # noqa: E402
from dialog import GuideDialog, Style, event_message, split_style  # noqa: E402
from engine import GuideEngine, strip_wake  # noqa: E402
from exhibits import load_exhibits  # noqa: E402
from fallback import RuleBackend  # noqa: E402
from llm import ToolCall  # noqa: E402

EXHIBITS = load_exhibits()


# ---------------- wake word ----------------

@pytest.mark.parametrize("heard, rest", [
    ("你好小吉", ""),
    ("小吉", ""),
    ("你好，小鸡。", ""),                         # ASR homophones still wake it
    ("晓吉带我去看青花瓷", "带我去看青花瓷"),
    ("小机，液泡是干什么的？", "液泡是干什么的"),
    ("你好小吉，讲简单点", "讲简单点"),
    ("嗨，小吉你好", "你好"),
])
def test_strip_wake_finds_wake_word(heard, rest):
    assert strip_wake(heard) == rest


@pytest.mark.parametrize("heard", ["你好", "带我去看青花瓷", "小心", "吉祥如意", "小朋友",
                                   "这个小机器人真可爱", "小机器人你好", "我家小鸡会唱歌"])
def test_strip_wake_ignores_other_speech(heard):
    assert strip_wake(heard) is None


def make_engine(wake=True):
    settings = Settings("deepseek", "m", "Realtek", "offline", "v", "localhost", 9090)
    log = []
    engine = GuideEngine(settings, use_robot=False, speak=False, listen=False, offline=True,
                         wake_word=wake, ui=lambda kind, text="": log.append((kind, text)))
    return engine, log


def queued(engine):
    return [text for kind, text in list(engine.inputs.queue) if kind == "user"]


def test_asleep_engine_ignores_speech_without_wake_word():
    engine, log = make_engine()
    assert engine.state == "sleeping"
    engine.handle_heard("带我去看青花瓷")
    assert queued(engine) == [] and any("待唤醒" in t for _, t in log)


def test_wake_word_alone_answers_immediately_then_stays_awake():
    engine, log = make_engine()
    engine.handle_heard("你好小吉")
    assert queued(engine) == []
    assert ("reply_chunk", "我在，请说。") in log
    engine.handle_heard("带我去看青花瓷")      # no wake word needed while awake
    assert queued(engine) == ["带我去看青花瓷"]


def test_wake_word_with_command_runs_the_command():
    engine, _ = make_engine()
    engine.handle_heard("小吉，带我去看青花瓷")
    assert queued(engine) == ["带我去看青花瓷"]


def test_session_times_out_and_goodbye_puts_it_to_sleep():
    engine, log = make_engine()
    engine.handle_heard("你好小吉")
    engine._awake_until = time.monotonic() - 1    # 30 s of silence
    assert not engine.awake and engine.state == "sleeping"

    engine.start()
    engine.handle_heard("你好小吉")
    engine.submit_text("好的谢谢，再见")
    engine.wait_idle()
    engine.keep_session()
    assert not engine.awake
    assert any("休眠" in t for _, t in log)
    engine.close()


def test_robot_walking_keeps_session_awake():
    engine, _ = make_engine()
    engine.handle_heard("你好小吉")
    engine._awake_until = time.monotonic() + 0.01
    engine.robot.status = {"state": "moving"}
    time.sleep(0.02)
    engine._awake_until = time.monotonic() + 0.5  # still inside the window when checked
    engine.keep_session()
    assert engine._awake_until > time.monotonic() + 20


def test_without_wake_word_everything_is_heard():
    engine, _ = make_engine(wake=False)
    assert engine.state != "sleeping"
    engine.handle_heard("带我去看青花瓷")
    assert queued(engine) == ["带我去看青花瓷"]


def test_typed_text_wakes_the_session():
    engine, _ = make_engine()
    engine.submit_text("你好")
    assert engine.awake


# ---------------- style ----------------

def test_style_tag_round_trip_and_clamping():
    style = Style("child", 7)
    parsed, rest = split_style(f"{style.tag()} 你好")
    assert parsed == style and rest == "你好"
    style.update(humor=42)
    assert style.humor == 10
    style.update(audience="alien")
    assert style.audience == "child"
    assert split_style("没有标签") == (None, "没有标签")


def test_set_style_tool_changes_style_and_notifies():
    seen = []

    class Robot:
        pass

    dialog = GuideDialog(None, Robot(), log=lambda _: None, on_style=seen.append)
    result = dialog.execute(ToolCall("1", "set_style", {"audience": "expert", "humor": 8}))
    assert dialog.style == Style("expert", 8) and seen == [dialog.style]
    assert "专业模式" in result


def test_every_turn_carries_the_style_tag():
    sent = []

    class Backend:
        def send(self, text, on_text):
            sent.append(text)
            return []

        def add_tool_results(self, results): ...

    dialog = GuideDialog(Backend(), None, log=lambda _: None, style=Style("child", 9))
    dialog.turn("你好", lambda _: None)
    assert sent == ["[风格：儿童模式，幽默9/10] 你好"]


def test_engine_set_style_slows_offline_speech_for_children():
    engine, _ = make_engine(wake=False)

    class Tts:
        speed = 1.0

    engine.speaker.engine = Tts()
    engine.set_style(audience="child")
    assert engine.speaker.engine.speed < 1.0
    engine.set_style(audience="normal")
    assert engine.speaker.engine.speed == 1.0


# ---------------- offline brain follows the style ----------------

def ask(brain, text):
    out = []
    calls = brain.send(text, out.append)
    return "".join(out), calls


@pytest.mark.parametrize("text, change", [
    ("讲简单点吧", {"audience": "child"}),
    ("我家小朋友听不懂", {"audience": "child"}),
    ("能讲专业一点吗", {"audience": "expert"}),
    ("正常讲就行", {"audience": "normal"}),
    ("幽默一点嘛", {"humor": 6}),
    ("严肃点", {"humor": 0}),
])
def test_offline_brain_switches_style(text, change):
    reply, calls = ask(RuleBackend(EXHIBITS), f"{Style().tag()} {text}")
    assert [(c.name, c.args) for c in calls] == [("set_style", change)] and reply


def test_offline_arrival_length_follows_audience():
    lengths = {}
    for audience in ("child", "normal", "expert"):
        reply, _ = ask(RuleBackend(EXHIBITS), f"{Style(audience).tag()} {event_message('arrived', EXHIBITS[0])}")
        lengths[audience] = len(reply)
    assert lengths["child"] < lengths["normal"] < lengths["expert"]


@pytest.mark.parametrize("chunks, spoken", [
    (["[风格：普通模式，幽默3/10]", "\n\n", "青花瓷", "是蓝的。"], "青花瓷是蓝的。"),
    (["[风", "格：儿童模式，幽默", "3/10] 你知道吗？"], "你知道吗？"),
    (["你好", "呀"], "你好呀"),
    (["[系统] 保留"], "[系统] 保留"),
])
def test_echoed_style_tag_is_never_spoken(chunks, spoken):
    from dialog import TagFilter

    out = []
    emit = TagFilter(out.append)
    for chunk in chunks:
        emit(chunk)
    assert "".join(out) == spoken
