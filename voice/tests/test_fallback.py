import sys
import threading
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from config import Settings  # noqa: E402
from dialog import GuideDialog, event_message  # noqa: E402
from exhibits import load_exhibits  # noqa: E402
from fallback import RuleBackend, overlap  # noqa: E402
from llm import ResilientBackend, TurnCancelled  # noqa: E402

EXHIBITS = load_exhibits()


def ask(brain, text):
    out = []
    calls = brain.send(text, out.append)
    return "".join(out), calls


def goto_target(calls):
    return [c.args.get("exhibit_id") for c in calls if c.name == "goto_exhibit"]


# ---------------- rule brain: navigation ----------------

@pytest.mark.parametrize("text, target", [
    ("带我去看地球仪", "E06"),
    ("带我去看的球仪吧", "E06"),          # ASR homophone error seen in testing
    ("我想看看植物细胞", "E03"),
    ("去三号展品", "E03"),
    ("带我去第五个", "E05"),
    ("我想去看那个电路的海报", "E02"),
    ("带我去看太阳和地球", "E01"),        # must not pick E06 because of 地球
    ("领我去看二进制", "E04"),
])
def test_goto_intents(text, target):
    reply, calls = ask(RuleBackend(EXHIBITS), text)
    assert goto_target(calls) == [target], reply
    assert "请跟我来" in reply


def test_stop_intent():
    _, calls = ask(RuleBackend(EXHIBITS), "等一下，先别走")
    assert [c.name for c in calls] == ["stop_robot"]


def test_continue_resumes_last_goal_and_next_follows_recommendation():
    brain = RuleBackend(EXHIBITS)
    ask(brain, "带我去看地球仪")
    ask(brain, "停一下")
    _, calls = ask(brain, "好了继续走吧")
    assert goto_target(calls) == ["E06"]
    ask(brain, event_message("arrived", EXHIBITS[5]))
    _, calls = ask(brain, "下一个")
    assert goto_target(calls) == [EXHIBITS[5].next_id]


def test_question_about_exhibit_is_not_navigation():
    reply, calls = ask(RuleBackend(EXHIBITS), "地球仪为什么是斜的")
    assert calls == [] and "23.5" in reply


# ---------------- rule brain: answers ----------------

@pytest.mark.parametrize("question, expected", [
    ("阳光到地球要多久", "8 分 20 秒"),
    ("欧姆定律是什么", "I = U / R"),
    ("液泡是干什么的", "储存水分"),
    ("01001000是什么意思", "H"),
    ("活字印刷是谁发明的", "毕昇"),
    ("地球上海洋占多少", "71%"),
])
def test_faq_answers(question, expected):
    reply, _ = ask(RuleBackend(EXHIBITS), question)
    assert expected in reply


def test_context_follows_current_exhibit():
    brain = RuleBackend(EXHIBITS)
    ask(brain, event_message("arrived", EXHIBITS[2]))
    reply, _ = ask(brain, "它的细胞壁有什么用")
    assert "支撑" in reply


def test_arrival_gives_intro_recommendation_and_gesture():
    reply, calls = ask(RuleBackend(EXHIBITS), event_message("arrived", EXHIBITS[0]))
    assert reply.startswith("我们到了") and "日地系统" in reply and "推荐" in reply
    assert [(c.name, c.args) for c in calls] == [("gesture", {"name": "present", "exhibit_id": "E01"})]


def test_interest_offer_then_yes_navigates():
    brain = RuleBackend(EXHIBITS)
    reply, calls = ask(brain, "我对地理比较感兴趣")
    assert calls == [] and "带你过去" in reply
    assert goto_target(ask(brain, "好啊")[1]) == ["E06"]
    ask(brain, "那个细胞海报在哪")
    assert ask(brain, "不用了")[1] == []


def test_polite_question_form_still_navigates():
    assert goto_target(ask(RuleBackend(EXHIBITS), "能带我去看看那本大书吗")[1]) == ["E05"]


def test_named_exhibit_breaks_generic_ties():
    assert "限制电流" in ask(RuleBackend(EXHIBITS), "电阻有什么用")[0]


def test_context_does_not_make_noise_relevant():
    brain = RuleBackend(EXHIBITS)
    ask(brain, event_message("arrived", EXHIBITS[2]))
    for text in ("这个展馆有多大", "量子纠缠是怎么回事", "我饿了"):
        assert "细胞" not in ask(brain, text)[0]


def test_unknown_question_admits_uncertainty():
    reply, calls = ask(RuleBackend(EXHIBITS), "量子纠缠是怎么回事")
    assert calls == [] and ("不太确定" in reply or "只能回答" in reply)


def test_list_greet_bye_repeat():
    brain = RuleBackend(EXHIBITS)
    assert "6件展品" in ask(brain, "这里有什么展品")[0]
    assert "小吉" in ask(brain, "你好")[0]
    first = ask(brain, "液泡是干什么的")[0]
    assert ask(brain, "没听清，再说一遍")[0] == first
    assert "再见" in ask(brain, "拜拜")[0]


def test_overlap_measures_bigram_share():
    assert overlap("地球仪", "这是地球仪") == 1.0
    assert overlap("你好", "地球仪") == 0.0


# ---------------- resilient backend ----------------

class Broken:
    def __init__(self, partial=""):
        self.partial = partial
        self.calls = 0

    def send(self, text, on_text):
        self.calls += 1
        if self.partial:
            on_text(self.partial)
        raise ConnectionError("offline")

    def add_tool_results(self, results): ...
    def warmup(self): raise ConnectionError("offline")


def test_llm_failure_falls_back_and_cools_down():
    primary = Broken()
    backend = ResilientBackend(primary, RuleBackend(EXHIBITS), log=lambda _: None)
    reply, calls = ask(backend, "带我去看地球仪")
    assert goto_target(calls) == ["E06"] and not backend.online
    ask(backend, "液泡是干什么的")
    assert primary.calls == 1  # second turn skipped the dead LLM


def test_failure_after_partial_reply_apologises_instead_of_repeating():
    backend = ResilientBackend(Broken(partial="好的，"), RuleBackend(EXHIBITS), log=lambda _: None)
    reply, calls = ask(backend, "带我去看地球仪")
    assert reply == "好的，……抱歉，网络断了一下。" and calls == []


def test_no_api_key_means_offline_brain():
    backend = ResilientBackend(None, RuleBackend(EXHIBITS), log=lambda _: None)
    assert not backend.online
    assert goto_target(ask(backend, "去一号展品")[1]) == ["E01"]


def test_cancel_propagates_through_resilient_backend():
    class Streaming:
        def send(self, text, on_text):
            on_text("第一句。")
            on_text("第二句。")
            return []

        def add_tool_results(self, results): ...

    backend = ResilientBackend(Streaming(), RuleBackend(EXHIBITS), log=lambda _: None)

    def on_text(chunk):
        raise TurnCancelled

    with pytest.raises(TurnCancelled):
        backend.send("hi", on_text)
    assert backend.online  # cancelling is not a failure


def test_dialog_with_rule_brain_drives_robot():
    class Robot:
        def __init__(self):
            self.goals = []

        def goto(self, exhibit_id):
            self.goals.append(exhibit_id)
            return "已出发"

        def stop(self): return "已停止"
        def describe_status(self): return "idle"

    robot = Robot()
    dialog = GuideDialog(RuleBackend(EXHIBITS), robot, log=lambda _: None)
    assert "请跟我来" in dialog.turn("带我去看书籍", lambda _: None)
    assert robot.goals == ["E05"]


# ---------------- engine: echo filter and cancellation ----------------

def make_engine():
    from engine import GuideEngine

    settings = Settings("deepseek", "m", "Realtek", "offline", "v", "localhost", 9090)
    log = []
    engine = GuideEngine(settings, use_robot=False, speak=False, listen=False, offline=True,
                         ui=lambda kind, text="": log.append((kind, text)))
    return engine, log


def test_echo_filter_uses_recent_robot_words():
    engine, _ = make_engine()
    engine.speaker.recent_text = lambda window=4.0: "我们到了，这就是地球仪。地球仪的轴是倾斜的。"
    assert engine.is_echo("这就是地球仪")
    assert engine.is_echo("地球仪的轴是倾斜")
    assert not engine.is_echo("带我去看植物细胞")


def test_engine_runs_turns_and_interrupt_cancels_stream():
    engine, log = make_engine()
    engine.start()
    engine.submit_text("带我去看地球仪")
    engine.wait_idle()
    assert ("user", "带我去看地球仪") in log
    assert any(k == "reply_chunk" and "请跟我来" in t for k, t in log)

    gate = threading.Event()

    class Slow:
        def send(self, text, on_text):
            on_text("第一句。")
            gate.wait(2)
            on_text("第二句。")  # arrives after the interrupt
            return []

        def add_tool_results(self, results): ...

    engine.dialog.backend = Slow()
    log.clear()
    engine.submit_text("讲个长故事")
    time.sleep(0.2)
    engine.interrupt()
    gate.set()
    engine.wait_idle()
    chunks = [t for k, t in log if k == "reply_chunk"]
    assert chunks == ["第一句。"]
    assert any("打断" in t for k, t in log if k == "log")
    engine.close()


def test_stop_words_stop_robot_before_the_llm_answers():
    engine, log = make_engine()
    order = []

    class Robot:
        connected = True

        def stop(self):
            order.append("stop")
            return "已停止"

        def close(self): ...

    class Slow:
        def send(self, text, on_text):
            order.append("llm")
            on_text("好的。")
            return []

        def add_tool_results(self, results): ...

    engine.robot = Robot()
    engine.dialog.backend = Slow()
    engine.start()
    engine.submit_text("等一下，先别走")
    engine.wait_idle()
    assert order == ["stop", "llm"]
    engine.close()
