"""Guide engine shared by the console (app.py) and the window (gui.py).

Threads:
- turn worker: takes visitor text / robot events from ``inputs`` and runs one dialog turn
- ear: microphone → VAD → ASR → ``inputs``; also watches for barge-in while the robot talks
- rosbridge callbacks: robot status events → ``inputs``
- crowd: lidar people (/guide/people) on arrival and while presenting, see crowd.py

Barge-in (``barge_in=True``): while the robot speaks, speech on the microphone is
transcribed every 0.4 s; when two polls in a row hear at least four characters that
are not an echo of what the robot is saying, playback stops, the reply in flight is
abandoned and the new sentence is handled.  Two layers keep the robot from
interrupting itself on open speakers: WebRTC echo cancellation (aec.py) removes the
loudspeaker signal from the microphone, and the remaining text is compared with the
robot's recent words by fuzzy pinyin.
"""

from __future__ import annotations

import queue
import re
import threading
import time
from typing import Callable

import crowd
from config import Settings
from dialog import AUDIENCES, GuideDialog, Style, build_system_prompt, build_tools, event_message
from exhibits import load_exhibits
from fallback import BYE_WORDS, STOP_WORDS, RuleBackend, _has, normalize
from llm import ResilientBackend, TurnCancelled, make_backend
from robot import PRESENT_MODES, RobotLink

# 唤醒模式 (PLAN M5.9): direct = the wake word alone (the original behaviour);
# vision = the wake word only counts while the front camera sees someone within
# 3 m facing the robot.  Without vision data, vision mode falls back to direct.
WAKE_MODES = {"direct": "直接唤醒", "vision": "视觉唤醒"}
VISION_WAKE_WINDOW_S = 2.0
# 手势 (PLAN M5.1, events from vision/gestures.py).  palm = stop works at any
# time; the others only while awake and not talking (speech has priority).
GESTURE_COUNTS = {"one": "1", "two": "2", "three": "3", "four": "4"}
from tts import SentenceBuffer, Speaker, make_engine

ECHO_OVERLAP = 0.5      # heard text sharing this much with the robot's recent words is echo
BARGE_POLL_S = 0.4      # how often to test live speech during playback
BARGE_MIN_CHARS = 4     # shorter fragments are usually garbled echo
BARGE_CONFIRM = 2       # consecutive non-echo polls needed before cutting the robot off
MUTE_TAIL_S = 0.3       # without barge-in: keep the mic closed this long after speaking (room echo)
WAKE_IDLE_S = 30.0      # with a wake word: fall asleep after this long without anyone talking
CHILD_SPEED = 0.85      # offline TTS speed in child mode
MOVING_STATES = {"requested", "aligning", "moving", "blocked", "backing_up"}

UiCallback = Callable[..., None]


_DIGIT_CN = "零一二三四五六七八九"


def _int_to_cn(n: int) -> str:
    if n < 10:
        return _DIGIT_CN[n]
    if n >= 10000:
        return "".join(_DIGIT_CN[int(d)] for d in str(n))
    out, zero = "", False
    for value, unit in ((1000, "千"), (100, "百"), (10, "十"), (1, "")):
        digit = n // value % 10
        if digit:
            out += ("零" if zero and out else "") + ("" if digit == 1 and unit == "十" and not out else _DIGIT_CN[digit]) + unit
            zero = False
        else:
            zero = True
    return out


def spell_numbers(text: str) -> str:
    """23.5 → 二十三点五, so ASR digits (use_itn) sound like the robot's spoken numbers."""
    def one(match: re.Match) -> str:
        whole, _, frac = match.group(0).partition(".")
        return _int_to_cn(int(whole)) + ("点" + "".join(_DIGIT_CN[int(d)] for d in frac) if frac else "")
    return re.sub(r"\d+(?:\.\d+)?", one, text)


# Sounds a distorted echo gets confused between; comparing them as equal makes the echo test fuzzy.
_FUZZY = [(re.compile(p), r) for p, r in (
    (r"^zh", "z"), (r"^ch", "c"), (r"^sh", "s"), (r"^n", "l"), (r"^r", "l"), (r"^f", "h"),
    (r"ing$", "in"), (r"eng$", "en"), (r"ang$", "an"), (r"^x", "s"), (r"^q", "c"), (r"^j", "z"),
)]


def _fuzzy(syllable: str) -> str:
    for pattern, repl in _FUZZY:
        syllable = pattern.sub(repl, syllable)
    return syllable


WAKE_PREFIXES = ("", "你好", "您好", "嗨", "喂", "哈喽", "嘿")


def strip_wake(text: str) -> str | None:
    """Text after the wake word "小吉" (matched by sound: 小鸡/小机/晓吉 count too).

    The wake word must open the sentence, optionally after a greeting, so that
    "这个小机器人真可爱" does not wake the guide; "小机器…" never counts.
    Returns None when the wake word is absent, "" when nothing follows it.
    """
    from pypinyin import lazy_pinyin

    chars = normalize(text)
    for prefix in WAKE_PREFIXES:
        if not chars.startswith(prefix):
            continue
        rest = chars[len(prefix):]
        syllables = [lazy_pinyin(c)[0] for c in rest[:3]]
        if syllables[:2] == ["xiao", "ji"] and not (len(syllables) > 2 and syllables[2] == "qi"):
            return rest[2:]
    return None


def _syllable_pairs(text: str) -> set[tuple[str, ...]]:
    from pypinyin import lazy_pinyin

    syllables = [_fuzzy(s) for s in lazy_pinyin(normalize(spell_numbers(text)))]
    pairs = {tuple(syllables[i:i + 2]) for i in range(len(syllables) - 1)}
    return pairs or {tuple(syllables)} if syllables else set()


def pinyin_overlap(heard: str, spoken: str) -> float:
    """Share of the heard syllable pairs that also occur in what the robot said.

    Compared by sound, not by character: ASR hears our own echo with homophone
    errors (倾斜 → 清晰, 地球仪 → 秋仪) that a character comparison would miss.
    """
    heard_pairs = _syllable_pairs(heard)
    return len(heard_pairs & _syllable_pairs(spoken)) / len(heard_pairs) if heard_pairs else 0.0


class NullSpeaker:
    busy = False

    def say(self, sentence: str) -> None: ...
    def wait(self) -> None: ...
    def stop(self) -> None: ...
    def recent_text(self, window: float = 4.0) -> str: return ""


def console_ui(kind: str, text: str = "") -> None:
    if kind == "reply_start":
        print("小吉：", end="", flush=True)
    elif kind == "reply_chunk":
        print(text, end="", flush=True)
    elif kind == "reply_end":
        print()
    elif kind == "user":
        print(f"你：{text}")
    else:  # event, log
        print(text)


class GuideEngine:
    def __init__(self, settings: Settings, *, use_robot: bool = True, speak: bool = True,
                 listen: bool = True, barge_in: bool = False, gain: float = 2.0,
                 offline: bool = False, wake_word: bool = False, style: Style | None = None,
                 ui: UiCallback = console_ui):
        self.settings = settings
        self.ui = ui
        self.listen = listen
        self.barge_in = barge_in
        self.gain = gain
        self.exhibits = load_exhibits()

        primary = None if offline else make_backend(
            settings, build_system_prompt(self.exhibits), build_tools(self.exhibits))
        if primary is None:
            self.log("[llm] 离线模式：规则理解 + 知识库问答" if offline else
                     "[llm] 没有找到 API Key，使用离线模式（规则理解 + 知识库问答）")
        else:
            self.log(f"[llm] {settings.provider} / {settings.model}")
        self.backend = ResilientBackend(primary, RuleBackend(self.exhibits), log=self.log)

        self.robot = RobotLink(self.exhibits, settings.rosbridge_host, settings.rosbridge_port,
                               on_event=self._on_robot_event, log=self.log)
        self.robot.on_gesture = self.on_gesture
        self.use_robot = use_robot
        self.speaker = Speaker(make_engine(settings.tts_engine, settings.tts_voice)) if speak else NullSpeaker()
        self.style = style or Style()
        self.dialog = GuideDialog(self.backend, self.robot, log=self.log,
                                  style=self.style, on_style=self._on_style,
                                  on_audience_request=self._on_audience_request)
        self._audience_manual = False  # a visitor or the operator chose it: no auto change
        self._on_style(self.style)
        # PLAN M5.5: lidar people drive arrival and goodbye when /guide/people is live.
        self.crowd_enabled = True      # GUI toggle
        self.present_mode = "point"    # GUI: "point" 指向展品 | "face" 面向观众
        self.presenting = None         # exhibit being presented to a live audience
        self._gone_since: float | None = None
        self.aec = None
        if listen and speak:
            try:
                from aec import EchoCanceller

                self.aec = EchoCanceller()
                self.speaker.player.on_render = self.aec.render
            except ImportError:
                self.log("[回声消除] 未安装 livekit，外放时打断容易误触发：pip install livekit")

        self.inputs: queue.Queue[tuple[str, str]] = queue.Queue()
        self.cancel = threading.Event()
        self.turn_active = threading.Event()
        self.listener = None
        self.mic_enabled = True  # GUI toggle; barge_in may also be flipped at runtime
        self.wake_word = wake_word  # GUI toggle too
        self.wake_mode = "direct"   # GUI: see WAKE_MODES
        self._vision_fallback_logged = False
        self._awake_until = 0.0
        self._sleep_after_turn = False
        self._running = True
        self._ready = threading.Event()

    # ---- lifecycle ----
    def start(self) -> None:
        threading.Thread(target=self.backend.warmup, daemon=True).start()
        if self.use_robot:
            self.robot.connect()
        threading.Thread(target=self._turn_worker, daemon=True).start()
        if self.use_robot:
            threading.Thread(target=self._crowd_watch, daemon=True).start()
        if self.listen:
            threading.Thread(target=self._ear, daemon=True).start()
            self._ready.wait(timeout=30)
        self.log("请说话…" if self.listen else "可以开始对话了")

    def close(self) -> None:
        self._running = False
        self.interrupt()
        if self.listener is not None:
            self.listener.stop()
        self.robot.close()

    # ---- external controls ----
    def submit_text(self, text: str) -> None:
        """Typed input (console or GUI): needs no wake word and cuts off whatever the robot is saying."""
        text = text.strip()
        if text:
            self.wake()
            self.interrupt()
            self.inputs.put(("user", text))

    def set_style(self, audience: str | None = None, humor: int | None = None) -> None:
        """Operator (GUI) change; an audience chosen here is not auto-changed later."""
        if audience is not None:
            self._audience_manual = True
        self.style.update(audience, humor)
        self._on_style(self.style)

    def set_present_mode(self, mode: str) -> None:
        if mode in PRESENT_MODES:
            self.present_mode = mode
            self.robot.set_present_mode(mode)
            self.log(f"[讲解模式] {PRESENT_MODES[mode]}")

    def _on_audience_request(self, _audience: str) -> None:
        self._audience_manual = True  # the visitor asked for it

    def _on_style(self, style: Style) -> None:
        tts = getattr(self.speaker, "engine", None)
        if hasattr(tts, "speed"):
            tts.speed = CHILD_SPEED if style.audience == "child" else 1.0
        self.log(f"[风格] {AUDIENCES[style.audience]}模式，幽默 {style.humor}/10")

    # ---- wake word ----
    def set_wake_mode(self, mode: str) -> None:
        if mode in WAKE_MODES:
            self.wake_mode = mode
            self._vision_fallback_logged = False
            self.log(f"[唤醒模式] {WAKE_MODES[mode]}")

    @property
    def vision_gated(self) -> bool:
        """Vision wake is on and has live camera data to decide with."""
        return self.wake_mode == "vision" and self.robot.vision_live

    @property
    def wake_required(self) -> bool:
        return self.wake_word or self.wake_mode == "vision"

    @property
    def awake(self) -> bool:
        return not self.wake_required or time.monotonic() < self._awake_until

    def wake(self) -> None:
        self._awake_until = time.monotonic() + WAKE_IDLE_S

    def keep_session(self) -> None:
        """Stay awake while the robot talks, thinks or walks; fall asleep after a goodbye."""
        busy = self.speaker.busy or self.turn_active.is_set() or not self.inputs.empty()
        if busy or self.robot.status.get("state") in MOVING_STATES:
            if self.awake:
                self.wake()
        elif self._sleep_after_turn:
            self._sleep_after_turn = False
            if self.wake_required:
                self._awake_until = 0.0
                self.log("[休眠] 说“你好小吉”再叫醒我")

    def interrupt(self) -> None:
        self.cancel.set()
        self.speaker.stop()

    def wait_idle(self) -> None:
        self.inputs.join()
        self.speaker.wait()

    @property
    def state(self) -> str:
        if self.wake_required and not self.awake:
            return "sleeping"
        if self.speaker.busy:
            return "speaking"
        if self.turn_active.is_set():
            return "thinking"
        if self.listener is not None and self.listener.muted.is_set():
            return "muted"
        if self.listener is not None and self.listener.speech_active:
            return "hearing"
        return "listening" if self.listen else "idle"

    def log(self, text: str) -> None:
        self.ui("log", text)

    # ---- turns ----
    def _on_robot_event(self, state: str, exhibit) -> None:
        if state == "arrived" and self.crowd_enabled and self.robot.people_near() is not None:
            threading.Thread(target=self._arrive, args=(exhibit,), daemon=True).start()
            return
        self.inputs.put(("event", event_message(state, exhibit, self.present_mode)))

    # ---- gestures (PLAN M5.1) ----
    def on_gesture(self, name: str) -> None:
        if name == "palm":
            self.log("[手势] 手掌：停止")
            self.interrupt()
            self.presenting = None
            if self.robot.connected:
                self.log(f"[手势] {self.robot.stop()}")
            return
        if not self.awake:
            self.log(f"[手势] {name}：未唤醒，忽略")
            return
        if self.speaker.busy or self.turn_active.is_set():
            self.log(f"[手势] {name}：正在说话，语音优先，忽略")
            return
        if name in GESTURE_COUNTS:
            exhibit = self.robot.by_key.get(GESTURE_COUNTS[name])
            if exhibit is None:
                return
            text = f"带我去看{exhibit.title}"
        elif name == "thumbs_up":
            text = "好的"
        elif name == "wave":
            text = "再见" if self.presenting is not None else "你好"
        else:
            return
        self.log(f"[手势] {name} → {text}")
        self.wake()
        self.inputs.put(("user", text))

    # ---- audience (PLAN M5.5) ----
    def _arrive(self, exhibit, wait_s: float = crowd.FOLLOW_WAIT_S, poll_s: float = 0.2) -> None:
        """Arrived: present only if someone is within 3 m, waiting a little for them."""
        people = self.robot.people_near(crowd.FOLLOW_RADIUS)
        if people is None:  # lidar people data stopped: behave as before
            self.inputs.put(("event", event_message("arrived", exhibit, self.present_mode)))
            return
        if not people:
            self.log("[观众] 到站时 3 m 内没有人")
            self._say_now(crowd.FOLLOW_PROMPT)
            deadline = time.monotonic() + wait_s
            while not people:
                time.sleep(poll_s)
                if self.robot.status.get("state") in MOVING_STATES:
                    return  # sent somewhere else meanwhile
                if time.monotonic() >= deadline:
                    self.log(f"[观众] 等了 {wait_s:.0f} 秒仍没有人，不讲解")
                    return
                people = self.robot.people_near(crowd.FOLLOW_RADIUS) or []
            if self.present_mode == "face":
                self.robot.face()
        self.log(f"[观众] {crowd.describe(people)}")
        self._apply_audience(people)
        self.presenting = exhibit
        self._gone_since = None
        self.inputs.put(("event", event_message("arrived", exhibit, self.present_mode)))

    def _apply_audience(self, people: list[dict]) -> None:
        target = crowd.audience_for(people)
        if self._audience_manual:
            self.log(f"[观众] 讲解风格已手动设定，保持{AUDIENCES[self.style.audience]}模式")
        elif target != self.style.audience:
            self.style.update(target)
            self._on_style(self.style)

    def _crowd_tick(self, now: float) -> None:
        """While presenting: everybody gone for GONE_S → wave and say goodbye."""
        if self.presenting is None:
            return
        if self.robot.status.get("state") in MOVING_STATES:
            self.presenting = None  # left for the next exhibit
            return
        people = self.robot.people_near(crowd.FOLLOW_RADIUS)
        if people is None or people:
            self._gone_since = None
            return
        if self._gone_since is None:
            self._gone_since = now
        elif now - self._gone_since >= crowd.GONE_S:
            self._goodbye()

    def _goodbye(self) -> None:
        self.log("[观众] 观众都离开了，挥手告别")
        self.presenting = None
        self._gone_since = None
        self._audience_manual = False  # the next group gets its own judgement
        self.interrupt()
        self.robot.gesture("wave")
        self._say_now(crowd.GOODBYE)
        self._sleep_after_turn = True

    def _crowd_watch(self) -> None:
        while self._running:
            try:
                self._crowd_tick(time.monotonic())
            except Exception as exc:  # never let the watcher die mid-demo
                self.log(f"[观众] {type(exc).__name__}: {exc}")
            time.sleep(0.2)

    def _turn_worker(self) -> None:
        while self._running:
            kind, text = self.inputs.get()
            try:
                self._run_turn(kind, text)
            finally:
                self.inputs.task_done()

    def _run_turn(self, kind: str, text: str) -> None:
        self.cancel.clear()
        self.turn_active.set()
        self.ui(kind, text)
        buffer = SentenceBuffer()
        started = [False]

        def on_text(chunk: str) -> None:
            if self.cancel.is_set():
                raise TurnCancelled
            if not started[0]:
                started[0] = True
                self.ui("reply_start")
            self.ui("reply_chunk", chunk)
            for sentence in buffer.feed(chunk):
                self.speaker.say(sentence)

        if kind == "user" and _has(normalize(text), BYE_WORDS):
            self._sleep_after_turn = True
        if kind == "user" and self.robot.connected and _has(normalize(text), STOP_WORDS):
            # Safety first: stop now instead of waiting ~3 s for the LLM to call stop_robot.
            self.log(f"[快速停止] {self.robot.stop()}")
        try:
            self.dialog.turn(text, on_text)
            for sentence in buffer.flush():
                self.speaker.say(sentence)
        except TurnCancelled:
            self.log("[打断] 放弃当前回复")
        except Exception as exc:  # a bug in one turn must not end the demo
            self.log(f"[错误] {type(exc).__name__}: {exc}")
            self.speaker.say("抱歉，我刚才出了点问题，请再说一遍。")
        finally:
            if started[0]:
                self.ui("reply_end")
            self.turn_active.clear()

    # ---- microphone ----
    def is_echo(self, heard: str) -> bool:
        recent = self.speaker.recent_text(window=5.0)
        return bool(recent) and pinyin_overlap(heard, recent) >= ECHO_OVERLAP

    def _ear(self) -> None:
        from asr import Listener, Recognizer
        from audio_io import input_device

        recognizer = Recognizer()
        self.listener = listener = Listener(input_device(self.settings.mic_hint), gain=self.gain,
                                            processor=self.aec.capture if self.aec else None)
        listener.start()
        self._ready.set()
        quiet_until = 0.0
        last_poll = 0.0
        strikes = 0  # consecutive polls that heard something other than our own voice
        while self._running:
            now = time.monotonic()
            self.keep_session()
            busy = self.speaker.busy or self.turn_active.is_set()
            if not self.mic_enabled:
                listener.muted.set()
                time.sleep(0.05)
                continue
            if not self.barge_in:
                if busy:
                    listener.muted.set()
                    quiet_until = now + MUTE_TAIL_S
                    time.sleep(0.05)
                    continue
                if now < quiet_until:
                    time.sleep(0.05)
                    continue
                listener.muted.clear()
            elif listener.muted.is_set():
                listener.muted.clear()
            elif not (self.speaker.busy and listener.speech_active):
                strikes = 0
            elif now - last_poll >= BARGE_POLL_S:
                last_poll = now
                heard = recognizer.transcribe(listener.recent(1.5))
                if len(normalize(heard)) >= BARGE_MIN_CHARS and not self.is_echo(heard):
                    strikes += 1
                else:
                    strikes = 0
                if strikes >= BARGE_CONFIRM:
                    strikes = 0
                    self.log(f"[打断] 听到：{heard}")
                    self.interrupt()
                    listener.restart_segment(keep_seconds=1.0)
            try:
                audio = listener.utterances.get(timeout=0.05)
            except queue.Empty:
                continue
            self.handle_heard(recognizer.transcribe(audio))

    def handle_heard(self, text: str) -> None:
        """One recognised utterance: drop echo, apply the wake word, then queue it as a turn."""
        if not normalize(text):
            return
        if self.barge_in:
            if self.speaker.busy and len(normalize(text)) < BARGE_MIN_CHARS:
                self.log(f"[回声] 说话中忽略短句：{text}")
                return
            if self.is_echo(text):
                self.log(f"[回声] 忽略：{text}")
                return
        if self.wake_required and not self.awake:
            rest = strip_wake(text)
            if rest is None:
                self.log(f"[待唤醒] 忽略：{text}")
                return
            if self.wake_mode == "vision":
                if not self.robot.vision_live:
                    if not self._vision_fallback_logged:
                        self.log("[视觉唤醒] 没有相机数据（vision/watch.py 没运行？），按直接唤醒处理")
                        self._vision_fallback_logged = True
                elif not self.robot.facing_recently(VISION_WAKE_WINDOW_S):
                    self.log(f"[视觉唤醒] 没看到 3 m 内有人面向机器人，忽略：{text}")
                    return
            self.wake()
            self.log("[唤醒]")
            if not rest:  # just "你好小吉": answer at once, no LLM round trip
                self.ui("user", text)
                self._say_now("我在，请说。")
                return
            text = rest
        self.wake()
        if self.barge_in and (self.speaker.busy or self.turn_active.is_set()):
            self.interrupt()
        self.inputs.put(("user", text))

    def _say_now(self, text: str) -> None:
        self.ui("reply_start")
        self.ui("reply_chunk", text)
        self.ui("reply_end")
        self.speaker.say(text)
