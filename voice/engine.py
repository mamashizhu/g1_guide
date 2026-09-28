"""Guide engine shared by the console (app.py) and the window (gui.py).

Threads:
- turn worker: takes visitor text / robot events from ``inputs`` and runs one dialog turn
- ear: microphone → VAD → ASR → ``inputs``; also watches for barge-in while the robot talks
- rosbridge callbacks: robot status events → ``inputs``

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

from config import Settings
from dialog import AUDIENCES, GuideDialog, Style, build_system_prompt, build_tools, event_message
from exhibits import load_exhibits
from fallback import BYE_WORDS, STOP_WORDS, RuleBackend, _has, normalize
from llm import ResilientBackend, TurnCancelled, make_backend
from robot import RobotLink
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
        self.use_robot = use_robot
        self.speaker = Speaker(make_engine(settings.tts_engine, settings.tts_voice)) if speak else NullSpeaker()
        self.style = style or Style()
        self.dialog = GuideDialog(self.backend, self.robot, log=self.log,
                                  style=self.style, on_style=self._on_style)
        self._on_style(self.style)
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
        self.style.update(audience, humor)
        self._on_style(self.style)

    def _on_style(self, style: Style) -> None:
        tts = getattr(self.speaker, "engine", None)
        if hasattr(tts, "speed"):
            tts.speed = CHILD_SPEED if style.audience == "child" else 1.0
        self.log(f"[风格] {AUDIENCES[style.audience]}模式，幽默 {style.humor}/10")

    # ---- wake word ----
    @property
    def awake(self) -> bool:
        return not self.wake_word or time.monotonic() < self._awake_until

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
            if self.wake_word:
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
        if self.wake_word and not self.awake:
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
        self.inputs.put(("event", event_message(state, exhibit)))

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
        if self.wake_word and not self.awake:
            rest = strip_wake(text)
            if rest is None:
                self.log(f"[待唤醒] 忽略：{text}")
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
