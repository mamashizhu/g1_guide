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
from dialog import GuideDialog, build_system_prompt, build_tools, event_message
from exhibits import load_exhibits
from fallback import RuleBackend, normalize
from llm import ResilientBackend, TurnCancelled, make_backend
from robot import RobotLink
from tts import SentenceBuffer, Speaker, make_engine

ECHO_OVERLAP = 0.5      # heard text sharing this much with the robot's recent words is echo
BARGE_POLL_S = 0.4      # how often to test live speech during playback
BARGE_MIN_CHARS = 4     # shorter fragments are usually garbled echo
BARGE_CONFIRM = 2       # consecutive non-echo polls needed before cutting the robot off
MUTE_TAIL_S = 0.3       # without barge-in: keep the mic closed this long after speaking (room echo)

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
                 offline: bool = False, ui: UiCallback = console_ui):
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
        self.dialog = GuideDialog(self.backend, self.robot, log=self.log)
        self.speaker = Speaker(make_engine(settings.tts_engine, settings.tts_voice)) if speak else NullSpeaker()
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
        """Typed input (console or GUI): cuts off whatever the robot is saying."""
        text = text.strip()
        if text:
            self.interrupt()
            self.inputs.put(("user", text))

    def interrupt(self) -> None:
        self.cancel.set()
        self.speaker.stop()

    def wait_idle(self) -> None:
        self.inputs.join()
        self.speaker.wait()

    @property
    def state(self) -> str:
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
            text = recognizer.transcribe(audio)
            if not normalize(text):
                continue
            if self.barge_in:
                if self.speaker.busy and len(normalize(text)) < BARGE_MIN_CHARS:
                    self.log(f"[回声] 说话中忽略短句：{text}")
                    continue
                if self.is_echo(text):
                    self.log(f"[回声] 忽略：{text}")
                    continue
                if self.speaker.busy or self.turn_active.is_set():
                    self.interrupt()
            self.inputs.put(("user", text))
