"""Text-to-speech: each sentence is synthesised (offline Matcha or edge-tts) and played as soon as it is ready."""

from __future__ import annotations

import asyncio
import queue
import re
import threading
import time

import edge_tts
import miniaudio
import numpy as np

from audio_io import Player
from config import MATCHA_DIR, VOCODER_MODEL

EDGE_RATE = 24000

_SENTENCE_END = re.compile(r"[。！？!?；;\n]")
_SOFT_BREAK = re.compile(r"[，,、：:]")


class SentenceBuffer:
    """Accumulates streamed LLM text and releases speakable sentences early."""

    def __init__(self, soft_limit: int = 40):
        self.soft_limit = soft_limit
        self._text = ""

    def feed(self, chunk: str) -> list[str]:
        self._text += chunk
        out = []
        while True:
            match = _SENTENCE_END.search(self._text)
            if match is None:
                break
            out.append(self._text[:match.end()])
            self._text = self._text[match.end():]
        # A very long clause without a full stop: break at the last comma to keep latency low.
        if len(self._text) > self.soft_limit:
            breaks = list(_SOFT_BREAK.finditer(self._text))
            if breaks:
                cut = breaks[-1].end()
                out.append(self._text[:cut])
                self._text = self._text[cut:]
        return [s.strip() for s in out if s.strip()]

    def flush(self) -> list[str]:
        rest, self._text = self._text.strip(), ""
        return [rest] if rest else []


_DIGITS = "零一二三四五六七八九"


def clean_for_speech(text: str) -> str:
    """Remove markdown/emoji-ish symbols, and turn exhibit IDs the offline voice cannot read into words."""
    text = re.sub(r"(?<![A-Za-z0-9])E0?([1-9])(?![0-9])",
                  lambda m: _DIGITS[int(m.group(1))] + "号展品", text)
    text = re.sub(r"[*#`>_~|\[\]]", "", text)
    return re.sub(r"[\U0001F000-\U0001FFFF☀-➿]", "", text).strip()


async def _synth_mp3(text: str, voice: str) -> bytes:
    audio = bytearray()
    async for part in edge_tts.Communicate(text, voice).stream():
        if part["type"] == "audio":
            audio.extend(part["data"])
    return bytes(audio)


class EdgeEngine:
    """Online Microsoft voices: nicer timbre, but ~2.2 s handshake per sentence."""

    sample_rate = EDGE_RATE

    def __init__(self, voice: str):
        self.voice = voice

    def synthesize(self, text: str) -> np.ndarray:
        mp3 = asyncio.run(_synth_mp3(text, self.voice))
        decoded = miniaudio.decode(mp3, output_format=miniaudio.SampleFormat.FLOAT32,
                                   nchannels=1, sample_rate=EDGE_RATE)
        return np.asarray(decoded.samples, dtype=np.float32)


class OfflineEngine:
    """sherpa-onnx Matcha (zh-baker) + Vocos: ~0.2 s per sentence on CPU, no network."""

    def __init__(self, speed: float = 1.0):
        import sherpa_onnx

        model = MATCHA_DIR
        config = sherpa_onnx.OfflineTtsConfig(
            model=sherpa_onnx.OfflineTtsModelConfig(
                matcha=sherpa_onnx.OfflineTtsMatchaModelConfig(
                    acoustic_model=str(model / "model-steps-3.onnx"),
                    vocoder=str(VOCODER_MODEL),
                    lexicon=str(model / "lexicon.txt"),
                    tokens=str(model / "tokens.txt"),
                    dict_dir=str(model / "dict")),
                num_threads=4),
            rule_fsts=",".join(str(model / f) for f in ("phone.fst", "date.fst", "number.fst")))
        self._tts = sherpa_onnx.OfflineTts(config)
        self.sample_rate = self._tts.sample_rate
        self.speed = speed

    def synthesize(self, text: str) -> np.ndarray:
        return np.asarray(self._tts.generate(text, sid=0, speed=self.speed).samples, dtype=np.float32)


def make_engine(kind: str, voice: str):
    return EdgeEngine(voice) if kind == "edge" else OfflineEngine()


class Speaker:
    """say() queues a sentence; a worker synthesises while the player plays the previous one."""

    def __init__(self, engine, output_device: int | None = None):
        self.engine = engine
        self.player = Player(engine.sample_rate, output_device)
        self._sentences: queue.Queue[str] = queue.Queue()
        self._said: list[tuple[float, str]] = []  # (estimated end of playback, sentence)
        self._play_end = 0.0
        self._generation = 0
        self._synthesising = threading.Event()
        threading.Thread(target=self._run, daemon=True).start()

    def say(self, sentence: str) -> None:
        sentence = clean_for_speech(sentence)
        if sentence:
            self._synthesising.set()
            self._sentences.put(sentence)

    def stop(self) -> None:
        self._generation += 1
        self._play_end = 0.0
        while True:
            try:
                self._sentences.get_nowait()
            except queue.Empty:
                break
        self.player.stop()

    def recent_text(self, window: float = 4.0) -> str:
        """What the speaker is saying now, has queued, or said in the last ``window`` seconds (echo filtering)."""
        now = time.monotonic()
        return "".join(sentence for end, sentence in self._said if end + window >= now)

    @property
    def busy(self) -> bool:
        return self._synthesising.is_set() or not self._sentences.empty() or self.player.busy

    def wait(self) -> None:
        while self.busy:
            threading.Event().wait(0.05)

    def _run(self) -> None:
        while True:
            sentence = self._sentences.get()
            generation = self._generation
            try:
                samples = self.engine.synthesize(sentence)
            except Exception as exc:  # network hiccup: skip the sentence, keep the dialog alive
                print(f"[tts] 合成失败：{exc}")
                samples = None
            if samples is not None and generation == self._generation:
                self.player.enqueue(samples)
                # Clips play back to back: this one ends after everything already queued.
                self._play_end = max(time.monotonic(), self._play_end) + len(samples) / self.engine.sample_rate
                self._said.append((self._play_end, sentence))
                self._said = self._said[-12:]
            if self._sentences.empty():
                self._synthesising.clear()
