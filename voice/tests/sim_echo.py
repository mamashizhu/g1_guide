"""Open-speaker echo simulation for barge-in tuning (no microphone needed).

    .venv/Scripts/python.exe tests/sim_echo.py            # echo only: counts false barge-ins
    .venv/Scripts/python.exe tests/sim_echo.py --visitor  # a visitor also talks: must interrupt
    .venv/Scripts/python.exe tests/sim_echo.py --no-aec   # compare without echo cancellation

The robot reads exhibit introductions through the real Speaker queue.  A timeline
player "plays" the clips and the microphone hears exactly what is playing, after
laptop-speaker distortion (band-limit, room reverb, clipping, noise).  Runs in real
time; about 20 s per exhibit.
"""

from __future__ import annotations

import argparse
import re
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.stdout.reconfigure(encoding="utf-8")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import asr  # noqa: E402
from config import SAMPLE_RATE, Settings  # noqa: E402
from engine import GuideEngine  # noqa: E402
from tts import OfflineEngine, Speaker  # noqa: E402

RNG = np.random.default_rng(1)
VISITOR = "等一下，带我去看星月夜。"


def resample(x: np.ndarray, src: int, dst: int = SAMPLE_RATE) -> np.ndarray:
    n = int(len(x) * dst / src)
    return np.interp(np.linspace(0, len(x) - 1, n), np.arange(len(x)), x).astype(np.float32)


def laptop_echo(x: np.ndarray, gain: float) -> np.ndarray:
    """Band-limit to a small speaker, add ~0.3 s room reverb, clip softly."""
    spectrum = np.fft.rfft(x)
    freqs = np.fft.rfftfreq(len(x), 1 / SAMPLE_RATE)
    spectrum[(freqs < 250) | (freqs > 4500)] = 0
    y = np.fft.irfft(spectrum, len(x))
    t = np.arange(int(0.3 * SAMPLE_RATE)) / SAMPLE_RATE
    impulse = RNG.normal(0, 1, len(t)) * np.exp(-t / 0.08)
    impulse[0] = 4.0
    y = np.convolve(y, impulse / np.abs(impulse).sum() * 3)[: len(x) + len(t)]
    return np.tanh(y * gain * 2).astype(np.float32) / 2


class TimelinePlayer:
    """Stands in for audio_io.Player: keeps clips on a clock instead of a sound card."""

    def __init__(self, sample_rate: int, echo_gain: float):
        self.sample_rate = sample_rate
        self.echo_gain = echo_gain
        self.clips: list[tuple[float, np.ndarray, np.ndarray]] = []  # (start, 16 kHz reference, 16 kHz echo)
        self.end = 0.0
        self.stopped_at: list[float] = []
        self.lock = threading.Lock()

    def enqueue(self, samples: np.ndarray) -> None:
        reference = resample(samples, self.sample_rate)
        echo = laptop_echo(reference, self.echo_gain)
        with self.lock:
            start = max(time.monotonic(), self.end)
            self.clips.append((start, reference, echo))
            self.end = start + len(samples) / self.sample_rate

    def stop(self) -> None:
        with self.lock:
            if self.busy:
                self.stopped_at.append(time.monotonic())
            self.clips.clear()
            self.end = 0.0

    @property
    def busy(self) -> bool:
        return time.monotonic() < self.end

    def mic_block(self, now: float, n: int) -> tuple[np.ndarray, np.ndarray]:
        """(what the loudspeaker plays, what the microphone hears of it) for the block starting now."""
        reference, heard = np.zeros(n, np.float32), np.zeros(n, np.float32)
        with self.lock:
            for start, ref, echo in self.clips:
                i = int((now - start) * SAMPLE_RATE)
                for src, dst in ((ref, reference), (echo, heard)):
                    if i + n > 0 and i < len(src):
                        lo, hi = max(i, 0), min(i + n, len(src))
                        dst[lo - i:hi - i] += src[lo:hi]
        return reference, heard


def run(exhibit_index: int, tts: OfflineEngine, echo_gain: float, visitor_at: float | None,
        use_aec: bool) -> dict:
    asr.Listener.start = lambda self: None
    asr.Listener.stop = lambda self: None
    log: list[tuple[str, str]] = []
    engine = GuideEngine(Settings("deepseek", "m", "Realtek", "offline", "v", "localhost", 9090),
                         use_robot=False, speak=False, listen=True, barge_in=True, offline=True,
                         ui=lambda kind, text="": log.append((kind, text)))
    speaker = Speaker.__new__(Speaker)
    Speaker.__init__(speaker, tts)
    player = TimelinePlayer(tts.sample_rate, echo_gain)
    speaker.player = player
    engine.speaker = speaker
    if use_aec:
        from aec import EchoCanceller

        engine.aec = EchoCanceller()
    engine.start()

    intro = engine.exhibits[exhibit_index].intro
    sentences = [s for s in re.findall(r"[^。！？]+[。！？]", intro.replace("\n", ""))]
    for sentence in sentences:          # an LLM streams the whole reply in a second or two
        speaker.say(sentence)
    visitor = None
    if visitor_at is not None:
        visitor = resample(tts.synthesize(VISITOR), tts.sample_rate)
    time.sleep(0.3)
    t0 = time.monotonic()
    while speaker.busy or time.monotonic() - t0 < 2:
        now = time.monotonic()
        reference, block = player.mic_block(now, 1600)
        block = block + RNG.normal(0, 0.004, 1600).astype(np.float32)
        if engine.aec is not None:
            engine.aec.render(reference, SAMPLE_RATE)
        if visitor is not None:
            i = int((now - t0 - visitor_at) * SAMPLE_RATE)
            if i + 1600 > 0 and i < len(visitor):
                lo, hi = max(i, 0), min(i + 1600, len(visitor))
                block[lo - i:hi - i] += visitor[lo:hi]
        engine.listener._on_audio(block[:, None], 1600, None, None)
        time.sleep(max(0.0, 0.1 - (time.monotonic() - now)))
        if now - t0 > 60:
            break
    time.sleep(1.0)
    engine.wait_idle()
    engine.close()
    return {
        "exhibit": engine.exhibits[exhibit_index].exhibit_id,
        "stops": [round(t - t0, 1) for t in player.stopped_at],
        "barge": [t for k, t in log if k == "log" and t.startswith("[打断] 听到")],
        "echo": [t for k, t in log if k == "log" and t.startswith("[回声]")],
        "user": [t for k, t in log if k == "user"],
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--visitor", action="store_true")
    parser.add_argument("--gain", type=float, default=0.8, help="echo loudness at the mic")
    parser.add_argument("--exhibits", default="0,1,2,3,4,5")
    parser.add_argument("--no-aec", action="store_true", help="disable echo cancellation")
    args = parser.parse_args()
    tts = OfflineEngine()
    false_stops = 0
    for index in map(int, args.exhibits.split(",")):
        result = run(index, tts, args.gain, 4.0 if args.visitor else None, not args.no_aec)
        if not args.visitor:
            false_stops += bool(result["stops"])
        print(f"{result['exhibit']}: 停止于 {result['stops']}  打断 {result['barge']}  "
              f"游客轮 {result['user']}  回声过滤 {len(result['echo'])} 次")
    if not args.visitor:
        print(f"误打断：{false_stops} 段")


if __name__ == "__main__":
    main()
