"""Acoustic echo cancellation with WebRTC's AEC3 (via livekit's AudioProcessingModule).

The player hands every block it plays to ``render``; the microphone passes every
block through ``capture`` before VAD/ASR.  AEC3 subtracts what the loudspeaker
contributed, so on open speakers the robot stops hearing (and interrupting) itself.
"""

from __future__ import annotations

import threading

import numpy as np

from config import SAMPLE_RATE

FRAME = SAMPLE_RATE // 100  # AEC works on 10 ms frames


def _resample(x: np.ndarray, src: int) -> np.ndarray:
    if src == SAMPLE_RATE:
        return x
    n = int(round(len(x) * SAMPLE_RATE / src))
    return np.interp(np.linspace(0, len(x) - 1, n), np.arange(len(x)), x).astype(np.float32)


def _to_int16(x: np.ndarray) -> np.ndarray:
    return (np.clip(x, -1.0, 1.0) * 32767).astype(np.int16)


class EchoCanceller:
    def __init__(self, delay_ms: int = 0):
        from livekit import rtc

        self._rtc = rtc
        self._apm = rtc.AudioProcessingModule(echo_cancellation=True, noise_suppression=True,
                                              high_pass_filter=True)
        self._lock = threading.Lock()
        self._render_rest = np.zeros(0, np.float32)
        self._capture_rest = np.zeros(0, np.float32)
        self.delay_ms = delay_ms

    def _frame(self, samples: np.ndarray):
        pcm = _to_int16(samples)
        return self._rtc.AudioFrame(pcm.tobytes(), SAMPLE_RATE, 1, FRAME)

    def render(self, samples: np.ndarray, sample_rate: int) -> None:
        """Far end: audio about to leave the loudspeaker."""
        with self._lock:
            data = np.concatenate([self._render_rest, _resample(samples, sample_rate)])
            usable = len(data) // FRAME * FRAME
            for start in range(0, usable, FRAME):
                self._apm.process_reverse_stream(self._frame(data[start:start + FRAME]))
            self._render_rest = data[usable:]

    def capture(self, samples: np.ndarray) -> np.ndarray:
        """Near end: microphone block (16 kHz float32) in, echo-reduced block of the same length out."""
        with self._lock:
            data = np.concatenate([self._capture_rest, samples])
            usable = len(data) // FRAME * FRAME
            out = []
            for start in range(0, usable, FRAME):
                frame = self._frame(data[start:start + FRAME])
                self._apm.set_stream_delay_ms(self.delay_ms)
                self._apm.process_stream(frame)
                out.append(np.frombuffer(bytes(frame.data), dtype=np.int16).astype(np.float32) / 32767)
            self._capture_rest = data[usable:]
        cleaned = np.concatenate(out) if out else np.zeros(0, np.float32)
        # Callers use multiples of 10 ms; otherwise the <10 ms remainder is emitted with the next block.
        if len(cleaned) < len(samples):
            cleaned = np.concatenate([np.zeros(len(samples) - len(cleaned), np.float32), cleaned])
        return cleaned[-len(samples):]
