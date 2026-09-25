"""Microphone selection and an interruptible audio player."""

from __future__ import annotations

import queue
import threading

import numpy as np
import sounddevice as sd


def find_device(hint: str, kind: str = "input") -> int | None:
    """Return the first device whose name contains ``hint`` (case-insensitive).

    The Windows default input on this machine is a silent ToDesk virtual
    microphone, so we always select by name instead of trusting the default.
    """
    channel_key = "max_input_channels" if kind == "input" else "max_output_channels"
    hint = hint.lower()
    for index, device in enumerate(sd.query_devices()):
        if device[channel_key] > 0 and hint in device["name"].lower():
            return index
    return None


def input_device(hint: str) -> int | None:
    device = find_device(hint, "input")
    if device is None:
        print(f"[audio] 没找到名称包含 {hint!r} 的麦克风，改用系统默认输入")
    return device


class Player:
    """Plays queued float32 mono clips on a background thread; ``stop()`` cuts off at once."""

    BLOCK = 1024

    def __init__(self, sample_rate: int = 24000, device: int | None = None):
        self.sample_rate = sample_rate
        self._clips: queue.Queue[np.ndarray | None] = queue.Queue()
        self._stop = threading.Event()
        self._busy = threading.Event()
        self._device = device
        self.on_render = None  # callback(samples, sample_rate) for each block sent to the speaker (AEC reference)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def enqueue(self, samples: np.ndarray) -> None:
        self._busy.set()
        self._clips.put(samples.astype(np.float32, copy=False))

    def stop(self) -> None:
        """Drop everything queued and silence the current clip within one block (~40 ms)."""
        self._stop.set()
        while True:
            try:
                self._clips.get_nowait()
            except queue.Empty:
                break

    @property
    def busy(self) -> bool:
        return self._busy.is_set() or not self._clips.empty()

    def wait(self) -> None:
        while self.busy:
            threading.Event().wait(0.05)

    def _run(self) -> None:
        with sd.OutputStream(samplerate=self.sample_rate, channels=1, dtype="float32",
                             device=self._device, blocksize=self.BLOCK) as stream:
            while True:
                try:
                    clip = self._clips.get(timeout=0.1)
                except queue.Empty:
                    self._busy.clear()
                    self._stop.clear()
                    continue
                for start in range(0, len(clip), self.BLOCK):
                    if self._stop.is_set():
                        break
                    block = clip[start:start + self.BLOCK]
                    if self.on_render is not None:
                        self.on_render(block, self.sample_rate)
                    stream.write(block)
                if self._clips.empty():
                    self._busy.clear()
                    self._stop.clear()
