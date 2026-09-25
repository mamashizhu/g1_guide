"""Speech recognition: silero VAD cuts utterances, SenseVoice transcribes them (both offline)."""

from __future__ import annotations

import queue
import threading

import numpy as np
import sherpa_onnx
import sounddevice as sd

from config import SAMPLE_RATE, SENSEVOICE_DIR, VAD_MODEL


class Recognizer:
    def __init__(self, num_threads: int = 2):
        self._rec = sherpa_onnx.OfflineRecognizer.from_sense_voice(
            model=str(SENSEVOICE_DIR / "model.int8.onnx"),
            tokens=str(SENSEVOICE_DIR / "tokens.txt"),
            language="zh",
            use_itn=True,
            num_threads=num_threads,
        )

    def transcribe(self, samples: np.ndarray, sample_rate: int = SAMPLE_RATE) -> str:
        stream = self._rec.create_stream()
        stream.accept_waveform(sample_rate, samples.astype(np.float32, copy=False))
        self._rec.decode_stream(stream)
        return stream.result.text.strip()


def make_vad(min_silence: float = 0.6, min_speech: float = 0.25) -> sherpa_onnx.VoiceActivityDetector:
    config = sherpa_onnx.VadModelConfig()
    config.silero_vad.model = str(VAD_MODEL)
    config.silero_vad.threshold = 0.5
    config.silero_vad.min_silence_duration = min_silence
    config.silero_vad.min_speech_duration = min_speech
    config.silero_vad.max_speech_duration = 20
    config.sample_rate = SAMPLE_RATE
    return sherpa_onnx.VoiceActivityDetector(config, buffer_size_in_seconds=60)


class Listener:
    """Streams the microphone through VAD and puts finished utterances (float32 arrays) on a queue.

    ``gain`` boosts quiet microphones; ``muted`` discards audio (used while the robot talks
    when barge-in is off).
    """

    WINDOW = 512  # silero VAD window at 16 kHz

    def __init__(self, device: int | None = None, gain: float = 1.0, processor=None):
        self.device = device
        self.gain = gain
        self.processor = processor  # e.g. EchoCanceller.capture, applied before VAD
        self.utterances: queue.Queue[np.ndarray] = queue.Queue()
        self.muted = threading.Event()
        self.level = 0.0
        self.speech_active = False  # VAD currently inside a speech segment
        self._ring = np.zeros(int(2.0 * SAMPLE_RATE), dtype=np.float32)  # last 2 s of audio
        self._prefix: np.ndarray | None = None
        self._lock = threading.Lock()
        self._vad = make_vad()
        self._pending = np.zeros(0, dtype=np.float32)
        self._stream: sd.InputStream | None = None

    def start(self) -> None:
        self._stream = sd.InputStream(samplerate=SAMPLE_RATE, channels=1, dtype="float32",
                                      device=self.device, blocksize=1600, callback=self._on_audio)
        self._stream.start()

    def stop(self) -> None:
        if self._stream is not None:
            self._stream.stop()
            self._stream.close()

    def recent(self, seconds: float) -> np.ndarray:
        with self._lock:
            return self._ring[-int(seconds * SAMPLE_RATE):].copy()

    def restart_segment(self, keep_seconds: float = 0.6) -> None:
        """Drop the VAD segment in progress (it holds robot echo) and start a fresh one.

        The last ``keep_seconds`` of audio are prepended to the next utterance so the
        visitor's first syllables, spoken before the barge-in fired, are not lost.
        """
        with self._lock:
            self._prefix = self._ring[-int(keep_seconds * SAMPLE_RATE):].copy()
            self._vad.reset()
            self._pending = np.zeros(0, dtype=np.float32)
            self.speech_active = False

    def _on_audio(self, data, frames, time_info, status) -> None:
        chunk = np.clip(data[:, 0] * self.gain, -1.0, 1.0)
        if self.processor is not None:
            chunk = self.processor(chunk)
        self.level = float(np.sqrt(np.mean(chunk ** 2)))
        with self._lock:
            self._ring = np.concatenate([self._ring[len(chunk):], chunk])
            if self.muted.is_set():
                self._pending = np.zeros(0, dtype=np.float32)
                self._vad.reset()
                self.speech_active = False
                return
            self._pending = np.concatenate([self._pending, chunk])
            while len(self._pending) >= self.WINDOW:
                self._vad.accept_waveform(self._pending[:self.WINDOW])
                self._pending = self._pending[self.WINDOW:]
            self.speech_active = self._vad.is_speech_detected()
            while not self._vad.empty():
                samples = np.array(self._vad.front.samples, dtype=np.float32)
                if self._prefix is not None:
                    samples = np.concatenate([self._prefix, samples])
                    self._prefix = None
                self.utterances.put(samples)
                self._vad.pop()
