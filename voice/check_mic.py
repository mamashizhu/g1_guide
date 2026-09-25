"""V1 麦克风检查：列出设备，录 4 秒，显示音量条，再回放给你听。

    python check_mic.py            # 默认选名称含 Realtek 的麦克风
    python check_mic.py --mic 耳机  # 按名称关键字选别的麦克风
"""

from __future__ import annotations

import argparse
import sys

import numpy as np
import sounddevice as sd

from audio_io import input_device
from config import SAMPLE_RATE


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mic", default="Realtek")
    parser.add_argument("--seconds", type=float, default=4.0)
    args = parser.parse_args()

    print("可用麦克风：")
    for index, device in enumerate(sd.query_devices()):
        if device["max_input_channels"] > 0 and device["hostapi"] == 0:
            print(f"  [{index}] {device['name']}")
    device = input_device(args.mic)
    print(f"\n使用设备 [{device}]。请在 {args.seconds:.0f} 秒内正常音量说一句话…")
    audio = sd.rec(int(args.seconds * SAMPLE_RATE), samplerate=SAMPLE_RATE, channels=1,
                   dtype="float32", device=device)
    sd.wait()
    audio = audio[:, 0]

    frames = audio[: len(audio) // 1600 * 1600].reshape(-1, 1600)
    for i, frame in enumerate(frames):
        rms = float(np.sqrt(np.mean(frame ** 2)))
        print(f"  {i / 10:4.1f}s {'#' * min(60, int(rms * 600))}")
    peak = float(np.max(np.abs(audio)))
    print(f"\n峰值 {peak:.3f}（说话时 0.1–0.9 为好；< 0.05 太小，要加增益或换麦克风）")
    suggested = min(20.0, 0.5 / peak) if peak > 0 else 20.0
    print(f"建议增益 --gain {suggested:.1f}")

    print("回放中…")
    sd.play(np.clip(audio * max(1.0, suggested), -1, 1), SAMPLE_RATE)
    sd.wait()


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
