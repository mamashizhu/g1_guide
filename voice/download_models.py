"""Download the offline speech models into voice/models (about 400 MB, from k2-fsa/sherpa-onnx releases)."""

from __future__ import annotations

import sys
import tarfile
import urllib.request

from config import MATCHA_DIR, MODELS_DIR, SENSEVOICE_DIR, VAD_MODEL, VOCODER_MODEL

BASE = "https://github.com/k2-fsa/sherpa-onnx/releases/download"
ITEMS = [
    (VAD_MODEL, f"{BASE}/asr-models/silero_vad.onnx", False),
    (SENSEVOICE_DIR, f"{BASE}/asr-models/{SENSEVOICE_DIR.name}.tar.bz2", True),
    (MATCHA_DIR, f"{BASE}/tts-models/{MATCHA_DIR.name}.tar.bz2", True),
    (VOCODER_MODEL, f"{BASE}/vocoder-models/{VOCODER_MODEL.name}", False),
]


def main() -> None:
    MODELS_DIR.mkdir(exist_ok=True)
    for target, url, is_archive in ITEMS:
        if target.exists():
            print(f"已存在 {target.name}")
            continue
        print(f"下载 {url}")
        download = MODELS_DIR / url.rsplit("/", 1)[1]
        urllib.request.urlretrieve(url, download)
        if is_archive:
            with tarfile.open(download) as archive:
                archive.extractall(MODELS_DIR, filter="data")
            download.unlink()
    print("模型齐全")


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
