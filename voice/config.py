"""Paths and runtime settings for the Windows-side voice guide.

All paths derive from this file so the project folder can move freely.
Settings come from environment variables so API keys never live in code.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

VOICE_DIR = Path(__file__).resolve().parent
PROJECT_DIR = VOICE_DIR.parent
MODELS_DIR = VOICE_DIR / "models"
KNOWLEDGE_DIR = PROJECT_DIR / "knowledge"
EXHIBITION_JSON = PROJECT_DIR / "ws" / "src" / "g1_guide" / "config" / "exhibition.json"

VAD_MODEL = MODELS_DIR / "silero_vad.onnx"
SENSEVOICE_DIR = MODELS_DIR / "sherpa-onnx-sense-voice-zh-en-ja-ko-yue-int8-2024-07-17"
MATCHA_DIR = MODELS_DIR / "matcha-icefall-zh-baker"
VOCODER_MODEL = MODELS_DIR / "vocos-22khz-univ.onnx"

SAMPLE_RATE = 16000

# provider -> (API key env var, default model, OpenAI-compatible base_url or None)
PROVIDERS = {
    "deepseek": ("DEEPSEEK_API_KEY", "deepseek-chat", "https://api.deepseek.com"),
    "qwen": ("DASHSCOPE_API_KEY", "qwen-plus", "https://dashscope.aliyuncs.com/compatible-mode/v1"),
    "claude": ("ANTHROPIC_API_KEY", "claude-sonnet-5", None),
}


@dataclass(frozen=True)
class Settings:
    provider: str
    model: str
    mic_hint: str
    tts_engine: str
    tts_voice: str
    rosbridge_host: str
    rosbridge_port: int


def detect_provider() -> str:
    """Explicit G1_LLM_PROVIDER wins; otherwise the first provider whose key is set."""
    chosen = os.environ.get("G1_LLM_PROVIDER", "").strip().lower()
    if chosen:
        if chosen not in PROVIDERS:
            raise SystemExit(f"G1_LLM_PROVIDER={chosen!r} 不支持，可选：{', '.join(PROVIDERS)}")
        return chosen
    for name, (key_env, _, _) in PROVIDERS.items():
        if os.environ.get(key_env):
            return name
    return "deepseek"


def load_settings() -> Settings:
    provider = detect_provider()
    return Settings(
        provider=provider,
        model=os.environ.get("G1_LLM_MODEL") or PROVIDERS[provider][1],
        mic_hint=os.environ.get("G1_MIC", "Realtek"),
        tts_engine=os.environ.get("G1_TTS", "offline").lower(),
        tts_voice=os.environ.get("G1_TTS_VOICE", "zh-CN-XiaoxiaoNeural"),
        rosbridge_host=os.environ.get("G1_ROSBRIDGE_HOST", "localhost"),
        rosbridge_port=int(os.environ.get("G1_ROSBRIDGE_PORT", "9090")),
    )
