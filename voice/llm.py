"""Streaming chat backends with tool calling.

Two flavours share one interface:
- OpenAICompatBackend: DeepSeek, Qwen (DashScope compatible mode), anything OpenAI-shaped
- ClaudeBackend: Anthropic Messages API

``send(text, on_text)`` streams text chunks to ``on_text`` and returns the tool calls the
model made.  The caller executes them and reports back with ``add_tool_results``; the
results are delivered with the next ``send`` (pass ``text=None`` to just continue).
"""

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass
from typing import Callable

from config import PROVIDERS, Settings

MAX_HISTORY = 30  # messages kept besides the system prompt
REQUEST_TIMEOUT = 10.0  # seconds; a visitor will not wait longer, the offline brain answers instead
OFFLINE_COOLDOWN = 30.0  # after a failure, skip the LLM for this long instead of timing out every turn


class TurnCancelled(Exception):
    """Raised from an on_text callback to abandon a streaming reply (visitor barged in)."""


@dataclass
class Tool:
    name: str
    description: str
    parameters: dict


@dataclass
class ToolCall:
    call_id: str
    name: str
    args: dict


def _parse_args(raw: str) -> dict:
    try:
        value = json.loads(raw or "{}")
    except json.JSONDecodeError:
        return {}
    return value if isinstance(value, dict) else {}


def _trim(messages: list[dict], is_user_text: Callable[[dict], bool]) -> list[dict]:
    """Keep the tail of the history, cutting only at a plain user turn so tool pairs stay intact."""
    if len(messages) <= MAX_HISTORY:
        return messages
    for start in range(len(messages) - MAX_HISTORY, len(messages)):
        if is_user_text(messages[start]):
            return messages[start:]
    return messages[-2:]


def bypass_proxy(base_url: str) -> None:
    """Domestic APIs are ~2 s slower through the local overseas proxy, so connect directly."""
    host = base_url.split("://", 1)[-1].split("/", 1)[0]
    current = [h for h in os.environ.get("NO_PROXY", "").split(",") if h]
    if host not in current:
        os.environ["NO_PROXY"] = os.environ["no_proxy"] = ",".join([*current, host])


class OpenAICompatBackend:
    def __init__(self, system: str, tools: list[Tool], model: str, base_url: str, api_key: str):
        from openai import OpenAI

        bypass_proxy(base_url)
        self.client = OpenAI(base_url=base_url, api_key=api_key, timeout=REQUEST_TIMEOUT, max_retries=0)
        self.model = model
        self.system = system
        self.tools = [{"type": "function",
                       "function": {"name": t.name, "description": t.description,
                                    "parameters": t.parameters}} for t in tools]
        self.messages: list[dict] = []

    def send(self, text: str | None, on_text: Callable[[str], None]) -> list[ToolCall]:
        if text is not None:
            self.messages.append({"role": "user", "content": text})
        self.messages = _trim(self.messages, lambda m: m["role"] == "user")
        stream = self.client.chat.completions.create(
            model=self.model, stream=True, tools=self.tools, temperature=0.6,
            messages=[{"role": "system", "content": self.system}, *self.messages])
        reply, pending = "", {}
        for chunk in stream:
            if not chunk.choices:
                continue
            delta = chunk.choices[0].delta
            if delta.content:
                reply += delta.content
                on_text(delta.content)
            for tc in delta.tool_calls or []:
                slot = pending.setdefault(tc.index, {"id": "", "name": "", "args": ""})
                slot["id"] = tc.id or slot["id"]
                if tc.function:
                    slot["name"] += tc.function.name or ""
                    slot["args"] += tc.function.arguments or ""
        calls = [ToolCall(s["id"], s["name"], _parse_args(s["args"])) for s in pending.values()]
        message = {"role": "assistant", "content": reply or None}
        if calls:
            message["tool_calls"] = [
                {"id": c.call_id, "type": "function",
                 "function": {"name": c.name, "arguments": json.dumps(c.args, ensure_ascii=False)}}
                for c in calls]
        self.messages.append(message)
        return calls

    def add_tool_results(self, results: list[tuple[ToolCall, str]]) -> None:
        for call, result in results:
            self.messages.append({"role": "tool", "tool_call_id": call.call_id, "content": result})

    def warmup(self) -> None:
        """Open the TLS connection and prime the provider's prompt cache before the first visitor."""
        self.client.chat.completions.create(
            model=self.model, max_tokens=1,
            messages=[{"role": "system", "content": self.system}, {"role": "user", "content": "你好"}])


class ClaudeBackend:
    def __init__(self, system: str, tools: list[Tool], model: str, api_key: str):
        import anthropic

        self.client = anthropic.Anthropic(api_key=api_key, timeout=REQUEST_TIMEOUT, max_retries=0)
        self.model = model
        self.system = system
        self.tools = [{"name": t.name, "description": t.description, "input_schema": t.parameters}
                      for t in tools]
        self.messages: list[dict] = []
        self._pending_results: list[dict] = []

    def send(self, text: str | None, on_text: Callable[[str], None]) -> list[ToolCall]:
        content = list(self._pending_results)
        self._pending_results = []
        if text is not None:
            content.append({"type": "text", "text": text})
        if content:
            self.messages.append({"role": "user", "content": content})
        self.messages = _trim(self.messages, lambda m: m["role"] == "user" and any(
            isinstance(b, dict) and b.get("type") == "text" for b in m["content"]))
        with self.client.messages.stream(model=self.model, max_tokens=600, system=self.system,
                                         tools=self.tools, messages=self.messages) as stream:
            for chunk in stream.text_stream:
                on_text(chunk)
            final = stream.get_final_message()
        self.messages.append({"role": "assistant",
                              "content": [b.model_dump(exclude_none=True) for b in final.content]})
        return [ToolCall(b.id, b.name, dict(b.input)) for b in final.content if b.type == "tool_use"]

    def warmup(self) -> None:
        self.client.messages.create(model=self.model, max_tokens=1, system=self.system,
                                    messages=[{"role": "user", "content": "你好"}])

    def add_tool_results(self, results: list[tuple[ToolCall, str]]) -> None:
        self._pending_results.extend(
            {"type": "tool_result", "tool_use_id": call.call_id, "content": result}
            for call, result in results)


class ResilientBackend:
    """Uses the LLM when it answers, the offline rule brain when it does not."""

    def __init__(self, primary, fallback, log: Callable[[str], None] = print):
        self.primary = primary
        self.fallback = fallback
        self.log = log
        self.offline_until = 0.0
        self._last = fallback if primary is None else primary

    @property
    def online(self) -> bool:
        return self.primary is not None and time.monotonic() >= self.offline_until

    def warmup(self) -> None:
        if self.primary is not None:
            try:
                self.primary.warmup()
            except Exception as exc:
                self._fail(exc)

    def send(self, text: str | None, on_text: Callable[[str], None]) -> list[ToolCall]:
        if text is None:  # follow-up after tool results goes to whoever made the calls
            return self._last.send(None, on_text)
        if not self.online:
            self._last = self.fallback
            return self.fallback.send(text, on_text)
        emitted = []

        def relay(chunk: str) -> None:
            emitted.append(chunk)
            on_text(chunk)

        try:
            self._last = self.primary
            return self.primary.send(text, relay)
        except TurnCancelled:
            raise
        except Exception as exc:
            self._fail(exc)
            self._last = self.fallback
            if emitted:
                on_text("……抱歉，网络断了一下。")
                return []
            return self.fallback.send(text, on_text)

    def add_tool_results(self, results) -> None:
        self._last.add_tool_results(results)

    def _fail(self, exc: Exception) -> None:
        self.offline_until = time.monotonic() + OFFLINE_COOLDOWN
        self.log(f"[llm] 大模型不可用（{type(exc).__name__}: {exc}），{OFFLINE_COOLDOWN:.0f} 秒内改用离线应答")


def make_backend(settings: Settings, system: str, tools: list[Tool]):
    """The configured LLM backend, or None when its API key is missing."""
    key_env, _, base_url = PROVIDERS[settings.provider]
    api_key = os.environ.get(key_env)
    if not api_key:
        return None
    if base_url is None:
        return ClaudeBackend(system, tools, settings.model, api_key)
    return OpenAICompatBackend(system, tools, settings.model, base_url, api_key)
