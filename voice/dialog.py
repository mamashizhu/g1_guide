"""Dialog core: system prompt, tool definitions, and one-turn orchestration.

Audio-free on purpose, so it can be driven from the keyboard or tests.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Callable, Protocol

from exhibits import Exhibit
from llm import Tool, ToolCall
from robot import GESTURES

MAX_TOOL_ROUNDS = 3
NAV_TOOLS = {"goto_exhibit", "stop_robot"}
AUDIENCES = {"normal": "普通", "child": "儿童", "expert": "专业"}
_STYLE_TAG = re.compile(r"^\[风格：(\S+?)模式，幽默(\d+)/10\]\s*")


@dataclass
class Style:
    """How the guide talks: audience level and a 0–10 humour dial."""
    audience: str = "normal"
    humor: int = 3

    def tag(self) -> str:
        return f"[风格：{AUDIENCES[self.audience]}模式，幽默{self.humor}/10]"

    def update(self, audience: str | None = None, humor=None) -> None:
        if audience in AUDIENCES:
            self.audience = audience
        if humor is not None:
            try:
                self.humor = max(0, min(10, int(humor)))
            except (TypeError, ValueError):
                pass


class TagFilter:
    """Drops a "[风格：…]" tag the model sometimes echoes at the start of a reply (streaming-safe)."""

    def __init__(self, on_text: Callable[[str], None]):
        self.on_text = on_text
        self._head = ""       # text held back while it might still be the start of a tag
        self._passed = False

    def __call__(self, chunk: str) -> None:
        if self._passed:
            self.on_text(chunk)
            return
        self._head += chunk
        head = self._head.lstrip()
        if not head:
            return
        if head.startswith("[风格") or "[风格".startswith(head):
            if "]" not in head:
                return  # wait for the rest of the tag
            head = head[head.index("]") + 1:].lstrip()
            if not head:
                self._head = ""
                return
        self._passed = True
        self.on_text(head)


def split_style(text: str) -> tuple[Style | None, str]:
    """Undo Style.tag(): (style, text without the tag).  Used by the offline brain."""
    match = _STYLE_TAG.match(text)
    if not match:
        return None, text
    audience = {v: k for k, v in AUDIENCES.items()}.get(match.group(1), "normal")
    return Style(audience, int(match.group(2))), text[match.end():]


class Robot(Protocol):
    def goto(self, exhibit_id: str) -> str: ...
    def stop(self) -> str: ...
    def gesture(self, name: str, exhibit_id: str = "") -> str: ...
    def describe_status(self) -> str: ...


def build_tools(exhibits: list[Exhibit]) -> list[Tool]:
    catalogue = "；".join(f"{e.exhibit_id}={e.title}" for e in exhibits)
    return [
        Tool("goto_exhibit",
             f"让机器人带游客走到某个展品前。展品：{catalogue}。"
             "只在游客明确想去、或同意你的推荐时调用。",
             {"type": "object",
              "properties": {"exhibit_id": {"type": "string",
                                            "enum": [e.exhibit_id for e in exhibits]}},
              "required": ["exhibit_id"]}),
        Tool("stop_robot", "立即停止机器人移动。游客说停、等一下、别走了时调用。",
             {"type": "object", "properties": {}}),
        Tool("get_robot_status", "查询机器人当前的移动状态和目标。",
             {"type": "object", "properties": {}}),
        Tool("set_style",
             "切换讲解风格。游客说“讲简单点”“我是小朋友”时 audience=child；“讲专业点”“详细讲讲”时 "
             "audience=expert；“正常讲就行”时 normal。“幽默一点”“搞笑点”在当前幽默值上加 3，"
             "“严肃点”“正经点”减 3 或设为 0。只改游客提到的那一项。",
             {"type": "object",
              "properties": {"audience": {"type": "string", "enum": list(AUDIENCES)},
                             "humor": {"type": "integer", "minimum": 0, "maximum": 10}}}),
        Tool("gesture",
             "让机器人做一个手臂动作，和说话同时进行。wave=挥手打招呼或道别；point=指向展品；"
             "present=摊手介绍；explain_loop=讲解时持续的自然手势；emphasize=强调重点；"
             "invite=邀请的手势；home=手臂放下。移动途中不要调用。",
             {"type": "object",
              "properties": {"name": {"type": "string", "enum": list(GESTURES)},
                             "exhibit_id": {"type": "string",
                                            "enum": [e.exhibit_id for e in exhibits],
                                            "description": "point 时指向哪件展品，可省略"}},
              "required": ["name"]}),
    ]


def build_system_prompt(exhibits: list[Exhibit]) -> str:
    knowledge = "\n\n".join(e.knowledge or f"# {e.exhibit_id} {e.title}" for e in exhibits)
    return f"""你是美术馆里的导览机器人“小吉”，是一台宇树人形机器人。你正在用语音和游客交流。

说话要求：
- 你的回复会被直接转成语音播放，所以只输出口语化的纯文本，不要用 markdown、列表、表情符号、括号注释。
- 每次回复尽量简短，一般一到三句话、不超过 80 个字。讲解展品时可以稍长，但不超过 150 个字，讲完问游客还想了解什么。
- 数字按口语习惯说，年份逐位读，例如“一八八九年”；尺寸说大概，例如“五米多长”。
- 语音合成读不出英文字母：不要说展品编号（如 E06），用展品名称；尽量不用英文和字母缩写。
- 只根据下面的展品资料回答展品问题；资料里没有的内容，如实说不确定，不要编造。
- 和展览无关的闲聊可以简短友好地回应，然后自然地引回展品。
- 讲解时你就站在展品前，多引导游客看画面或器物的细节，例如“请看左边这道巨浪”。
- 书和瓷瓶两件立体展品是仿制品；被问到是不是真迹、值多少钱时如实说明。

讲解风格：
- 每条消息开头的“[风格：…]”是当前讲解风格设置，不是游客说的话。回复里绝对不要复述这个标签，直接按它调整说法。
- 普通模式：面向一般游客，通俗准确。
- 儿童模式：面向小学生。用简单的词和生活里的比方，句子短，多问“你知道吗”这类互动问题，不用专业术语，数字取整说。
- 专业模式：面向成人爱好者。可以用术语，多讲原理、数据和来历，讲解可以到 200 字。
- 幽默值 0 到 10：0 完全严肃；3 左右偶尔轻松一句；6 以上多用俏皮比喻和小玩笑；9 以上像脱口秀。无论多幽默，知识必须准确，不拿游客开玩笑。
- 游客要求换风格时调用 set_style，并用新风格回一句话。

行动规则：
- 游客想去某个展品时，调用 goto_exhibit，同时用一句话告诉游客“请跟我来”之类的话。
- 调用 goto_exhibit 后机器人才刚出发，不要说“已经到了”。
- 到达后你会收到一条以“[系统事件]”开头的消息，这不是游客说的话。收到“已到达”后，直接开始讲解该展品，结尾推荐下一站。
- 游客让你停下时调用 stop_robot。
- 说话时可以配合 gesture：打招呼、道别时 wave；到达展品开始讲解时先 point 指向该展品，再 explain_loop；讲完一段、等游客提问时 home。调用 goto_exhibit 的同一轮不要做手势。手势要和话一起调用，不要单独调用。
- 游客的话由语音识别得到，可能有同音错字，请按最合理的意思理解；实在听不懂就请游客再说一遍。

展馆共有 {len(exhibits)} 件展品，资料如下：

{knowledge}
"""


def event_message(state: str, exhibit: Exhibit | None, present_mode: str = "point") -> str:
    name = f"{exhibit.exhibit_id} {exhibit.title}" if exhibit else "目标"
    if state == "arrived" and present_mode == "face":
        return (f"[系统事件] 已到达 {name}，机器人正在转身面向观众（面向观众模式）。"
                "这次讲解不要用 point 指向展品，改用 present 或 explain_loop，请开始讲解。")
    return {
        "arrived": f"[系统事件] 已到达 {name}，请开始讲解。",
        "blocked": f"[系统事件] 前往 {name} 的路线被挡住了，机器人正在停车重新规划。请向游客简短说明。",
        "invalid_goal": "[系统事件] 导航目标无效。",
        "sensor_stale": "[系统事件] 机器人传感器数据中断，暂时无法移动。请向游客致歉。",
    }.get(state, f"[系统事件] 机器人状态：{state}")


class GuideDialog:
    def __init__(self, backend, robot: Robot, log: Callable[[str], None] = print,
                 style: Style | None = None, on_style: Callable[[Style], None] | None = None,
                 on_audience_request: Callable[[str], None] | None = None):
        self.backend = backend
        self.robot = robot
        self.log = log
        self.style = style or Style()
        self.on_style = on_style
        self.on_audience_request = on_audience_request  # the visitor asked for an audience mode

    def execute(self, call: ToolCall) -> str:
        if call.name == "goto_exhibit":
            return self.robot.goto(str(call.args.get("exhibit_id", "")))
        if call.name == "stop_robot":
            return self.robot.stop()
        if call.name == "get_robot_status":
            return self.robot.describe_status()
        if call.name == "set_style":
            self.style.update(call.args.get("audience"), call.args.get("humor"))
            if call.args.get("audience") in AUDIENCES and self.on_audience_request:
                self.on_audience_request(call.args["audience"])
            if self.on_style:
                self.on_style(self.style)
            return f"已切换为{AUDIENCES[self.style.audience]}模式，幽默{self.style.humor}/10，从这句开始按新风格说"
        if call.name == "gesture":
            result = self.robot.gesture(str(call.args.get("name", "")), str(call.args.get("exhibit_id", "")))
            return result + "。如果话还没说完就接着说；已经说完了就不要再输出任何内容。"
        return f"失败：未知工具 {call.name}"

    def turn(self, text: str, on_text: Callable[[str], None]) -> str:
        """Run one user (or system-event) turn; returns the full spoken reply."""
        spoken: list[str] = []

        def show(chunk: str) -> None:
            spoken.append(chunk)
            on_text(chunk)

        emit = TagFilter(show)

        calls = self.backend.send(f"{self.style.tag()} {text}", emit)
        for _ in range(MAX_TOOL_ROUNDS):
            if not calls:
                break
            results = []
            for call in calls:
                result = self.execute(call)
                self.log(f"[tool] {call.name}({call.args}) -> {result}")
                results.append((call, result))
            self.backend.add_tool_results(results)
            if "".join(spoken).strip() and any(c.name in NAV_TOOLS | {"set_style"} for c in calls):
                # "请跟我来" has been said and the robot is moving; the results ride along
                # with the next turn, so we do not pay for an extra round trip.
                break
            # Gestures accompany speech: the model often says one line, gestures, and
            # expects to go on talking once the tool results are back.
            calls = self.backend.send(None, emit)
        return "".join(spoken)
