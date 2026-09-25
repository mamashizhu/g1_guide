"""Dialog core: system prompt, tool definitions, and one-turn orchestration.

Audio-free on purpose, so it can be driven from the keyboard or tests.
"""

from __future__ import annotations

from typing import Callable, Protocol

from exhibits import Exhibit
from llm import Tool, ToolCall
from robot import GESTURES

MAX_TOOL_ROUNDS = 3
NAV_TOOLS = {"goto_exhibit", "stop_robot"}


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
    return f"""你是科技馆里的导览机器人“小吉”，是一台宇树人形机器人。你正在用语音和游客交流。

说话要求：
- 你的回复会被直接转成语音播放，所以只输出口语化的纯文本，不要用 markdown、列表、表情符号、括号注释。
- 每次回复尽量简短，一般一到三句话、不超过 80 个字。讲解展品时可以稍长，但不超过 150 个字，讲完问游客还想了解什么。
- 数字按口语习惯说，例如“一点五亿公里”。
- 语音合成读不出英文字母：不要说展品编号（如 E06），用展品名称；尽量不用英文和字母缩写。
- 只根据下面的展品资料回答展品问题；资料里没有的内容，如实说不确定，不要编造。
- 和展览无关的闲聊可以简短友好地回应，然后自然地引回展品。

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


def event_message(state: str, exhibit: Exhibit | None) -> str:
    name = f"{exhibit.exhibit_id} {exhibit.title}" if exhibit else "目标"
    return {
        "arrived": f"[系统事件] 已到达 {name}，请开始讲解。",
        "blocked": f"[系统事件] 前往 {name} 的路线被挡住了，机器人正在停车重新规划。请向游客简短说明。",
        "invalid_goal": "[系统事件] 导航目标无效。",
        "sensor_stale": "[系统事件] 机器人传感器数据中断，暂时无法移动。请向游客致歉。",
    }.get(state, f"[系统事件] 机器人状态：{state}")


class GuideDialog:
    def __init__(self, backend, robot: Robot, log: Callable[[str], None] = print):
        self.backend = backend
        self.robot = robot
        self.log = log

    def execute(self, call: ToolCall) -> str:
        if call.name == "goto_exhibit":
            return self.robot.goto(str(call.args.get("exhibit_id", "")))
        if call.name == "stop_robot":
            return self.robot.stop()
        if call.name == "get_robot_status":
            return self.robot.describe_status()
        if call.name == "gesture":
            result = self.robot.gesture(str(call.args.get("name", "")), str(call.args.get("exhibit_id", "")))
            return result + "。如果话还没说完就接着说；已经说完了就不要再输出任何内容。"
        return f"失败：未知工具 {call.name}"

    def turn(self, text: str, on_text: Callable[[str], None]) -> str:
        """Run one user (or system-event) turn; returns the full spoken reply."""
        spoken: list[str] = []

        def emit(chunk: str) -> None:
            spoken.append(chunk)
            on_text(chunk)

        calls = self.backend.send(text, emit)
        for _ in range(MAX_TOOL_ROUNDS):
            if not calls:
                break
            results = []
            for call in calls:
                result = self.execute(call)
                self.log(f"[tool] {call.name}({call.args}) -> {result}")
                results.append((call, result))
            self.backend.add_tool_results(results)
            if "".join(spoken).strip() and any(c.name in NAV_TOOLS for c in calls):
                # "请跟我来" has been said and the robot is moving; the results ride along
                # with the next turn, so we do not pay for an extra round trip.
                break
            # Gestures accompany speech: the model often says one line, gestures, and
            # expects to go on talking once the tool results are back.
            calls = self.backend.send(None, emit)
        return "".join(spoken)
