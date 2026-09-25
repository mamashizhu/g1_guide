"""Offline dialog brain used when the LLM is unreachable (or no API key is set).

Rules pick the intent, the knowledge base supplies the words.  It speaks the same
backend interface as llm.py, so navigation still goes through ToolCalls and the
robot code does not care which brain is in charge.
"""

from __future__ import annotations

import math
import re
from collections import Counter
from typing import Callable

from dialog import Style, split_style
from exhibits import Exhibit
from llm import ToolCall

_PUNCT = re.compile(r"[\s，。？！、,.?!：:；;“”\"'（）()]")
_CN_NUM = {"一": 1, "二": 2, "两": 2, "三": 3, "四": 4, "五": 5, "六": 6, "七": 7, "八": 8, "九": 9}

CONTEXT_BONUS = 0.1   # head start for the exhibit being talked about
FAQ_THRESHOLD = 0.12  # TF-IDF cosine; unrelated questions score below 0.1 on our FAQ set
STOP_WORDS = ("停", "等一下", "等等", "别走", "站住", "先别", "暂停")
GOTO_WORDS = ("带我", "去", "领我", "走", "想看", "看看", "参观")
NEXT_WORDS = ("下一个", "下一站", "下一件", "接下来")
CONTINUE_WORDS = ("继续走", "接着走", "走吧", "继续")
LIST_WORDS = ("有什么展品", "有哪些", "都有什么", "有什么好看", "展品有", "介绍一下展馆")
GREET_WORDS = ("你好", "您好", "嗨", "哈喽", "早上好", "下午好")
WHO_WORDS = ("你是谁", "叫什么", "你是什么")
BYE_WORDS = ("再见", "拜拜", "谢谢你", "我走了")
REPEAT_WORDS = ("再说一遍", "没听清", "重复一下", "再讲一遍")
CHILD_WORDS = ("简单点", "简单一点", "小朋友", "听不懂", "我是小孩")
EXPERT_WORDS = ("专业一点", "专业点", "详细讲", "详细一点", "深入一点")
NORMAL_WORDS = ("正常讲", "普通模式", "正常模式")
FUNNY_WORDS = ("幽默一点", "幽默点", "搞笑", "有趣一点", "好玩一点")
SERIOUS_WORDS = ("严肃点", "严肃一点", "正经点", "正经一点", "别开玩笑")
INTRO_SENTENCES = {"child": 2, "normal": 4, "expert": 99}  # how much of the intro to read out
INTRO_WORDS = ("介绍", "讲讲", "讲一下", "说说", "是什么")
LOCATE_WORDS = ("在哪", "怎么走", "怎么去", "在什么地方")
INTEREST_WORDS = ("感兴趣", "喜欢", "想了解", "想知道", "有意思")
YES_WORDS = ("好", "可以", "行", "走吧", "去吧", "嗯", "要", "是的", "对")
NO_WORDS = ("不", "算了", "不用", "别")


def normalize(text: str) -> str:
    return _PUNCT.sub("", text)


def bigrams(text: str) -> set[str]:
    text = normalize(text)
    return {text[i:i + 2] for i in range(len(text) - 1)} or ({text} if text else set())


def overlap(query: str, candidate: str) -> float:
    """Fraction of the query's character bigrams found in the candidate."""
    q = bigrams(query)
    return len(q & bigrams(candidate)) / len(q) if q else 0.0


def char_grams(text: str) -> list[str]:
    text = normalize(text)
    return list(text) + [text[i:i + 2] for i in range(len(text) - 1)]


class FaqIndex:
    """Character unigram+bigram TF-IDF over every FAQ (question weighted twice)."""

    def __init__(self, exhibits: list[Exhibit]):
        self.entries = [(e, q, a) for e in exhibits for q, a in e.faqs]
        df = Counter()
        for _, q, a in self.entries:
            df.update(set(char_grams(q + a)))
        n = len(self.entries)
        self._idf = {g: math.log((n + 1) / (c + 1)) + 1 for g, c in df.items()}
        self._unseen = math.log(n + 1) + 1
        self._vectors = [self._vector(q + q + a) for _, q, a in self.entries]

    def _vector(self, text: str) -> dict[str, float]:
        counts = Counter(char_grams(text))
        vec = {g: c * self._idf.get(g, self._unseen) for g, c in counts.items()}
        norm = math.sqrt(sum(w * w for w in vec.values())) or 1.0
        return {g: w / norm for g, w in vec.items()}

    def search(self, query: str, focus: Exhibit | None = None) -> tuple[float, Exhibit, str] | None:
        """Best (raw score, exhibit, answer).  The focus bonus only reorders candidates;
        relevance is judged on the raw score so context cannot make noise look relevant."""
        qv = self._vector(query)
        best, best_rank = None, -1.0
        for (exhibit, _, answer), dv in zip(self.entries, self._vectors):
            score = sum(w * dv.get(g, 0.0) for g, w in qv.items())
            rank = score + (CONTEXT_BONUS if exhibit is focus else 0.0)
            if rank > best_rank:
                best, best_rank = (score, exhibit, answer), rank
        return best


def _fuzzy_contains(text: str, word: str) -> bool:
    """Substring match that forgives one wrong character (ASR homophones: 的球仪 → 地球仪)."""
    if word in text:
        return True
    if len(word) < 3:
        return False
    for start in range(len(text) - len(word) + 1):
        window = text[start:start + len(word)]
        if sum(a != b for a, b in zip(window, word)) <= 1:
            return True
    return False


def _has(text: str, words) -> bool:
    return any(w in text for w in words)


class RuleBackend:
    def __init__(self, exhibits: list[Exhibit]):
        self.exhibits = exhibits
        self.by_id = {e.exhibit_id: e for e in exhibits}
        self.current: Exhibit | None = None   # exhibit we are at, or talking about
        self.goal: Exhibit | None = None      # last navigation target
        self.last_reply = ""
        self.offer: Exhibit | None = None     # exhibit we just suggested visiting
        self.style = Style()                  # read from the tag GuideDialog puts on every message
        self.faq = FaqIndex(exhibits)
        self._call_seq = 0

    # ---- backend interface ----
    def warmup(self) -> None: ...

    def add_tool_results(self, results) -> None: ...

    def send(self, text: str | None, on_text: Callable[[str], None]) -> list[ToolCall]:
        if text is None:
            return []
        reply, calls = self.respond(text)
        self.last_reply = reply or self.last_reply
        on_text(reply)
        return calls

    # ---- understanding ----
    def find_exhibit(self, text: str) -> Exhibit | None:
        text = normalize(text)
        match = (re.search(r"第([1-6一二两三四五六])个", text)
                 or re.search(r"([1-6一二两三四五六])号", text)
                 or re.search(r"E0?([1-6])", text))
        if match:
            digit = match.group(1)
            index = int(digit) if digit.isdigit() else _CN_NUM[digit]
            if 1 <= index <= len(self.exhibits):
                return self.exhibits[index - 1]
        best, best_len = None, 0
        for exhibit in self.exhibits:
            for name in (exhibit.title, *exhibit.aliases):
                if len(name) >= 2 and len(name) > best_len and _fuzzy_contains(text, name):
                    best, best_len = exhibit, len(name)
        return best

    def respond(self, text: str) -> tuple[str, list[ToolCall]]:
        style, text = split_style(text)
        if style is not None:
            self.style = style
        event = re.match(r"\[系统事件\]", text)
        if event:
            return self._on_event(text)
        norm = normalize(text)
        exhibit = self.find_exhibit(norm)
        offer, self.offer = self.offer, None

        restyle = self._restyle(norm)
        if restyle:
            return restyle

        if offer and len(norm) <= 6 and not exhibit:
            if _has(norm, NO_WORDS):
                return "好的，那你想看看别的吗？", []
            if _has(norm, YES_WORDS):
                return self._goto(offer)
        if _has(norm, STOP_WORDS):
            return "好的，我停下来了。", [self._call("stop_robot", {})]
        if _has(norm, REPEAT_WORDS):
            return self.last_reply or "我刚才还没说什么呢。", []
        if _has(norm, NEXT_WORDS):
            target = self.by_id.get(self.current.next_id) if self.current else self.exhibits[0]
            return self._goto(target)
        if exhibit and _has(norm, GOTO_WORDS) and not _has(norm, ("为什么", "怎么", "什么")):
            return self._goto(exhibit)
        if exhibit and _has(norm, LOCATE_WORDS):
            self.offer = exhibit
            return f"{exhibit.title}就在展厅里，要我带你过去吗？", []
        if exhibit and _has(norm, INTEREST_WORDS):
            self.current = self.offer = exhibit
            return (f"{exhibit.title}确实很有意思。{self._first_sentences(exhibit.intro, 1)}"
                    "要我带你过去看看吗？"), []
        if _has(norm, CONTINUE_WORDS) and self.goal:
            return self._goto(self.goal)
        if _has(norm, LIST_WORDS):
            names = "、".join(e.title for e in self.exhibits)
            return f"这里一共有{len(self.exhibits)}件展品：{names}。你想先看哪一件？", []
        if _has(norm, WHO_WORDS):
            return "我是科技馆的导览机器人小吉，可以带你参观展品、给你讲解。", []
        if _has(norm, BYE_WORDS):
            return "谢谢你的参观，欢迎下次再来，再见！", [self._call("gesture", {"name": "wave"})]
        if _has(norm, GREET_WORDS) and len(norm) <= 6:
            return "你好呀，我是导览机器人小吉。想看什么展品，跟我说就行。", [self._call("gesture", {"name": "wave"})]
        return self._answer(norm, exhibit)

    # ---- actions ----
    def _call(self, name: str, args: dict) -> ToolCall:
        self._call_seq += 1
        return ToolCall(f"rule_{self._call_seq}", name, args)

    def _goto(self, exhibit: Exhibit | None) -> tuple[str, list[ToolCall]]:
        if exhibit is None:
            return "你想去哪一件展品呢？", []
        self.goal = exhibit
        return (f"好的，请跟我来，我们去看{exhibit.title}。",
                [self._call("goto_exhibit", {"exhibit_id": exhibit.exhibit_id})])

    def _answer(self, norm: str, exhibit: Exhibit | None) -> tuple[str, list[ToolCall]]:
        # Search every FAQ; the exhibit named in the question (or the one we stand at) gets a
        # head start, but a clearly better match elsewhere still wins ("阳光到地球" is not the globe).
        hit = self.faq.search(norm, focus=exhibit or self.current)
        if hit and hit[0] >= FAQ_THRESHOLD:
            _, best_exhibit, answer = hit
            self.current = best_exhibit
            return answer, []
        if exhibit and _has(norm, INTRO_WORDS + ("这个", "它")):
            self.current = exhibit
            return self._first_sentences(exhibit.intro, 3) + "你想去看看吗？", []
        if exhibit:
            self.current = exhibit
            return f"关于{exhibit.title}，这个问题我不太确定。你可以问问它的原理、用途，或者让我带你去看看。", []
        return "抱歉，我现在网络不太好，只能回答展品相关的问题。你可以说“有什么展品”，或者“带我去看地球仪”。", []

    def _restyle(self, norm: str) -> tuple[str, list[ToolCall]] | None:
        change, reply = {}, ""
        if _has(norm, CHILD_WORDS):
            change, reply = {"audience": "child"}, "好呀，那我讲得简单一点，小朋友也能听懂。"
        elif _has(norm, EXPERT_WORDS):
            change, reply = {"audience": "expert"}, "好的，接下来我讲得专业、详细一些。"
        elif _has(norm, NORMAL_WORDS):
            change, reply = {"audience": "normal"}, "好的，恢复正常讲解。"
        elif _has(norm, FUNNY_WORDS):
            change, reply = {"humor": min(10, self.style.humor + 3)}, "没问题，我尽量风趣一点。"
        elif _has(norm, SERIOUS_WORDS):
            change, reply = {"humor": 0}, "好的，我认真讲。"
        if not change:
            return None
        self.style.update(**change)
        return reply, [self._call("set_style", change)]

    def _on_event(self, text: str) -> tuple[str, list[ToolCall]]:
        match = re.search(r"(E\d\d)", text)
        exhibit = self.by_id.get(match.group(1)) if match else None
        if "已到达" in text and exhibit:
            self.current = exhibit
            self.goal = None
            nxt = f"看完之后，推荐你去看{exhibit.next_hint.split('：')[0]}。" if exhibit.next_hint else ""
            intro = self._first_sentences(exhibit.intro, INTRO_SENTENCES[self.style.audience])
            return (f"我们到了，这就是{exhibit.title}。{intro}{nxt}有问题可以随时问我。",
                    [self._call("gesture", {"name": "present", "exhibit_id": exhibit.exhibit_id})])
        if "挡住" in text:
            return "前面的路被挡住了，我先停一下，重新找条路。", []
        if "传感器" in text:
            return "抱歉，我的传感器出了点问题，暂时走不了。", []
        return "", []

    @staticmethod
    def _first_sentences(text: str, count: int) -> str:
        sentences = re.findall(r"[^。！？]+[。！？]", text.replace("\n", ""))
        return "".join(sentences[:count])
