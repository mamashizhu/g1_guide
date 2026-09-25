"""Exhibit catalogue: IDs and stations from exhibition.json, prose from knowledge/*.md."""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path

from config import EXHIBITION_JSON, KNOWLEDGE_DIR

_FAQ = re.compile(r"^- 问：(.+?)\s*答：(.+)$")


@dataclass(frozen=True)
class Exhibit:
    exhibit_id: str
    title: str
    kind: str
    station_key: str
    knowledge: str
    aliases: tuple[str, ...] = ()
    intro: str = ""
    faqs: tuple[tuple[str, str], ...] = ()
    next_hint: str = ""
    next_id: str = ""


def _section(markdown: str, heading: str) -> str:
    match = re.search(rf"^## {heading}\s*$(.*?)(?=^## |\Z)", markdown, re.M | re.S)
    return match.group(1).strip() if match else ""


def parse_knowledge(markdown: str) -> dict:
    alias_line = re.search(r"^别名：(.+)$", markdown, re.M)
    faqs = tuple(m.groups() for line in _section(markdown, "常见问题").splitlines()
                 if (m := _FAQ.match(line.strip())))
    next_hint = _section(markdown, "推荐下一站")
    next_id = re.match(r"(E\d\d)", next_hint)
    return {
        "aliases": tuple(a.strip() for a in alias_line.group(1).split("、")) if alias_line else (),
        "intro": _section(markdown, "讲解"),
        "faqs": faqs,
        "next_hint": re.sub(r"^E\d\d\s*", "", next_hint),
        "next_id": next_id.group(1) if next_id else "",
    }


def load_exhibits(scene_path: Path = EXHIBITION_JSON,
                  knowledge_dir: Path = KNOWLEDGE_DIR) -> list[Exhibit]:
    scene = json.loads(scene_path.read_text(encoding="utf-8"))
    exhibits = []
    for item in scene["exhibits"]:
        md = knowledge_dir / f"{item['exhibit_id']}.md"
        knowledge = md.read_text(encoding="utf-8").strip() if md.exists() else ""
        exhibits.append(Exhibit(
            exhibit_id=item["exhibit_id"],
            title=item["title"],
            kind=item["kind"],
            station_key=str(item["station"]["key"]),
            knowledge=knowledge,
            **parse_knowledge(knowledge),
        ))
    return exhibits
