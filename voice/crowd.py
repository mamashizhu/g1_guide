"""Who is listening, from the lidar people on /guide/people (PLAN M5.5).

The engine uses these rules on arrival and while presenting:
- nobody within FOLLOW_RADIUS on arrival: ask FOLLOW_PROMPT, wait FOLLOW_WAIT_S,
  and skip the explanation if still nobody comes;
- children mode only for exactly one visitor who is a child, else normal;
- everybody gone for GONE_S while presenting: wave and say GOODBYE.

``facing_people`` fuses the front camera (/guide/vision, vision/vision_core.py)
with the lidar: 视觉唤醒 (PLAN M5.9) needs a frontal face of someone within
FOLLOW_RADIUS.
"""

from __future__ import annotations

FOLLOW_RADIUS = 3.0
FOLLOW_WAIT_S = 10.0
GONE_S = 5.0
FOLLOW_PROMPT = "大家跟上了吗？"
GOODBYE = "谢谢大家的参观，再见！"
MATCH_BEARING_DEG = 8.0  # face and lidar person this close in bearing = same person
# A face-width distance this many times the lidar distance is someone farther
# away behind that person (simulated faces are small and read ~2x as far).
MATCH_MAX_DISTANCE_RATIO = 2.5


def head_count(people: list[dict]) -> int:
    """A cluster of several people standing together counts as its ``group``."""
    return sum(int(p.get("group", 1)) for p in people)


def audience_for(people: list[dict]) -> str:
    """``child`` only for a single visitor who is a child; everyone else is normal."""
    if head_count(people) == 1 and people[0].get("child"):
        return "child"
    return "normal"


def describe(people: list[dict]) -> str:
    children = sum(1 for p in people if p.get("child"))
    text = f"3 m 内 {head_count(people)} 人"
    return text + (f"（{children} 名儿童）" if children else "")


def facing_people(vision: dict, people: list[dict] | None, radius: float = FOLLOW_RADIUS) -> list[dict]:
    """Frontal faces of people within ``radius``.

    With lidar people (``/guide/people``, bearings in the same left-positive
    convention) a face counts when a person within ``radius`` lies within
    MATCH_BEARING_DEG of it and the face is not far behind it (a distant face
    in line with a near person who turned away), and takes that person's
    distance.  Without lidar
    data the face-width distance is used.
    """
    found = []
    for face in vision.get("faces", ()):
        if not face.get("facing"):
            continue
        if people is None:
            if face["distance"] <= radius:
                found.append(dict(face))
            continue
        near = [p for p in people if p["distance"] <= radius
                and abs(p["bearing_deg"] - face["bearing_deg"]) <= MATCH_BEARING_DEG
                and face["distance"] <= MATCH_MAX_DISTANCE_RATIO * p["distance"]]
        if near:
            person = min(near, key=lambda p: abs(p["bearing_deg"] - face["bearing_deg"]))
            found.append({**face, "distance": person["distance"], "person_id": person.get("id")})
    return found

