"""PLAN M5.9: 视觉唤醒 -- the wake word counts only while someone near faces the robot."""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from config import Settings  # noqa: E402
from crowd import facing_people  # noqa: E402
from engine import GuideEngine  # noqa: E402
from robot import RobotLink  # noqa: E402

FRONT = {"box": [300, 100, 330, 130], "conf": 0.9, "distance": 3.5, "bearing_deg": 2.0, "facing": True}
PROFILE = dict(FRONT, facing=False)
NEAR_PERSON = {"id": 1, "distance": 1.8, "bearing_deg": 0.0}
FAR_PERSON = {"id": 2, "distance": 4.5, "bearing_deg": 0.0}


def make(mode="vision"):
    settings = Settings("deepseek", "m", "Realtek", "offline", "v", "localhost", 9090)
    log = []
    engine = GuideEngine(settings, use_robot=False, speak=False, listen=False, offline=True,
                         ui=lambda kind, text="": log.append((kind, text)))
    engine.set_wake_mode(mode)
    return engine, log


def feed(engine, faces, people):
    robot = engine.robot
    robot.people = {"people": people}
    robot.people_at = time.monotonic()
    robot.on_vision({"persons": [], "faces": faces})


def queued(engine):
    return [text for kind, text in list(engine.inputs.queue) if kind == "user"]


def test_fusion_uses_lidar_distance_and_bearing():
    assert facing_people({"faces": [FRONT]}, [NEAR_PERSON])[0]["distance"] == 1.8
    assert facing_people({"faces": [FRONT]}, [FAR_PERSON]) == []
    assert facing_people({"faces": [PROFILE]}, [NEAR_PERSON]) == []
    assert facing_people({"faces": [dict(FRONT, bearing_deg=20.0)]}, [NEAR_PERSON]) == []
    assert facing_people({"faces": [FRONT]}, []) == []                     # lidar sees nobody
    far_face_behind = dict(FRONT, distance=12.0)                          # walker behind a near person
    assert facing_people({"faces": [far_face_behind]}, [NEAR_PERSON]) == []
    assert facing_people({"faces": [dict(FRONT, distance=2.5)]}, None)      # no lidar: face width


def test_vision_mode_needs_a_wake_word_even_when_facing():
    engine, _ = make()
    feed(engine, [FRONT], [NEAR_PERSON])
    assert engine.state == "sleeping"
    engine.handle_heard("带我去看青花瓷")
    assert queued(engine) == []


def test_wake_word_with_someone_facing_wakes():
    engine, _ = make()
    feed(engine, [FRONT], [NEAR_PERSON])
    engine.handle_heard("你好小吉，带我去看青花瓷")
    assert queued(engine) == ["带我去看青花瓷"] and engine.awake


def test_wake_word_while_nobody_faces_is_ignored():
    for faces, people in (([PROFILE], [NEAR_PERSON]), ([FRONT], [FAR_PERSON]), ([], [NEAR_PERSON])):
        engine, log = make()
        feed(engine, faces, people)
        engine.handle_heard("你好小吉，带我去看青花瓷")
        assert queued(engine) == [] and not engine.awake
        assert any("没看到" in text for _, text in log)


def test_recent_look_is_enough():
    engine, _ = make()
    feed(engine, [FRONT], [NEAR_PERSON])
    feed(engine, [PROFILE], [NEAR_PERSON])       # turned away a moment ago
    engine.handle_heard("你好小吉")
    assert engine.awake
    engine.robot.facing_at -= 5.0
    engine._awake_until = 0.0
    engine.handle_heard("你好小吉")
    assert not engine.awake


def test_no_camera_data_falls_back_to_direct_wake():
    engine, log = make()
    engine.handle_heard("你好小吉，带我去看青花瓷")
    assert queued(engine) == ["带我去看青花瓷"]
    assert any("没有相机数据" in text for _, text in log)


def test_direct_mode_ignores_the_camera():
    engine, _ = make("direct")
    feed(engine, [], [NEAR_PERSON])
    engine.handle_heard("带我去看青花瓷")                 # wake word off: always listening
    assert queued(engine) == ["带我去看青花瓷"]


def test_typed_text_is_never_gated():
    engine, _ = make()
    feed(engine, [], [NEAR_PERSON])
    engine.submit_text("带我去看青花瓷")
    assert queued(engine) == ["带我去看青花瓷"]


def test_robot_link_parses_vision_message():
    robot = RobotLink([], on_event=None, log=lambda _: None)
    robot._on_vision({"data": '{"persons": [], "faces": []}'})
    robot._on_vision({"data": "not json"})
    assert robot.vision_live and not robot.facing_recently()


# ---------------- PLAN M5.1 gestures ----------------

def test_count_gesture_goes_to_station_when_awake():
    engine, log = make("direct")
    engine.robot.on_vision({"faces": [], "gesture": "three"})
    assert queued(engine) == ["带我去看星月夜"]


def test_gestures_need_wake_except_palm():
    engine, log = make("direct")
    engine.wake_word = True
    engine.robot.on_vision({"faces": [], "gesture": "two"})
    assert queued(engine) == [] and any("未唤醒" in t for _, t in log)
    stopped = []
    engine.interrupt = lambda: stopped.append(True)
    engine.robot.on_vision({"faces": [], "gesture": "palm"})
    assert stopped


def test_speech_has_priority_over_gestures():
    engine, log = make("direct")
    engine.turn_active.set()
    engine.robot.on_vision({"faces": [], "gesture": "thumbs_up"})
    assert queued(engine) == [] and any("语音优先" in t for _, t in log)


def test_wave_greets_or_says_goodbye():
    engine, _ = make("direct")
    engine.robot.on_vision({"faces": [], "gesture": "wave"})
    engine.presenting = object()
    engine.robot.on_vision({"faces": [], "gesture": "wave"})
    engine.robot.on_vision({"faces": [], "gesture": "thumbs_up"})
    assert queued(engine) == ["你好", "再见", "好的"]

