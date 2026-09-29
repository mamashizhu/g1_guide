r"""PLAN M5.9 视觉唤醒端到端：Gazebo 前置相机 → vision\watch.py → /guide/vision → 语音引擎。

先在 WSL 后台启动：VISION=true bash tools/crowd_e2e_sim.sh vision （rosbridge 端口 9097）
再在 Windows 运行：voice\.venv\Scripts\python.exe voice\tests\sim_vision_wake.py

本脚本自己启动 vision\watch.py，并在机器人正前方 1.8 m 放一个静态参观者模型
（Fuel「Standing person」，模型朝向 = 机器人朝向 − 90° 时正对机器人）。
唤醒词用文字注入，等同于语音识别结果；离线大脑、不出声。
  A 视觉唤醒 + 参观者正对机器人 → 说「你好小吉」应唤醒
  B 视觉唤醒 + 参观者背对       → 应忽略
  C 视觉唤醒 + 参观者侧身       → 应忽略
  D 直接唤醒 + 参观者背对       → 应唤醒（不看相机）
"""
import math
import os
import subprocess
import sys
import time
from pathlib import Path

sys.stdout.reconfigure(encoding="utf-8")
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "voice"))
os.environ.setdefault("G1_ROSBRIDGE_PORT", "9097")

from config import Settings  # noqa: E402
from engine import VISION_WAKE_WINDOW_S, GuideEngine  # noqa: E402

SIM = "bash /mnt/d/g1_guide/tools/crowd_e2e_sim.sh"
T0 = time.monotonic()


def ui(kind, text=""):
    if kind == "log":
        print(f"{time.monotonic() - T0:6.1f}s {text}", flush=True)


def wsl(command: str) -> None:
    subprocess.run(["wsl", "-e", "bash", "-lc", command], capture_output=True, timeout=90)


def wait_for(predicate, timeout, label):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            print(f"{time.monotonic() - T0:6.1f}s ✓ {label}", flush=True)
            return True
        time.sleep(0.2)
    print(f"{time.monotonic() - T0:6.1f}s ✗ 超时：{label}", flush=True)
    return False


def try_wake(engine) -> bool:
    engine._awake_until = 0.0
    engine.handle_heard("你好小吉")
    return engine.awake


def main() -> int:
    port = os.environ["G1_ROSBRIDGE_PORT"]
    watch = subprocess.Popen([str(ROOT / "vision/.venv/Scripts/python.exe"), str(ROOT / "vision/watch.py"),
                              "--port", port], cwd=str(ROOT / "vision"))
    try:
        settings = Settings("deepseek", "m", "Realtek", "offline", "v", "localhost", int(port))
        engine = GuideEngine(settings, use_robot=True, speak=False, listen=False, offline=True, ui=ui)
        engine.crowd_enabled = False   # this test is about waking only
        engine.start()
        robot = engine.robot
        if not wait_for(lambda: robot.connected and robot.vision_live and robot.people_near() is not None,
                        180, "rosbridge、雷达人群、相机数据都到了"):
            return 1
        rx, ry, ryaw = robot.people["robot"]
        px, py = rx + 1.8 * math.cos(ryaw), ry + 1.8 * math.sin(ryaw)
        engine.set_wake_mode("vision")
        results = {}
        for label, turn, mode, expect in (("A 正对 → 唤醒", -math.pi / 2, "vision", True),
                                          ("B 背对 → 忽略", math.pi / 2, "vision", False),
                                          ("C 侧身 → 忽略", 0.0, "vision", False),
                                          ("D 直接唤醒 + 背对 → 唤醒", math.pi / 2, "direct", True)):
            wsl(f"{SIM} remove visitor_probe")
            wsl(f"{SIM} person visitor_probe {px:.3f} {py:.3f} {ryaw + turn:.4f}")
            engine.set_wake_mode(mode)
            engine.wake_word = True
            # Let the last "facing" moment age out, then look for a while.
            time.sleep(VISION_WAKE_WINDOW_S + 3.0)
            vision = robot.vision
            print(f"  相机：{len(vision.get('persons', ()))} 人，人脸 "
                  f"{[(f['bearing_deg'], f['facing']) for f in vision.get('faces', ())]}，"
                  f"3 m 内面向 {len(vision.get('facing_near', ()))}；雷达 3 m 内 "
                  f"{[(p['distance'], p['bearing_deg']) for p in robot.people_near() or ()]}", flush=True)
            woke = try_wake(engine)
            results[label] = woke == expect
            print(f"{time.monotonic() - T0:6.1f}s {label}：{'唤醒' if woke else '未唤醒'}", flush=True)
        wsl(f"{SIM} remove visitor_probe")
        engine.close()
        print("\n结果：" + "，".join(f"{k} {'✓' if v else '✗'}" for k, v in results.items()))
        return 0 if all(results.values()) else 1
    finally:
        watch.terminate()


if __name__ == "__main__":
    sys.exit(main())
