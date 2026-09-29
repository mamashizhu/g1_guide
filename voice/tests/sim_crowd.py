r"""PLAN M5.5 端到端：语音引擎经 rosbridge 连 Gazebo（真机雷达版），走一遍观众流程。

先在 WSL 后台启动：bash tools/crowd_e2e_sim.sh crowd （rosbridge 端口 9097）
再在 Windows 运行：voice\.venv\Scripts\python.exe voice\tests\sim_crowd.py

流程：面向观众模式去 3 号展位 → 到站没人，说「大家跟上了吗」→ 放一个 1.1 m 圆柱当儿童
→ 机器人转向它、切儿童模式、开始讲解 → 删掉圆柱 → 5 秒后挥手、说再见。
离线大脑、不出声；北厅行走的 actor 经过时也会被算作观众，只会让某一步晚一点发生。
"""
import os
import subprocess
import sys
import time
from pathlib import Path

sys.stdout.reconfigure(encoding="utf-8")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
os.environ.setdefault("G1_ROSBRIDGE_PORT", "9097")

import crowd  # noqa: E402
from config import Settings  # noqa: E402
from engine import GuideEngine  # noqa: E402

SIM = "bash /mnt/d/g1_guide/tools/crowd_e2e_sim.sh"
KID = ("kid", -6.0, 8.0, 1.1)   # 1.6 m from station 3, behind-left
T0 = time.monotonic()
log: list[str] = []


def ui(kind, text=""):
    if kind in ("log", "event", "reply_chunk"):
        line = f"{time.monotonic() - T0:6.1f}s [{kind}] {text}"
        log.append(line)
        print(line, flush=True)


def wsl(command: str) -> None:
    subprocess.run(["wsl", "-e", "bash", "-lc", command], capture_output=True, timeout=60)


def wait_for(predicate, timeout: float, label: str) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            print(f"{time.monotonic() - T0:6.1f}s ✓ {label}", flush=True)
            return True
        time.sleep(0.2)
    print(f"{time.monotonic() - T0:6.1f}s ✗ 超时：{label}", flush=True)
    return False


def logged(text: str):
    return lambda: any(text in line for line in log)


def main() -> int:
    settings = Settings("deepseek", "m", "Realtek", "offline", "v", "localhost",
                        int(os.environ["G1_ROSBRIDGE_PORT"]))
    engine = GuideEngine(settings, use_robot=True, speak=False, listen=False, offline=True, ui=ui)
    engine.start()
    robot = engine.robot
    ok = wait_for(lambda: robot.connected and robot.people_near() is not None, 120, "rosbridge 已连上，有雷达人群数据")
    if not ok:
        return 1
    engine.set_present_mode("face")
    print(robot.goto("E03"), flush=True)
    checks = {}
    checks["到站"] = wait_for(lambda: engine.presenting is not None or logged("3 m 内没有人")(), 240, "到达 3 号展位")
    if engine.presenting is None:
        checks["没人时提醒"] = logged(crowd.FOLLOW_PROMPT)()
        wsl(f"{SIM} spawn {' '.join(map(str, KID))}")
        print(f"{time.monotonic() - T0:6.1f}s 放圆柱 {KID}", flush=True)
    checks["开始讲解"] = wait_for(lambda: engine.presenting is not None, 15, "有观众，开始讲解")
    checks["转向观众"] = wait_for(lambda: robot.status.get("state") == "faced"
                               or robot.status.get("faced"), 20, "转向观众")
    time.sleep(1.0)
    print(f"{time.monotonic() - T0:6.1f}s 导航状态 {robot.status}", flush=True)
    checks["儿童模式"] = engine.style.audience == "child" or print("当前风格", engine.style) or False
    wsl(f"{SIM} remove {KID[0]}")
    removed = time.monotonic()
    print(f"{removed - T0:6.1f}s 删掉圆柱", flush=True)
    checks["挥手告别"] = wait_for(logged("观众都离开了"), 30, "观众离开 → 挥手告别")
    checks["告别语"] = logged(crowd.GOODBYE)()
    engine.close()
    print("\n结果：" + "，".join(f"{k} {'✓' if v else '✗'}" for k, v in checks.items()))
    return 0 if all(checks.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
