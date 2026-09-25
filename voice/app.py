"""Console entry point of the voice guide (the window version is gui.py).

    run.cmd                  语音对话（麦克风 → 识别 → 大模型 → 语音 → 机器人）
    run.cmd --barge-in       允许在机器人说话时插话打断（建议戴耳机）
    run.cmd --text           键盘打字对话（加 --mute 不出声）
    run.cmd --offline        不用大模型，只用离线规则和知识库
    run.cmd --no-robot       不连接 rosbridge
"""

from __future__ import annotations

import argparse
import sys

from config import load_settings
from engine import GuideEngine, console_ui


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--text", action="store_true", help="键盘输入代替麦克风")
    parser.add_argument("--mute", action="store_true", help="不播放语音")
    parser.add_argument("--no-robot", action="store_true", help="不连接 rosbridge")
    parser.add_argument("--offline", action="store_true", help="不调用大模型")
    parser.add_argument("--barge-in", action="store_true", help="说话时可被打断（建议戴耳机）")
    parser.add_argument("--gain", type=float, default=2.0, help="麦克风增益")
    args = parser.parse_args()

    def ui(kind: str, text: str = "") -> None:
        if not (args.text and kind == "user"):  # typed text is already on screen
            console_ui(kind, text)

    engine = GuideEngine(load_settings(), use_robot=not args.no_robot, speak=not args.mute,
                         listen=not args.text, barge_in=args.barge_in, gain=args.gain,
                         offline=args.offline, ui=ui)
    engine.start()
    try:
        if args.text:
            while True:
                engine.wait_idle()
                engine.submit_text(input("你："))
                engine.wait_idle()
        else:
            print("（按回车可打断机器人，Ctrl+C 退出）")
            while True:
                input()
                engine.interrupt()
    except (KeyboardInterrupt, EOFError):
        print("\n再见")
    finally:
        engine.close()


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
