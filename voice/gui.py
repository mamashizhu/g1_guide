"""Control console window for the voice guide (same engine as app.py).

    gui.cmd                 麦克风 + 大模型 + 机器人
    gui.cmd --no-robot      不连接 rosbridge
    gui.cmd --offline       只用离线规则
    gui.cmd --no-mic        不开麦克风，只用输入框和按钮
    gui.cmd --wake          需要唤醒词“你好小吉”（窗口里也能随时开关）
"""

from __future__ import annotations

import argparse
import html
import sys
import threading
import time

from PySide6.QtCore import QObject, Qt, QTimer, Signal
from PySide6.QtGui import QFont, QTextCursor
from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox, QGridLayout, QGroupBox,
                               QHBoxLayout, QLabel, QLineEdit, QMainWindow, QPlainTextEdit,
                               QProgressBar, QPushButton, QSlider, QTextBrowser, QVBoxLayout,
                               QWidget)

from config import load_settings
from crowd import describe
from dialog import AUDIENCES
from robot import PRESENT_MODES
from engine import WAKE_MODES, GuideEngine

STATE_STYLE = {
    "listening": ("倾听中", "#2e7d32"),
    "hearing": ("听到声音", "#1565c0"),
    "thinking": ("思考中", "#ef6c00"),
    "speaking": ("说话中", "#6a1b9a"),
    "muted": ("麦克风关闭", "#757575"),
    "idle": ("等待输入", "#757575"),
    "sleeping": ("待唤醒：说“你好小吉”", "#37474f"),
}
NAV_STATE = {
    "unknown": "未知", "requested": "收到目标", "moving": "移动中", "aligning": "调整朝向",
    "arrived": "已到达", "blocked": "路线受阻", "backing_up": "后退避让", "cancelled": "已停止", "invalid_goal": "目标无效",
    "sensor_stale": "传感器中断", "invalid_command": "指令异常",
    "facing": "转向观众", "faced": "已面向观众",
}


class Bridge(QObject):
    """Moves engine callbacks (worker threads) onto the Qt thread."""
    message = Signal(str, str)


def pill(label: QLabel, text: str, color: str) -> None:
    label.setText(text)
    label.setStyleSheet(f"background:{color};color:white;border-radius:9px;padding:3px 10px;"
                        "font-weight:bold;")


class GuideWindow(QMainWindow):
    def __init__(self, engine: GuideEngine, bridge: Bridge):
        super().__init__()
        self.engine = engine
        self.setWindowTitle("小吉 · 展馆导览控制台")
        self.resize(1100, 700)
        bridge.message.connect(self.on_message)

        # ---- left: conversation ----
        self.chat = QTextBrowser()
        self.chat.setFont(QFont("Microsoft YaHei", 11))
        self.input = QLineEdit()
        self.input.setPlaceholderText("也可以在这里打字，回车发送")
        self.input.returnPressed.connect(self.send_typed)
        send = QPushButton("发送")
        send.clicked.connect(self.send_typed)
        stop_talk = QPushButton("打断")
        stop_talk.clicked.connect(engine.interrupt)
        row = QHBoxLayout()
        row.addWidget(self.input, 1)
        row.addWidget(send)
        row.addWidget(stop_talk)
        left = QVBoxLayout()
        left.addWidget(self.chat, 1)
        left.addLayout(row)

        # ---- right: status ----
        self.state_label = QLabel()
        self.level = QProgressBar()
        self.level.setRange(0, 100)
        self.level.setTextVisible(False)
        self.level.setFixedHeight(10)
        self.llm_label = QLabel()
        self.robot_label = QLabel()
        self.nav_label = QLabel()
        self.nav_label.setWordWrap(True)
        status = QGridLayout()
        status.addWidget(QLabel("语音"), 0, 0)
        status.addWidget(self.state_label, 0, 1)
        status.addWidget(self.level, 1, 0, 1, 2)
        status.addWidget(QLabel("大模型"), 2, 0)
        status.addWidget(self.llm_label, 2, 1)
        status.addWidget(QLabel("机器人"), 3, 0)
        status.addWidget(self.robot_label, 3, 1)
        status.addWidget(self.nav_label, 4, 0, 1, 2)
        self.people_label = QLabel()
        status.addWidget(self.people_label, 5, 0, 1, 2)
        status_box = QGroupBox("状态")
        status_box.setLayout(status)

        self.mic_box = QCheckBox("麦克风开启")
        self.mic_box.setChecked(True)
        self.mic_box.setEnabled(engine.listen)
        self.mic_box.toggled.connect(lambda on: setattr(engine, "mic_enabled", on))
        self.barge_box = QCheckBox("允许插话打断（建议戴耳机）")
        self.barge_box.setChecked(engine.barge_in)
        self.barge_box.setEnabled(engine.listen)
        self.barge_box.toggled.connect(lambda on: setattr(engine, "barge_in", on))
        self.wake_box = QCheckBox("需要唤醒词“你好小吉”（30 秒无人说话自动休眠）")
        self.wake_box.setChecked(engine.wake_word)
        self.wake_box.setEnabled(engine.listen)
        self.wake_box.toggled.connect(self.toggle_wake)
        self.audience = QComboBox()
        for key, name in AUDIENCES.items():
            self.audience.addItem(f"{name}模式", key)
        self.audience.currentIndexChanged.connect(
            lambda _: engine.set_style(audience=self.audience.currentData()))
        self.humor = QSlider(Qt.Orientation.Horizontal)
        self.humor.setRange(0, 10)
        self.humor.setPageStep(1)
        self.humor_label = QLabel()
        self.humor.valueChanged.connect(self.on_humor)
        style_row = QHBoxLayout()
        style_row.addWidget(QLabel("讲解"))
        style_row.addWidget(self.audience)
        style_row.addWidget(QLabel("幽默"))
        style_row.addWidget(self.humor, 1)
        style_row.addWidget(self.humor_label)
        self.present = QComboBox()
        for key, name in PRESENT_MODES.items():
            self.present.addItem(name, key)
        self.present.currentIndexChanged.connect(
            lambda _: engine.set_present_mode(self.present.currentData()))
        self.crowd_box = QCheckBox("按雷达人数讲解：没人不讲、自动判断儿童、人走光告别（需 lidar:=mid360）")
        self.crowd_box.setChecked(engine.crowd_enabled)
        self.crowd_box.toggled.connect(lambda on: setattr(engine, "crowd_enabled", on))
        self.wake_mode = QComboBox()
        for key, name in WAKE_MODES.items():
            self.wake_mode.addItem(name, key)
        self.wake_mode.setToolTip("视觉唤醒：说唤醒词时，前置相机要看到 3 m 内有人面向机器人"
                                  "（需要运行 vision/watch.py）；没有相机数据时按直接唤醒处理")
        self.wake_mode.currentIndexChanged.connect(
            lambda _: engine.set_wake_mode(self.wake_mode.currentData()))
        present_row = QHBoxLayout()
        present_row.addWidget(QLabel("到站后"))
        present_row.addWidget(self.present)
        present_row.addWidget(QLabel("唤醒"))
        present_row.addWidget(self.wake_mode)
        present_row.addStretch(1)
        options = QVBoxLayout()
        options.addWidget(self.mic_box)
        options.addWidget(self.barge_box)
        options.addWidget(self.wake_box)
        options.addLayout(style_row)
        options.addLayout(present_row)
        options.addWidget(self.crowd_box)
        options_box = QGroupBox("选项")
        options_box.setLayout(options)

        goto = QGridLayout()
        for i, exhibit in enumerate(engine.exhibits):
            button = QPushButton(f"{exhibit.station_key}. {exhibit.title}")
            button.clicked.connect(lambda _=False, e=exhibit: engine.submit_text(f"带我去看{e.title}"))
            goto.addWidget(button, i // 2, i % 2)
        stop_robot = QPushButton("停下")
        stop_robot.clicked.connect(lambda: engine.submit_text("停下"))
        goto.addWidget(stop_robot, (len(engine.exhibits) + 1) // 2, 0, 1, 2)
        goto_box = QGroupBox("快捷指令（等同于说“带我去看…”）")
        goto_box.setLayout(goto)

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(500)
        self.log_view.setFont(QFont("Consolas", 9))
        log_box = QGroupBox("日志（工具调用、打断、回声过滤）")
        log_layout = QVBoxLayout()
        log_layout.addWidget(self.log_view)
        log_box.setLayout(log_layout)

        right = QVBoxLayout()
        for box in (status_box, options_box, goto_box):
            right.addWidget(box)
        right.addWidget(log_box, 1)
        right_widget = QWidget()
        right_widget.setLayout(right)
        right_widget.setFixedWidth(400)

        root = QHBoxLayout()
        root.addLayout(left, 1)
        root.addWidget(right_widget)
        central = QWidget()
        central.setLayout(root)
        self.setCentralWidget(central)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)
        self.refresh()

    # ---- options ----
    def toggle_wake(self, on: bool) -> None:
        self.engine.wake_word = on
        if on:
            self.engine._awake_until = 0.0  # start asleep so the wake word is needed right away

    def on_humor(self, value: int) -> None:
        self.humor_label.setText(f"{value}/10")
        if value != self.engine.style.humor:
            self.engine.set_style(humor=value)

    def sync_style(self) -> None:
        """Follow style changes made by voice ("讲简单点", "幽默一点")."""
        style = self.engine.style
        index = self.audience.findData(style.audience)
        if index != self.audience.currentIndex():
            self.audience.blockSignals(True)
            self.audience.setCurrentIndex(index)
            self.audience.blockSignals(False)
        if self.humor.value() != style.humor:
            self.humor.blockSignals(True)
            self.humor.setValue(style.humor)
            self.humor.blockSignals(False)
        self.humor_label.setText(f"{style.humor}/10")

    # ---- conversation ----
    def send_typed(self) -> None:
        text = self.input.text().strip()
        if text:
            self.input.clear()
            self.engine.submit_text(text)

    def _header(self, who: str, color: str) -> None:
        if not self.chat.document().isEmpty():
            self.chat.append("")
        stamp = time.strftime("%H:%M:%S")
        self.chat.append(f'<span style="color:{color};font-weight:bold">{who}</span> '
                         f'<span style="color:#999">{stamp}</span>')

    def on_message(self, kind: str, text: str) -> None:
        if kind == "user":
            self._header("你", "#1565c0")
            self.chat.append(html.escape(text))
        elif kind == "event":
            self.chat.append("")
            self.chat.append(f'<span style="color:#888">⚙ {html.escape(text)}</span>')
        elif kind == "reply_start":
            self._header("小吉", "#6a1b9a")
            self.chat.append("")  # streamed chunks go into this paragraph
        elif kind == "reply_chunk":
            self.chat.moveCursor(QTextCursor.MoveOperation.End)
            self.chat.insertPlainText(text)
        elif kind == "log":
            self.log_view.appendPlainText(f"{time.strftime('%H:%M:%S')} {text}")
        self.chat.moveCursor(QTextCursor.MoveOperation.End)

    # ---- status ----
    def refresh(self) -> None:
        engine = self.engine
        self.sync_style()
        text, color = STATE_STYLE.get(engine.state, (engine.state, "#757575"))
        pill(self.state_label, text, color)
        level = engine.listener.level if engine.listener is not None else 0.0
        self.level.setValue(min(100, int(level * 400)))

        backend = engine.backend
        if backend.primary is None:
            pill(self.llm_label, "离线规则", "#757575")
        elif backend.online:
            pill(self.llm_label, f"{engine.settings.provider} 在线", "#2e7d32")
        else:
            pill(self.llm_label, "暂时离线（规则兜底）", "#c62828")

        robot = engine.robot
        if robot.connected:
            pill(self.robot_label, "已连接", "#2e7d32")
            status = dict(robot.status)
            exhibit = robot.by_key.get(str(status.get("goal_key")))
            state = NAV_STATE.get(status.get("state"), status.get("state"))
            target = f"　目标：{exhibit.title}" if exhibit else ""
            distance = status.get("distance_m")
            far = f"　剩余 {distance:.1f} m" if isinstance(distance, (int, float)) else ""
            self.nav_label.setText(f"导航：{state}{target}{far}")
            people = robot.people_near()
            text = ("雷达观众：无数据（简化雷达版没有人群感知）" if people is None
                    else f"雷达观众：{describe(people)}")
            if robot.vision_live:
                vision = robot.vision
                text += (f"　相机：{len(vision.get('persons', ()))} 人，"
                         f"{len(vision.get('facing_near', ()))} 人面向机器人")
            else:
                text += "　相机：无数据"
            self.people_label.setText(text)
        else:
            pill(self.robot_label, "未连接", "#757575")
            self.nav_label.setText("导航：—（在 WSL 启动 rosbridge 后重开本窗口）")
            self.people_label.setText("")

    def closeEvent(self, event) -> None:
        self.timer.stop()
        self.engine.close()
        super().closeEvent(event)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-robot", action="store_true")
    parser.add_argument("--offline", action="store_true")
    parser.add_argument("--no-mic", action="store_true")
    parser.add_argument("--mute", action="store_true")
    parser.add_argument("--barge-in", action="store_true")
    parser.add_argument("--gain", type=float, default=2.0)
    parser.add_argument("--wake", action="store_true")
    args = parser.parse_args()

    app = QApplication(sys.argv)
    bridge = Bridge()
    engine = GuideEngine(load_settings(), use_robot=not args.no_robot, speak=not args.mute,
                         listen=not args.no_mic, barge_in=args.barge_in, gain=args.gain,
                         offline=args.offline, wake_word=args.wake,
                         ui=lambda kind, text="": bridge.message.emit(kind, text))
    window = GuideWindow(engine, bridge)
    window.show()
    threading.Thread(target=engine.start, daemon=True).start()  # model loading must not freeze the window
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
