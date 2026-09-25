# 语音导览（Windows 端）

麦克风 → VAD → 语音识别 → 大模型（工具调用）→ 语音合成 → 扬声器，同时通过 rosbridge 驱动仿真里的机器人。

## 首次安装

```bash
D:\python\python.exe -m venv voice\.venv
voice\.venv\Scripts\python.exe -m pip install -r voice\requirements.txt
voice\.venv\Scripts\python.exe voice\download_models.py
setx DEEPSEEK_API_KEY "你的key"
```

模型大约 400MB，下载后放在 `voice/models/`，不进 git。

## 运行

| 命令 | 作用 |
|------|------|
| `voice\gui.cmd` | 控制台窗口：对话记录、语音状态、大模型/机器人状态、快捷按钮、日志 |
| `voice\run.cmd` | 终端版：直接对麦克风说话，按回车打断机器人 |
| `voice\run.cmd --text` | 打字对话，用来调提示词 |
| `voice\check_mic.py` | 麦克风检查：录 4 秒，显示音量，回放，给出建议增益 |

常用参数（两个入口都支持）：`--no-robot` 不连机器人；`--offline` 不用大模型；`--barge-in` 允许插话打断（建议戴耳机）；`--wake` 需要唤醒词；`--gain 2` 麦克风增益；`--mute` 不出声。终端版另有 `--audience child|normal|expert` 和 `--humor 0-10`。

## 唤醒词、讲解模式、幽默值

| 功能 | 怎么用 |
|------|--------|
| 唤醒词 | 开启后说“你好小吉”或“小吉，带我去看地球仪”才开始对话；机器人说话、思考、走路时保持唤醒，空闲 30 秒或说“再见”后休眠。唤醒词要在句首（前面可带“你好/嗨/喂”），“小机器人”不会误唤醒；按读音匹配，所以识别成“小鸡”“晓吉”也能唤醒 |
| 讲解模式 | 普通 / 儿童（短句、打比方、语速放慢到 0.85）/ 专业（术语、原理、数据）。说“讲简单点”“我是小朋友”“讲专业一点”“正常讲就行”即可切换 |
| 幽默值 | 0 完全严肃，3 偶尔轻松，6 以上常开玩笑，9 以上像脱口秀；知识始终保持准确。说“幽默一点”（+3）或“严肃点”（归零） |

窗口里有对应的开关、下拉框和滑块，语音切换后会自动同步。离线兜底同样听得懂这些说法，讲解长度随模式变化（离线时没有幽默效果）。

环境变量：`G1_LLM_PROVIDER`（deepseek / qwen / claude，默认按已设置的 Key 自动选），`G1_LLM_MODEL`，`G1_TTS=edge` 换成在线微软语音（音质更好，每句多约 2 秒），`G1_MIC` 麦克风名称关键字（默认 Realtek）。

## 连接仿真里的机器人

WSL 两个终端：

```bash
ros2 launch g1_guide exhibition_nav.launch.py
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

再在 Windows 启动 `voice\gui.cmd`，状态栏显示"机器人 已连接"。说"带我去看地球仪"，机器人走到地球仪，到站后自动开始讲解。

| 话题 | 方向 | 用途 |
|------|------|------|
| `/nav/goal_key` | 语音 → 机器人 | 展位键 `1`–`6` 或 `cancel` |
| `/nav/status` | 机器人 → 语音 | 到站、受阻等事件，触发主动讲解 |
| `/guide/gesture` | 语音 → 机器人 | 手势，如 `{"name":"point","exhibit":"E03"}` |

## 结构

| 文件 | 内容 |
|------|------|
| `engine.py` | 核心：线程编排、打断、回声过滤（拼音比对） |
| `dialog.py` | 系统提示词、工具定义、一轮对话（含工具调用） |
| `llm.py` | DeepSeek/千问（OpenAI 兼容）和 Claude 后端；失败自动切到离线规则 |
| `fallback.py` | 离线大脑：规则意图 + 知识库 TF-IDF 问答 |
| `asr.py` / `tts.py` / `audio_io.py` | SenseVoice 识别、Matcha 离线合成、可中断播放 |
| `robot.py` | rosbridge 连接、导航和手势 |
| `../knowledge/E0*.md` | 展品讲解资料（别名、正文、问答、推荐下一站） |

## 测试

```bash
voice\.venv\Scripts\python.exe -m pytest voice\tests
voice\.venv\Scripts\python.exe voice\tests\sim_barge_in.py
```

第二条用合成语音模拟麦克风，验证三种打断场景（约 45 秒）。

## 实测数据（2026-09-25）

- 识别：合成语音 3/3 句完全正确，速度约为实时的 10 倍
- 离线合成：每句约 0.2 秒；停止播放 33ms 生效
- 说完到开口：1.5–3 秒，主要取决于 DeepSeek 的响应速度（直连 0.5–2.5 秒；走本机代理会多 2 秒，所以国内厂商自动直连）
- 打断：戴耳机约 0.6 秒停，外放约 0.9 秒停；只有回声时不会误打断
