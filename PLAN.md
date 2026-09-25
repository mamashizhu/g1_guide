# G1 展馆导览机器人：任务清单与执行计划

分支：`claude/guide-robot`　维护者：Claude　创建日期：2026-09-25

本计划不沿用 `docs/agent-workboard.md` 的旧清单，按课程要求重新拆分。原则：
1. **先做必做项，再做亮点**，每一项都要在本机上真实跑通。
2. 只做仿真与交互，**不接真机**。
3. 项目目录是 `D:\g1_guide`（从 `D:\智能机器人\claude_course` 迁出，保留 git 历史）。旧目录只作参考，不再修改。
4. 每项任务标注执行方：🤖 = Claude 能在本机独立完成并验证；👤 = 需要你本人操作。

---

## 0. 本机条件（2026-09-25 实测）

| 项目 | 状态 | 对计划的影响 |
|------|------|-------------|
| WSL Ubuntu 26.04 + ROS lyrical + Gazebo Jetty | ✅ 可用 | 仿真与导航都在 WSL 里跑 |
| RTX 4060 8GB，WSL 内存约 7.5GB | ✅ | YOLO 训练可行；全栈同时运行要注意内存 |
| WSL Python 3.14：有 cv2、numpy、PySide6、cv_bridge，**无 pip** | ⚠️ | 需要 `apt install python3-venv python3-pip` |
| apt 可装：slam_toolbox、nav2、rosbridge_server、espeak-ng、portaudio | ✅ | 建图和跨系统通信有现成包 |
| Windows Python 3.13（`D:\python`） | ✅ | **语音、摄像头、大模型调用放在 Windows 端**，原生支持麦克风和扬声器 |
| WSL 声音（WSLg PulseServer） | ⚠️ 不稳定 | 不在 WSL 里录音或播放 |
| WSL NAT 模式不镜像 localhost 代理 | ⚠️ | 需要代理的网络请求放在 Windows 端发 |

### 已有底座（可以直接复用）
- 展馆世界：导入 Fuel 的真实博物馆网格 `iscas_museum`（22.5×20.5m），4 张海报贴真实墙面、2 个立体展品、3 个静态参观者、2 个行走 actor（`config/exhibition.json`）。原版 16×12m 长方体房间保留在 `config/exhibition_simple.json`，SLAM 仍用它。详见 [docs/exhibition-hall.md](docs/exhibition-hall.md)
- 平移代理：VelocityControl 加关闭重力，机器人平移不倒
- 低阶控制：固定骨盆的 29 关节力矩 PD 控制，以及手臂指向（在单独的 `lowlevel` 模型里）
- 传感器：RGB-D、2D 雷达 `/scan` 和 `/nav_scan`、IMU、odom，另有传感器面板
- 导航：A* 栅格、路径跟踪、数字键入口的代码都已写好，**但尚未在 Gazebo 中验收**

### 已有基础与课程要求对照（2026-09-25 盘点）

状态说明：✅ 已完成且有 Gazebo 实测证据；🟡 代码已写，但未通过验收；❌ 还没做。

**任务一：仿真环境与机器人控制**

| 课程要求 | 状态 | 现有实现 | 证据 |
|---------|------|---------|------|
| 虚拟环境（墙、障碍物） | ✅（2026-09-25 换成导入网格，画面待用户肉眼确认） | `exhibition_world.py` 导入 `iscas_museum` 网格（22.5×20.5m，层高 4m）；该网格只有墙，地面仍由代码生成；墙体烘焙成 320 个矩形供 `nav_grid` 使用，与网格可达区域逐格一致（泄漏 0） | `docs/exhibition-hall.md`；旧证据 `docs/evidence/f01/` 对应已弃用的 16×12m 房间 |
| 展品 ≥4 张、实物 ≥2 个 | ✅ | E01–E04 四张海报（`scripts/generate_exhibit_assets.py` 生成图片）贴在真实墙面（北墙 2、西墙 1、南墙 1），E05 书籍、E06 地球仪两个立体展品 | 同上 |
| 环境里有人（加分项） | ✅ 代码完成，画面待确认 | 3 个带碰撞的静态参观者（进导航栅格，雷达可见）+ 2 个骨骼动画行走 actor（相机可见，**雷达不可见**，Gazebo actor 无碰撞体） | `docs/exhibition-hall.md` |
| 低阶控制 | ✅ | `make_controlled_urdf.py` 生成完整 29 关节模型（固定骨盆支撑），通过 ros2_control 做力矩 PD；配套 `joint_targets.py` 限速与失联保护，`render_lowlevel_report.py` 画曲线 | `docs/evidence/f02/`、`docs/reviews/f02-review.md` |
| 机器人在环境内移动 | ✅ | `make_glide_urdf.py` 平移模型（VelocityControl + 关闭重力），`g1_teleop.py` 键盘遥控 | 重力修复后你已手动确认不倒 |
| 多传感器获取与展示 | ✅ | RGB-D 相机、雷达 `/scan`（360 线）、导航雷达 `/nav_scan`、IMU、odom；`sensor_dashboard.py` 面板，`g1_camera_viewer.py` 相机查看，`frame_sync.py` 时间戳配对 | `docs/evidence/f03/`、`docs/reviews/f03-review.md` |
| 数字键 1–6 去展位 | ✅（2026-09-25 用户手动验证：`exhibition_nav.launch.py` + `nav_keyboard` 可到站；旧自动验收脚本作废） | `nav_keyboard.py`（数字键）、`exhibition_nav.py`（ROS 节点）、`exhibition_nav.launch.py` | 自动验收 `tmp/f04-a`…`k` 共 11 轮全部失败，而且都是**重力修复之前**跑的 |
| 路径规划 | 🟡 | `nav_grid.py`：按配置生成占据栅格、膨胀后做 A* | 纯逻辑单测通过，仿真内未验收 |
| 避障 | 🟡 | `nav_grid.py` 用实时雷达点更新栅格并重规划；`nav_control.py` 前方 < 0.8m 停车，速度有上限 | 同上；之前失败原因是机身俯仰导致地面回波被当成障碍 |
| 机器人姿态 | 🟡 | `nav_control.py` 先转向再前进，到站后对准展品朝向 | 未验收 |
| 手指向展品 | ✅ 已在展馆平移模型中重做（M2.3，2026-09-25 用户目视验收通过；旧 `/g1_pointing/select` 实现作废） | `pointing_kinematics.py` 手臂 IK，`pointing_node.py` 左右指向 | `docs/evidence/f02b/`：误差约 5°。**只在低阶模型（骨盆固定）里做过**，展馆平移模型的手臂仍是焊死的 |
| SLAM 建图（亮点） | ✅ 简单展馆，两个雷达版本都已建图（见 M2.1） | `slam.launch.py lidar:=simple/mid360`（slam_toolbox online_async）+ `slam_explore.py` 自动巡航 + `slam_map.py` 偏差检查 + `mid360_to_scan.py` 点云压 2D | `maps/exhibition_simple_map*`、`maps/exhibition_simple_map_mid360*` |

**任务二：语音对话**：🟡 2026-09-25 已完成 `voice/`（以大模型工具调用为主、离线规则兜底），用户已亲测语音对话；连接仿真的端到端测试待做。详见下方 M3 开头的说明和 `voice/README.md`。

**任务三：视觉与综合**：只有底层数据。相机画面已能取到（`/camera/image`、深度、点云），但识别、人脸、手势、网络控制都是 ❌。

**测试**：迁移后 `g1_guide/test/` 共 15 个测试文件，142 项全部通过（指向和旧验收脚本的测试已随代码一起舍弃）。

**迁移说明**：下表的「现有实现」文件名指旧包；迁移后包名为 `g1_guide`，文件名不变。

**结论**：任务一只差「导航与避障在仿真中验收」和「展馆模型里的手臂动作」；任务二、三从零开始。下面的清单按这个结论排：M1 只做收尾，不重写已有内容。

### 关于「仿真能否接大模型 API」
**能。** 大模型调用只是对话程序发出的一个 HTTP 请求，和 Gazebo 没有关系。真正的限制只有两点：需要网络，需要你提供 API Key。所以大模型只作为**可选增强**；每个功能都要有离线兜底，没有 Key 也能完整演示。

---

## 1. 整体架构

```
┌──────────── Windows（原生 Python 3.13）────────────┐     ┌──────────── WSL（ROS2 + Gazebo）─────────────┐
│ 麦克风 → VAD → 流式ASR ─┐                            │     │                                              │
│                        ▼                             │     │  rosbridge_server (ws://localhost:9090)      │
│  NLU(意图+槽位) → 对话状态跟踪 → 策略 → TTS → 扬声器   │◀──▶│   │                                          │
│        │             │           ↑ 可被打断           │ WS  │   ├─ guide_manager  到站/讲解/手势编排       │
│        ▼             ▼                               │     │   ├─ exhibition_nav A*+避障 → /cmd_vel      │
│  知识库检索 + 阅读理解      [可选] 大模型 API          │     │   ├─ gesture_player 手臂关键帧 → 关节控制器   │
│  摄像头 → 人脸/手势（任务三）                          │     │   ├─ exhibit_detector YOLO ← /camera/image  │
│  PySide6 控制台（对话、状态、机器人画面）              │     │   └─ slam_toolbox ← /nav_scan + /odom       │
└──────────────────────────────────────────────────────┘     └──────────────────────────────────────────────┘
```

**Windows 和 WSL 之间只走一条通道：** rosbridge WebSocket，Windows 端用 `roslibpy` 连接。

**上下半身接口分离：**
- 下半身订阅 `/cmd_vel`（现在是平移，以后换成步行也不影响上层）
- 上半身订阅 `/guide/gesture`（手势名称）

**对外话题约定：**

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/guide/goto` | std_msgs/String | 对话→机器人 | 展品 ID，如 `"E03"` |
| `/guide/status` | std_msgs/String（JSON） | 机器人→对话 | `{state: idle/moving/arrived/failed, exhibit, pose}` |
| `/guide/gesture` | std_msgs/String | 对话→机器人 | `point` / `open_palm` / `wave` / `explain_loop` / `home` |
| `/guide/detections` | std_msgs/String（JSON） | 视觉→对话 | 识别到的展品和人员 |

---

## 2. 目录规划（`D:\g1_guide`）

```
g1_guide/
├── PLAN.md                  ← 本文件
├── ws/src/g1_guide/         ROS2 主包（已迁入全部已验证代码，见 README.md）
│   ├── g1_guide/            guide_manager、gesture_player、exhibit_detector …
│   ├── scripts/make_glide_urdf.py   平移模型生成器（M2.3 在这里放开手臂关节）
│   ├── worlds/ config/ launch/ test/
│   └── build.sh             colcon --symlink-install，安装到独立目录
├── voice/                   Windows 端语音对话系统（纯 Python）
│   ├── audio/ asr/ tts/ nlu/ dst/ qa/ llm/ app.py gui.py
│   └── tests/               用录音 WAV 注入做自动化测试
├── knowledge/               展品资料 E01–E06（正文 + FAQ + 别名）
├── vision/                  数据集生成、YOLO 训练、摄像头人脸/手势
├── tools/                   一键启动、录屏、证据采集
└── report/                  实验报告素材（图、表、日志）
```

---

## 3. 任务清单

每项都写明：做什么、怎么做、验收标准、执行方。

### 阶段 M0：基础设施（第 1 周前半）

| # | 任务 | 方案 | 验收 | 执行 |
|---|------|------|------|------|
| M0.1 | ~~提交现有未提交修改~~ ✅ 改为迁移 | 已验证代码迁入 `claude_course/ws`，包名改为 g1_guide | 142 项测试通过，导航和低阶控制的 headless 冒烟测试通过 | 🤖 已完成 |
| M0.2 | WSL 装依赖 | `apt install python3-venv python3-pip ros-lyrical-slam-toolbox ros-lyrical-nav2-map-server ros-lyrical-rosbridge-server`；建 venv 时加 `--system-site-packages`，以便复用 rclpy | `ros2 pkg list` 中能看到这几个包 | 🤖（通过 `wsl -u root`，若要密码则 👤） |
| M0.3 | Windows 端建 venv | `D:\python\python.exe -m venv voice\.venv`，安装 roslibpy、sounddevice、numpy、PySide6 | `import` 全部成功 | 🤖 |
| M0.4 | 打通 Windows↔WSL | WSL 启动 rosbridge，Windows 端用 roslibpy 发布 `/guide/goto`，WSL 端 `ros2 topic echo` 能收到 | 往返延迟 < 100ms | 🤖 |
| M0.5 | ✅ 新建 `g1_guide` 包和构建脚本 | `ws/build.sh`，使用 `--symlink-install`，独立安装到 g1-guide-runtime | `ros2 launch g1_guide …` 可以启动 | 🤖 |

### 阶段 M1：任务一 必做项收尾（第 1–2 周）

| # | 任务 | 方案 | 验收 | 执行 |
|---|------|------|------|------|
| ✅ M1.1 | **导航到站验收**（数字 1–6） | 重力修复后机身不再俯仰，原先"地面回波被当成障碍"的根因很可能已经消失。先用 `/nav_scan` 重新跑 `verify_navigation.py`；如果仍有假障碍，就按高度和距离过滤雷达点，只保留机身高度附近的回波 | 6 个展位均由真实 `/odom` 到站，位置误差 ≤ 0.25m，全程不碰撞 | 🤖 |
| ✅ M1.2 | 避障验收 | 用 Gazebo 服务在路径上临时生成一个箱子。雷达检测到后停车并用 A* 重规划；路完全封死时停车并报 failed | 绕行后到站；封路时速度为 0 | 🤖 |
| M1.3 | 环境丰富化 | 在 g1_guide 里新建一个 world 生成器（读取 exhibition.json 并扩展），增加：隔墙和门洞形成两个展厅、栏杆、展柜、服务台、垃圾桶、立牌、**一个来回移动的动态障碍**（Gazebo actor 或周期性运动的物体）、地面指引线、分区灯光 | 至少 15 类元素；俯视图截图；导航仍然通过 | 🤖 |
| ✅ M1.4 | 低阶控制展示（**已有，仅整理**） | 复用已验收的 F02a（固定骨盆，29 关节力矩 PD），整理出关节正负轨迹跟踪曲线图 | 曲线图、误差表格 | 🤖 |
| ✅ M1.5 | 传感器数据展示（**已有，仅整理**） | 复用 F03 面板，另外配一份 RViz 配置（点云、雷达、TF、路径、地图），用于演示 | 截图，每路数据频率表 | 🤖 |

### 阶段 M2：任务一 亮点（第 2–3 周）

| # | 任务 | 方案 | 验收 | 执行 |
|---|------|------|------|------|
| ✅ M2.1 | **SLAM 建图** | 运行 slam_toolbox 的 online_async 模式，输入 `/nav_scan` 和 `/odom`。用预设巡航路线自动走遍展馆，结束后用 map_saver 保存 `map.pgm/yaml` | 地图和真实布局叠加对比，墙体偏差 < 0.2m | 🤖 |
| ✅ M2.1 结果 | 2026-09-25 简单展馆（`exhibition_simple.json`）实测，两个雷达版本 | **简化雷达版** `lidar:=simple`（默认）：虚构的水平单线雷达，8m。墙体 p95 = 0.03m 合格，但长椅、书展台、花盆低于 1.32m 扫描平面，没有进地图，中部还有未知区。**真机雷达版** `lidar:=mid360`：MID-360 按真机规格做成倒装 3D 雷达（360°×−7°～52°，40m，30 线仿真），`mid360_to_scan` 取离地 0.08～2m 的点压成 `/nav_scan`，过滤 0.5m 内的机器人自身回波。6 站全部到达，墙体 p95 = 0.06m、99.7% 在 0.2m 内，**14 个物体全部进地图**，没有未知区 | 两版都走完 6 站 | 🤖 |
| ✅ M2.2 | 用 SLAM 地图导航（**可选功能，主流程不用**） | `slam_nav.launch.py`：map_server 读 SLAM 地图 + AMCL 定位 + `exhibition_nav` 地图模式（`map_yaml`、`global_frame:=map`、到站圈 0.15m）；真机雷达版可开里程计漂移（`odom_drift`） | 简单展馆实测：6/6 到站，里程计漂到 2.6m 时 AMCL 定位误差平均 0.05m；到站圈收紧后复跑的前 2 站真实误差 0.20/0.13m（之后 WSL 崩溃未跑完）。地图与展馆绑定（`*_meta.json`），错配会报错 | 🤖 |
| ✅ M2.2b | 导航提速 | 直线 0.35→0.6 m/s，转向 0.8→1.2 rad/s；巡航不再被 0.2m 路径点间距限速，只在接近终点时减速 | 新展馆布局模式 headless 实测：速度指令达到 0.6 m/s，4、6 号站真实到站误差 0.23/0.24m | 🤖 |
| ✅ M2.3 | **讲解时的手臂动作** | `make_glide_urdf.py` 中只把左右臂 14 个关节改为 revolute，并关闭手臂连杆的重力；每个关节配一个 gz `JointPositionController`（速度指令模式，免调参）。`gesture_player` 用关键帧加插值实现 7 个动作 `home/wave/point/present/explain_loop/emphasize/invite`，关键帧全在 `config/gestures.yaml`，每轮叠加随机扰动。指向用新写的 `arm_kinematics.py`（numpy FK + 阻尼最小二乘 IK） | 到站后能指向展品，末端指向误差 < 10°；平移过程中手臂不乱甩 | 🤖 |
| M2.3 结果 | 2026-09-25 大展馆实测：7 个动作都能播放，关节跟随指令；站在出生点指向最近展品 E04，按真实关节角算的指向误差 0.0°；行走中发 `wave` 被拒绝（手臂保持 home）。用户目视验收通过：7 个动作、行走中手臂下垂、到站自动指向 | `test/test_gestures.py` 17 项 | 🤖 / 👤 目视 |
| M2.4 | 路径与姿态可视化 | 在 RViz 中显示规划路径（`nav_msgs/Path`）、机器人朝向、代价地图 | 截图或录屏 | 🤖 |
| M2.5 | （可选）MuJoCo 真步行 | 使用 `unitree_rl_gym` 自带的 G1 预训练策略，配合 `deploy_mujoco.py`，在 Windows 上 pip 安装 mujoco 和 torch(CPU) 即可运行，用键盘控制速度实现双足行走。作为「步行可行性」的独立演示，**不并入 Gazebo 主线** | 能稳定走 10 秒以上，并能转向 | 🤖（需要从 GitHub 下载） |

### 阶段 M3：任务二 语音对话系统（第 3–5 周）

所有模块都放在 Windows 端 `voice/`，并且都有离线实现。

> **2026-09-25 路线调整（用户确认）：以大模型为主。** 大模型（默认 DeepSeek）直接理解意图，通过工具调用 `goto_exhibit / stop_robot / gesture / get_robot_status` 驱动机器人，不解析自由文本。规则 NLU 和知识库问答（M3.5、M3.7）降级为离线兜底：没有 Key、断网或超时 10 秒时自动接管，30 秒后再尝试大模型。M4.4 因此并入主线。
>
> | 项 | 状态 | 实际做法与结果 |
> |----|------|----------------|
> | M3.1 知识库 | 🟡 待审 | `knowledge/E01..E06.md`（Markdown），每件 7 条问答，共 42 条 |
> | M3.2 音频 | ✅ | 按名称选 Realtek 麦克风（系统默认是 ToDesk 虚拟麦克风，没有声音）；停止播放 33ms 生效 |
> | M3.3 ASR | ✅ | silero VAD + SenseVoice int8（离线），实时率约 0.1；用户实测"显示准确" |
> | M3.4 TTS | ✅ | 默认离线 Matcha，每句约 0.2 秒；edge-tts 每句约 2.2 秒，改为可选 |
> | M3.5/M3.7 离线 NLU 与问答 | ✅ 兜底 | `fallback.py`：规则意图（去/停/继续/下一个/感兴趣/确认等）+ 字符 n-gram TF-IDF 问答，容忍一个同音错字 |
> | M3.6 对话状态 | ✅ | 由大模型的对话历史承担（记得目的地、当前展品）；离线兜底有 current/goal/offer |
> | M3.9 对接机器人 | 🟡 | 复用 `/nav/goal_key`、`/nav/status`、`/guide/gesture`，WSL 端不用改代码；到站事件触发主动讲解。rosbridge 往返约 8ms；端到端待用户在仿真中验证 |
> | M3.10 GUI | ✅ | `voice/gui.cmd`：对话、语音状态灯、音量条、大模型/机器人状态、快捷按钮、日志 |
> | M4.2 打断 | ✅ 模拟通过 | `--barge-in`：播放时每 0.4 秒识别一次人声，用拼音比对排除回声。合成语音注入测试：戴耳机约 0.6 秒停，外放约 0.9 秒停，只有回声时不误停。真人验证待做 |
> | M4.4 大模型 | ✅ 主线 | 见上 |

| # | 任务 | 方案 | 验收 | 执行 |
|---|------|------|------|------|
| M3.1 | 展品知识库 | `knowledge/E01..E06.yaml`，每个展品包含：名称和别名、300–600 字正文、≥ 6 条 FAQ（年代/原理/用途/趣闻/适合人群/相关展品）、推荐下一站。**内容由 Claude 编写**，你审阅 | 6 个展品共 36 条以上 FAQ | 🤖 写，👤 审 |
| M3.2 | 音频 I/O | sounddevice 以 16kHz 单声道流式录音；播放放在独立线程，支持 `stop()` 立即中断 | 单测：播放过程中调用 stop，50ms 内静音 | 🤖 |
| M3.3 | VAD 与 ASR | **sherpa-onnx**（pip 即可安装，CPU 实时）：silero VAD 加流式中文 zipformer 模型。备选方案 Vosk 中文小模型 | 用 20 条录音 WAV 测试，字错率 < 15% | 🤖（下载模型要联网） |
| M3.4 | TTS | 离线：sherpa-onnx 中文 VITS/matcha；在线可选：edge-tts（音质更好）。句子级合成，边合成边播放 | 首字延迟 < 1s | 🤖 |
| M3.5 | **NLU：意图和槽位** | 规则、同义词、拼音模糊匹配三者结合（pypinyin 可以容错 ASR 同音错字）。意图：`greet / bye / self_intro / list_exhibits / express_interest / ask_question / goto_exhibit / next / repeat / stop / affirm / deny / chitchat / unknown`；槽位：`exhibit`、`aspect`（年代、原理……）、`ordinal`（第三个）、`ref`（"它"、"这个"，由对话状态消解） | 自建 150 条标注测试句，意图准确率 ≥ 90%，槽位 F1 ≥ 85% | 🤖 |
| M3.6 | **对话状态跟踪（DST）** | 状态包括：`visited[]`、`interested[]`、`current_exhibit`、`robot_state`、`pending_confirm`、`qa_count[exhibit]`、`turn_history`。策略：确认兴趣 → 规划参观顺序（感兴趣的展品优先，其余按距离排序）→ 到站讲解 → 答疑 → 推荐下一站 → 全部看完后总结并道别 | 5 个多轮剧本测试全部走通，状态转移正确 | 🤖 |
| M3.7 | **展品问答（阅读理解）** | 三级回答：① FAQ 语义匹配（字符 n-gram TF-IDF，纯 Python 实现）；② 抽取式阅读理解：用 `uer/roberta-base-chinese-extractive-qa`（transformers + CPU torch）从检索到的正文段落里抽答案；③ 置信度低时如实回答"这个我不太确定"。可选第 ④ 级：大模型基于检索内容生成答案 | 每个展品 ≥ 3 个问题能正确回答；另加一组 20 条超出 FAQ 的问题集，统计阅读理解正确率 | 🤖（模型下载要联网，可用 hf-mirror） |
| M3.8 | 闲聊 | 打招呼、道别、自我介绍、问时间、夸奖、"你是谁做的"，用模板随机回复；对 unknown 引导回展品话题 | 剧本测试 | 🤖 |
| M3.9 | 对接机器人 | 对话通过 roslibpy 发 `/guide/goto` 和 `/guide/gesture`，订阅 `/guide/status`；在 WSL 端 `guide_manager` 编排"到站 → 指向 → 讲解动作循环 → 结束归位" | 语音说"带我去看地球仪"，仿真里机器人走到 E06 并指向，同时开始讲解 | 🤖 |
| M3.10 | 控制台 GUI | PySide6 界面：实时 ASR 文本、意图和槽位、对话状态表、机器人状态、相机画面（订阅压缩图像）、"说话中/倾听中"指示灯 | 截图，录视频时可用 | 🤖 |

### 阶段 M4：任务二 亮点（第 5–6 周）

| # | 任务 | 方案 | 验收 | 执行 |
|---|------|------|------|------|
| M4.1 | **连续对话，只唤醒一次** | sherpa-onnx KWS 检测唤醒词"你好小G"，唤醒后进入会话；会话内不再需要唤醒词。静默 30s 或说"再见"时结束会话 | 录音测试：一次唤醒后连续 5 轮对话 | 🤖 |
| M4.2 | **语音打断（不靠唤醒词）** | 播放期间保持 VAD 和 ASR 运行。满足以下条件即判定为打断：持续人声 ≥ 300ms，且识别出的文字与正在播放的 TTS 文本相似度低（排除回声）。打断后立即停止播放，丢弃旧回复，用新的一句重新走 NLU。**演示时建议戴耳机**，可从物理上消除回声；外放时依赖相似度过滤，可选接入 WebRTC AEC | 录音注入测试：打断后 < 0.5s 停止播放，并切换到新话题 | 🤖 测逻辑，👤 真人验证 |
| M4.3 | 受众模式 | 儿童模式（语速慢、短句、打比方）和专业模式（加细节）；由用户说"讲简单点"来切换 | 同一问题两种模式回答不同 | 🤖 |
| M4.4 | （可选）大模型增强 | 抽象出 `LLMClient` 接口，兼容 OpenAI 格式（DeepSeek/通义等国内服务无需代理）。用于 NLU 兜底和开放式问答；只基于知识库检索内容回答，超时 3s 自动退回离线结果 | 有 Key 时开放问题回答更自然；无 Key 时功能不受影响 | 🤖 写代码，👤 提供 Key |

### 阶段 M5：任务三 视觉与综合（第 7–10 周）

| # | 任务 | 方案 | 验收 | 执行 |
|---|------|------|------|------|
| M5.1 | **自动生成展品数据集** | 在 Gazebo 中让相机从随机位置和角度拍摄海报与立体展品；已知展品 3D 包围盒和相机内参，直接投影出 YOLO 标注框，**无需人工标注**。同时加入光照、遮挡、距离等随机变化 | ≥ 2000 张带标注图片，随机抽查 50 张标注正确 | 🤖 |
| M5.2 | 训练 YOLO | 在 Windows 端用 ultralytics yolov8n/yolo11n 微调，RTX 4060 上约 30 分钟；导出 ONNX | 独立测试集 mAP50 ≥ 0.9 | 🤖 |
| M5.3 | 仿真内展品识别节点 | `exhibit_detector` 订阅 `/camera/image`，用 onnxruntime 推理，结合深度图计算展品距离，发布 `/guide/detections` | 到站后识别结果与目标展品一致，测距误差 < 0.3m | 🤖 |
| M5.4 | 视觉确认到站 | guide_manager 到站后用视觉核对展品，不一致就微调位置或报告；用户问"这是什么"时，按相机当前看到的展品回答 | 剧本演示 | 🤖 |
| M5.5 | 文字识别（OCR） | 每块展牌上放标题文字，用 RapidOCR（onnx 版）读取，作为识别的第二证据 | 6 块展牌识别率 ≥ 90% | 🤖 |
| M5.6 | 仿真里的参观者 | 放置人物模型（Gazebo actor）；用 YOLO 预训练模型检测人，机器人转身面向对方并主动打招呼 | 人进入视野后 3s 内打招呼 | 🤖（人物模型要联网下载） |
| M5.7 | **摄像头非语言交互** | Windows 端用笔记本摄像头。人脸位置由 OpenCV YuNet 检测（纯 cv2 实现）→ 机器人朝向跟随；手势由 MediaPipe Hands 识别（若 3.13 不兼容，改用 YOLO-pose 手部关键点）：挥手 = 打招呼/再见，竖拇指 = 肯定，手掌 = 停止讲解，比数字 1–6 = 去对应展位；视线/人脸朝向偏离过久 → 询问"要不要换一个展品" | 5 种手势识别率 ≥ 85% | 🤖 写代码，👤 对着摄像头测试 |
| M5.8 | 网络远程控制 | 用 roslibjs 做一个网页控制台（手机也能打开）：查看机器人位置和画面、点选展品、文字对话 | 手机浏览器能完成一次导览 | 🤖 |
| M5.9 | 多模态融合策略 | 统一事件总线，给信号定优先级：打断 > 手势停止 > 语音指令 > 视觉事件 > 空闲主动搭话。冲突处理写进报告 | 融合剧本演示 | 🤖 |

### 阶段 M6：交付（第 11–13 周）

| # | 任务 | 执行 |
|---|------|------|
| M6.1 | 一键启动脚本：仿真、rosbridge、Windows 语音端、GUI | 🤖 |
| M6.2 | 全流程回归测试脚本，保证代码可复现；另写 README 安装说明 | 🤖 |
| M6.3 | 三份阶段报告草稿：功能描述与概要设计、详细设计、使用方法、结果分析，含图表和数据 | 🤖 起草 |
| M6.4 | 报告里的「经验总结」、个人心得，以及最终审定 | 👤 |
| M6.5 | 无声演示片段（Gazebo 导航、建图、识别），用 ffmpeg 录屏 | 🤖 |
| M6.6 | 带真人语音的结果视频：对话、打断、手势，加旁白讲解 | 👤（Claude 提供分镜脚本） |

---

## 4. 时间表

学期末考试前后截止。**三次阶段实验的具体日期请你补充**，我会据此调整。

| 周次 | 日期 | 内容 | 里程碑 |
|------|------|------|--------|
| W1 | 09/28–10/04 | M0 全部，M1.1–M1.2 | 数字键导航和避障验收通过 |
| W2 | 10/05–10/11 | M1.3–M1.5，M2.1 | **阶段一可交**（环境、低阶控制、传感器、导航） |
| W3 | 10/12–10/18 | M2.2–M2.4，M3.1–M3.2 | 建图导航、讲解手势 |
| W4 | 10/19–10/25 | M3.3–M3.6 | 语音链路打通，NLU 和 DST 通过测试集 |
| W5 | 10/26–11/01 | M3.7–M3.10 | 语音导览全流程 |
| W6 | 11/02–11/08 | M4.1–M4.4 | **阶段二可交**（连续对话、打断） |
| W7 | 11/09–11/15 | M5.1–M5.3 | 展品识别模型 |
| W8 | 11/16–11/22 | M5.4–M5.6 | 视觉融合导览 |
| W9 | 11/23–11/29 | M5.7–M5.8 | 摄像头手势、网页控制 |
| W10 | 11/30–12/06 | M5.9，M2.5（可选） | 综合演示剧本 |
| W11–W13 | 12/07–12/27 | M6 | **阶段三**：报告、视频、代码 |
| 缓冲 | 12/28 至期末 | 返工、答辩准备 | |

---

## 5. 需要你本人完成的事项（Claude 无法代劳）

| 事项 | 原因 | 时间点 |
|------|------|--------|
| 对着麦克风做真人对话和打断测试 | Claude 只能用录音文件注入测试，无法说话 | W4 起每周一次 |
| 对着摄像头做手势测试 | 需要真人 | W9 |
| 提供大模型 API Key（可选） | 账号和费用归你 | W6 前 |
| 录制带语音的结果视频、配旁白 | 需要真人出镜和出声 | W11–W13 |
| 报告的经验总结与最终审定 | 个人内容 | W12–W13 |
| 确认三次阶段实验的截止日期 | 课程信息 | 尽快 |
| 如果模型下载被墙，开代理或手动下载 | 网络环境 | 按需 |
| sudo 密码（如果 `wsl -u root` 不可用） | 系统权限 | M0.2 |
| 真机接入 | 本计划明确不做 | — |

---

## 6. 风险与备选

| 风险 | 备选方案 |
|------|---------|
| 外放时回声导致误打断 | 演示戴耳机；加文本相似度过滤；可选 WebRTC AEC |
| sherpa-onnx 模型下载失败 | 改用 Vosk 中文小模型（约 42MB） |
| 阅读理解模型下载或推理太慢 | 退回 FAQ 匹配加检索句子直接作答 |
| WSL 内存不足（Gazebo、SLAM、YOLO 同时运行） | YOLO 推理放 Windows 端；Gazebo 使用 headless 模式加 RViz |
| MediaPipe 不支持 Python 3.13 | 改用 YOLO-pose，或单独建一个 3.12 的 venv |
| 雷达地面回波问题没有被重力修复解决 | 按高度过滤，或降低 `/nav_scan` 安装高度 |

---

## 7. 进度记录

| 日期 | 完成 | 提交 |
|------|------|------|
| 2026-09-25 | 创建分支 `claude/guide-robot` 和本计划 | — |
| 2026-09-25 | 用户可视化验证：导航、避障、低阶控制、传感器均通过；指向不可用，舍弃 | — |
| 2026-09-25 | 已验证代码迁移到 `claude_course/ws`，包名改为 g1_guide，旧目录弃用 | 见 git log |
| 2026-09-25 | 项目整体搬到 `D:\g1_guide`，去掉写死路径，新建独立仓库 | 见 git log |
| 2026-09-25 | M2.1 SLAM 建图（简单展馆）：slam_toolbox + 自动巡航 + 地图偏差检查，墙体合格，矮障碍待解决 | 见 git log |
| 2026-09-25 | M2.1 新增真机雷达版（MID-360 3D，按真机规格），矮障碍全部建进地图；简化雷达版保留 | 见 git log |
| 2026-09-25 | M2.2 SLAM 地图导航（AMCL + 里程计漂移）作为可选功能完成；主流程仍用布局导航。导航提速到 0.6 m/s。新展馆建图因 Gazebo 渲染内存泄漏失败，按用户决定不再在新展馆建图 | 见 git log |
