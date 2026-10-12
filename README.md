# G1 展馆导览机器人

宇树 G1 人形机器人在虚拟美术馆里给参观者做导览：用语音和人对话，按指令或对话内容走到对应展位，到站后用手臂指向展品并讲解，同时用雷达感知周围有多少观众、有没有儿童，用前置相机判断有没有人正对着它、看懂简单手势。

原则是**仿真先行，接口按真机设计**：所有和传感器、执行器打交道的代码只依赖 ROS 话题，不依赖 Gazebo 专有功能，以后可以直接换成真机驱动。真机是可选的加分项。

- **仿真端（WSL）**：ROS 2 lyrical + Gazebo（`gz sim`），ROS 包名 `g1_guide`
- **交互端（Windows）**：语音对话、视觉、控制台 GUI，通过 rosbridge（WebSocket，端口 9090）和仿真端通信
- **步态实验（独立）**：Isaac Lab 在租用服务器上训练崎岖地形行走策略，本地 MuJoCo 验证

项目位置 `D:\g1_guide`（WSL：`/mnt/d/g1_guide`）。代码里不写死路径，整个文件夹可以搬动，搬动后删掉 `~/.local/share/g1-guide-runtime` 再运行一次 `ws/build.sh`。

详细的任务计划和每项的验收记录见 [PLAN.md](PLAN.md)。

---

## 目录

1. [当前进展](#1-当前进展)
2. [系统架构](#2-系统架构)
3. [目录结构](#3-目录结构)
4. [安装](#4-安装)
5. [启动方式](#5-启动方式)
6. [功能详解与修改方法](#6-功能详解与修改方法)
   - 6.1 展馆与展品 · 6.2 机器人模型 · 6.3 导航 · 6.4 讲解手势 · 6.5 雷达人群感知
   - 6.6 语音对话 · 6.7 视觉 · 6.8 SLAM · 6.9 低阶控制 · 6.10 步态训练（MuJoCo / Isaac）
7. [测试](#7-测试)
8. [工具脚本](#8-工具脚本)
9. [已知问题与限制](#9-已知问题与限制)
10. [常见错误](#10-常见错误)

---

## 1. 当前进展

状态：✅ 已完成并实测；🟡 代码完成，部分验收未做；❌ 未开始。

| 模块 | 状态 | 说明 |
|---|---|---|
| 展馆环境 | ✅ | 导入的真实博物馆网格（22.5×20.5 m），4 幅名画 + 2 件立体展品，3 个静态参观者 + 2 个走动的人 |
| 机器人控制 | ✅ | 平移模型键盘遥控；29 关节低阶力矩控制（骨盆固定） |
| 多传感器 | ✅ | 头部 RGB-D、前置相机、2D 雷达、MID-360 3D 雷达、IMU、里程计，配传感器面板 |
| 导航（数字键 1–6） | ✅ | A* + 雷达避障，0.6 m/s；修复了 3 种卡死情况。离线仿真和 Gazebo 都验证过 |
| 讲解手势 | ✅ | 7 个动作，到站后用 IK 自动指向展品。用户目视验收通过 |
| SLAM（实验功能） | ✅ | 简单展馆两种雷达都建好图，可用 SLAM 地图 + AMCL 导航 |
| 语音对话 | ✅ / 🟡 | 离线识别和合成、大模型 + 离线兜底、唤醒词、打断、讲解风格、GUI 都已完成，用户亲测过；接仿真的端到端测试待做 |
| 雷达人群感知 | ✅ | 数 3 m 内人数、判断是否单个儿童，决定是否开讲、用哪种讲解模式、观众走光后告别 |
| 视觉 | 🟡 | 视觉唤醒仿真里已打通；手势指令代码完成，等对着摄像头验收；展品识别按决定取消 |
| 网页远程控制 | ❌ | 计划用 roslibjs 做手机可打开的控制台 |
| 交付 | ❌ | 一键启动、回归脚本、报告、演示视频 |
| 崎岖地形步态 | 🟡 | 第三版策略在 Isaac 13 项测试全部 ≥ 98.4%、零摔倒；MuJoCo 里 6 条跑道过 3 条，差距未查清 |

---

## 2. 系统架构

```
┌──────────── Windows（Python 3.13）────────────┐        ┌──────────────── WSL（ROS 2 + Gazebo）────────────────┐
│ 麦克风 → 回声消除 → VAD → SenseVoice 识别      │        │  rosbridge_server  ws://localhost:9090                │
│        ↓                                      │        │    │                                                  │
│ 唤醒判定 → 大模型（工具调用）/ 离线规则兜底    │◀─WS──▶│    ├─ exhibition_nav   A* + 避障 → /cmd_vel           │
│        ↓                    ↓                 │        │    ├─ gesture_player   手臂关键帧 / IK → 关节控制器   │
│   Matcha 合成 → 扬声器    导航 / 手势指令     │        │    ├─ people_tracker   MID-360 点云 → /guide/people   │
│                                               │        │    ├─ mid360_to_scan   3D 点云 → /nav_scan            │
│ 前置相机 → YOLO 人 + YuNet 人脸 + MediaPipe 手 │        │    ├─ camera_jpeg      前置相机 → JPEG                │
│ PySide6 控制台                                │        │    └─ Gazebo（展馆、G1、传感器）                       │
└───────────────────────────────────────────────┘        └───────────────────────────────────────────────────────┘
```

### 主要话题

| 话题 | 类型 | 方向 | 内容 |
|---|---|---|---|
| `/nav/goal_key` | String | → 导航 | 展位键 `1`–`6`；`0` / `cancel` 取消；`face` 转向人群 |
| `/nav/status` | String（JSON） | 导航 → | `state`、`goal_key`、`replans` 等；state 见 [6.3](#63-导航) |
| `/guide/gesture` | String | → 手势 | 动作名如 `wave`，或 JSON `{"name":"point","exhibit":"E03"}` |
| `/guide/gesture_status` | String（JSON） | 手势 → | 动作执行 / 拒绝的原因 |
| `/guide/present_mode` | String | → 导航、手势 | 到站后 `point` 指向展品 / `face` 面向观众 |
| `/guide/people` | String（JSON） | 人群 → | 每人的位置、方位、距离、身高、是否儿童、组内人数 |
| `/guide/vision` | String（JSON） | 视觉 → | 人框、人脸（方位、距离、是否正对）、当前手势、手势事件 |
| `/front_camera/image/compressed` | CompressedImage | → 视觉 | 前置相机 5 Hz JPEG（`vision:=true` 时） |
| `/cmd_vel` | Twist | 导航 → Gazebo | 平移模型速度 |
| `/scan`、`/nav_scan` | LaserScan | Gazebo → | 360 线雷达；导航用雷达 |
| `/odom`、`/odom_truth` | Odometry | | 里程计；真机雷达版下 `/odom` 带漂移，真值在 `/odom_truth` |
| `/g1/arm/<关节>/cmd_pos` | Float64 | 手势 → Gazebo | 14 个手臂关节目标角 |

---

## 3. 目录结构

```
g1_guide/
├── README.md / PLAN.md           说明 / 任务计划与验收记录
├── docs/
│   ├── exhibition-hall.md        展馆网格导入、烘焙导航地图、布局坐标、排错
│   └── G1 第三版.md              崎岖地形步态训练的方法与结果
├── knowledge/E01–E06.md          展品讲解资料（语音端读取）
├── voice/                        Windows 语音端（见 6.6）
├── vision/                       Windows 视觉端（见 6.7）
├── maps/                         SLAM 建好的地图（pgm/yaml）与报告
├── mujoco/                       MuJoCo 步态实验（见 6.10）
│   ├── g1_walk.py                宇树预训练平地策略
│   ├── g1_rough_walk.py          自训练崎岖地形策略（sim2sim）
│   ├── terrain.py                6 条斜坡 / 楼梯跑道
│   ├── replay_demo.py            把 Isaac 里录的位姿渲染成 mp4
│   └── policies/                 3 版训练好的策略
├── tools/                        测试、排查、展馆烘焙脚本（见第 8 节）
│   ├── hall/                     展馆网格探测、烘焙、布局核对
│   └── isaac_server/             云服务器 Isaac Lab 安装脚本与训练记录
└── ws/                           ROS 2 工作区
    ├── build.sh                  构建（生成 URDF + colcon --symlink-install）
    └── src/
        ├── g1_description/       宇树官方 G1 29 自由度模型与网格
        ├── g1_bridge/            Gazebo ↔ ROS 桥接（C++）
        └── g1_guide/             主包
            ├── g1_guide/         节点与算法（下表）
            ├── scripts/          make_glide_urdf.py（平移模型）、make_controlled_urdf.py（29 关节）、frame_paintings.py（画框合成）
            ├── launch/           g1_sim / exhibition / exhibition_nav / slam / slam_nav / lowlevel
            ├── config/           exhibition.json、exhibition_simple.json、gestures.yaml、hall_nav_obstacles.json、
            │                     bridge*.yaml、slam_toolbox*.yaml、amcl.yaml、slam.rviz
            └── test/             ROS 端单元测试
```

主包 `g1_guide/g1_guide/` 的模块：

| 文件 | 作用 |
|---|---|
| `exhibition_world.py`、`scene_config.py` | 按 `config/*.json` 生成展馆 SDF；场景配置加载与占地几何（世界和导航共用） |
| `exhibition_nav.py` | 导航 ROS 节点，只做收发 |
| `nav_core.py` | 导航决策（与 ROS 无关，离线仿真共用）：目标、规划、脱困、障碍记忆、面向观众 |
| `nav_grid.py` | 占据栅格、膨胀、A* |
| `nav_control.py` | 速度指令、前方停车、后退脱困（纯函数） |
| `nav_keyboard.py` | 数字键选展位 |
| `gesture_player.py`、`gestures.py`、`arm_kinematics.py` | 讲解手势节点、关键帧引擎、手臂 FK/IK |
| `people_core.py`、`people_tracker.py` | 雷达人群感知（纯逻辑 / 节点） |
| `camera_jpeg.py` | 前置相机转 JPEG，给 Windows 视觉端 |
| `mid360_to_scan.py`、`odom_drift.py`、`lidar_variants.py` | 真机雷达版：点云压 2D、里程计漂移、启动片段 |
| `slam_explore.py`、`slam_map.py` | SLAM 自动巡航建图；地图读写与墙体偏差检查 |
| `g1_teleop.py` | WASD 遥控 |
| `lowlevel_*.py`、`joint_targets.py` | 29 关节低阶力矩 PD 控制 |
| `sensor_dashboard.py`、`sensor_metrics.py`、`g1_camera_viewer.py`、`frame_sync.py` | 传感器面板、相机查看、时间戳配对 |

---

## 4. 安装

### 4.1 WSL（仿真端）

环境：Ubuntu 26.04 + ROS 2 lyrical + Gazebo（命令 `gz sim`）。额外的 apt 包：

```bash
sudo apt install ros-lyrical-slam-toolbox ros-lyrical-nav2-map-server ros-lyrical-nav2-amcl ros-lyrical-nav2-lifecycle-manager ros-lyrical-rosbridge-server
```

`~/.bashrc` 已配置（新终端自动生效）：

```bash
source /opt/ros/lyrical/setup.bash
source /home/ma/.local/share/g1-guide-runtime/install/setup.bash
export LD_LIBRARY_PATH=/usr/lib/wsl/lib:$LD_LIBRARY_PATH
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export GALLIUM_DRIVER=d3d12
```

展馆网格 `OpenRobotics/iscas_museum` 和人物模型第一次启动时从 Gazebo Fuel 下载，之后缓存在 `~/.gz/fuel`，可离线使用。

### 4.2 构建

```bash
bash /mnt/d/g1_guide/ws/build.sh
```

构建用 `--symlink-install`，**只改已有的 Python、launch、config 文件时立即生效**，不用重新构建。以下情况必须重新运行 `build.sh`：

- 新增文件
- 改了 `setup.py`
- 改了 `make_glide_urdf.py`（它生成 URDF）；改了 `gestures.yaml` 里的 `home` 姿势（它也是 Gazebo 里手臂的初始角）
- 项目文件夹搬了位置

`ros2 launch` 只读安装目录（`~/.local/share/g1-guide-runtime/install/`），不读源码目录。不要单独运行 `make_glide_urdf.py` 来"更新模型"。

### 4.3 Windows 语音端

```bash
D:\python\python.exe -m venv voice\.venv
```
```bash
voice\.venv\Scripts\python.exe -m pip install -r voice\requirements.txt
```
```bash
voice\.venv\Scripts\python.exe voice\download_models.py
```

模型约 400 MB，放在 `voice/models/`（不入库）：silero VAD、SenseVoice int8、Matcha 中文合成 + vocos 声码器。外放时的回声消除要额外装 `livekit`（`pip install livekit`），没装也能运行，只是外放打断容易误触发。

大模型是可选的，设置任意一个 Key 即可（没有 Key 自动用离线规则）：

```bash
setx DEEPSEEK_API_KEY "你的key"
```

### 4.4 Windows 视觉端

```bash
D:\python\python.exe -m venv vision\.venv
```
```bash
vision\.venv\Scripts\python.exe -m pip install ultralytics opencv-contrib-python mediapipe roslibpy
```

`vision/models/` 下需要 3 个模型文件（不入库）：

| 文件 | 来源 |
|---|---|
| `yolo11n.pt` | ultralytics 官方 COCO 预训练模型 |
| `face_detection_yunet_2023mar.onnx` | OpenCV Zoo 的 YuNet 人脸检测 |
| `hand_landmarker.task` | MediaPipe Hand Landmarker |

### 4.5 MuJoCo 步态实验

```bash
D:\python\python.exe -m pip install mujoco torch pyyaml numpy
```
```bash
git clone --depth 1 https://github.com/unitreerobotics/unitree_rl_gym mujoco\unitree_rl_gym
```

自训练策略要用的 MuJoCo Menagerie 2024-07 版 G1（commit `cddb24f0db`）放在 `mujoco/menagerie_g1_2024/`，用 `gh api` 下载（本机 git 直连 GitHub 会 SSL 握手失败）。两者都不入库。

---

## 5. 启动方式

标 **WSL** 的在 WSL 终端运行；标 **Windows** 的在 `D:\g1_guide` 下用 cmd 运行。每条命令开一个新终端。

### 5.1 主流程：展馆 + 导航（WSL）

```bash
ros2 launch g1_guide exhibition_nav.launch.py
```

| 参数 | 默认 | 作用 |
|---|---|---|
| `headless:=true` | false | 不开窗口，省内存，适合长时间运行和自动测试 |
| `lidar:=mid360` | simple | 用真机规格 3D 雷达，同时启动人群感知 `people_tracker` |
| `vision:=true` | false | 启动 `camera_jpeg`，把前置相机画面发给 Windows 视觉端 |
| `gui_render:=software` | gpu | 窗口改软件渲染：不泄漏内存但很卡 |
| `camera_rate:=5` | 10 | 头部相机帧率 |
| `scene_config:=<路径>` | `config/exhibition.json` | 换场景；简单展馆是 `exhibition_simple.json` |

完整感知版（雷达数人 + 视觉）：
```bash
ros2 launch g1_guide exhibition_nav.launch.py lidar:=mid360 vision:=true
```

数字键选展位（1–6 去对应展位，0 或空格取消，q 退出）：
```bash
ros2 run g1_guide nav_keyboard
```

做一个讲解手势（`wave` / `point` / `present` / `explain_loop` / `emphasize` / `invite` / `home`）：
```bash
ros2 topic pub --once /guide/gesture std_msgs/String "data: wave"
```

查看导航状态：
```bash
ros2 topic echo /nav/status
```

### 5.2 只看展馆、遥控、传感器（WSL）

| 功能 | 命令 |
|---|---|
| 只开展馆和机器人（不带导航） | `ros2 launch g1_guide exhibition.launch.py` |
| WASD 遥控（W/S 前后，A/D 转向，空格停，+/- 调速度，q 退出） | `ros2 run g1_guide g1_teleop` |
| 传感器面板 | `ros2 run g1_guide sensor_dashboard` |
| 相机查看 | `ros2 run g1_guide g1_camera_viewer` |
| 放 / 删一个障碍箱（测试避障） | `bash /mnt/d/g1_guide/tools/spawn_box.sh box1 0 -2.5` / `bash /mnt/d/g1_guide/tools/remove_box.sh box1` |
| 只开 Gazebo | `gz sim` |

### 5.3 语音导览（WSL + Windows）

先在 WSL 开着 5.1 的展馆导航，再开通信桥：
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Windows 端二选一。图形控制台：
```bash
voice\gui.cmd
```
终端版（对麦克风说话，按回车打断机器人）：
```bash
voice\run.cmd
```

| 参数 | 作用 |
|---|---|
| `--no-robot` | 不连仿真，只测对话 |
| `--offline` | 不用大模型，只用离线规则 |
| `--text` | 键盘打字代替麦克风（调提示词用） |
| `--barge-in` | 机器人说话时允许插话打断（建议戴耳机） |
| `--wake` | 需要唤醒词"你好小吉" |
| `--gain 2` | 麦克风增益（默认 2） |
| `--mute` | 不播放语音 |
| `--audience child\|normal\|expert` | 初始讲解风格（终端版） |
| `--humor 0-10` | 初始幽默值（终端版，默认 3） |

麦克风检查（录 4 秒、显示音量、回放、给出建议增益）：
```bash
voice\.venv\Scripts\python.exe voice\check_mic.py
```

### 5.4 视觉（Windows）

接仿真前置相机（WSL 端要带 `vision:=true`，并开着 rosbridge）：
```bash
vision\.venv\Scripts\python.exe vision\watch.py --show
```

用笔记本摄像头测手势和人脸（不需要仿真）：
```bash
vision\.venv\Scripts\python.exe vision\watch.py --camera 0 --show --no-ros
```

参数：`--host` / `--port` rosbridge 地址；`--topic` 图像话题；`--hfov 70` 相机水平视野（度）。退出：画面窗口按 q / Esc，或终端 Ctrl+C。

### 5.5 一套完整演示（推荐顺序）

1. WSL：`ros2 launch g1_guide exhibition_nav.launch.py lidar:=mid360 vision:=true`
2. WSL：`ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
3. Windows：`vision\.venv\Scripts\python.exe vision\watch.py --show`（不需要视觉可跳过）
4. Windows：`voice\gui.cmd`，状态栏显示"机器人 已连接"后说"带我去看青花瓷"

### 5.6 SLAM（实验功能，WSL）

| 功能 | 命令 |
|---|---|
| 手动开车建图（简单展馆） | `ros2 launch g1_guide slam.launch.py rviz:=true`，再开 `nav_keyboard` 或 `g1_teleop` |
| 自动巡航建图并保存（简化雷达） | `ros2 launch g1_guide slam.launch.py explore:=true rviz:=true` |
| 自动巡航建图并保存（真机雷达） | `ros2 launch g1_guide slam.launch.py lidar:=mid360 explore:=true rviz:=true` |
| SLAM 地图导航（MID-360 + 漂移 + AMCL） | `ros2 launch g1_guide slam_nav.launch.py rviz:=true`，再开 `nav_keyboard` |
| SLAM 地图导航自动验收（走完 6 站出报告） | `ros2 launch g1_guide slam_nav.launch.py tour:=true` → `maps/nav_report.json` |
| 手动保存当前地图 | `ros2 run nav2_map_server map_saver_cli -f /mnt/d/g1_guide/maps/my_map` |
| 检查地图墙体偏差 | `ros2 run g1_guide slam_map_check /mnt/d/g1_guide/maps/exhibition_simple_map.yaml --scene <场景json>` |

`slam.launch.py` 其他参数：`odom_drift`、`order`（巡航顺序）、`output_dir`、`map_name`。`slam_nav.launch.py` 其他参数：`map`（地图 yaml）、`lidar`、`odom_drift`、`order`、`report_name`。

### 5.7 低阶控制（WSL）

```bash
ros2 launch g1_guide lowlevel.launch.py
```
发一个关节目标：
```bash
ros2 topic pub --once /g1_lowlevel/target sensor_msgs/msg/JointState '{name: [left_elbow_joint], position: [0.8]}'
```

### 5.8 MuJoCo 步态（Windows）

| 功能 | 命令 |
|---|---|
| 宇树预训练平地步态（也可双击 `mujoco\run_g1_walk.bat`） | `D:\python\python.exe mujoco\g1_walk.py` |
| 平地步态上跑道 | `mujoco\run_g1_walk.bat --course stairs15` |
| 自训练崎岖地形策略（第三版，目前最好） | `D:\python\python.exe mujoco\g1_rough_walk.py --policy g1_rough_rand070_n16384_mb8` |
| 站到某条跑道前自动走 | 上一条加 `--course slope10` |
| 无窗口跑完 6 条跑道 | 上一条加 `--check-all` |
| 核对跑道尺寸 | `D:\python\python.exe mujoco\terrain.py` |
| 回放 Isaac 里录的位姿 | `D:\python\python.exe mujoco\replay_demo.py <demo目录>` |

窗口按键：↑↓ 前进速度 ±0.1 m/s，←→ 转向，PgUp/PgDn 横移（平地版），Home 速度清零，End 复位，空格暂停。

### 5.9 多个仿真同时运行

和其他会话同时跑时用环境变量隔离，否则话题串线：
```bash
export ROS_DOMAIN_ID=42 GZ_PARTITION=mine
```

在 WSL 里跑仿真要让 `wsl` 命令常驻，不要 `nohup … &` 后退出，进程会被回收。

---

## 6. 功能详解与修改方法

### 6.1 展馆与展品

**主题**：古今名画美术馆（2026-09-26 由科普馆改成；旧版保存在 git 标签 `hall-science-v1`）。

| 编号 | 展品 | 类型 | 展位键 | 站位 (x, y) |
|---|---|---|---|---|
| E01 | 清明上河图 | 画（北墙） | 1 | (−3.7, 9.1) |
| E02 | 千里江山图 | 画（北墙） | 2 | (3.9, 9.1) |
| E03 | 星月夜 | 画（西墙） | 3 | (−7.1, 9.1) |
| E04 | 神奈川冲浪里 | 画（南墙） | 4 | (0.7, −8.6) |
| E05 | 芥子园画传 | 立体：摊开的古籍放在木书托上 | 5 | (3.4, 5.0) |
| E06 | 青花瓷瓶 | 立体：玉壶春瓶仿制品 | 6 | (−3.2, −7.3) |

机器人出生在 (0, −9)，面朝北。画作都是公有领域作品，来源见 `assets/exhibition/paintings_src/SOURCES.md`，`scripts/frame_paintings.py` 合成画框和说明牌。

**人物**：3 个带碰撞的静态参观者（会进导航栅格，雷达能看到）；2 个骨骼动画行走的人（北厅沿 y = 9.2 走，正好穿过 1、2 号展位；南廊一个）。行走的人没有碰撞体，但雷达按渲染测距，**导航雷达能扫到它**，所以会让机器人临时停车或后退让路。

**配置文件** `ws/src/g1_guide/config/exhibition.json`（schema v2）：

| 字段 | 内容 |
|---|---|
| `hall` | 展馆网格（Fuel `iscas_museum`）、位姿；`collision: baked` 用烘焙好的 320 个墙体盒子做碰撞（快），`mesh` 用原网格（慢） |
| `nav_obstacles_file` | 烘焙好的墙体矩形 `hall_nav_obstacles.json`，导航规划用 |
| `room` | 地面范围、墙高（网格只有墙，没有地面和屋顶，地面由代码生成） |
| `robot_clearance_radius` | 规划净空半径 0.55 m |
| `spawn` | 机器人出生位姿 |
| `exhibits[]` | 每件展品：`exhibit_id`、`title`、`kind`（poster / book / vase）、`asset`、`position`、`size`、`yaw`、`station`（`key`、站位 `x`、`y`、到站朝向 `yaw`） |
| `people[]` | `kind: static`（`model_uri`、`position`、`yaw`、`size`）或 `kind: actor`（`skin_uri`、`speed`、`waypoints`） |
| `obstacles[]` | 额外的静态障碍 |

**怎么改**：

- 改展品位置或站位：改 `exhibits[].position` / `station`，**不要靠看图估坐标**，用 `tools/hall/check_layout.py` 对网格实测，再用 `tools/hall/check_tour.py` 确认 6 个展位仍然两两可达，最后跑一遍 `python3 tools/nav_sim.py --scene hall`。
- 换展馆网格：必须重新烘焙墙体并验证，流程见 [docs/exhibition-hall.md](docs/exhibition-hall.md)。
- 加人物：actor 的 `skin_uri` 路径**不能带空格**（如 `male%20visitor`），否则 Gazebo 加载卡死、黑屏；`validate_layout` 会拦截。
- 改展品名称或讲解内容：同时改 `knowledge/E0x.md`（见 6.6）。

### 6.2 机器人模型

| 模型 | 生成脚本 | 用途 | 要点 |
|---|---|---|---|
| 平移模型 `g1_glide.urdf` | `make_glide_urdf.py` | 展馆导航（主流程） | 腿和腰焊死，身体是一个刚体，由 VelocityControl 直接设速度；**base_link 必须关闭重力**，否则会倒或"起飞"。左右臂 14 个关节保持可动，手臂连杆也关重力，每个关节由 gz `JointPositionController` 保持在 `/g1/arm/<关节>/cmd_pos` 给的角度 |
| 真机雷达版 `g1_glide_mid360.urdf` | 同上（`build.sh` 一起生成） | `lidar:=mid360` | 头部倒装 MID-360；不发 odom TF，由 `odom_drift` 发布，所以必须和 `odom_drift` 一起启动（`lidar_variants` 负责） |
| 低阶模型 | `make_controlled_urdf.py` | `lowlevel.launch.py` | 29 关节，骨盆固定，ros2_control 力矩 PD |

**传感器**：头部 RGB-D（D435i，按官方 URDF 向下俯 47.6°，默认 10 Hz）；前置水平相机 `front_camera`（torso_link 上 0.09/0/0.50 m，离地约 1.34 m，视野 70°，640×480，10 Hz）；导航雷达；MID-360（360°×−7°～52°，40 m，仿真 30 线）；IMU；里程计。

`config/bridge.yaml` 里相机、`/scan` 和俯视相机都设为 `lazy`：没人订阅时 Gazebo 不渲染，大幅减少内存泄漏。**以后加视觉订阅时帧率能低就低。**

### 6.3 导航

**算法**：`nav_grid` 把展馆墙体、展品、静态参观者按矩形生成占据栅格，按机器人净空半径（0.55 m）+ 余量膨胀，做 A*。雷达点中不在静态几何上的点被当作动态障碍，叠加进栅格并在路线被挡时提前重规划。`nav_control` 先原地转向再前进，前方太近时停车，接近终点减速，到站后对准展品朝向。

所有决策都在 `nav_core.NavigatorCore`（与 ROS 无关），ROS 节点 `exhibition_nav` 只负责收发，离线仿真 `tools/nav_sim.py` 用的是同一份代码。

**导航状态**（`/nav/status` 的 `state`）：

| state | 含义 |
|---|---|
| `requested` | 收到目标 |
| `aligning` / `moving` | 原地对准 / 沿路径前进 |
| `blocked` | 前方太近被迫停车，等待重规划 |
| `backing_up` | 后退脱困中 |
| `arrived` | 到站 |
| `facing` | 面向观众模式：到站后原地转向 3 m 内人群中心 |
| `cancelled` / `invalid_goal` / `invalid_command` / `sensor_stale` | 取消 / 目标无效 / 指令无效 / 传感器数据中断 |

**主要参数和判定**（改哪个文件哪个常量）：

| 判定 | 值 | 位置 |
|---|---|---|
| 最高速度 | 直线 0.6 m/s，转向 1.2 rad/s | `nav_control.py` `MAX_LINEAR_SPEED` / `MAX_ANGULAR_SPEED` |
| 前方停车距离 | 前方回波 < 0.8 m 停车 | `OBSTACLE_STOP_DISTANCE` |
| 允许原地转身 | 周围 0.5 m 内没有东西 | `ROTATE_CLEARANCE` |
| 到站 | 离站位 < 0.25 m 且朝向误差 < 0.2 rad | `ARRIVAL_DISTANCE` / `ARRIVAL_YAW_ERROR`；SLAM 模式用 launch 参数 `arrival_distance:=0.15` |
| 先转后走 | 朝向误差 > 0.8 rad 时原地转 | `TURN_IN_PLACE_ERROR` |
| 规划净空 | 0.55 m | `exhibition.json` `robot_clearance_radius` |
| 路径点到达 | 0.25 m | `nav_core.py` `WAYPOINT_REACHED` |
| 失败后重试间隔 | 1 s | `REPLAN_RETRY_S` |
| 路线被挡提前重规划 | 最快 1 s 一次 | `REPLAN_BLOCKED_MIN_S` |
| 动态障碍记忆 | 看不见后仍记 15 s，除非之后有光束穿过那里；自身 0.6 m 内不记 | `MEMORY_S` / `MEMORY_SELF_RADIUS` |
| 面向观众 | 3 m 内人群中心，误差 0.07 rad，最多转 8 s | `FACE_RADIUS` / `FACE_YAW_TOLERANCE` / `FACE_TIMEOUT_S` |

**三种脱困手段**（2026-09-26 修复了三种锁死）：

1. **原地转身**：周围 0.5 m 内没东西就允许转，即使前方 < 0.8 m。
2. **起点吸附**（`plan_with_escape`）：起点落在**静态**障碍膨胀区（比如雷达看不到的 0.9 m 矮展台）时，把起点吸附到最近的可通行点。
3. **后退脱困**（`back_off`）：被**动态**障碍（人、箱子、自己抬起的手臂）逼得太近时触发。
   - 触发：规划失败或 `blocked` 时，前方 ±60° 最近回波 < 规划净空 + 0.1 m（0.95 m）。
   - 动作：直线倒退 0.15 m/s，不转向；每个控制周期（10 Hz）检查一次。
   - 停止：前方 ≥ 0.95 m → 立即重规划；后方 ±30° 回波 < 0.6 m → 停止后退，每 1 s 重试规划。
   - 参数：`nav_core.py` `BACK_OFF_HALF_ANGLE` / `BACK_OFF_MARGIN`，`nav_control.py` `REVERSE_SPEED` / `REAR_STOP_DISTANCE`。
   - **只在仿真里验证过**。上真机前要确认：MID-360 对正后方和低处有没有盲区、倒退步态是否稳定、后方 ±30° 对 0.5 m 宽的身体够不够、稀疏点云会不会让机器人来回进退。

到站做指向手势时手臂会抬进雷达平面（前方约 0.45 m 的回波），所以出发时先后退一小段，属正常。

**两种导航模式**：

| | 布局模式（主流程）`exhibition_nav.launch.py` | SLAM 地图模式 `slam_nav.launch.py` |
|---|---|---|
| 地图 | 从 `exhibition.json` 算出栅格 | 读 SLAM 建好的 `maps/*.yaml` |
| 定位 | Gazebo 精确 `/odom` | AMCL，发布 `map → odom` |
| 里程计 | 无误差 | 真机雷达版默认漂移：距离 +3%、转角 +2%、航向每秒 0.005 rad |
| 雷达 | 简化 2D | 默认 MID-360 |
| 到站 | 0.25 m | 0.15 m（给定位误差留余量） |

**怎么验证改动**：先离线，后 Gazebo。

```bash
python3 tools/nav_sim.py
```
简单展馆全部展位两两互达，约 20 s。加 `--scene hall` 跑大展馆（约 3 分钟）；`--tour 3 1 --box-ahead 2:1.3 -v` 在行进中前方放箱子并打印状态；`--face --people-at-stations` 测面向观众。20 s 没离开 0.5 m 范围自动判为卡死。

离线全过后在 Gazebo 里确认（慢，日志在 `.navrun/`）：
```bash
bash tools/nav_gz_tour.sh <名字> 4 5
```

### 6.4 讲解手势

`gesture_player` 订阅 `/guide/gesture`，按 `config/gestures.yaml` 的关键帧插值播放，每轮叠加小的随机扰动（`jitter_deg`）显得自然。

| 动作 | 效果 |
|---|---|
| `home` | 手臂自然下垂（也是 Gazebo 里的初始姿势） |
| `wave` | 右手挥手，打招呼 / 道别 |
| `point` | 指向展品：由 `arm_kinematics.py`（numpy FK + 阻尼最小二乘 IK）实时计算，展品在哪就指哪；只发 `point` 时指向最近的展品 |
| `present` | 右手摊开介绍 |
| `explain_loop` | 讲解时持续循环的自然手势 |
| `emphasize` | 右手强调一下（左手保持不动） |
| `invite` | 双手邀请 |

**规则**：导航处于 `requested` / `aligning` / `moving` / `blocked` / `backing_up` 时只允许 `home`，其他动作被拒绝；到站（`arrived`）后自动指向展品（参数 `auto_point`），但讲解模式是"面向观众"时不自动指向。

**怎么改 / 加动作**：编辑 `config/gestures.yaml`，保存即生效（symlink 安装）。

- 角度单位为度，每只手臂 7 个值：肩 pitch、肩 roll、肩 yaw、肘、腕 roll、腕 pitch、腕 yaw。
- 全部按**左臂约定**写，右臂自动镜像：肩 pitch 负 = 向前抬，肩 roll 正 = 向外张，肘 0 = 前后臂成 90°、肘 90 = 伸直，腕 roll −90 = 掌心朝上。
- 一只手臂可以写另一个姿势名（如 `home`，取那个姿势里这只手臂的角度）或 `keep`（保持当前角度）。
- 动作由 `frames`（`pose`、`time` 秒、可选 `hold`）组成，`then` 指定结束后回 `home` 还是 `hold`，`loop: true` 循环播放。
- 改了 `home` 要重新 `build.sh`。新动作要让语音端能调用，还要加进 `voice/robot.py` 的 `GESTURES`。
- 关节限速等参数：节点参数 `rate_hz`（50）、`max_speed`（1.5 rad/s）。

### 6.5 雷达人群感知

`lidar:=mid360` 时 `people_tracker` 随导航一起启动，发布 `/guide/people`（约 2 Hz）和 RViz 标记 `/guide/people_markers`。

**算法**（`people_core.py`）：MID-360 点云 → 去掉地面、展馆墙体和展品等静态几何、机器人自身 → 0.3 m 网格聚类（相邻格为一簇）→ 按高度和宽度判断是不是人 → 跨帧跟踪给稳定的 id。

| 判定 | 值 | 参数（`PeopleParams`） |
|---|---|---|
| 地面回波 | 离地 < 0.15 m 丢弃 | `min_height` |
| 自身过滤半径 | 0.75 m（指向手势时手伸到前方 0.5 m，曾被当成 0.97 m 的"儿童"） | `self_radius` |
| 最远距离 | 6 m | `range_max` |
| 一簇至少 | 4 个点 | `min_points` |
| 簇顶高度 | 0.7～2.05 m 才算人（更矮是包、长椅，更高是门框） | `min_top` / `max_top` |
| 一簇几个人 | 宽于 0.7 m 按多人算；宽于 2.5 m 是家具，不算 | `person_width` / `max_extent` |
| 儿童 | 整个人都在雷达视野内且身高 < 1.3 m | `CHILD_MAX_HEIGHT` |

**限制**：MID-360 倒装在头部（约 1.27 m 高），向上只看约 7°，**成人头顶在视野外，只能测出身高下限**（标 `height_clipped`，永远不算儿童）；儿童比雷达矮，能测全高，正好用于区分成人和儿童。

**到站讲解流程**（语音端 `voice/crowd.py` + `engine.py`，GUI 的"按雷达人数讲解"开关控制）：

1. 到站时 3 m 内没人 → 说"大家跟上了吗？"，等 10 秒；仍然没人就不讲。
2. 有人 → 开始讲解。**只有 1 个人且是儿童**时自动切儿童模式，其余情况用普通模式（游客手动要求过的风格优先）。
3. GUI 选"面向观众"时，机器人先原地转向人群中心再讲，不做指向手势。
4. 讲解中观众全部离开超过 5 秒 → 挥手说"谢谢大家的参观，再见！"，进入休眠。
5. 没有雷达人群数据（例如 `lidar:=simple`）时，到站直接讲解，和原来一样。

参数在 `voice/crowd.py`：`FOLLOW_RADIUS = 3.0`、`FOLLOW_WAIT_S = 10.0`、`GONE_S = 5.0`、`FOLLOW_PROMPT`、`GOODBYE`。

### 6.6 语音对话

**链路**：麦克风（按名称选，默认关键字 Realtek）→ WebRTC 回声消除（可选）→ silero VAD 切句 → SenseVoice int8 离线识别（实时率约 0.1）→ 唤醒判定 → 大模型工具调用（失败自动转离线规则）→ Matcha 离线合成（每句约 0.2 秒）→ 扬声器；同时通过 rosbridge 控制机器人。

**两个"大脑"**：

| | 大模型（主线） | 离线规则（兜底） |
|---|---|---|
| 实现 | `llm.py`：DeepSeek / 通义千问（OpenAI 兼容接口）、Claude | `fallback.py`：关键词意图 + 字符 n-gram TF-IDF 问答 |
| 知识来源 | 6 份展品资料全部放进系统提示词 | 同一份资料的"常见问题" |
| 对话状态 | 由对话历史承担（保留最近 30 条） | 记录当前展品、目标、推荐的下一站 |
| 何时使用 | 设置了 API Key | 没有 Key、`--offline`，或大模型请求失败 |

大模型请求超时 10 秒（连接 4 秒，重试 1 次），失败后 30 秒内直接用离线规则，不让游客每句都等超时。参数在 `llm.py`：`REQUEST_TIMEOUT`、`CONNECT_TIMEOUT`、`CONNECT_RETRIES`、`OFFLINE_COOLDOWN`、`MAX_HISTORY`。

**大模型能调用的工具**（`dialog.py` `build_tools`）：

| 工具 | 作用 |
|---|---|
| `goto_exhibit(exhibit_id)` | 带游客去某件展品（只在游客明确想去或同意推荐时调用） |
| `stop_robot()` | 立即停车 |
| `get_robot_status()` | 查询机器人状态 |
| `set_style(audience, humor)` | 切换讲解风格和幽默值 |
| `gesture(name, exhibit_id)` | 配合说话做手臂动作（移动途中不调用） |

到站、受阻、传感器中断时，机器人状态会以 `[系统事件]` 消息发给大模型，触发主动讲解或向游客解释。说话规则（口语化、每次 1～3 句不超过 80 字、讲解不超过 150 字、年份逐位读、不念英文编号、资料里没有的如实说不确定等）写在 `dialog.py` 的 `build_system_prompt`。

**离线规则能听懂的话**（`fallback.py`，按关键词匹配）：

| 意图 | 关键词举例 |
|---|---|
| 停 | 停、等一下、等等、别走、站住、暂停 |
| 去某展品 | 带我、去、领我、想看、看看、参观 + 展品名或别名 / 编号 |
| 下一个 / 继续 | 下一个、下一站、接下来 / 继续走、接着走、走吧 |
| 展品列表 | 有什么展品、有哪些、都有什么 |
| 介绍 / 位置 / 原作 | 介绍、讲讲、是什么 / 在哪、怎么走 / 原画、真迹、收藏在 |
| 打招呼 / 自我介绍 / 再见 / 再说一遍 | 你好 / 你是谁 / 再见、谢谢你 / 再说一遍、没听清 |
| 讲解风格 | 简单点、小朋友 → 儿童；专业一点、详细讲 → 专业；正常讲 → 普通；幽默一点 / 严肃点 |
| 确认 / 拒绝推荐 | 好、可以、走吧、嗯 / 不、算了、不用 |

问答判定：问题和"常见问题"的 TF-IDF 余弦相似度 ≥ 0.15 才回答（无关问题在本展品集上一般 ≤ 0.13），当前正在讲的展品加 0.1 优先；能容忍一个同音错字。关键词和阈值都是 `fallback.py` 开头的常量（`FAQ_THRESHOLD`、`CONTEXT_BONUS`、`*_WORDS`）。

**唤醒**（`--wake` 或 GUI 开关）：

- 说"你好小吉"或"小吉，带我去看青花瓷"才开始对话。唤醒词要在句首（前面可带你好、您好、嗨、喂、哈喽、嘿），"小机器人"不会误唤醒。
- **按读音匹配**，并做了模糊音归并（zh/z、ch/c、sh/s、n/l、r/l、f/h、ing/in、eng/en、x/s、q/c、j/z），所以识别成"小鸡""晓吉"也能唤醒。
- 机器人说话、思考、走路时保持唤醒；空闲 30 秒或说"再见"后休眠。
- 两种唤醒方式（GUI 选择）：**直接唤醒**（说唤醒词即可）；**视觉唤醒**（说唤醒词前 2 秒内，相机要看到 3 m 内有人正对机器人，见 6.7）。没有相机数据时自动退回直接唤醒并提示。
- 参数在 `voice/engine.py`：`WAKE_IDLE_S`、`WAKE_PREFIXES`、`_FUZZY`、`VISION_WAKE_WINDOW_S`。

**讲解风格**：

| 风格 | 说法 |
|---|---|
| 普通 | 面向一般游客，通俗准确 |
| 儿童 | 短句、生活比方、多互动提问、不用术语，合成语速放慢到 0.85 |
| 专业 | 可以用术语，多讲原理、数据、来历，讲解可到 200 字 |

幽默值 0～10：0 完全严肃，3 偶尔轻松，6 以上常开玩笑，9 以上像脱口秀，知识始终准确。说"幽默一点"+3，"严肃点"归零。离线规则没有幽默效果，但讲解长度随风格变化（儿童读 2 句、普通 4 句、专业全文）。

**打断**（`--barge-in`）：机器人说话时每 0.4 秒识别一次麦克风；识别到的字数 ≥ 4、与机器人刚说的话按拼音比对重合度 < 50%（不是回声），并且连续 2 次都这样，才打断。实测戴耳机约 0.6 秒停、外放约 0.9 秒停，只有回声时不误打断。不开打断时，机器人说完后麦克风再关 0.3 秒防房间回声。终端版按回车、GUI 按按钮可以随时打断。参数在 `engine.py`：`BARGE_POLL_S`、`BARGE_MIN_CHARS`、`BARGE_CONFIRM`、`ECHO_OVERLAP`、`MUTE_TAIL_S`。

**GUI**（`voice\gui.cmd`）：对话记录、语音状态灯、音量条、大模型 / 机器人 / 相机状态、唤醒开关和方式、讲解风格下拉框、幽默滑块、"到站后：指向展品 / 面向观众"、"按雷达人数讲解"开关、快捷按钮、打字输入、日志。语音里切换的风格会自动同步到界面。

**环境变量**：

| 变量 | 作用 |
|---|---|
| `DEEPSEEK_API_KEY` / `DASHSCOPE_API_KEY` / `ANTHROPIC_API_KEY` | 大模型 Key，设了哪个用哪个 |
| `G1_LLM_PROVIDER` | 强制选 `deepseek` / `qwen` / `claude` |
| `G1_LLM_MODEL` | 换模型（默认 deepseek-chat / qwen-plus / claude-sonnet-5） |
| `G1_TTS=edge` | 换在线微软语音（音质更好，每句多约 2 秒）；`G1_TTS_VOICE` 选音色 |
| `G1_MIC` | 麦克风名称关键字（默认 Realtek；系统默认的 ToDesk 虚拟麦克风没有声音） |
| `G1_ROSBRIDGE_HOST` / `G1_ROSBRIDGE_PORT` | rosbridge 地址（默认 localhost:9090） |

**知识库**：`knowledge/E01.md`～`E06.md`，格式如下，每件 7 条问答。改内容直接编辑，重启语音端生效。

```markdown
# E06 青花瓷瓶

别名：青花瓷瓶、青花瓷、青花、瓷瓶、花瓶、瓶子      ← 用来识别游客说的是哪件
形式：立体展品（瓷器仿制品）

## 讲解
（正文，到站时讲）

## 常见问题
- 问：青花是怎么烧出来的？ 答：……                  ← 离线问答用，格式要严格一致

## 推荐下一站
E04 神奈川冲浪里：……
```

**语音端代码**：

| 文件 | 内容 |
|---|---|
| `engine.py` | 核心：线程编排、唤醒、打断、回声过滤、到站讲解流程、手势事件 |
| `dialog.py` | 系统提示词、工具定义、一轮对话（含工具调用） |
| `llm.py` | 大模型后端与失败降级 |
| `fallback.py` | 离线大脑 |
| `crowd.py` | 雷达人群规则、人脸与雷达人员匹配 |
| `asr.py` / `tts.py` / `audio_io.py` / `aec.py` | 识别、合成、可中断播放、回声消除 |
| `robot.py` | rosbridge 连接：导航、手势、人群、视觉 |
| `exhibits.py` | 读取展品配置和知识库 |
| `gui.py` / `app.py` | 图形控制台 / 终端入口 |

### 6.7 视觉

**划界原则**：视觉只做数据库里没有的东西。展品位置、名称、机器人位姿都已知，不再用视觉确认；**避障只靠雷达**（雷达直接测距 10 Hz，视觉延迟大得多）。展品识别、OCR 读展牌已按决定取消。

**链路**：WSL `camera_jpeg`（前置相机 5 Hz JPEG）→ rosbridge → Windows `vision/watch.py`（YOLO 人、YuNet 人脸、MediaPipe 手势，CPU 约 1.6 帧/秒，处理不过来就丢旧帧）→ `/guide/vision` → 语音端。视觉代码只吃图像，不依赖 ROS 和 Gazebo，真机换 USB 摄像头的话题即可。

**是否正对机器人**（`vision/vision_core.py`）：YuNet 检出正脸（侧脸本来就检不出），且两眼间距 ≥ 脸宽的 0.25、鼻尖偏离两眼中点 < 两眼间距的 0.35。参数：`FACING_EYE_RATIO`、`FACING_NOSE_OFFSET`。

**距离**：优先用雷达。人脸方位和雷达人员方位相差 ≤ 8°，并且按脸宽（成人约 0.16 m）估的距离不超过雷达距离 2.5 倍（排除后面远处的正脸），就取那个人的雷达距离；没有雷达数据时才用脸宽估算。参数在 `voice/crowd.py`：`MATCH_BEARING_DEG`、`MATCH_MAX_DISTANCE_RATIO`。

**手势指令**（`vision/gestures.py`）：

| 手势 | 判定 | 效果 |
|---|---|---|
| 手掌 | 五指张开、手不动 | 停止讲解并停车（**任何时候都生效**，不用唤醒） |
| 挥手 | 五指张开，2 秒内手腕左右摆幅 ≥ 0.6 个掌宽、换向 ≥ 2 次 | 讲解中 = "再见"，否则 = "你好" |
| 竖拇指 | 只竖拇指且朝上 | "好的" |
| 比 1～4 | 伸出 1～4 根手指（不含拇指） | 去 1～4 号展位（五指和"手掌"冲突，5、6 号只能用说的） |

静态手势要保持 0.5 秒（且至少 2 帧）才触发，每次保持只触发一次。除"手掌"外都要先唤醒；机器人说话时语音优先，手势忽略。参数：`HOLD_S`、`WAVE_WINDOW_S`、`WAVE_SPAN`、`WAVE_REVERSALS`；手势到语音指令的映射在 `voice/engine.py` `on_gesture`。

**为什么加前置相机**：宇树官方 G1 的头部 D435i 固定向下俯 47.6°，只能看到 2.7 m 内的地面，看不到人脸；头部没有关节，腰部后仰也不可靠。所以仿真里在胸前加了水平朝前的相机，对应真机外接 USB 摄像头。

### 6.8 SLAM（实验功能）

主流程用布局模式导航（用户决定），SLAM 只作为实验功能保留，**只在简单展馆（16×12 m，`config/exhibition_simple.json`）建图**，不在新展馆建图。slam_toolbox（online_async）输入 `/nav_scan`，坐标系 `map → odom → base_link`；仿真里 odom 和世界坐标重合，所以地图坐标 = 世界坐标，可以直接和布局对比。

| | 简化雷达版 `lidar:=simple`（默认） | 真机雷达版 `lidar:=mid360` |
|---|---|---|
| 建模 | 虚构的水平单线雷达，8 m | 头部倒装 3D：360°×（−7°～52°），40 m，仿真 30 线 |
| `/nav_scan` 来源 | 额外加的水平雷达（真机没有） | `mid360_to_scan` 取离地 0.08～2 m 的点压成 2D |
| 矮障碍（长椅、花盆） | 扫不到 | 能建进地图、能避障 |
| 实测 | 墙体偏差 p95 0.03 m，但矮物体没进地图 | 墙体 p95 0.06 m，14 个物体全部进地图，6 站全部到达 |

- MID-360 会扫到自己的肩膀和手臂（0.3～0.4 m），`mid360_to_scan` 的 `self_radius` 必须 ≥ 0.5，否则导航一直 blocked。
- 简化雷达版建的地图没有矮物体，**不要用它做 SLAM 地图导航**。
- 地图旁的 `*_meta.json` 记录了建图的展馆，地图和展馆不一致时启动会直接报错。
- SLAM 地图导航实测：里程计漂 2.6 m 时 AMCL 定位误差平均 0.05 m。

**换真机时**：① 建图——把 Gazebo 和桥接换成 MID-360 驱动和里程计，`slam.launch.py lidar:=mid360` 开车走遍展馆后保存地图；② 标注展位（待做）——真机地图原点是开始建图的位置，要把机器人开到每个展品前记录位姿；③ 导航——`slam_nav.launch.py map:=<地图.yaml>`。

### 6.9 低阶控制

`make_controlled_urdf.py` 生成完整 29 关节模型（骨盆固定支撑），通过 ros2_control 做力矩 PD；`joint_targets.py` 做限速和失联保护，`lowlevel_guard` 节点监控，`lowlevel_metrics.py` 统计跟踪误差。向 `/g1_lowlevel/target` 发 `JointState` 即可（见 5.7）。

### 6.10 步态训练（MuJoCo / Isaac）

与 ROS / Gazebo 主线无关的独立实验：让 G1 真正用双腿走路，并能上台阶和坡。

**宇树预训练平地策略**（`mujoco/g1_walk.py`）：12 自由度，用 `unitree_rl_gym` 的 `motion.pt`。官方 `deploy_mujoco.py` 依赖 Isaac Gym，Windows 和 WSL 都装不了，所以改写成不依赖它的版本。速度指令范围 vx −0.5～1.0、vy ±0.5、wz ±1.0，超出会走不稳。实测直走 10 秒 4.66 m（0.47 m/s），转圈、横移都不倒；但 6 条跑道全部不合格，作为基线。

**测试跑道**（`mujoco/terrain.py`，用 `MjSpec` 在宇树 `scene.xml` 上加几何体，不改第三方文件）：机器人从 x = 0 出发，跑道从 x = 2 m 开始，宽 1.6 m。

| 跑道 | 内容 |
|---|---|
| `slope5` / `slope10` / `slope15` | 上坡 3 m → 平台 1.5 m → 下坡 3 m |
| `stairs10` / `stairs15` / `stairs18` | 上 5 级 → 平台 1.5 m → 下 5 级，踏面 30 cm |

摔倒判定：机身倾斜超过 60°，或骨盆离脚下地面（按跑道设计高度算）< 0.45 m。

**自训练崎岖地形策略**（方法详见 [docs/G1 第三版.md](docs/G1%20第三版.md)，服务器环境见 [tools/isaac_server/README.md](tools/isaac_server/README.md)）：

- Isaac Lab 2.3.2 / Isaac Sim 5.1，PPO（rsl_rl），16384 个机器人并行，1500 轮，单卡 RTX 4090 约 2 小时。本机配置低于 Isaac 最低要求，只在租用服务器上训练。
- 模型 `g1_minimal`（37 关节），观测 310 维（含 187 点地形高度扫描），动作 = 默认姿态 + 0.5 × 输出，控制 50 Hz。
- 地形课程：上下楼梯、随机方块、随机起伏、上下坡，台阶 5～23 cm、坡度 0～23°，10 级自动调难度。

| 版本（`mujoco/policies/`） | 改动 | 骨盆高度 | Isaac 结果 |
|---|---|---|---|
| `g1_rough_n16384_mb8` | 基线 | 0.49 m（半蹲） | 台阶、坡基本 ≥ 98% |
| `g1_rough_height072_n16384_mb8` | + 机身高度奖励（0.72 m，权重 −10） | 0.68 m | 20° 坡降到 78% |
| `g1_rough_rand070_n16384_mb8`（**最好**） | 高度目标 0.70 m、权重 −5 + 摩擦 / 质量 / 质心 / 推力 / PD 增益随机化 | 0.64 m | 13 项全部 ≥ 98.4%，零摔倒 |

**sim2sim**：用关节定义一致的 MuJoCo Menagerie 2024-07 版 G1；高度扫描用射线，转向指令按训练时的方式由朝向误差计算（系数 0.5），物理步长默认 0.002 s（0.005 s 时接触不准）。第三版在 MuJoCo 平地能走，6 条跑道过 3 条（10°、15° 坡和 10 cm 台阶），15/18 cm 台阶摔倒、5° 坡向一侧偏出跑道，原因未查清。

Isaac 自己录不了视频：服务器上用 `record_demo.py` 记录位姿，本地 `mujoco/replay_demo.py` 渲染成 mp4。

---

## 7. 测试

ROS 端单元测试（WSL）：
```bash
cd /mnt/d/g1_guide/ws/src/g1_guide && python3 -m pytest -q -p no:cacheprovider test
```

语音端单元测试（Windows）：
```bash
voice\.venv\Scripts\python.exe -m pytest voice\tests
```

视觉手势分类与防抖：
```bash
vision\.venv\Scripts\python.exe -m pytest vision\tests
```

模拟测试（用合成语音代替麦克风）：

| 脚本 | 验证内容 |
|---|---|
| `voice\tests\sim_barge_in.py` | 三种打断场景（约 45 秒） |
| `voice\tests\sim_echo.py` | 只有回声时不误打断 |
| `voice\tests\sim_crowd.py` + `tools/crowd_e2e_sim.sh` | Gazebo 端到端：到站没人 → "大家跟上了吗" → 放人 → 儿童模式、转身讲解 → 删人 → 挥手告别 |
| `voice\tests\sim_vision_wake.py` + `VISION=true bash tools/crowd_e2e_sim.sh vision` | 视觉唤醒：正对 → 唤醒，背对、侧身 → 忽略 |

导航离线 / Gazebo 测试见 [6.3](#63-导航)；MuJoCo 跑道无窗口测试见 5.8 的 `--check-all`。

---

## 8. 工具脚本

| 工具 | 用途 |
|---|---|
| `tools/nav_sim.py` | 离线导航仿真（与 Gazebo 同一份决策代码） |
| `tools/nav_gz_tour.sh` | Gazebo 里自动走一条路线，日志写到 `.navrun/` |
| `tools/nav_speed_log.py` | 记录每秒速度和实时率 |
| `tools/spawn_box.sh` / `remove_box.sh` | 放 / 删障碍箱 |
| `tools/people_gz_check.sh`、`people_watch.py` | 人群感知 Gazebo 检查与实时查看 |
| `tools/crowd_e2e_sim.sh` | 人群 / 视觉端到端测试的仿真端 |
| `tools/cv_capture_sim.sh`、`cv_capture.py` | 在展位用相机拍帧，给视觉可行性测试 |
| `tools/render_map.py maps/<地图>.yaml` | 把 SLAM 地图画成带展品框、展位箭头、米刻度的 PNG |
| `tools/mem_probe.sh <秒> <间隔>` | 采样 Gazebo 进程内存，排查泄漏 |
| `tools/run_detached.sh` | 用 systemd 托管一条 ros2 命令 |
| `tools/hall/probe_museum.py`、`bake_nav_obstacles.py`、`verify_bake.py` | 探测展馆网格、烘焙墙体矩形、验证烘焙 |
| `tools/hall/check_layout.py`、`check_tour.py`、`plan_layout.py`、`propose_spots.py` | 布局坐标实测、展位可达性检查、布局规划 |
| `tools/hall/bisect_world.py`、`bisect_load.py` | 二分定位导致世界加载卡死的元素 |
| `tools/isaac_server/` | 服务器安装脚本、版本锁、自定义任务 `G1-Rough-Height-v0`、踩坑记录 |

---

## 9. 已知问题与限制

- **Gazebo 渲染内存泄漏**（WSL 的 d3d12 显卡转译层，上游未修：gazebosim/gz-sim#2873）：大展馆开窗口时窗口进程约 +21 MB/s，WSL 只有 7.5 GB，开久了会被 OOM 杀掉（`dmesg | grep -i oom` 可查）。长时间运行用 `headless:=true` 或定期重启。仿真主进程已通过传感器 `lazy` 降到约 1 MB/s。全软件渲染实时率只有 0.005，不能用。
- **实时率**：大展馆 headless 约 0.47–0.49，开窗口 0.40–0.44。2026-09-29 把展馆碰撞从 8 MB 网格换成烘焙盒子后，不再越走越慢。
- **雷达与 actor**：Gazebo 的 actor 没有碰撞体，但雷达按渲染测距仍能扫到它。
- **矮展台**：书和花瓶展台只有 0.9 m 高，简化雷达看不到，靠静态地图和起点吸附处理。
- **成人身高**：MID-360 视野向上只有 7°，成人只能测出身高下限。
- **后退脱困**：只在仿真验证，上真机前要逐项确认（见 6.3）。
- **sim2sim 差距**：崎岖地形策略在 MuJoCo 的高台阶和缓坡上失败，原因未查清。
- **代码里的绝对路径**：`tools/` 下少数测试脚本写了 `/home/ma/.local/...`，换机器要改。

---

## 10. 常见错误

| 错误 | 原因 | 解决 |
|---|---|---|
| `gazebo: command not found` | 新版命令是 `gz sim` | 用 `gz sim` |
| `Package 'g1_guide' not found` | 没有构建或没有 source | 运行 `build.sh` 后重开终端 |
| Gazebo 卡顿 | GPU 路径未启用 | `glxinfo \| grep renderer` 检查 |
| 改了代码仿真没变化 | 新增了文件或改了 URDF 生成器，没重新构建 | 运行 `build.sh` |
| 机器人倒下或"起飞" | 安装目录里是旧 URDF（base_link 没关重力） | 运行 `build.sh`，不要单独跑 `make_glide_urdf.py` |
| 搬文件夹后启动报找不到文件 | 软链接指向旧位置 | 删掉 `~/.local/share/g1-guide-runtime` 后重新 `build.sh` |
| 黑屏、Entity Tree 空、`SceneBroadcaster: Timed out waiting for state` | actor 的 Fuel 路径带空格，网格解析器挂住 | 换路径不含空格的模型；用 `tools/hall/bisect_world.py` 定位 |
| 导航一直报 blocked（mid360） | 雷达扫到自己的肩膀和手臂 | `mid360_to_scan` 的 `self_radius` ≥ 0.5 |
| 语音端"机器人 未连接" | 没开 rosbridge 或端口不对 | WSL 运行 `ros2 launch rosbridge_server rosbridge_websocket_launch.xml` |
| 麦克风没声音 | 系统默认是虚拟麦克风 | 设 `G1_MIC` 为真实麦克风名称关键字，用 `check_mic.py` 检查 |
| 外放时机器人自己打断自己 | 没装回声消除 | `pip install livekit`，或戴耳机 |
| `.bat` 运行报奇怪的命令错误 | cmd 按 GBK 读批处理，中文注释被当成命令 | `.bat` 只写 ASCII |
| WSL 报 `E_UNEXPECTED` | 内存被 Gazebo 耗尽 | 用 headless 或缩短运行时间，重启 WSL |
