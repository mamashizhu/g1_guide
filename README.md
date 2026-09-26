# G1 展馆导览机器人

项目位置：`D:\g1_guide`（WSL：`/mnt/d/g1_guide`）。代码里没有写死路径，整个文件夹可以随意搬动，搬动后重新运行 `ws/build.sh` 即可。

## 目录

```
g1_guide/
├── PLAN.md                       任务清单与计划
├── docs/
│   └── exhibition-hall.md        展馆网格模型与人物模型：导入、烘焙导航地图、布局坐标、排错
├── ws/                           ROS2 工作区
│   ├── build.sh                  构建脚本（生成 URDF + colcon --symlink-install）
│   └── src/
│       ├── g1_description/       Unitree 官方 G1 29 自由度模型和网格
│       ├── g1_bridge/            Gazebo↔ROS 桥接（C++，可正常退出）
│       └── g1_guide/             主包
│           ├── g1_guide/         节点与算法
│           │   ├── exhibition_world.py   按 config 生成展馆 SDF（导入展馆网格 + 人物）
│           │   ├── scene_config.py        场景配置加载与占地几何（世界与导航共用）
│           │   ├── exhibition_nav.py     导航节点（A* + 雷达避障 + 路径跟踪）
│           │   ├── nav_grid.py           占据栅格与 A*
│           │   ├── nav_control.py        速度指令与安全停车
│           │   ├── nav_keyboard.py       数字键 1–6 选展位
│           │   ├── slam_explore.py       SLAM 自动巡航建图、保存地图
│           │   ├── slam_map.py           地图读写（pgm/yaml）与墙体偏差检查
│           │   ├── mid360_to_scan.py     真机雷达版：MID-360 点云压成 2D /nav_scan
│           │   ├── odom_drift.py         真机雷达版：模拟腿式里程计漂移，发布 /odom
│           │   ├── lidar_variants.py     两个雷达版本共用的启动片段
│           │   ├── g1_teleop.py          WASD 键盘遥控
│           │   ├── lowlevel_*.py, joint_targets.py   低阶关节力矩 PD 控制
│           │   ├── sensor_dashboard.py, sensor_metrics.py   传感器面板
│           │   └── g1_camera_viewer.py, frame_sync.py      相机查看与时间戳配对
│           ├── scripts/          make_glide_urdf.py（平移模型）、make_controlled_urdf.py（29 关节模型）等
│           ├── launch/           g1_sim / exhibition / exhibition_nav / lowlevel
│           ├── config/           exhibition.json（展馆布局）、exhibition_simple.json（原版 16×12m 简单展馆）、
│           │                     bridge.yaml（话题桥接）、slam_toolbox.yaml / slam.rviz（建图）
│           ├── worlds/ assets/ urdf/ test/
└── tools/                        spawn_box.sh / remove_box.sh（避障测试用障碍）
```

## 构建

```bash
bash "/mnt/d/g1_guide/ws/build.sh"
```

构建使用 `--symlink-install`。改 Python、launch、config 后立即生效，无需重新构建。以下情况需要重新运行 `build.sh`：
- 新增文件
- 改了 `setup.py`
- 改了 `make_glide_urdf.py`（它负责生成 URDF）

`~/.bashrc` 已改为加载 `~/.local/share/g1-guide-runtime/install/setup.bash`，原文件备份在 `~/.bashrc.bak-before-g1-guide`。

## 运行（每条命令开一个新终端）

| 功能 | 命令 |
|------|------|
| 展馆 + 导航 | `ros2 launch g1_guide exhibition_nav.launch.py` |
| 数字键选展位（1–6，0 取消） | `ros2 run g1_guide nav_keyboard` |
| 讲解手势（wave / point / present / explain_loop / emphasize / invite / home） | `ros2 topic pub --once /guide/gesture std_msgs/String "data: wave"`；动作定义在 `config/gestures.yaml` |
| WASD 遥控 | `ros2 run g1_guide g1_teleop` |
| 传感器面板 | `ros2 run g1_guide sensor_dashboard` |
| 导航状态 | `ros2 topic echo /nav/status` |
| 放障碍 / 删障碍 | `bash /mnt/d/g1_guide/tools/spawn_box.sh box1 0 -2.5` / `remove_box.sh box1` |
| SLAM 建图（手动开车，简单展馆） | `ros2 launch g1_guide slam.launch.py rviz:=true`，再开 `nav_keyboard` 或 `g1_teleop` |
| SLAM 自动巡航建图并保存（简化雷达版） | `ros2 launch g1_guide slam.launch.py explore:=true rviz:=true` → `maps/exhibition_simple_map.*` |
| SLAM 自动巡航建图并保存（真机雷达版） | `ros2 launch g1_guide slam.launch.py lidar:=mid360 explore:=true rviz:=true` → `maps/exhibition_simple_map_mid360.*` |
| **SLAM 地图导航**（真机雷达版 + 里程计漂移 + AMCL） | `ros2 launch g1_guide slam_nav.launch.py rviz:=true`，再开 `nav_keyboard` 选展位 |
| SLAM 地图导航自动验收（走完 6 站出报告） | `ros2 launch g1_guide slam_nav.launch.py tour:=true` → `maps/nav_report.json` |
| 手动保存当前地图 | `ros2 run nav2_map_server map_saver_cli -f /mnt/d/g1_guide/maps/my_map` |
| 检查地图墙体偏差 | `ros2 run g1_guide slam_map_check /mnt/d/g1_guide/maps/exhibition_simple_map.yaml --scene <场景json>` |
| 低阶控制（骨盆固定） | `ros2 launch g1_guide lowlevel.launch.py` |
| 发关节目标 | `ros2 topic pub --once /g1_lowlevel/target sensor_msgs/msg/JointState '{name: [left_elbow_joint], position: [0.8]}'` |

## 测试

```bash
cd /mnt/d/g1_guide/ws/src/g1_guide && python3 -m pytest -q -p no:cacheprovider test
```

## 两个雷达版本（SLAM）

| | 简化雷达版 `lidar:=simple`（默认） | 真机雷达版 `lidar:=mid360` |
|---|---|---|
| 模型文件 | `urdf/g1_glide.urdf` | `urdf/g1_glide_mid360.urdf`（`build.sh` 同时生成） |
| MID-360 位置 | 真机位置（头部倒装） | 真机位置（头部倒装） |
| MID-360 建模 | 单线 2D，量程 8m | 3D：360°×（−7°～52°），量程 40m，仿真 30 线 |
| `/nav_scan` 来源 | 额外加的水平雷达 `nav_lidar_link`（真机没有） | `mid360_to_scan` 从点云取离地 0.08～2m 的点压成 2D |
| 矮障碍（长椅、花盆） | 扫不到 | 能建进地图、能避障 |
| 仿真负担 | 小 | 点云约 1 万点/帧，8Hz |

## 两种导航模式

| | 布局模式（原版，`exhibition_nav.launch.py`） | SLAM 地图模式（`slam_nav.launch.py`） |
|---|---|---|
| 地图 | 从布局文件 `exhibition.json` 算出栅格（事先知道所有墙和家具） | 读 SLAM 建好的 `maps/*.yaml/pgm`，只用机器人自己扫到的东西 |
| 定位 | 直接用 Gazebo 的精确 `/odom` | AMCL：地图 + 雷达修正里程计，发布 `map → odom` |
| 里程计 | 精确，无误差 | 真机雷达版默认开启漂移：距离 +3%、转角 +2%、行进时航向每秒漂 0.005 rad |
| 雷达 | 简化版 2D 雷达 | 默认真机版 MID-360（`lidar:=simple` 可切回） |
| 到站判定 | 离目标 0.25m 内 | 0.15m 内（给定位误差留余量，真实误差仍在 0.25m 内） |
| 速度上限 | 直线 0.6 m/s，转向 1.2 rad/s | 相同 |
| 规划、避障、跟踪 | A* + 雷达动态障碍 + 前方 0.8m 停车 | 相同 |

**日常导航用布局模式**；SLAM 地图模式是可选的实验功能，目前只有简单展馆的地图（`maps/exhibition_simple_map*`）。

SLAM 地图模式的完整流程与真机一致：先建图（`slam.launch.py lidar:=mid360 explore:=true`）→ 保存地图 → 用地图定位导航。
简化雷达版建的地图里没有长椅、花盆（雷达扫不到），**不要用 `lidar:=simple` 的地图做 SLAM 地图导航**。

## 换真机时怎么用 SLAM

仿真里日常用布局地图（`exhibition_nav.launch.py`）。真机没有布局文件、也没有精确里程计，按下面的顺序用 SLAM：

1. **建图**：`slam.launch.py lidar:=mid360`。真机上把 Gazebo 和仿真桥接换成 Unitree 的 MID-360 驱动（发布点云）和里程计；`mid360_to_scan` 把点云压成 `/nav_scan`，`slam_toolbox` 建图。开车走遍展馆（`g1_teleop` 或 `nav_keyboard`），用 `map_saver_cli` 或 `slam_explore` 保存地图。
2. **标注展位**（待做）：真机地图的原点是开始建图的位置，布局文件里的展位坐标不能直接用，需要把机器人开到每个展品前记录位姿。仿真里地图坐标恰好等于世界坐标，所以暂时直接用布局坐标。
3. **导航**：`slam_nav.launch.py map:=<地图.yaml>`，map_server + AMCL 定位，`exhibition_nav` 在地图上规划、避障。
4. 地图旁的 `*_meta.json` 记录了建图的展馆，地图和展馆不一致时启动会直接报错。

仿真里验证过的数据：简单展馆 SLAM 建图墙体偏差 p95 0.06m，14 个物体全部建进地图；里程计漂 2.6m 时 AMCL 定位误差平均 0.05m。

## 工具

| 工具 | 用途 |
|------|------|
| `python tools/render_map.py maps/<地图>.yaml` | 把 SLAM 地图画成带展品框、展位箭头、米刻度的 PNG（Windows/WSL 均可） |
| `bash tools/mem_probe.sh <秒> <间隔>` | 采样 Gazebo 进程内存并按堆/显卡映射/文件分类，排查内存泄漏 |
| `wsl -u root -e bash tools/run_detached.sh …` | 用系统级 systemd 托管一条 ros2 命令（仍需一条 wsl 连接保持发行版不被关闭，见脚本注释） |

