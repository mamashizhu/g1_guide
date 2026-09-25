# G1 展馆导览机器人

项目位置：`D:\g1_guide`（WSL：`/mnt/d/g1_guide`）。代码里没有写死路径，整个文件夹可以随意搬动，搬动后重新运行 `ws/build.sh` 即可。

## 目录

```
g1_guide/
├── PLAN.md                       任务清单与计划
├── ws/                           ROS2 工作区
│   ├── build.sh                  构建脚本（生成 URDF + colcon --symlink-install）
│   └── src/
│       ├── g1_description/       Unitree 官方 G1 29 自由度模型和网格
│       ├── g1_bridge/            Gazebo↔ROS 桥接（C++，可正常退出）
│       └── g1_guide/             主包
│           ├── g1_guide/         节点与算法
│           │   ├── exhibition_world.py   按 config 生成展馆 SDF
│           │   ├── exhibition_nav.py     导航节点（A* + 雷达避障 + 路径跟踪）
│           │   ├── nav_grid.py           占据栅格与 A*
│           │   ├── nav_control.py        速度指令与安全停车
│           │   ├── nav_keyboard.py       数字键 1–6 选展位
│           │   ├── slam_explore.py       SLAM 自动巡航建图、保存地图
│           │   ├── slam_map.py           地图读写（pgm/yaml）与墙体偏差检查
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
| WASD 遥控 | `ros2 run g1_guide g1_teleop` |
| 传感器面板 | `ros2 run g1_guide sensor_dashboard` |
| 导航状态 | `ros2 topic echo /nav/status` |
| 放障碍 / 删障碍 | `bash /mnt/d/g1_guide/tools/spawn_box.sh box1 0 -2.5` / `remove_box.sh box1` |
| SLAM 建图（手动开车，简单展馆） | `ros2 launch g1_guide slam.launch.py rviz:=true`，再开 `nav_keyboard` 或 `g1_teleop` |
| SLAM 自动巡航建图并保存 | `ros2 launch g1_guide slam.launch.py explore:=true rviz:=true` → 地图存到 `maps/` |
| 手动保存当前地图 | `ros2 run nav2_map_server map_saver_cli -f /mnt/d/g1_guide/maps/my_map` |
| 检查地图墙体偏差 | `ros2 run g1_guide slam_map_check /mnt/d/g1_guide/maps/exhibition_map.yaml --scene <场景json>` |
| 低阶控制（骨盆固定） | `ros2 launch g1_guide lowlevel.launch.py` |
| 发关节目标 | `ros2 topic pub --once /g1_lowlevel/target sensor_msgs/msg/JointState '{name: [left_elbow_joint], position: [0.8]}'` |

## 测试

```bash
cd /mnt/d/g1_guide/ws/src/g1_guide && python3 -m pytest -q -p no:cacheprovider test
```
