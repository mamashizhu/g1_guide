# 项目规则

## 项目位置（2026-09-25 起）

- 项目目录：`D:\g1_guide`，WSL 路径 `/mnt/d/g1_guide`，git 分支 `main`
- 本项目从 `D:\智能机器人` 迁出。旧目录只作历史参考，**不修改、不引用**。
- 任务计划：`PLAN.md`；使用说明：`README.md`
- ROS 包名：**`g1_guide`**（旧包名 `g1_course` 已弃用）
- 代码里不写死绝对路径，一律从 `__file__` 推算，保证整个文件夹可以搬动。

## 启动命令

```bash
ros2 launch g1_guide exhibition_nav.launch.py   # 展馆 + 导航
ros2 run g1_guide nav_keyboard                  # 数字键 1–6 选展位
ros2 run g1_guide g1_teleop                     # WASD
ros2 run g1_guide sensor_dashboard              # 传感器面板
ros2 launch g1_guide lowlevel.launch.py         # 低阶关节控制（骨盆固定）
ros2 launch g1_guide slam.launch.py explore:=true rviz:=true   # SLAM 自动巡航建图，地图存到 maps/
gz sim                                          # 只开 Gazebo
ros2 launch g1_guide slam_nav.launch.py rviz:=true              # SLAM 地图导航（真机雷达 + 漂移 + AMCL）
```

## 环境说明

- ROS 发行版：**lyrical**（Ubuntu 26.04），Gazebo 命令是 `gz sim`（不是 `gazebo`）
- WSL 里的 Python 3.14 没有 pip；Windows 端有 Python 3.13（`D:\python`），语音、摄像头等功能放在 Windows 端运行

## ~/.bashrc 已配置（新终端自动生效）

```bash
source /opt/ros/lyrical/setup.bash
source /home/ma/.local/share/g1-guide-runtime/install/setup.bash
export LD_LIBRARY_PATH=/usr/lib/wsl/lib:$LD_LIBRARY_PATH
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export GALLIUM_DRIVER=d3d12
```

改之前的备份：`~/.bashrc.bak-before-g1-guide`

## 修改代码后的构建流程

```bash
bash /mnt/d/g1_guide/ws/build.sh
```

- 构建使用 `--symlink-install`。只改已有的 Python、launch、config 文件时，立即生效，不用重新构建。
- 以下情况**必须**重新运行 `build.sh`：
  - 新增文件
  - 改了 `setup.py`
  - 改了 `make_glide_urdf.py`（它负责生成 URDF）
  - **项目文件夹搬了位置**（软链接会断）
- 不要单独运行 `python3 make_glide_urdf.py` 来"更新模型"，要走 `build.sh`。

## ROS2 包系统：源码目录 vs 安装目录

**核心原则：`ros2 launch` 只读安装目录，不读源码目录。**

| 位置 | 路径 |
|------|------|
| 源码目录 | `ws/src/g1_guide/...` |
| 安装目录 | `~/.local/share/g1-guide-runtime/install/g1_guide/...` |

**真实案例（教训）：** 旧项目没有使用 symlink 构建。修改 `make_glide_urdf.py` 加了重力修复后，只单独运行了 python 脚本，安装目录仍是旧 URDF，结果机器人一直倒下，排查很久才发现。

## 机器人模型要点

- 平移模型（`make_glide_urdf.py`）：所有关节都是 fixed，整个机器人是一个刚体；由 VelocityControl 直接设定速度；base_link **必须关闭重力**，否则会倒或"起飞"。
- 低阶模型（`make_controlled_urdf.py`）：29 个关节，骨盆固定，ros2_control 做力矩 PD。
- 旧的手臂指向实现用户实测不可用，已舍弃，需要重做（PLAN M2.3）。

## 展馆模型要点

- 展馆是导入的 Fuel 网格 `OpenRobotics/iscas_museum`（已缓存在 `~/.gz/fuel`，离线可用）。**它只有墙，没有地面也没有屋顶**，地面仍由 `exhibition_world.py` 生成。
- `nav_grid` 按矩形规划，吃不了网格。墙体已离线烘焙成 `config/hall_nav_obstacles.json`（320 个矩形）。**改了展馆模型必须重新烘焙并验证**，流程见 `docs/exhibition-hall.md`。
- 改布局坐标不要靠读图估，用 `tools/hall/check_layout.py` 对网格实测，再用 `tools/hall/check_tour.py` 确认 6 个展位仍可达。
- 行走的 actor 没有碰撞体，雷达看不见，只有摄像头能拍到；参与避障的是 3 个静态参观者。

## SLAM 要点

- `slam.launch.py` 默认用 `config/exhibition_simple.json`（原版 16×12m 简单展馆）；导入的大展馆用 `scene_config:=.../exhibition.json`。
- slam_toolbox 输入 `/nav_scan`，坐标系 `map → odom → base_link`；odom 来自 Gazebo OdometryPublisher，与世界坐标重合，所以地图坐标 = 世界坐标，可以直接和布局对比。
- 两个雷达版本：`lidar:=simple`（默认，虚构水平 2D 雷达）和 `lidar:=mid360`（真机规格 3D MID-360，`mid360_to_scan` 压成 `/nav_scan`）。两版都要保留，不要删简化版。
- MID-360 会扫到机器人自己的肩膀和手臂（0.3～0.4m），`mid360_to_scan` 的 `self_radius` 必须 ≥ 0.5，否则导航一直报 blocked。
- **主流程导航用布局模式**（用户决定，SLAM 只作为实验功能保留，不在新展馆建图）。
- 两种导航模式都保留：布局模式 `exhibition_nav.launch.py`（原版，用户验证过）和 SLAM 地图模式 `slam_nav.launch.py`（`exhibition_nav` 加参数 `map_yaml`、`global_frame:=map`、`arrival_distance:=0.15`）。
- 真机雷达版的 URDF 不发 odom TF：Gazebo 真值在 `/odom_truth`，`/odom` 和 `odom→base_link` 由 `odom_drift` 节点发布，所以 mid360 模型必须和 `odom_drift` 一起启动（`lidar_variants.lidar_actions` 负责）。
- 依赖 `ros-lyrical-slam-toolbox`、`ros-lyrical-nav2-map-server`、`ros-lyrical-nav2-amcl`、`ros-lyrical-nav2-lifecycle-manager`（已 apt 安装）。
- 在 WSL 里跑仿真要让 `wsl` 命令常驻（后台任务），不要 `nohup … &` 后退出，进程会被回收。
- 与其他会话同时跑仿真时，用 `ROS_DOMAIN_ID` 和 `GZ_PARTITION` 隔离，避免话题串线。

## 验证原则

- 用户要求**可视化验证**：每完成一项，给出用户能亲眼看到效果的命令和合格标准。
- headless 模式（`headless:=true`）2026-09-25 实测新展馆实时率约 0.43，可以跑自动验收；看效果仍用图形窗口。
- **Gazebo 渲染内存泄漏（WSL GPU 转译层）**：机器人静止时，新展馆图形窗口约 +21 MB/s、仿真主进程约 +8 MB/s（headless 约 +10 MB/s），简单展馆约 +4 / +1.4 MB/s。WSL 只有 7.5GB，新展馆开窗口约 3 分钟就会被 OOM 杀掉，WSL 服务也可能随之报 `E_UNEXPECTED`。长时间运行用 headless，或分段重启；排查看 `dmesg | grep -i oom`。

## 渲染配置

- 当前使用 `ogre` 并关闭阴影（流畅优先）。恢复高画质：把 SDF 里的 `ogre` 改为 `ogre2`，`shadows` 改为 `true`。

## 常见错误

| 错误 | 原因 | 解决 |
|------|------|------|
| `gazebo: command not found` | 新版命令是 `gz sim` | 用 `gz sim` |
| `Package 'g1_guide' not found` | 没有构建或没有 source | 运行 `build.sh` 后重开终端 |
| Gazebo 卡顿 | GPU 路径未启用 | 验证：`glxinfo \| grep renderer` |
| 改了代码仿真没变化 | 新增了文件或改了 URDF 生成器，但没重新构建 | 运行 `build.sh` |
| 搬文件夹后启动报找不到文件 | 软链接指向旧位置 | 删掉 `~/.local/share/g1-guide-runtime` 后重新运行 `build.sh` |
| Gazebo 黑屏、Entity Tree 空、报 `SceneBroadcaster: Timed out waiting for state` | 物理线程卡在加载期。已知诱因：`<actor><skin><filename>` 的 Fuel 路径带空格（如 `male%20visitor`），网格解析器不报错直接挂住 | 换路径不含空格的模型。`validate_layout` 已拦截 `skin_uri` 里的空格；定位其他诱因用 `tools/hall/bisect_world.py` |
