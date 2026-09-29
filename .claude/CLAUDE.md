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
ros2 topic pub --once /guide/gesture std_msgs/String "data: wave"   # 讲解手势（exhibition_nav 已带 gesture_player）
ros2 run g1_guide g1_teleop                     # WASD
ros2 run g1_guide sensor_dashboard              # 传感器面板
ros2 launch g1_guide lowlevel.launch.py         # 低阶关节控制（骨盆固定）
ros2 launch g1_guide slam.launch.py explore:=true rviz:=true   # SLAM 自动巡航建图，地图存到 maps/
ros2 launch g1_guide slam_nav.launch.py rviz:=true              # SLAM 地图导航（真机雷达 + 漂移 + AMCL）
gz sim                                          # 只开 Gazebo
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

- 平移模型（`make_glide_urdf.py`）：腿和腰焊死（fixed），身体是一个刚体，由 VelocityControl 直接设定速度；base_link **必须关闭重力**，否则会倒或"起飞"。左右臂 14 个关节保持 revolute，手臂连杆也关闭重力，每个关节由 gz `JointPositionController`（速度指令模式）保持在 `/g1/arm/<关节>/cmd_pos` 给的角度。
- 低阶模型（`make_controlled_urdf.py`）：29 个关节，骨盆固定，ros2_control 做力矩 PD。
- 讲解手势（PLAN M2.3）：`gesture_player` 订阅 `/guide/gesture`，动作定义全在 `config/gestures.yaml`（角度按左臂约定写，右臂自动镜像）。`home` 姿势也是 Gazebo 里手臂的初始角，改了 home 要重新 `build.sh`。行走中（`/nav/status` 为 moving 等状态）只允许 home；到站后自动指向展品。指向用 `arm_kinematics.py` 的 IK，不要再用旧的 `pointing_kinematics.py`。

## 展馆模型要点

- 展馆是导入的 Fuel 网格 `OpenRobotics/iscas_museum`（已缓存在 `~/.gz/fuel`，离线可用）。**它只有墙，没有地面也没有屋顶**，地面仍由 `exhibition_world.py` 生成。
- `nav_grid` 按矩形规划，吃不了网格。墙体已离线烘焙成 `config/hall_nav_obstacles.json`（320 个矩形）。**改了展馆模型必须重新烘焙并验证**，流程见 `docs/exhibition-hall.md`。
- 改布局坐标不要靠读图估，用 `tools/hall/check_layout.py` 对网格实测，再用 `tools/hall/check_tour.py` 确认 6 个展位仍可达。
- 行走的 actor 没有碰撞体，但 `gpu_lidar` 按渲染测距，**导航雷达能扫到它**（2026-09-26 在 2 号展位旁看到它路线上的回波）。北厅 actor 沿 y=9.2 走，正好穿过 1、2 号展位，会让机器人临时 blocked / 后退让路。

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

## 导航测试（先离线，后 Gazebo）

- 决策逻辑在 `nav_core.NavigatorCore`（与 ROS 无关），`exhibition_nav` 节点只做收发。改导航先跑离线仿真，同一份代码：
  `python3 tools/nav_sim.py`（简单展馆全部展位两两互达，约 20 s）；`--scene hall`（大展馆，约 3 分钟）；`--tour 3 1 --box-ahead 2:1.3 -v`（行进中前方放箱子并打印状态）。自动判卡死：20 s 没离开 0.5 m 范围。
- 离线全过后，再用 `bash tools/nav_gz_tour.sh <名> 4 5` 在 Gazebo 里确认（慢，日志在 `.navrun/`）。
- 已知的锁死机制（2026-09-26 修复）：①前方 < 0.8 m 时连原地转也禁止；②起点被动态点吞没时规划永远失败；③书、花瓶展台只有 0.9 m 高，雷达看不到，跟踪时切角进入其膨胀区，重新规划后永远失败。现在分别靠原地转身、后退脱困（`back_off`）、起点吸附到最近可通行点（`plan_with_escape`）处理。
- 到站指向手势会把手臂抬进雷达平面（前方约 0.45 m 的回波），出发时会先后退一小段，属正常。

### 后退脱困（`back_off`）算法说明

目前只在仿真里验证过。**上真机前先不要依赖它**，必须按下面的“真机前待确认”逐项核对。代码：`nav_control.back_off`（纯函数）+ `nav_core.NavigatorCore._start_back_off / _back_off`。

- **什么时候触发**：规划失败，或跟踪中因前方净空 < 0.8 m 被迫停车（`blocked`）时，检查前方 ±60°（`BACK_OFF_HALF_ANGLE` = 1.05 rad）内最近的雷达回波。距离小于“规划净空 + 0.1 m”（0.55 + 0.30 + 0.10 = 0.95 m，`BACK_OFF_MARGIN`）就进入后退，状态 `backing_up`。
  - 为什么需要：离障碍物比规划净空还近时，机器人起点落在障碍的膨胀区里，A* 永远规划不出路；前方太近又不能前进。以前就这样一直锁死。
- **每一步怎么做**：直线倒退 `REVERSE_SPEED` = 0.15 m/s，不转向。每个控制周期（10 Hz）重新检查：
  - 前方 ±60° 最近回波 ≥ 0.95 m → 停止后退，立即重新规划；
  - 后方 ±30°（`forward_clearance` 朝向 yaw+π，默认半角 0.52 rad）最近回波 < `REAR_STOP_DISTANCE` = 0.6 m → 停止后退（后面也不通，后退帮不上），按 `REPLAN_RETRY_S` = 1 s 的节奏继续尝试规划、等路让开；
  - 输入有 NaN 或负数 → `invalid`，停车。
- **与其他脱困手段的分工**：周围 0.5 m 内没东西时允许原地转身（`ROTATE_CLEARANCE`）；起点落在**静态**障碍的膨胀区（雷达看不到的矮展台）时，由 `plan_with_escape` 把起点吸附到最近的可通行点；只有被**动态**障碍（人、箱子、自己的手臂）逼得太近时才后退。
- **真机前待确认**：
  1. 真机 MID-360 装在头部，要确认它对**正后方和低处**有没有盲区。后方看不全时，后退必须关掉，或限制为只退一小段固定距离（例如 0.3 m）。
  2. 真机倒退步态的稳定性和实际速度，0.15 m/s 是否合适。
  3. 后方 ±30° 的检查范围对 G1 的身体宽度（约 0.5 m）够不够，是否需要放宽到 ±45° 以上。
  4. 真机点云稀疏、噪声大，前方回波判断会不会让机器人来回“进一步退一步”。

## 验证原则

- 用户要求**可视化验证**：每完成一项，给出用户能亲眼看到效果的命令和合格标准。
- headless 模式（`headless:=true`）2026-09-25 实测新展馆实时率约 0.43，可以跑自动验收；看效果仍用图形窗口。
- **Gazebo 渲染内存泄漏（WSL GPU 转译层）**：机器人静止时，新展馆图形窗口约 +21 MB/s、仿真主进程约 +8 MB/s（headless 约 +10 MB/s），简单展馆约 +4 / +1.4 MB/s。WSL 只有 7.5GB，新展馆开窗口约 3 分钟就会被 OOM 杀掉，WSL 服务也可能随之报 `E_UNEXPECTED`。长时间运行用 headless，或分段重启；排查看 `dmesg | grep -i oom`。
  - 2026-09-25 定位（`tools/mem_probe.sh`）：涨的全是进程堆内存；只开展馆不放机器人时主进程不涨、窗口照涨；改 `LIBGL_ALWAYS_SOFTWARE=1` 后两边都不涨 → 泄漏在 WSL 的 d3d12 显卡转译层，不是我们的代码。上游同类问题未修复：gazebosim/gz-sim#2873、PX4/PX4-Autopilot#27296。
  - 全部软件渲染实时率只有 0.005，不能用。
- **2026-09-29 已处理，大展馆长时间运行不再越来越慢**（`tools/nav_gz_tour.sh` 同一路线 1→4→1→4 实测，每秒速度和实时率由 `tools/nav_speed_log.py` 记录）：
  - 真正拖慢的主因是**物理碰撞**：Fuel 的 iscas_museum 用整个 8 MB 显示网格做碰撞，机器人约 20 个网格碰撞体每步都要和它做检测。走到墙面复杂的区域，实时率就掉到约 0.2（三次运行的曲线完全一致，只和位置有关）。现在 `exhibition.json` 的 `hall.collision: baked`：显示仍用原网格，碰撞改用烘焙好的 320 个墙体盒子（和规划用的是同一份）。雷达按渲染测距，看到的不变。改回原样用 `collision: mesh`。效果：实时率稳定在 0.47–0.49（开窗口 0.40–0.44），每段路墙钟时间 162–192 s → 105–111 s。
  - 渲染泄漏：`config/bridge.yaml` 的 `/camera/*` 和 `/scan`、俯视相机 `/exhibition/overview` 都设为 `lazy`，没人订阅时 Gazebo 不渲染这些传感器。仿真主进程内存增长从约 8 MB/s 降到约 1 MB/s（10 分钟约 1.3 GB）。头部相机 `camera_rate` 默认 10 Hz（模型里是 30 Hz，启动时改）。**以后做视觉订阅相机时，渲染和泄漏会回来，帧率能低就低。**
  - 窗口：**用户决定（2026-09-29）窗口保持显卡渲染**（默认 `gui_render:=gpu`，和原来一样是单个 `gz sim` 进程）。窗口自身仍有约 21 MB/s 的泄漏，长时间开窗口要定期重启。可选 `gui_render:=software`：服务端 `gz sim -s` 加软件渲染窗口 `gz sim -g`（`LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe`），窗口不泄漏，但大展馆里帧率很低（窗口进程约占 3 个核仍然卡顿），不要设为默认。

## MuJoCo 实验（2026-09-28）

- 独立于 ROS/Gazebo，在 **Windows 端**运行：双击 `mujoco\run_g1_walk.bat`，或 `D:\python\python.exe mujoco\g1_walk.py`；无窗口自检加 `--check`。
- 用宇树 `unitree_rl_gym` 的 12 自由度模型 `scene.xml` 和预训练策略 `deploy/pre_train/g1/motion.pt`，参数读它的 `deploy_mujoco/configs/g1.yaml`。仓库克隆在 `mujoco/unitree_rl_gym/`，已加入 `.gitignore`。
- 官方 `deploy_mujoco.py` 依赖 `legged_gym`/Isaac Gym，Windows 和 WSL 都装不了，**不要直接运行它**，用 `g1_walk.py`。
- 斜坡楼梯场地 `mujoco/terrain.py`：用 `MjSpec` 在宇树 `scene.xml` 上加几何体（不改第三方文件），`g1_walk.py --course <跑道>` 加载；`python mujoco/terrain.py` 用射线核对尺寸。改了场地要重新跑核对。平地策略在 6 条跑道上全部不合格，这是基线。
- 摔倒判定：机身倾斜超过 60°，或骨盆离脚下地面（按跑道设计高度算）< 0.45 m。
- 策略的速度指令范围：vx −0.5～1.0，vy ±0.5，wz ±1.0，超出会走不稳。
- `.bat` 文件只写 ASCII：cmd 按 GBK 读批处理，中文注释会被当成命令执行。
- 训练新步态（上坡、上楼梯）：用户决定**本地不装 Isaac**（本机低于 Isaac Sim 5.1 最低配置：要求 RTX 4080 16GB、32GB 内存、驱动 ≥580.88，本机 4060 8GB、15GB、566.26）。Isaac Lab v2.3.x（Isaac Sim 5.1、Python 3.11）只装在租的 Linux 服务器上训练，训练好的策略拿回本地 MuJoCo 验证。服务器显卡必须带 RT 核心（4090/L40/A6000，不能用 A100/H100）。
- 服务器环境已装好（2026-09-28，AutoDL RTX 4090，`ssh -p 53309 root@connect.bjb2.seetacloud.com`，公钥登录）。安装脚本、版本锁和踩坑记录在 `tools/isaac_server/`。要点：pip 用清华源（阿里云限速 HTTP/1.1）；装任何包都要带 `PIP_CONSTRAINT`，否则 torch 会被升级掉；远程杀进程先查 PID 再 kill，不要 `pkill -f`；容器缺 `libglu1-mesa libxt6`，不装会卡在材质加载；Vulkan 用不了 NVIDIA 卡，但不影响无画面训练。实测 G1 崎岖地形 4096 个机器人每轮 2.13 s，3000 轮约 1.8 h。
- 2026-09-29 起环境在**系统盘 `/root/isaac`**（为了保存 AutoDL 镜像），数据盘只放临时文件。第一次训练（16384 个机器人、mini-batch 8、1500 轮）在 Isaac 里上 25 cm 台阶、20° 坡通过率都在 98% 以上，策略和评估结果在 `mujoco/policies/g1_rough_n16384_mb8/`。
- sim2sim 用的 MuJoCo 模型：Isaac Lab 的 G1（`g1_minimal`，37 关节：`torso_joint`、`elbow_pitch/roll`、手指 `zero~six`）对应 MuJoCo Menagerie 2024-07 版（commit `cddb24f0db`），已下载到 `mujoco/menagerie_g1_2024/`（不入库，用 `gh api` 下载，本机 git 直连 GitHub 会 SSL 握手失败）。宇树现在的模型是 29 关节新命名，对不上。策略观测 310 维（含 187 点地形高度扫描和机身线速度），动作缩放 0.5，控制 50 Hz，参数见 `exported/deploy.json`。这个策略学成了**半蹲步态**（骨盆约 0.48 m，Isaac 和 MuJoCo 都一样，不是移植错误）；在 MuJoCo 里 6 条跑道只过 1 条（2026-09-29，`mujoco/g1_rough_walk.py --check-all`），sim2sim 差距未查清。看 Isaac 里的真实表现：服务器 `record_demo.py` 记录位姿 → 本地 `mujoco/replay_demo.py <demo目录>` 渲染 mp4（Isaac 自己录不了视频）。当前服务器是 2026-09-29 从镜像克隆的新实例，端口 43659。第二次训练加了机身高度奖励（自定义任务 `G1-Rough-Height-v0`，在 `tools/isaac_server/g1_custom/`，不改 Isaac Lab 源码），骨盆离地 0.49 → 0.68 m，台阶和平地基本不退步，20° 坡通过率 98% → 78%；结果在 `mujoco/policies/g1_rough_height072_n16384_mb8/`，对比见 `tools/isaac_server/README.md`。高度奖励不能用 Isaac Lab 自带的 `base_height_l2`（射线没打中返回 inf，训练约 90 轮崩溃）。第三版（`g1_rough_rand070_n16384_mb8`，身高目标 0.70/权重 −5 + 摩擦、质量、质心、推力、PD 增益随机化）在 Isaac 13 项测试全部 ≥98.4% 且零摔倒，骨盆约 0.64 m，是目前最好的。sim2sim（2026-09-29）：`g1_rough_walk.py` 必须按训练时的方式用朝向误差算转向指令（`heading`，系数 0.5），否则机器人会偏出跑道；物理步长 0.005 s 时 MuJoCo 接触不准，默认改成 0.002 s。第三版在 MuJoCo 平地能走，6 条跑道过 3 条（10°、15° 坡和 10 cm 台阶），15/18 cm 台阶摔倒、5° 坡往 −y 偏出跑道，未查清。本机 Git Bash 调 ssh 要加 `MSYS_NO_PATHCONV=1`，并用 Windows 的 `ssh.exe`（Git 自带的 ssh 因用户名是中文找不到 `~/.ssh`）。

## 渲染配置

- 注意：展馆世界（`exhibition_world.py` 生成）的传感器渲染用的是 `ogre2` 并开了阴影，和下面这条不一致；下面这条说的是其他世界文件。

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
