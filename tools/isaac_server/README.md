# 云服务器 Isaac Lab 环境（AutoDL）

用于训练 G1 上坡、上楼梯。2026-09-28 在 AutoDL 北京 B 区 RTX 4090 上装好。

- 登录：`ssh -p 53309 root@connect.bjb2.seetacloud.com`（公钥认证，本机 `~/.ssh/id_rsa`）。实例重开后端口可能会变。
- 版本：Isaac Sim 5.1.0、Isaac Lab v2.3.2、Python 3.11、torch 2.7.0+cu128、rsl-rl-lib 3.1.2
- 位置（2026-09-29 起）：**系统盘 `/root/isaac`**，为了能保存 AutoDL 镜像（镜像只含系统盘）。conda 环境 `/root/isaac/envs/isaaclab`（20 GB），源码和训练记录 `/root/isaac/IsaacLab`，脚本在 `/root/isaac/*.sh`。系统盘共用约 21 GB / 30 GB。
  - 数据盘 `/root/autodl-tmp` 上还留着搬迁前的原件（环境、缓存、torch 安装包），新实例没有这些也能用。
  - 临时文件：有数据盘就放 `/root/autodl-tmp/tmp`，没有就放 `/root/isaac/tmp`（conda 环境的 `activate.d/tmpdir.sh`）。**装大包时一定要有数据盘**，否则会撑满系统盘。
- 进入环境：登录后输入 `isaac`（`~/.bashrc` 里的别名），或 `source /root/miniconda3/etc/profile.d/conda.sh && conda activate /root/isaac/envs/isaaclab && cd /root/isaac/IsaacLab`。EULA 和版本锁已写进 conda 环境变量。
- 2026-09-29 已从镜像克隆到新实例（RTX 4090 D，驱动 595.71），`ssh -p 43659 root@connect.bjb2.seetacloud.com`。系统盘内容完整带过去，数据盘是新的空盘。
- **搬到系统盘后的坑**：conda 的二进制程序（`tabs`、openssl 等）把旧路径 `/root/autodl-tmp/envs/isaaclab` 编译在里面，文本替换改不到。旧机器上数据盘原件还在所以没暴露，新机器上 `isaaclab.sh` 第一步 `tabs 4` 就报 `terminals database is inaccessible` 退出。修复在 `conda_activate_tmpdir.sh`（服务器上是环境的 `etc/conda/activate.d/tmpdir.sh`）：每次激活时在旧路径建软链接指回 `/root/isaac`，并设 `TERMINFO`。另外 SSH 远程执行时 `TERM=dumb`，所以 `*.sh` 里都强制 `export TERM=xterm`。
- 新实例从镜像启动后仍要做的：`git config --global http.https://github.com.proxy` 里的学术加速代理地址可能变了，要重新从 `/etc/network_turbo` 取。

## 常用命令（服务器上）

```bash
bash /root/isaac/train_g1_rough.sh 16384 8 1500 <运行名>                 # 训练：机器人数、mini-batch 数、轮数
bash /root/isaac/eval_suite_full.sh <model_*.pt> /root/isaac/eval.jsonl  # 导出策略 + 12 项固定难度地形评估
bash /root/isaac/run_eval.sh <model_*.pt> stairs_up 0.18 --out <文件>    # 单项评估
```

训练记录在 `/root/isaac/IsaacLab/logs/rsl_rl/g1_rough/<日期>_<运行名>/`，导出的策略在其中 `exported/`（`policy.pt`、`policy.onnx`、`deploy.json`）。

## 实测速度（2026-09-29）

`Isaac-Velocity-Rough-G1-v0`，无画面：

| 机器人数 | 每秒步数 | 每轮 | GPU 平均占用 | 显存 |
|---------|---------|------|-------------|------|
| 4096 | 46,000 | 2.13 s | — | — |
| 8192 | 76,800 | 2.56 s | 47% | 8.3 GB |
| 16384 | 110,000 | 3.58 s | 61% | 9.1 GB |

机器人太少时 GPU 吃不满；16384 个以上涨幅变小，瓶颈转到 CPU 一侧。

## 第一次训练结果（2026-09-29，`n16384_mb8`）

16384 个机器人、mini-batch 8 个、1500 轮，用时 1 小时 43 分钟。本地副本在 `mujoco/policies/g1_rough_n16384_mb8/`。
固定难度地形评估（`eval_terrain.py`，64 个机器人，指令 0.5 m/s，20 秒，走出台阶或坡区且没摔倒算通过）：

| 地形 | 第 1000 轮 | 第 1499 轮 |
|------|----------|----------|
| 上 10 / 15 / 18 cm 台阶 | 100% / 100% / 100% | 100% / 100% / 98.4% |
| 上 23 cm 台阶 | 98.4% | 100% |
| 上 25 cm 台阶（超出训练范围） | — | 100% |
| 下 15 cm 台阶 | 100% | 98.4% |
| 下 23 cm 台阶 | — | 96.9%（摔倒 3.1%） |
| 上 10° / 15° / 20° 坡 | 98.4% / 100% / 100% | 100% / 100% / 98.4% |
| 18 cm 台阶，1.0 m/s | — | 100% |
| 平地，1.0 m/s | — | 100% |

**步态问题**：这个策略一直**蹲着走**，骨盆离地只有约 0.48 m（直立约 0.74 m）。Isaac Lab 的 G1 崎岖地形任务没有"保持身高"的奖励，策略自己学成了半蹲。要改得在奖励里加机身高度项再训。

**演示视频**：Isaac 自己录不了（相机渲染要 Vulkan，`play.py --video` 卡在渲染初始化，超时退出）。改用 `record_demo.py` 记录每帧位姿和地形网格，本地 `mujoco/replay_demo.py` 回放渲染成 mp4（`mujoco/policies/g1_rough_n16384_mb8/demo/demo.mp4`）。

## 第二次训练：加机身高度奖励（2026-09-29，`height072_n16384_mb8`）

任务 `G1-Rough-Height-v0`（`g1_custom/`，不改 Isaac Lab 源码，`train_custom.py` 注册后调原版 train.py）：在原版基础上只加一项骨盆离地高度惩罚（目标 0.72 m，权重 −10，地面 = 高度扫描有效命中点平均，单步惩罚限幅 1）。另加 `agent.policy.noise_std_type=log`。本地副本 `mujoco/policies/g1_rough_height072_n16384_mb8/`，演示视频在其中 `demo/demo.mp4`。

- 第一次试训用 Isaac Lab 自带的 `mdp.base_height_l2`，约 90 轮崩溃（`normal expects all elements of std >= 0.0`）：没打中的射线返回 inf，惩罚变无穷。改成自己的 `base_height_l2_safe` 后正常。
- 用了 log_std 的 checkpoint 里存的是 `log_std` 而不是 `std`，评估、导出、录制脚本已改为自动识别。
- 训练曲线：平均奖励 33.3（原版 31.1），速度误差 0.26 m/s（原版 0.33），地形等级 5.76（原版 5.71），高度偏差约 5 cm。

| 地形（0.5 m/s） | 新版通过率 | 骨盆离地 | 原版通过率 | 原版骨盆离地 |
|------|------|------|------|------|
| 平地 | — | 0.686 m（第 500 轮） | 100% | 0.49 m |
| 上 10 / 15 / 18 / 23 cm 台阶 | 96.9% / 100% / 100% / 98.4% | 0.68 m | 100% / 100% / 98.4% / 100% | 0.56 m（18 cm） |
| 上 25 cm 台阶 | 96.9%（摔 3.1%） | 0.69 m | 100% | |
| 下 15 / 23 cm 台阶 | 100% / 98.4%（不摔） | 0.69 m | 98.4% / 96.9%（摔 3.1%） | |
| 上 10° / 15° / **20°** 坡 | 98.4% / 100% / **78.1%**（不摔，没爬上去） | 0.67–0.68 m | 100% / 100% / 98.4% | |
| 18 cm 台阶 1.0 m/s、平地 1.0 m/s | 100% / 100% | 0.68 / 0.67 m | 100% / 100% | |

结论：站直了（骨盆 0.49 → 0.68 m），平地和台阶基本不退步，**20° 陡坡明显变差**（重心高了，爬不上去但不摔）。

注意：平均地形等级在 6 级附近不再上升，**不代表学不动了**。通关最高级的机器人会被送回随机等级，满分策略的平均等级也只有约 4.5。

## 文件

| 文件 | 服务器上的位置 | 作用 |
|------|--------------|------|
| `pip.conf` | `~/.config/pip/pip.conf` | pip 用清华源 |
| `constraints.txt` | `/root/isaac/` | 锁住 torch 2.7.0+cu128、packaging 23.0 |
| `train_g1_rough.sh` | `/root/isaac/` | 训练 |
| `eval_terrain.py`、`run_eval.sh`、`eval_suite*.sh` | `/root/isaac/` | 固定难度地形评估 |
| `export_policy.py`、`run_export.sh` | `/root/isaac/` | 导出策略和 sim2sim 参数（`deploy.json`） |
| `run_g1_test.sh`、`run_g1_bench.sh` | `/root/isaac/` | 试跑 30 轮、测速并记录 GPU/CPU 占用 |
| `migrate_to_sysdisk.sh` | `/root/autodl-tmp/` | 从数据盘搬到系统盘（已执行过） |
| `install_isaaclab.sh`、`fix_isaaclab.sh` | — | 当初在数据盘上的安装过程，路径已被替换成 `/root/isaac`，**重装前先把路径和 TMPDIR 改回数据盘** |

## 踩过的坑

1. **阿里云镜像对 HTTP/1.1 限速到约 0.7 MB/s**，HTTP/2 有 10 MB/s。pip 和 Python 只用 HTTP/1.1，所以 curl 快、pip 慢。pip 改用清华源（HTTP/1.1 下 9 MB/s）。
2. `pypi.nvidia.com` 会跳转到国内的 `pypi.nvidia.cn`，HTTP/1.1 下 7.5 MB/s，可以直接用。
3. cu128 版 torch 只有阿里云的 pytorch-wheels 镜像有，用 curl 先下到 `/root/autodl-tmp/wheels`，再让 pip 从本地装。
4. pip 临时文件默认在系统盘 `/tmp`，装 Isaac Sim 会撑满 30 GB 系统盘，要设 `TMPDIR` 到数据盘。
5. `flatdict` 编译要 `pkg_resources`，而 setuptools ≥ 81 已经删了它：先装 `setuptools<81`，再 `pip install flatdict==4.0.1 --no-build-isolation`。
6. `isaaclab_rl[all]` 的依赖会把 torch 升到 CUDA 13 新版，破坏 Isaac Sim。必须带 `PIP_CONSTRAINT=constraints.txt`。
7. GitHub 依赖（`rl_games`）直连会卡住：用 `git config --global http.https://github.com.proxy <学术加速代理>`，代理地址在 `/etc/network_turbo` 里。
8. 远程执行时 `~/.bashrc` 里的 `export` 不生效（Ubuntu 的 `.bashrc` 非交互直接返回），所以环境变量写进了 conda 环境（`conda env config vars`）。
9. 杀进程别用 `pkill -f <关键字>`：远程命令行本身也包含那个关键字，会把自己的 shell 一起杀掉。先列出 PID，再单独 `kill`。
10. **容器缺 `libGLU.so.1` 和 `libXt.so.6`**：Isaac Sim 的材质库（`libneuray`、MaterialX）加载失败，训练启动 39 秒后卡在材质加载上不动。修复：`apt-get install -y libglu1-mesa libxt6 libsm6 libice6 libxrandr2 libxinerama1 libxcursor1 libxi6 libxkbcommon0`。**换实例或重装系统盘后要重新装。**
11. **Vulkan 用不了 NVIDIA 显卡**（`vk_icdGetInstanceProcAddr` 拿不到 `vkCreateInstance`，退回 CPU 的 llvmpipe）：驱动库齐全、版本一致，手动建 `/dev/nvidia-modeset` 也没用，原因没查清。**不影响无画面训练**（物理在 CUDA 上跑，日志里的 `Unable to get IGpuFoundation` 警告可以忽略），但录视频、相机观测可能会很慢或失败。
12. 不在 Isaac Sim 应用里直接 `import isaaclab_tasks` 会报 `No module named 'pxr'`，这是正常现象，要通过 `isaaclab.sh -p` 脚本启动。
