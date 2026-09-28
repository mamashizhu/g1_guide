# 云服务器 Isaac Lab 环境（AutoDL）

用于训练 G1 上坡、上楼梯。2026-09-28 在 AutoDL 北京 B 区 RTX 4090 上装好。

- 登录：`ssh -p 53309 root@connect.bjb2.seetacloud.com`（公钥认证，本机 `~/.ssh/id_rsa`）。实例重开后端口可能会变。
- 版本：Isaac Sim 5.1.0、Isaac Lab v2.3.2、Python 3.11、torch 2.7.0+cu128、rsl-rl-lib 3.1.2
- 位置：全部在数据盘 `/root/autodl-tmp`（系统盘只有 30 GB）。conda 环境 `envs/isaaclab`，源码 `IsaacLab/`，`~/.cache` 软链到数据盘。
- 进入环境：`source /root/miniconda3/etc/profile.d/conda.sh && conda activate /root/autodl-tmp/envs/isaaclab && cd /root/autodl-tmp/IsaacLab`。环境变量（EULA、TMPDIR、版本锁）已写进 conda 环境，激活后自动生效。

## 实测速度（2026-09-29）

`Isaac-Velocity-Rough-G1-v0`，4096 个机器人，无画面：46000 步/秒，每轮 2.13 秒。默认 3000 轮约 1.8 小时。试跑脚本 `/root/autodl-tmp/run_g1_test.sh`（30 轮）。

## 文件

| 文件 | 服务器上的位置 | 作用 |
|------|--------------|------|
| `pip.conf` | `~/.config/pip/pip.conf` | pip 用清华源 |
| `install_isaaclab.sh` | `/root/autodl-tmp/` | 装 torch、Isaac Sim、克隆 Isaac Lab |
| `fix_isaaclab.sh` | `/root/autodl-tmp/` | 带版本锁装 Isaac Lab 各模块 |
| `constraints.txt` | `/root/autodl-tmp/` | 锁住 torch 2.7.0+cu128、packaging 23.0 |

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
