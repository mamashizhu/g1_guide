#!/bin/bash
export TERM=xterm  # isaaclab.sh 开头调用 tabs，TERM 为空或 dumb 会直接退出
# 续装 Isaac Lab。constraints.txt 锁住 Isaac Sim 5.1 需要的 torch 2.7.0+cu128 和 packaging 23.0，
# 否则 isaaclab_rl[all] 的依赖会把 torch 升到 CUDA 13 新版。GitHub 依赖走学术加速（git 全局代理只对 github.com）。
set -e
D=/root/isaac
export TMPDIR=$D/tmp PIP_CONSTRAINT=$D/constraints.txt PIP_FIND_LINKS=$D/wheels
source /root/miniconda3/etc/profile.d/conda.sh
conda activate $D/envs/isaaclab
pip install "packaging==23.0"
cd $D/IsaacLab
./isaaclab.sh --install
python -c "import torch, isaaclab, isaaclab_tasks, isaaclab_rl, rsl_rl; print(\"import ok, torch\", torch.__version__)"
pip check | grep -iE "torch|packaging|isaac" || true
echo "== 全部完成"
df -h / $D
