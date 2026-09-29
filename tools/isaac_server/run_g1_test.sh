#!/bin/bash
export TERM=xterm  # isaaclab.sh 开头调用 tabs，TERM 为空或 dumb 会直接退出
# 试跑 G1 崎岖地形训练 30 轮，测速度、确认 G1 资源能下载
source /root/miniconda3/etc/profile.d/conda.sh
conda activate /root/isaac/envs/isaaclab
cd /root/isaac/IsaacLab
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Velocity-Rough-G1-v0 --headless --num_envs 4096 --max_iterations 30
echo "== 退出码 $?"
