#!/bin/bash
export TERM=xterm  # isaaclab.sh 开头调用 tabs，TERM 为空或 dumb 会直接退出
# G1 崎岖地形（楼梯、坡、方块、起伏）PPO 训练
# 用法：train_g1_rough.sh <机器人数> <mini-batch 个数> <轮数> <运行名>
N=${1:-16384}; MB=${2:-8}; IT=${3:-1500}; NAME=${4:-n${N}_mb${MB}}
D=/root/isaac
source /root/miniconda3/etc/profile.d/conda.sh
conda activate $D/envs/isaaclab
cd $D/IsaacLab
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Velocity-Rough-G1-v0 --headless \
  --num_envs $N --max_iterations $IT --run_name $NAME agent.algorithm.num_mini_batches=$MB
echo "== 退出码 $?"
