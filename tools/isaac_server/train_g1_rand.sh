#!/bin/bash
export TERM=xterm  # isaaclab.sh 开头调用 tabs，TERM 为空或 dumb 会直接退出
# G1 崎岖地形 + 放宽的机身高度奖励 + 域随机化（g1_custom/g1_rand_cfg.py）
# noise_std_type=log：动作噪声标准差用对数参数化，不会变负（第一次试训在第 90 轮因 std 无效崩溃）
# 用法：train_g1_height.sh <机器人数> <mini-batch 个数> <轮数> <运行名>
N=${1:-16384}; MB=${2:-8}; IT=${3:-1500}; NAME=${4:-rand_n${N}_mb${MB}}
D=/root/isaac
source /root/miniconda3/etc/profile.d/conda.sh
conda activate $D/envs/isaaclab
cd $D/IsaacLab
./isaaclab.sh -p $D/g1_custom/train_custom.py --task G1-Rough-Rand-v0 --headless \
  --num_envs $N --max_iterations $IT --run_name $NAME agent.algorithm.num_mini_batches=$MB agent.policy.noise_std_type=log
echo "== 退出码 $?"
