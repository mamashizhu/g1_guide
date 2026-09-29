#!/bin/bash
export TERM=xterm  # isaaclab.sh 开头调用 tabs，TERM 为空或 dumb 会直接退出
# 测速：G1 崎岖地形，机器人数量 $1，跑 20 轮，每秒记录 GPU/CPU 占用到 bench_<N>_mon.csv
N=${1:-8192}
D=/root/isaac
source /root/miniconda3/etc/profile.d/conda.sh
conda activate $D/envs/isaaclab
cd $D/IsaacLab
nvidia-smi --query-gpu=timestamp,utilization.gpu,memory.used,power.draw --format=csv,noheader -l 1 > $D/bench_${N}_gpu.csv &
GM=$!
( while true; do top -bn1 | awk "NR>7 && \$9>5 {printf \"%s %s%% \", \$12, \$9} END{print \"\"}" | sed "s/^/$(date +%T) /"; sleep 1; done ) > $D/bench_${N}_cpu.log &
CM=$!
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Velocity-Rough-G1-v0 --headless --num_envs $N --max_iterations 20
echo "== 退出码 $?"
kill $GM $CM
