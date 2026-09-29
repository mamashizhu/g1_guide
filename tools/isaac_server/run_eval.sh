#!/bin/bash
export TERM=xterm  # isaaclab.sh 开头调用 tabs，TERM 为空或 dumb 会直接退出
# 用法：run_eval.sh <checkpoint> <terrain> <value> [更多参数]
D=/root/isaac
source /root/miniconda3/etc/profile.d/conda.sh
conda activate $D/envs/isaaclab
cd $D/IsaacLab
./isaaclab.sh -p $D/eval_terrain.py --checkpoint "$1" --terrain $2 --value $3 "${@:4}"
