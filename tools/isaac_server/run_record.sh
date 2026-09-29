#!/bin/bash
export TERM=xterm  # isaaclab.sh 开头调用 tabs，TERM 为空或 dumb 会直接退出
# 用法：run_record.sh <checkpoint> <输出目录> [更多参数]
D=/root/isaac
source /root/miniconda3/etc/profile.d/conda.sh
conda activate $D/envs/isaaclab
cd $D/IsaacLab
./isaaclab.sh -p $D/record_demo.py --checkpoint "$1" --out "$2" "${@:3}"
echo "== 退出码 $?"
