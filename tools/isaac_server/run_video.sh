#!/bin/bash
export TERM=xterm  # isaaclab.sh 开头调用 tabs，TERM 为空或 dumb 会直接退出
# 试录 Isaac 演示视频（需要相机渲染；AutoDL 容器里 Vulkan 用不了显卡，可能失败）
D=/root/isaac
source /root/miniconda3/etc/profile.d/conda.sh
conda activate $D/envs/isaaclab
cd $D/IsaacLab
timeout 360 ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py --task Isaac-Velocity-Rough-G1-Play-v0 \
  --headless --enable_cameras --video --video_length 300 --num_envs 16 --checkpoint "$1"
echo "== 退出码 $?"
