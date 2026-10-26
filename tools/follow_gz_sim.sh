#!/usr/bin/env bash
# 跟随端到端测试的 WSL 端：简单展馆 + 真机雷达版 + 前置相机 JPEG + rosbridge（端口 9097）+ set_pose 服务桥。
#   bash tools/follow_gz_sim.sh <输出名>          # 启动（无窗口），直到 .navrun/<输出名>/stop 出现（最长 20 分钟）
#   GUI=1 bash tools/follow_gz_sim.sh <输出名>    # 同上，开 Gazebo 窗口，亲眼看机器人跟人走
#   bash tools/follow_gz_sim.sh leader <输出名> [--route short|walk]   # 另开终端：放领路人、发 follow、判定
# Windows 端运行 vision\watch.py --port 9097 [--show]（YOLO）。
# 只结束本脚本启动的进程（独立进程组）；用 ROS_DOMAIN_ID / GZ_PARTITION 和其他会话隔开。
source /opt/ros/lyrical/setup.bash
source /home/ma/.local/share/g1-guide-runtime/install/setup.bash
export LD_LIBRARY_PATH=/usr/lib/wsl/lib:$LD_LIBRARY_PATH MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA GALLIUM_DRIVER=d3d12
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-86} GZ_PARTITION=${GZ_PARTITION:-followtest}

if [ "$1" = leader ]; then
  shift
  exec python3 /mnt/d/g1_guide/tools/follow_gz_leader.py "$@"
fi

scene=$(ros2 pkg prefix g1_guide)/share/g1_guide/config/${SCENE:-exhibition_simple}.json
headless=true
[ "${GUI:-0}" = 1 ] && headless=false

out=/mnt/d/g1_guide/.navrun/${1:-follow}; rm -rf $out; mkdir -p $out; cd $out
setsid bash -c "ros2 launch g1_guide exhibition_nav.launch.py headless:=$headless lidar:=mid360 vision:=true \
    vision_rate:=${VISION_RATE:-10} scene_config:=$scene > launch.log 2>&1 &
  ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9097 > rosbridge.log 2>&1 &
  ros2 run ros_gz_bridge parameter_bridge /world/exhibition/set_pose@ros_gz_interfaces/srv/SetEntityPose > bridge.log 2>&1 &
  wait" &
LP=$!
trap 'kill -- -$LP 2>/dev/null; sleep 3; kill -9 -- -$LP 2>/dev/null' EXIT
while [ ! -f stop ] && [ $SECONDS -lt 1200 ]; do sleep 2; done
echo "stopped after ${SECONDS}s"
