#!/usr/bin/env bash
# PLAN M5.5 端到端测试的 WSL 端：大展馆 + 真机雷达版 + rosbridge（端口 9097），headless。
# 配合 Windows 端 voice\tests\sim_crowd.py 使用；也支持手动放/删「观众」：
#   VISION=true bash tools/crowd_e2e_sim.sh <输出名>   # 同时发前置相机 JPEG（视觉唤醒测试）
#   bash tools/crowd_e2e_sim.sh <输出名>            # 启动，直到 .navrun/<输出名>/stop 出现（最长 15 分钟）
#   bash tools/crowd_e2e_sim.sh spawn <名> <x> <y> <高>   # 放一个圆柱人
#   bash tools/crowd_e2e_sim.sh person <名> <x> <y> <yaw>   # 放一个静态参观者模型
#   bash tools/crowd_e2e_sim.sh remove <名>
# 只结束本脚本启动的进程（独立进程组）。
source /opt/ros/lyrical/setup.bash
source /home/ma/.local/share/g1-guide-runtime/install/setup.bash
export LD_LIBRARY_PATH=/usr/lib/wsl/lib:$LD_LIBRARY_PATH MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA GALLIUM_DRIVER=d3d12
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-84} GZ_PARTITION=${GZ_PARTITION:-peoplecheck}

case $1 in
  spawn)
    sdf="<?xml version='1.0'?><sdf version='1.9'><model name='$2'><static>true</static><link name='l'><collision name='c'><geometry><cylinder><radius>0.18</radius><length>$5</length></cylinder></geometry></collision><visual name='v'><geometry><cylinder><radius>0.18</radius><length>$5</length></cylinder></geometry><material><ambient>1 0.5 0 1</ambient><diffuse>1 0.5 0 1</diffuse></material></visual></link></model></sdf>"
    ros2 run ros_gz_sim create -world exhibition -name "$2" -x "$3" -y "$4" -z "$(python3 -c "print($5/2)")" -string "$sdf" >/dev/null 2>&1
    exit;;
  person)  # person <名> <x> <y> <yaw>：放一个 Fuel「Standing person」静态参观者（有碰撞，雷达可见）
    sdf="<?xml version='1.0'?><sdf version='1.9'><include><name>$2</name><static>true</static><uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing person</uri></include></sdf>"
    ros2 run ros_gz_sim create -world exhibition -name "$2" -x "$3" -y "$4" -z 0 -Y "$5" -string "$sdf" >/dev/null 2>&1
    exit;;
  remove)
    exec bash /mnt/d/g1_guide/tools/remove_box.sh "$2";;
esac

out=/mnt/d/g1_guide/.navrun/${1:-crowd}; rm -rf $out; mkdir -p $out; cd $out
setsid bash -c "ros2 launch g1_guide exhibition_nav.launch.py headless:=true lidar:=mid360 vision:=${VISION:-false} > launch.log 2>&1 &
  ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9097 > rosbridge.log 2>&1 &
  wait" &
LP=$!
trap 'kill -- -$LP 2>/dev/null; sleep 3; kill -9 -- -$LP 2>/dev/null' EXIT
while [ ! -f stop ] && [ $SECONDS -lt 900 ]; do sleep 2; done
echo "stopped after ${SECONDS}s"
