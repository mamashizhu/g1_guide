#!/usr/bin/env bash
# PLAN M5.0：headless 大展馆，机器人在几处位置对着参观者和行走 actor 拍照，图存到 .navrun/cv/
# 用法：bash tools/cv_capture_sim.sh ；只结束本脚本启动的进程（独立进程组）
source /opt/ros/lyrical/setup.bash
source /home/ma/.local/share/g1-guide-runtime/install/setup.bash
export LD_LIBRARY_PATH=/usr/lib/wsl/lib:$LD_LIBRARY_PATH MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA GALLIUM_DRIVER=d3d12
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-84} GZ_PARTITION=${GZ_PARTITION:-peoplecheck}
TOOLS=/mnt/d/g1_guide/tools
out=/mnt/d/g1_guide/.navrun/cv; rm -rf $out; mkdir -p $out; cd $out
setsid ros2 launch g1_guide exhibition_nav.launch.py headless:=true lidar:=mid360 > launch.log 2>&1 &
LP=$!
trap 'kill -- -$LP 2>/dev/null; sleep 3; kill -9 -- -$LP 2>/dev/null' EXIT
sleep 45

go() {  # go <key>：发目标并等到 arrived/faced
  ros2 topic pub --once /nav/goal_key std_msgs/String "data: '$1'" >/dev/null
  local t0=$SECONDS
  while [ $((SECONDS-t0)) -lt 200 ]; do
    tail -5 launch.log | grep -q "\"state\": \"\(arrived\|faced\)\", \"goal_key\": \"$1\"" && { echo "reached $1 after $((SECONDS-t0))s"; return; }
    sleep 1
  done
  echo "!! $1 not reached"
}

ros2 topic pub --once /guide/present_mode std_msgs/String "data: 'face'" >/dev/null
go 5                                        # 5 号展位，转向旁边的两位静态参观者
python3 $TOOLS/cv_capture.py $out 6 2 s5_face
go 2                                        # 2 号展位（北厅），actor 沿 y=9.2 走过
ros2 topic pub --once /guide/present_mode std_msgs/String "data: 'point'" >/dev/null
python3 $TOOLS/cv_capture.py $out 8 3 s2
go 6                                        # 6 号展位（南廊），另一个 actor 来回走
python3 $TOOLS/cv_capture.py $out 8 3 s6
ls $out/*.png | wc -l
