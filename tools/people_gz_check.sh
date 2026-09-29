#!/usr/bin/env bash
# Gazebo 验收 PLAN M5.2 雷达人群感知 + M5.3 面向观众（headless，约 4 分钟）
# 用法：bash tools/people_gz_check.sh [输出名]；日志和记录在 .navrun/<输出名>/
# 流程：大展馆 + 真机雷达版 → 去 5 号展位（旁边有 2 个静态参观者）→ 记录 /guide/people
#       → 放一个 1.1 m 高的圆柱当「儿童」→ 删掉它 → 切到面向观众，发 face 让机器人转向人群
# 只结束本脚本启动的进程（独立进程组），不影响其他会话的仿真。
source /opt/ros/lyrical/setup.bash
source /home/ma/.local/share/g1-guide-runtime/install/setup.bash
export LD_LIBRARY_PATH=/usr/lib/wsl/lib:$LD_LIBRARY_PATH MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA GALLIUM_DRIVER=d3d12
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-84} GZ_PARTITION=${GZ_PARTITION:-peoplecheck}
TOOLS=/mnt/d/g1_guide/tools
out=/mnt/d/g1_guide/.navrun/${1:-people}; rm -rf $out; mkdir -p $out; cd $out
setsid ros2 launch g1_guide exhibition_nav.launch.py headless:=true lidar:=mid360 > launch.log 2>&1 &
LP=$!
trap 'kill -- -$LP 2>/dev/null; sleep 3; kill -9 -- -$LP 2>/dev/null' EXIT
sleep 45

wait_state() {  # wait_state <state> <goal_key> <秒>
  local t0=$SECONDS
  while [ $((SECONDS-t0)) -lt $3 ]; do
    grep -q "\"state\": \"$1\", \"goal_key\": \"$2\"" launch.log && { echo "  $1 ($2) after $((SECONDS-t0))s"; return 0; }
    sleep 1
  done
  echo "  !! 等不到 $1 ($2)"; return 1
}
phase() { echo "== $1"; python3 $TOOLS/people_watch.py $2 people_$3.jsonl | tail -${4:-3}; }

echo "== 去 5 号展位"
ros2 topic pub --once /nav/goal_key std_msgs/String "data: '5'" >/dev/null
wait_state arrived 5 200
phase "到站：应检出 visitor_reading 和 visitor_centre" 6 station

kid="<?xml version='1.0'?><sdf version='1.9'><model name='kid'><static>true</static><link name='l'><collision name='c'><geometry><cylinder><radius>0.18</radius><length>1.1</length></cylinder></geometry></collision><visual name='v'><geometry><cylinder><radius>0.18</radius><length>1.1</length></cylinder></geometry><material><ambient>1 0.5 0 1</ambient><diffuse>1 0.5 0 1</diffuse></material></visual></link></model></sdf>"
ros2 run ros_gz_sim create -world exhibition -name kid -x 4.4 -y 3.6 -z 0.55 -string "$kid" >/dev/null 2>&1
phase "放 1.1 m 圆柱（4.4, 3.6）：应多一个儿童" 6 kid

bash $TOOLS/remove_box.sh kid >/dev/null 2>&1
echo "== 删掉圆柱（$(date +%T)）：3 秒内应消失"
python3 $TOOLS/people_watch.py 5 people_removed.jsonl

echo "== 面向观众：present_mode=face，goal=face"
ros2 topic pub --once /guide/present_mode std_msgs/String "data: 'face'" >/dev/null
ros2 topic pub --once /nav/goal_key std_msgs/String "data: 'face'" >/dev/null
wait_state faced face 30
grep -o '"state": "faced"[^}]*' launch.log | tail -1
phase "转向后" 3 faced 1
python3 - people_faced.jsonl <<'PY'
import json, math, sys
payload = json.loads(open(sys.argv[1], encoding="utf-8").readlines()[-1])
rx, ry, ryaw = payload["robot"]
near = [p for p in payload["people"] if p["distance"] <= 3.0]
if near:
    cx = sum(p["x"] for p in near) / len(near); cy = sum(p["y"] for p in near) / len(near)
    err = math.degrees(math.atan2(math.sin(math.atan2(cy-ry, cx-rx)-ryaw), math.cos(math.atan2(cy-ry, cx-rx)-ryaw)))
    print(f"  朝向与 3 m 内人群中心的夹角 {err:+.1f}°（{len(near)} 人）")
PY
grep -i -E "error|traceback" launch.log | grep -v -i "ogre\|rendering" | head -5
