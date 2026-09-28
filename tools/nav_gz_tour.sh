#!/usr/bin/env bash
# Gazebo 导航验收（慢，最后确认用；日常先跑 tools/nav_sim.py）：tools/nav_gz_tour.sh <输出名> <步骤...>
# 步骤：展位号（发目标并等到 arrived，最多 200s）| go<k>（只发目标）| until<k>（只等到达）
#       | waitN | ahead,<名>,<距离>（机器人正前方放箱子）| rm_<名>
# 场景默认简单展馆，SCENE=<json 路径> 可换；日志在 .navrun/<输出名>/launch.log
# 例：bash tools/nav_gz_tour.sh s1 go3 wait6 ahead,b1,1.3 until3 5
source /opt/ros/lyrical/setup.bash
source /home/ma/.local/share/g1-guide-runtime/install/setup.bash
export LD_LIBRARY_PATH=/usr/lib/wsl/lib:$LD_LIBRARY_PATH MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA GALLIUM_DRIVER=d3d12
export ROS_DOMAIN_ID=83 GZ_PARTITION=navlock
SCENE=${SCENE:-/mnt/d/g1_guide/ws/src/g1_guide/config/exhibition_simple.json}
out=/mnt/d/g1_guide/.navrun/$1; rm -rf $out; mkdir -p $out; cd $out; shift
ros2 launch g1_guide exhibition_nav.launch.py headless:=true scene_config:=$SCENE > launch.log 2>&1 &
LP=$!
sleep 40
for k in "$@"; do
  case $k in
    ahead,*) IFS=, read _ n d <<< "$k"
      read bx by < <(python3 - $d <<'PY'
import sys, math, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
rclpy.init(); n = Node('probe'); got = []
n.create_subscription(Odometry, '/odom', got.append, qos_profile_sensor_data)
while not got: rclpy.spin_once(n, timeout_sec=1)
p = got[-1].pose.pose; q = p.orientation
yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)); d = float(sys.argv[1])
print(p.position.x+d*math.cos(yaw), p.position.y+d*math.sin(yaw))
PY
)
      bash /mnt/d/g1_guide/tools/spawn_box.sh $n $bx $by >/dev/null 2>&1; echo "box $n at $bx $by"; continue;;
    rm_*) bash /mnt/d/g1_guide/tools/remove_box.sh ${k:3} >/dev/null 2>&1; echo "remove ${k:3}"; continue;;
    wait*) sleep ${k:4}; continue;;
    go*) ros2 topic pub --once /nav/goal_key std_msgs/String "data: '${k:2}'" >/dev/null; continue;;
    until*) k=${k:5};;
    *) ros2 topic pub --once /nav/goal_key std_msgs/String "data: '$k'" >/dev/null;;
  esac
  t0=$SECONDS; last=""
  while [ $((SECONDS-t0)) -lt 200 ]; do
    last=$(grep -o '"state": "[a-z_]*", "goal_key": "'$k'"' launch.log | tail -1)
    case $last in *arrived*) break;; esac
    sleep 2
  done
  echo "goal $k -> $last after $((SECONDS-t0))s"
done
kill $LP; sleep 5; pkill -f "gz sim"; pkill -f exhibition_nav; pkill -f gesture_player; sleep 2
grep -o '"state": "[a-z_]*", "goal_key": "[0-9]"' launch.log | uniq -c
