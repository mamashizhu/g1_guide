#!/usr/bin/env bash
# 在 WSL 内用系统级 systemd 托管运行一条 ros2 命令（以用户 ma 身份），不依赖 wsl.exe 连接。
#
# 为什么需要：多个会话同时调用 wsl 时，Windows 端 WSL 服务偶尔报 E_UNEXPECTED，
# 挂在那条 wsl 连接上的仿真会被一起回收；systemd --user 也不行，最后一个登录断开时用户
# 管理器会退出并停掉它的单元。系统级单元不挂在任何一条连接上。
# 注意：WSL 在没有任何 wsl.exe 连接时会关闭整个发行版，所以仍要留一条连接等待单元结束，
# 例如：wsl -u root -e bash -c '… run_detached.sh …; while systemctl is-active -q <单元名>; do sleep 5; done'
#
# 用法（需要 root）：
#   wsl -u root -e bash /mnt/d/g1_guide/tools/run_detached.sh <单元名> <日志> [VAR=值 ...] -- <命令...>
#   例：… run_detached.sh g1-navtour /home/ma/navrun/launch.log ROS_DOMAIN_ID=77 GZ_PARTITION=slamtest \
#         -- ros2 launch g1_guide slam_nav.launch.py tour:=true
# 查看：systemctl status <单元名>；停止：systemctl stop <单元名>
set -eo pipefail
[ "$(id -u)" = 0 ] || { echo "请用 wsl -u root 运行" >&2; exit 2; }
unit=$1; log=$2; shift 2
user=${G1_USER:-ma}
home=$(getent passwd "$user" | cut -d: -f6)
env_args=(--setenv=HOME="$home" --setenv=USER="$user" --setenv=DISPLAY=:0
          --setenv=WAYLAND_DISPLAY=wayland-0 --setenv=XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir
          --setenv=PULSE_SERVER=unix:/mnt/wslg/PulseServer)
while [ $# -gt 0 ] && [ "$1" != "--" ]; do env_args+=(--setenv="$1"); shift; done
[ "$1" = "--" ] && shift
sudo -u "$user" mkdir -p "$(dirname "$log")"
systemctl reset-failed "$unit" 2>/dev/null || true
systemd-run --collect --unit="$unit" --uid="$user" --gid="$(id -g "$user")" \
  --working-directory="$home" "${env_args[@]}" \
  bash -c 'source /opt/ros/lyrical/setup.bash
source "$HOME/.local/share/g1-guide-runtime/install/setup.bash"
export LD_LIBRARY_PATH=/usr/lib/wsl/lib:$LD_LIBRARY_PATH MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA GALLIUM_DRIVER=d3d12
exec "$@" > "'"$log"'" 2>&1' run_detached "$@"
echo "已在 systemd 单元 $unit 中启动，日志：$log"
