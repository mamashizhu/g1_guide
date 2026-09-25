#!/usr/bin/env bash
# 采样 Gazebo 进程的内存，按映射类别分解，判断是否泄漏以及泄漏在哪。
# 用法：bash tools/mem_probe.sh <秒数> <间隔秒>
# 每次采样输出：进程、RSS、匿名堆内存、显卡驱动映射（/dev/dxg）、其他文件映射（MB）。
dur=${1:-120}; step=${2:-15}
sample() {
  for pid in $(pgrep -f "[g]z-sim-main|[g]z-sim-gui-client"); do
    name=$(ps -o comm= -p "$pid")
    awk -v name="$name" -v t="$1" '
      /^[0-9a-f]+-[0-9a-f]+ / { path = $6 }
      /^Rss:/ { if (path ~ /dxg|d3d12|dri|nvidia|libd3d/) gpu += $2;
                else if (path == "" || path ~ /^\[/) anon += $2; else file += $2; total += $2 }
      END { printf "t=%4ss %-16s rss=%5d anon=%5d gpu_map=%5d file=%5d MB\n", t, name,
            total/1024, anon/1024, gpu/1024, file/1024 }' "/proc/$pid/smaps" 2>/dev/null
  done
}
for ((t = 0; t <= dur; t += step)); do sample "$t"; sleep "$step"; done
