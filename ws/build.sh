#!/usr/bin/env bash
# 构建 claude_course 工作区：先生成平移模型 URDF，再 colcon 构建。
# 使用 --symlink-install：之后改 Python、launch、config 立即生效，
# 只有新增文件、改 setup.py 或改 make_glide_urdf.py 时才需要重新运行本脚本。
set -eo pipefail

# WSL 会追加 Windows PATH，CMake 可能误用 Windows 的 Anaconda 头文件。
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/lyrical/setup.bash

ws=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
runtime="${G1_GUIDE_RUNTIME:-$HOME/.local/share/g1-guide-runtime}"

python3 "$ws/src/g1_guide/scripts/make_glide_urdf.py" --g1-ws "$ws"

cd -- "$ws"
colcon --log-base "$runtime/log" build \
  --build-base "$runtime/build" \
  --install-base "$runtime/install" \
  --symlink-install \
  --packages-up-to g1_guide
printf '构建完成。环境：source %s/install/setup.bash\n' "$runtime"
