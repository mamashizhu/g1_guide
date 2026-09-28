#!/bin/bash
# 在数据盘安装 Isaac Sim 5.1 + Isaac Lab v2.3.2。系统盘只有 30 GB，环境和缓存都放 /root/autodl-tmp。
# 下载源（2026-09-28 实测，pip 只用 HTTP/1.1）：清华 PyPI 9 MB/s；pypi.nvidia.com 会跳到 pypi.nvidia.cn，7.5 MB/s；
# cu128 版 torch 只有阿里云镜像有，但阿里云限速 HTTP/1.1，所以用 curl（HTTP/2，10 MB/s）先下到本地。
set -e
D=/root/autodl-tmp
export TMPDIR=$D/tmp  # pip 临时文件默认在系统盘 /tmp，会撑满系统盘
mkdir -p $TMPDIR $D/wheels
source /root/miniconda3/etc/profile.d/conda.sh
conda activate $D/envs/isaaclab

echo "== [1/3] PyTorch 2.7.0 cu128"
cd $D/wheels
for f in torch-2.7.0%2Bcu128-cp311-cp311-manylinux_2_28_x86_64.whl torchvision-0.22.0%2Bcu128-cp311-cp311-manylinux_2_28_x86_64.whl; do
  out=${f//%2B/+}
  [ -f "$out.done" ] || { curl -fL --retry 10 -C - -o "$out" "https://mirrors.aliyun.com/pytorch-wheels/cu128/$f" && touch "$out.done"; }
done
pip install ./torch-2.7.0+cu128-*.whl ./torchvision-0.22.0+cu128-*.whl

echo "== [2/3] Isaac Sim 5.1.0"
cd $D
# torch==2.7.0 已由 2.7.0+cu128 满足，不会再下一份
pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com

echo "== [3/3] Isaac Lab v2.3.2"
[ -d $D/IsaacLab ] || (source /etc/network_turbo >/dev/null; git clone --branch v2.3.2 --depth 1 https://github.com/isaac-sim/IsaacLab.git $D/IsaacLab)
cd $D/IsaacLab
./isaaclab.sh --install
python -c "import torch; print(\"torch\", torch.__version__)"
echo "== 全部完成"
df -h / $D
