# 1) 环境最初装在数据盘 /root/autodl-tmp/envs/isaaclab，后来复制到系统盘 /root/isaac。
#    conda 的二进制程序（tabs、openssl 等）把旧路径编译在里面，文本替换改不到，
#    所以在旧路径建一个软链接指回新位置。新实例的数据盘是空的，每次激活时补建。
if [ -d /root/autodl-tmp ] && [ ! -e /root/autodl-tmp/envs/isaaclab ]; then
  mkdir -p /root/autodl-tmp/envs && ln -s /root/isaac/envs/isaaclab /root/autodl-tmp/envs/isaaclab
fi
export TERMINFO=/root/isaac/envs/isaaclab/share/terminfo
# 2) 有数据盘就把临时文件放数据盘（系统盘只有 30 GB，pip 装大包会撑满），否则放 /root/isaac/tmp
if [ -d /root/autodl-tmp ]; then export TMPDIR=/root/autodl-tmp/tmp; else export TMPDIR=/root/isaac/tmp; fi
mkdir -p "$TMPDIR"
