#!/bin/bash
# 把 Isaac Lab 环境从数据盘复制到系统盘 /root/isaac，便于保存 AutoDL 镜像（镜像只含系统盘）。
# 数据盘原件保留不动，验证通过后再决定删不删。
set -e
S=/root/autodl-tmp; T=/root/isaac
echo "== [1/6] 复制 conda 环境和 Isaac Lab $(date +%T)"
mkdir -p $T/envs
cp -a $S/envs/isaaclab $T/envs/
cp -a $S/IsaacLab $T/
rm -rf $T/envs/isaaclab/lib/python3.11/site-packages/isaacsim/kit/logs/*   # 运行日志，不必进镜像
echo "== [2/6] 替换绝对路径 $(date +%T)"
for pair in "$S/envs/isaaclab#$T/envs/isaaclab" "$S/IsaacLab#$T/IsaacLab"; do
  old=${pair%%#*}; new=${pair##*#}
  grep -rlI --null -F "$old" $T/envs/isaaclab $T/IsaacLab 2>/dev/null | xargs -0 -r sed -i "s#$old#$new#g"
done
echo "剩余旧路径的文本文件数：$(grep -rlI -F "$S/" $T/envs/isaaclab $T/IsaacLab 2>/dev/null | grep -v "/logs/" | wc -l)"
echo "== [3/6] 脚本和版本锁 $(date +%T)"
for f in constraints.txt run_eval.sh run_export.sh run_g1_test.sh run_g1_bench.sh train_g1_rough.sh eval_suite.sh eval_suite_full.sh eval_terrain.py export_policy.py install_isaaclab.sh fix_isaaclab.sh; do
  [ -f $S/$f ] && sed "s#/root/autodl-tmp#$T#g" $S/$f > $T/$f
done
chmod +x $T/*.sh
echo "== [4/6] conda 环境变量 $(date +%T)"
source /root/miniconda3/etc/profile.d/conda.sh
conda env config vars unset -p $T/envs/isaaclab TMPDIR PIP_FIND_LINKS >/dev/null
conda env config vars set -p $T/envs/isaaclab OMNI_KIT_ACCEPT_EULA=YES PIP_CONSTRAINT=$T/constraints.txt >/dev/null
mkdir -p $T/envs/isaaclab/etc/conda/activate.d
cat > $T/envs/isaaclab/etc/conda/activate.d/tmpdir.sh <<"EOS"
# 有数据盘就把临时文件放数据盘（系统盘只有 30 GB，pip 装大包会撑满），否则放 /root/isaac/tmp
if [ -d /root/autodl-tmp ]; then export TMPDIR=/root/autodl-tmp/tmp; else export TMPDIR=/root/isaac/tmp; fi
mkdir -p "$TMPDIR"
EOS
echo "== [5/6] 缓存目录改回系统盘上的真实目录 $(date +%T)"
[ -L ~/.cache ] && { rm ~/.cache; mkdir -p ~/.cache; }
[ -L ~/.local/share/ov ] && { rm ~/.local/share/ov; mkdir -p ~/.local/share/ov; }
sed -i "s#^alias isaac=.*#alias isaac=\"source /root/miniconda3/etc/profile.d/conda.sh \&\& conda activate $T/envs/isaaclab \&\& cd $T/IsaacLab\"#" ~/.bashrc
sed -i "s#^export TMPDIR=/root/autodl-tmp/tmp#[ -d /root/autodl-tmp ] \&\& export TMPDIR=/root/autodl-tmp/tmp#" ~/.bashrc
echo "== [6/6] 验证 $(date +%T)"
conda activate $T/envs/isaaclab
which python pip
python -c "import torch, isaacsim, rsl_rl; print(\"torch\", torch.__version__, torch.cuda.is_available())"
pip list 2>/dev/null | grep -E "^isaaclab " 
df -h / | tail -1
echo "== 完成 $(date +%T)"
