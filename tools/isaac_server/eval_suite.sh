#!/bin/bash
export TERM=xterm  # isaaclab.sh 开头调用 tabs，TERM 为空或 dumb 会直接退出
# 对一个 checkpoint 跑一组固定难度地形评估，结果追加到 <out>
# 用法：eval_suite.sh <checkpoint> <out.jsonl>
CK=$1; OUT=$2
for t in "stairs_up 0.10" "stairs_up 0.15" "stairs_up 0.18" "stairs_up 0.23" "stairs_down 0.15" "slope_up 10" "slope_up 15" "slope_up 20"; do
  set -- $t
  bash /root/isaac/run_eval.sh $CK $1 $2 --out $OUT > /root/isaac/eval_last.log 2>&1
done
echo "== 完成" >> $OUT.done
