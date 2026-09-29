#!/bin/bash
export TERM=xterm  # isaaclab.sh 开头调用 tabs，TERM 为空或 dumb 会直接退出
# 最终模型：导出策略 + 12 项固定难度地形评估
# 用法：eval_suite_full.sh <checkpoint> <out.jsonl>
CK=$1; OUT=$2; D=/root/isaac
bash $D/run_export.sh $CK > $D/export.log 2>&1
for t in "stairs_up 0.10" "stairs_up 0.15" "stairs_up 0.18" "stairs_up 0.23" "stairs_down 0.15" "slope_up 10" "slope_up 15" "slope_up 20" \
         "stairs_down 0.23" "stairs_up 0.25" "stairs_up 0.18 --vx 1.0" "flat 0 --vx 1.0"; do
  bash $D/run_eval.sh $CK $t --out $OUT > $D/eval_last.log 2>&1
done
echo 完成 > $OUT.done
