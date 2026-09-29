#!/bin/bash
# rand070 训练期间每 5 分钟刷新摘要表；结束后导出策略 + 12 项地形评估 + 录制演示数据
D=/root/isaac; LOG=$D/train_rand070.log; SUM=$D/summary_rand070.tsv
PY=$D/envs/isaaclab/bin/python
sleep 60
while pgrep -f "[t]rain_custom.py" > /dev/null; do $PY $D/summarize_log.py $LOG --weight -5 --out $SUM > /dev/null 2>&1; sleep 300; done
$PY $D/summarize_log.py $LOG --weight -5 --out $SUM > /dev/null 2>&1
R=$D/IsaacLab/logs/rsl_rl/g1_rough/$(ls $D/IsaacLab/logs/rsl_rl/g1_rough/ | grep rand070 | tail -1)
C=$R/$(ls $R | grep "^model_" | sort -V | tail -1)
echo "最终模型 $C" > $D/after_rand070.status
bash $D/eval_suite_full.sh $C $D/eval_rand070.jsonl
bash $D/run_eval.sh $C flat 0 --out $D/eval_rand070.jsonl > $D/eval_last.log 2>&1
echo "评估完成" >> $D/after_rand070.status
bash $D/run_record.sh $C $D/demo_rand070 > $D/record_rand070.log 2>&1
echo "录制完成" >> $D/after_rand070.status
