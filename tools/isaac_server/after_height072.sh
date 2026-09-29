#!/bin/bash
# 等 height072 训练结束后：导出策略 + 12 项地形评估 + 录制演示数据
D=/root/isaac
R=$D/IsaacLab/logs/rsl_rl/g1_rough/$(ls $D/IsaacLab/logs/rsl_rl/g1_rough/ | grep height072 | tail -1)
while pgrep -f "[t]rain_custom.py" > /dev/null; do sleep 20; done
C=$R/$(ls $R | grep "^model_" | sort -V | tail -1)
echo "最终模型 $C" > $D/after_height072.status
bash $D/eval_suite_full.sh $C $D/eval_h1499.jsonl
echo "评估完成" >> $D/after_height072.status
bash $D/run_record.sh $C $D/demo_height072 > $D/record_h.log 2>&1
echo "录制完成" >> $D/after_height072.status
