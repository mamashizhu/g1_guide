"""先注册自定义任务，再原样运行 Isaac Lab 的 rsl_rl/train.py（参数全部透传）。

    ./isaaclab.sh -p /root/isaac/g1_custom/train_custom.py --task G1-Rough-Height-v0 --headless ...
"""
import os
import runpy
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TRAIN = "/root/isaac/IsaacLab/scripts/reinforcement_learning/rsl_rl/train.py"

sys.path.insert(0, HERE)                    # g1_height_cfg 等模块
sys.path.insert(0, os.path.dirname(TRAIN))  # train.py 要 import 同目录的 cli_args
import g1_tasks  # noqa: E402,F401  注册任务

sys.argv = [TRAIN] + sys.argv[1:]
runpy.run_path(TRAIN, run_name="__main__")
