"""固定难度地形评估：训练好的 G1 崎岖地形策略在指定台阶高度 / 坡度上的通过率。

地形只保留一种子地形、难度固定，机器人从每块地形（8×8 m）中央出发，
按固定速度沿 +x 走 --seconds 秒，走出台阶/坡区域且没摔倒算通过。
    上楼梯 stairs_up   = 倒金字塔台阶（中央最低，往外一级级升高）
    下楼梯 stairs_down = 金字塔台阶（中央最高）
    上坡 slope_up / 下坡 slope_down 同理

用法（服务器，Isaac Lab 环境内）：
    ./isaaclab.sh -p /root/autodl-tmp/eval_terrain.py --checkpoint <model_*.pt> --terrain stairs_up --value 0.15
输出最后一行是 JSON，方便汇总。
"""
import argparse
import json
import math

from isaaclab.app import AppLauncher

TASK = "Isaac-Velocity-Rough-G1-Play-v0"

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--checkpoint", required=True, help="rsl_rl 保存的 model_*.pt")
parser.add_argument("--terrain", required=True, choices=["stairs_up", "stairs_down", "slope_up", "slope_down", "flat"])
parser.add_argument("--value", type=float, default=0.0, help="台阶高（米）或坡度（度）")
parser.add_argument("--num_envs", type=int, default=64)
parser.add_argument("--vx", type=float, default=0.5, help="前进速度指令 m/s")
parser.add_argument("--seconds", type=float, default=20.0)
parser.add_argument("--out", default="/root/autodl-tmp/eval_results.jsonl", help="结果追加写入的文件（Isaac Sim 会吞终端输出）")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
simulation_app = AppLauncher(args).app

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

import isaaclab.terrains as terrain_gen  # noqa: E402
import isaaclab_tasks  # noqa: E402,F401  注册任务
from isaaclab.terrains.config.rough import ROUGH_TERRAINS_CFG  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry  # noqa: E402

TILE = 8.0
# 平台半宽 + 台阶/坡区宽度：走出这个半径就算越过了障碍（与 rough.py 里的尺寸一致）
STAIR_PLATFORM, STAIR_BORDER = 3.0, 1.0
SLOPE_PLATFORM, SLOPE_BORDER = 2.0, 0.25


def sub_terrain():
    v = args.value
    if args.terrain.startswith("stairs"):
        cls = terrain_gen.MeshInvertedPyramidStairsTerrainCfg if args.terrain == "stairs_up" \
            else terrain_gen.MeshPyramidStairsTerrainCfg
        cfg = cls(proportion=1.0, step_height_range=(v, v), step_width=0.3,
                  platform_width=STAIR_PLATFORM, border_width=STAIR_BORDER, holes=False)
        goal = TILE / 2 - STAIR_BORDER + 0.2
    elif args.terrain.startswith("slope"):
        cls = terrain_gen.HfInvertedPyramidSlopedTerrainCfg if args.terrain == "slope_up" \
            else terrain_gen.HfPyramidSlopedTerrainCfg
        r = math.radians(v)
        cfg = cls(proportion=1.0, slope_range=(r, r), platform_width=SLOPE_PLATFORM, border_width=SLOPE_BORDER)
        goal = TILE / 2 - SLOPE_BORDER
    else:
        cfg = terrain_gen.MeshPlaneTerrainCfg(proportion=1.0)
        goal = 3.0
    return cfg, goal


def main():
    env_cfg = parse_env_cfg(TASK, device=args.device, num_envs=args.num_envs)
    cfg, goal = sub_terrain()
    env_cfg.scene.terrain.terrain_generator = ROUGH_TERRAINS_CFG.replace(
        sub_terrains={"test": cfg}, num_rows=4, num_cols=4, curriculum=False, use_cache=False)
    env_cfg.scene.terrain.max_init_terrain_level = None
    env_cfg.commands.base_velocity.ranges.lin_vel_x = (args.vx, args.vx)
    env_cfg.episode_length_s = args.seconds + 5.0  # 评估期间不会因超时重置

    env = RslRlVecEnvWrapper(gym.make(TASK, cfg=env_cfg))
    agent_cfg = load_cfg_from_registry(TASK, "rsl_rl_cfg_entry_point")
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=args.device)
    runner.load(args.checkpoint)
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    robot = env.unwrapped.scene["robot"]
    origins = env.unwrapped.scene.env_origins[:, :2].clone()
    n = env.num_envs
    dev = env.unwrapped.device
    max_dist = torch.zeros(n, device=dev)
    fallen = torch.zeros(n, dtype=torch.bool, device=dev)
    ended = torch.zeros(n, dtype=torch.bool, device=dev)

    obs = env.get_observations()
    for _ in range(int(args.seconds / env.unwrapped.step_dt)):
        with torch.inference_mode():
            obs, _, dones, extras = env.step(policy(obs))
        dones = dones.bool()
        timeouts = extras.get("time_outs", torch.zeros_like(dones)).bool()
        live = ~ended
        dist = torch.norm(robot.data.root_pos_w[:, :2] - origins, dim=1)
        # 刚重置的机器人位置已回到出发点，只统计第一个回合
        max_dist = torch.where(live & ~dones, torch.maximum(max_dist, dist), max_dist)
        fallen |= live & dones & ~timeouts
        ended |= dones

    passed = (~fallen) & (max_dist >= goal)
    res = {
        "terrain": args.terrain, "value": args.value, "vx": args.vx, "robots": n,
        "pass_rate": round(passed.float().mean().item(), 3),
        "fall_rate": round(fallen.float().mean().item(), 3),
        "median_dist": round(max_dist.median().item(), 2), "goal_dist": goal,
    }
    print("EVAL_RESULT " + json.dumps(res))
    with open(args.out, "a") as f:
        f.write(json.dumps(res) + "\n")
    env.close()


if __name__ == "__main__":
    try:
        main()
    except BaseException:
        # Isaac Sim 会吞掉终端输出，错误写到文件
        import traceback
        with open(args.out + ".err", "w") as f:
            traceback.print_exc(file=f)
        raise
    finally:
        simulation_app.close()
