"""在 Isaac 里跑策略，逐帧记录机器人位姿，并导出地形网格，供本地 MuJoCo 回放成视频。

AutoDL 容器里 Vulkan 用不了显卡，Isaac 自己录不了视频，所以只记运动数据：
    <out>/terrain.stl   地形网格（世界坐标）
    <out>/traj.npz      每个控制步（50 Hz）的骨盆位置、四元数（wxyz）、37 个关节角，及地形标签
用法（服务器，Isaac Lab 环境内）：
    ./isaaclab.sh -p /root/isaac/record_demo.py --checkpoint <model_*.pt> --out /root/isaac/demo
"""
import argparse
import json
import math
import os

from isaaclab.app import AppLauncher

TASK = "Isaac-Velocity-Rough-G1-Play-v0"

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--checkpoint", required=True)
parser.add_argument("--out", default="/root/isaac/demo")
parser.add_argument("--terrains", default="stairs_up:0.18,stairs_up:0.23,slope_up:20,stairs_down:0.15",
                    help="逗号分隔的 类型:数值，每种地形一列、一个机器人")
parser.add_argument("--vx", type=float, default=0.5)
parser.add_argument("--seconds", type=float, default=20.0)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
simulation_app = AppLauncher(args).app

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

import isaaclab.terrains as terrain_gen  # noqa: E402
import isaaclab_tasks  # noqa: E402,F401
from isaaclab.terrains import TerrainGenerator  # noqa: E402
from isaaclab.terrains.config.rough import ROUGH_TERRAINS_CFG  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry  # noqa: E402


def sub_terrain(kind, v):
    if kind.startswith("stairs"):
        cls = terrain_gen.MeshInvertedPyramidStairsTerrainCfg if kind == "stairs_up" \
            else terrain_gen.MeshPyramidStairsTerrainCfg
        return cls(proportion=1.0, step_height_range=(v, v), step_width=0.3,
                   platform_width=3.0, border_width=1.0, holes=False)
    r = math.radians(v)
    cls = terrain_gen.HfInvertedPyramidSlopedTerrainCfg if kind == "slope_up" else terrain_gen.HfPyramidSlopedTerrainCfg
    return cls(proportion=1.0, slope_range=(r, r), platform_width=2.0, border_width=0.25)


def main():
    os.makedirs(args.out, exist_ok=True)
    items = [t.split(":") for t in args.terrains.split(",")]
    subs = {f"{k}_{v}": sub_terrain(k, float(v)) for k, v in items}
    # curriculum=True 时按比例逐列摆放子地形：一列一种，顺序固定；num_rows=1 只有一行
    terrain_cfg = ROUGH_TERRAINS_CFG.replace(sub_terrains=subs, num_rows=1, num_cols=len(subs),
                                             curriculum=True, use_cache=False, seed=42)
    for s in terrain_cfg.sub_terrains.values():
        s.proportion = 1.0 / len(subs)
    TerrainGenerator(cfg=terrain_cfg, device="cpu").terrain_mesh.export(os.path.join(args.out, "terrain.stl"))

    env_cfg = parse_env_cfg(TASK, device=args.device, num_envs=len(subs))
    env_cfg.scene.terrain.terrain_generator = terrain_cfg
    env_cfg.scene.terrain.max_init_terrain_level = None
    env_cfg.commands.base_velocity.ranges.lin_vel_x = (args.vx, args.vx)
    env_cfg.episode_length_s = args.seconds + 5.0
    env = RslRlVecEnvWrapper(gym.make(TASK, cfg=env_cfg))
    agent_cfg = load_cfg_from_registry(TASK, "rsl_rl_cfg_entry_point")
    # 训练时若用了 noise_std_type=log，checkpoint 里存的是 log_std，网络要按同样方式建
    if "log_std" in torch.load(args.checkpoint, map_location="cpu", weights_only=False)["model_state_dict"]:
        agent_cfg.policy.noise_std_type = "log"
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=args.device)
    runner.load(args.checkpoint)
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    u = env.unwrapped
    robot = u.scene["robot"]
    terrain = u.scene.terrain
    cols = terrain.terrain_types.cpu().numpy().tolist() if hasattr(terrain, "terrain_types") else list(range(len(subs)))
    pos, quat, qj, done = [], [], [], []
    obs = env.get_observations()
    for _ in range(int(args.seconds / u.step_dt)):
        with torch.inference_mode():
            obs, _, dones, _ = env.step(policy(obs))
        pos.append(robot.data.root_pos_w.cpu().numpy())
        quat.append(robot.data.root_quat_w.cpu().numpy())
        qj.append(robot.data.joint_pos.cpu().numpy())
        done.append(dones.cpu().numpy())
    np.savez_compressed(
        os.path.join(args.out, "traj.npz"),
        pos=np.array(pos), quat=np.array(quat), joint_pos=np.array(qj), done=np.array(done),
        joint_names=np.array(robot.data.joint_names), dt=u.step_dt,
        env_origins=u.scene.env_origins.cpu().numpy(),
        labels=np.array([list(subs)[c] for c in cols]),
    )
    with open(os.path.join(args.out, "info.json"), "w") as f:
        json.dump({"checkpoint": args.checkpoint, "terrains": list(subs), "robot_cols": cols, "vx": args.vx}, f)
    env.close()


if __name__ == "__main__":
    try:
        main()
    except BaseException:
        import traceback
        with open(os.path.join(args.out, "record.err"), "w") as f:
            traceback.print_exc(file=f)
        raise
    finally:
        simulation_app.close()
