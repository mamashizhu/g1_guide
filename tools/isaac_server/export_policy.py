"""导出训练好的 G1 策略（TorchScript + ONNX），并记录 sim2sim 需要对齐的全部参数。

输出到 checkpoint 同目录的 exported/：
    policy.pt / policy.onnx   策略网络
    deploy.json               关节顺序、默认姿态、PD 增益、动作缩放、观测组成、控制周期、高度扫描点
用法（服务器，Isaac Lab 环境内）：
    ./isaaclab.sh -p /root/isaac/export_policy.py --checkpoint <model_*.pt>
"""
import argparse
import json
import os

from isaaclab.app import AppLauncher

TASK = "Isaac-Velocity-Rough-G1-Play-v0"

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--checkpoint", required=True)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
simulation_app = AppLauncher(args).app

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

import isaaclab_tasks  # noqa: E402,F401
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, export_policy_as_jit, export_policy_as_onnx  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry  # noqa: E402


def tolist(x):
    return x.detach().cpu().tolist() if torch.is_tensor(x) else x


def main():
    out_dir = os.path.join(os.path.dirname(args.checkpoint), "exported")
    env_cfg = parse_env_cfg(TASK, device=args.device, num_envs=1)
    env = RslRlVecEnvWrapper(gym.make(TASK, cfg=env_cfg))
    agent_cfg = load_cfg_from_registry(TASK, "rsl_rl_cfg_entry_point")
    # 训练时若用了 noise_std_type=log，checkpoint 里存的是 log_std，网络要按同样方式建
    if "log_std" in torch.load(args.checkpoint, map_location="cpu", weights_only=False)["model_state_dict"]:
        agent_cfg.policy.noise_std_type = "log"
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=args.device)
    runner.load(args.checkpoint)
    policy_nn = runner.alg.policy
    normalizer = getattr(policy_nn, "actor_obs_normalizer", None)
    export_policy_as_jit(policy_nn, normalizer=normalizer, path=out_dir, filename="policy.pt")
    export_policy_as_onnx(policy_nn, normalizer=normalizer, path=out_dir, filename="policy.onnx")

    u = env.unwrapped
    robot = u.scene["robot"]
    act_term = u.action_manager.get_term("joint_pos")
    obs_mgr = u.observation_manager
    policy_terms = []
    for name, cfg, dims in zip(obs_mgr.active_terms["policy"], obs_mgr._group_obs_term_cfgs["policy"],
                               obs_mgr.group_obs_term_dim["policy"]):
        policy_terms.append({
            "name": name, "dim": list(dims), "func": getattr(cfg.func, "__name__", str(cfg.func)),
            "scale": tolist(cfg.scale) if cfg.scale is not None else None,
            "clip": list(cfg.clip) if cfg.clip is not None else None,
        })
    stiffness, damping = {}, {}
    for act in robot.actuators.values():
        for i, j in enumerate(act.joint_names):
            stiffness[j] = float(act.stiffness[0, i])
            damping[j] = float(act.damping[0, i])
    scanner = u.scene.sensors.get("height_scanner")
    info = {
        "task": TASK,
        "checkpoint": os.path.abspath(args.checkpoint),
        "sim_dt": u.cfg.sim.dt,
        "decimation": u.cfg.decimation,
        "control_dt": u.step_dt,
        "joint_names": robot.data.joint_names,
        "default_joint_pos": tolist(robot.data.default_joint_pos[0]),
        "stiffness": stiffness,
        "damping": damping,
        "action": {
            "joint_names": act_term._joint_names,
            "scale": tolist(act_term._scale) if torch.is_tensor(act_term._scale) else act_term._scale,
            "use_default_offset": act_term.cfg.use_default_offset,
        },
        "obs_dim": int(obs_mgr.group_obs_dim["policy"][0]),
        "obs_terms": policy_terms,
        "height_scan": None if scanner is None else {
            "offset_z": scanner.cfg.offset.pos[2],
            "ray_starts_body": tolist(scanner.ray_starts[0]),
            "attach_yaw_only": getattr(scanner.cfg, "ray_alignment", getattr(scanner.cfg, "attach_yaw_only", None)),
        },
        "commands_range": {
            "lin_vel_x": list(u.cfg.commands.base_velocity.ranges.lin_vel_x),
            "ang_vel_z": list(u.cfg.commands.base_velocity.ranges.ang_vel_z),
        },
    }
    with open(os.path.join(out_dir, "deploy.json"), "w") as f:
        json.dump(info, f, indent=1, ensure_ascii=False, default=str)
    env.close()


if __name__ == "__main__":
    try:
        main()
    except BaseException:
        import traceback
        with open("/root/isaac/export_policy.err", "w") as f:
            traceback.print_exc(file=f)
        raise
    finally:
        simulation_app.close()
