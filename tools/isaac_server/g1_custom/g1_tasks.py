"""注册自定义 G1 任务（不改 Isaac Lab 源码）。只用字符串入口，导入时不依赖 Isaac Sim 已启动。"""
import gymnasium as gym

_AGENT = "isaaclab_tasks.manager_based.locomotion.velocity.config.g1.agents.rsl_rl_ppo_cfg:G1RoughPPORunnerCfg"

gym.register(
    id="G1-Rough-Height-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={"env_cfg_entry_point": "g1_height_cfg:G1RoughHeightEnvCfg", "rsl_rl_cfg_entry_point": _AGENT},
)
gym.register(
    id="G1-Rough-Rand-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={"env_cfg_entry_point": "g1_rand_cfg:G1RoughRandEnvCfg", "rsl_rl_cfg_entry_point": _AGENT},
)
gym.register(
    id="G1-Rough-Height-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={"env_cfg_entry_point": "g1_height_cfg:G1RoughHeightEnvCfg_PLAY", "rsl_rl_cfg_entry_point": _AGENT},
)
