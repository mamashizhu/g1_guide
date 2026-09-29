"""G1 崎岖地形 + 放宽的机身高度奖励 + 域随机化（为 sim2sim / 真机做准备）。

在 g1_height_cfg 的基础上：
- 身高目标 0.72 → 0.70 m，权重 −10 → −5：上一版站直后 20° 坡通过率从 98% 掉到 78%。
- 打开 Isaac Lab 的 G1 配置特意关掉的随机化（推、躯干质量、质心），并随机摩擦和关节 PD 增益。
  上一版策略只在固定参数下练过，换到 MuJoCo 后 6 条跑道只过 1 条。
"""
import isaaclab.envs.mdp as mdp
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab_tasks.manager_based.locomotion.velocity.config.g1.rough_env_cfg import (
    G1RoughEnvCfg,
    G1RoughEnvCfg_PLAY,
    G1Rewards,
)

from g1_height_cfg import base_height_l2_safe

BASE_HEIGHT_TARGET = 0.70
BASE_HEIGHT_WEIGHT = -5.0


@configclass
class G1RandRewards(G1Rewards):
    base_height = RewTerm(
        func=base_height_l2_safe,
        weight=BASE_HEIGHT_WEIGHT,
        params={
            "target_height": BASE_HEIGHT_TARGET,
            "asset_cfg": SceneEntityCfg("robot"),
            "sensor_cfg": SceneEntityCfg("height_scanner"),
        },
    )


def add_randomization(cfg):
    ev = cfg.events
    # 脚与地面的摩擦（原来固定 0.8 / 0.6）
    ev.physics_material.params["static_friction_range"] = (0.5, 1.25)
    ev.physics_material.params["dynamic_friction_range"] = (0.4, 1.0)
    ev.physics_material.params["make_consistent"] = True   # 保证动摩擦 ≤ 静摩擦
    # 躯干质量（torso_link 约 7.5 kg，整机约 32 kg）
    ev.add_base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="torso_link"),
            "mass_distribution_params": (-2.0, 3.0),
            "operation": "add",
        },
    )
    # 躯干质心
    ev.base_com = EventTerm(
        func=mdp.randomize_rigid_body_com,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="torso_link"),
            "com_range": {"x": (-0.05, 0.05), "y": (-0.05, 0.05), "z": (-0.01, 0.01)},
        },
    )
    # 每 10～15 s 随机推一下（直接改机身速度）
    ev.push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(10.0, 15.0),
        params={"velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)}},
    )
    # 关节 PD 增益 ×0.8～1.2（MuJoCo 和真机的执行器都和 Isaac 不完全一样）
    ev.actuator_gains = EventTerm(
        func=mdp.randomize_actuator_gains,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "stiffness_distribution_params": (0.8, 1.2),
            "damping_distribution_params": (0.8, 1.2),
            "operation": "scale",
        },
    )


@configclass
class G1RoughRandEnvCfg(G1RoughEnvCfg):
    rewards: G1RandRewards = G1RandRewards()

    def __post_init__(self):
        super().__post_init__()
        add_randomization(self)


@configclass
class G1RoughRandEnvCfg_PLAY(G1RoughEnvCfg_PLAY):
    rewards: G1RandRewards = G1RandRewards()
