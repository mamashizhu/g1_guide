"""G1 崎岖地形任务 + 机身高度奖励。

原版 Isaac-Velocity-Rough-G1-v0 没有约束机身高度，策略学成了半蹲（骨盆离地约 0.49 m，直立约 0.74 m）。
这里只加一项：骨盆高度偏离目标的平方惩罚。目标高度相对脚下地面——用地形高度扫描的命中高度做基准，
楼梯、坡上也成立。其余奖励、地形、网络都和原版一样，便于对比。

第一次试训（2026-09-29）用 Isaac Lab 自带的 mdp.base_height_l2，约 90 轮后崩溃
（"normal expects all elements of std >= 0.0"）：它直接对所有射线命中高度求平均，
没打中东西的射线返回 inf，惩罚变成无穷大，梯度变 NaN。这里改用只统计有效命中、并限幅的版本。
"""
import torch

from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab_tasks.manager_based.locomotion.velocity.config.g1.rough_env_cfg import (
    G1RoughEnvCfg,
    G1RoughEnvCfg_PLAY,
    G1Rewards,
)

BASE_HEIGHT_TARGET = 0.72   # 骨盆离地目标高度（m）；默认站姿约 0.74，走路时膝盖略弯
BASE_HEIGHT_WEIGHT = -10.0  # 半蹲到 0.49 m 时每秒约扣 0.53；速度跟踪每秒最多 +1
PENALTY_MAX = 1.0           # 单步惩罚上限（高度偏差 1 m），防止异常值把训练带崩


def base_height_l2_safe(env, target_height: float, asset_cfg: SceneEntityCfg, sensor_cfg: SceneEntityCfg):
    """骨盆离地高度偏差的平方。地面 = 有效射线命中高度的平均（忽略没打中的 inf/nan）。"""
    asset = env.scene[asset_cfg.name]
    hits = env.scene.sensors[sensor_cfg.name].data.ray_hits_w[..., 2]
    valid = torch.isfinite(hits)
    ground = torch.where(valid, hits, 0.0).sum(dim=1) / valid.sum(dim=1).clamp(min=1)
    err = asset.data.root_pos_w[:, 2] - (ground + target_height)
    return torch.nan_to_num(torch.square(err), nan=PENALTY_MAX).clamp(max=PENALTY_MAX)


@configclass
class G1HeightRewards(G1Rewards):
    base_height = RewTerm(
        func=base_height_l2_safe,
        weight=BASE_HEIGHT_WEIGHT,
        params={
            "target_height": BASE_HEIGHT_TARGET,
            "asset_cfg": SceneEntityCfg("robot"),
            "sensor_cfg": SceneEntityCfg("height_scanner"),
        },
    )


@configclass
class G1RoughHeightEnvCfg(G1RoughEnvCfg):
    rewards: G1HeightRewards = G1HeightRewards()


@configclass
class G1RoughHeightEnvCfg_PLAY(G1RoughEnvCfg_PLAY):
    rewards: G1HeightRewards = G1HeightRewards()
