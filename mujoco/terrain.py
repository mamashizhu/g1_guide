"""MuJoCo 斜坡、楼梯测试场地，用来验证上坡和上楼梯的策略。

6 条平行跑道，机器人从 x=0 出发沿 +x 走，跑道从 x=START 开始：
    坡道：上坡 → 平台 → 下坡；楼梯：上台阶 → 平台 → 下台阶。
场地叠加在宇树 scene.xml（G1 + 地面）上，用 MjSpec 加几何体，不改第三方文件。

    python mujoco/terrain.py            # 用射线实测每条跑道，与设计尺寸对比
    python mujoco/g1_walk.py --course stairs15   # 让机器人去走某条跑道
"""
import math
import os
from dataclasses import dataclass

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = os.path.join(HERE, "unitree_rl_gym", "resources", "robots", "g1_description", "scene.xml")

START = 2.0         # 跑道起点 x
LANE_WIDTH = 1.6    # 跑道宽度
LANE_GAP = 2.5      # 相邻跑道中心距
PLATFORM = 1.5      # 顶部平台长度
RAMP_RUN = 3.0      # 坡道水平长度
TREAD = 0.30        # 台阶踏面深度（与 Isaac Lab 楼梯地形一致）
STEPS = 5           # 单侧台阶数
THICK = 0.2         # 坡道板厚

SLOPE_RGBA = [0.55, 0.45, 0.35, 1]
STAIR_RGBA = [0.45, 0.50, 0.58, 1]
TOP_RGBA = [0.62, 0.62, 0.55, 1]


@dataclass
class Course:
    name: str
    kind: str       # "slope" | "stairs"
    value: float    # 坡度（度）或台阶高（米）
    y: float = 0.0

    @property
    def rise(self):
        if self.kind == "slope":
            return RAMP_RUN * math.tan(math.radians(self.value))
        return STEPS * self.value

    @property
    def run(self):
        return RAMP_RUN if self.kind == "slope" else STEPS * TREAD

    @property
    def end(self):
        return START + 2 * self.run + PLATFORM

    def height(self, x):
        """跑道中线上 x 处的设计地面高度。"""
        up_end = START + self.run
        down_start = up_end + PLATFORM
        if x < START or x > self.end:
            return 0.0
        if up_end <= x <= down_start:
            return self.rise
        d = x - START if x < up_end else self.end - x   # 离最近的坡脚/梯脚的水平距离
        if self.kind == "slope":
            return d * math.tan(math.radians(self.value))
        return min(STEPS, math.floor(d / TREAD) + 1) * self.value


COURSES = [
    Course("slope5", "slope", 5), Course("slope10", "slope", 10), Course("slope15", "slope", 15),
    Course("stairs10", "stairs", 0.10), Course("stairs15", "stairs", 0.15), Course("stairs18", "stairs", 0.18),
]
for _i, _c in enumerate(COURSES):
    _c.y = (_i - (len(COURSES) - 1) / 2) * LANE_GAP
BY_NAME = {c.name: c for c in COURSES}


def _box(body, name, center, half, rgba, pitch=0.0):
    g = body.add_geom()
    g.name = name
    g.type = mujoco.mjtGeom.mjGEOM_BOX
    g.pos = center
    g.size = half
    g.rgba = rgba
    if pitch:
        g.quat = [math.cos(pitch / 2), 0, math.sin(pitch / 2), 0]


def _add_course(wb, c):
    hw = LANE_WIDTH / 2
    top_x0 = START + c.run
    _box(wb, f"{c.name}_top", [top_x0 + PLATFORM / 2, c.y, c.rise / 2], [PLATFORM / 2, hw, c.rise / 2], TOP_RGBA)
    if c.kind == "slope":
        th = math.radians(c.value)
        L = c.run / math.cos(th)
        n = np.array([-math.sin(th), math.cos(th)])  # 上坡板面法向（x, z）
        for side, x_foot, sgn in (("up", START, 1), ("down", c.end, -1)):
            # 板面中点，再沿法向往下退半个板厚得到盒子中心
            mid = np.array([x_foot + sgn * c.run / 2, c.rise / 2])
            normal = np.array([sgn * n[0], n[1]])
            ctr = mid - normal * THICK / 2
            _box(wb, f"{c.name}_{side}", [ctr[0], c.y, ctr[1]], [L / 2, hw, THICK / 2], SLOPE_RGBA,
                 pitch=-sgn * th)
    else:
        for k in range(1, STEPS + 1):
            h = k * c.value
            for side, x0 in (("up", START + (k - 1) * TREAD), ("down", c.end - k * TREAD)):
                _box(wb, f"{c.name}_{side}{k}", [x0 + TREAD / 2, c.y, h / 2], [TREAD / 2, hw, h / 2], STAIR_RGBA)


def build_spec(scene=SCENE):
    """在给定机器人场景（默认宇树 12 自由度 scene.xml）上加 6 条跑道。场地几何体都在 group 0。"""
    if not os.path.isfile(scene):
        raise SystemExit(f"找不到 {scene}\n先按 README 的 MuJoCo 一节下载模型。")
    spec = mujoco.MjSpec.from_file(scene)
    for c in COURSES:
        _add_course(spec.worldbody, c)
    return spec


def measure(m, d, c, step=0.01):
    """沿跑道中线从上往下打射线，返回 (x 数组, 实测高度数组)。只看场地几何体。"""
    xs = np.arange(START - 0.5, c.end + 0.5, step)
    hs = np.empty_like(xs)
    gid = np.zeros(1, dtype=np.int32)
    down = np.array([0.0, 0.0, -1.0])
    for i, x in enumerate(xs):
        dist = mujoco.mj_ray(m, d, np.array([x, c.y, 5.0]), down, None, 1, -1, gid)
        hs[i] = 5.0 - dist if dist >= 0 else np.nan
    return xs, hs


def check_course(m, d, c):
    xs, hs = measure(m, d, c)
    design = np.array([c.height(x) for x in xs])
    # 台阶竖面正好落在采样点上时两边都对，只比较离竖面 > 1 cm 的点
    edges = [START + k * TREAD for k in range(STEPS + 1)] + [c.end - k * TREAD for k in range(STEPS + 1)]
    mask = np.ones_like(xs, bool)
    if c.kind == "stairs":
        for e in edges:
            mask &= np.abs(xs - e) > 0.011
    err = np.nanmax(np.abs(hs - design)[mask])
    if c.kind == "slope":
        seg = (xs > START + 0.2) & (xs < START + c.run - 0.2)
        k = np.polyfit(xs[seg], hs[seg], 1)[0]
        got = f"实测坡度 {math.degrees(math.atan(k)):.2f}°"
    else:
        risers = np.diff(hs[:len(hs) // 2])
        jumps = risers[risers > 0.02]
        got = f"实测台阶 {len(jumps)} 级，高 {jumps.min() * 100:.1f}～{jumps.max() * 100:.1f} cm"
    want = f"{c.value:.0f}°" if c.kind == "slope" else f"{STEPS} 级 × {c.value * 100:.0f} cm"
    ok = err < 0.005
    print(f"{c.name:9s} y={c.y:+5.2f}  设计 {want:>12s}  {got}  顶高 {np.nanmax(hs):.3f} m"
          f"  最大高度误差 {err * 1000:.1f} mm  {'合格' if ok else '不合格'}")
    return ok


def main():
    m = build_spec().compile()
    d = mujoco.MjData(m)
    d.qpos[:3] = [-3, 0, 0.8]          # 机器人放远处，免得挡住射线
    mujoco.mj_forward(m, d)
    print(f"场地：{len(COURSES)} 条跑道，从 x={START} m 开始，宽 {LANE_WIDTH} m，踏面 {TREAD * 100:.0f} cm")
    ok = all([check_course(m, d, c) for c in COURSES])
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
