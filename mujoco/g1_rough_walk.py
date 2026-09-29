"""MuJoCo 里运行 Isaac Lab 训练的 G1 崎岖地形策略（上坡、上楼梯），做 sim2sim 验证。

策略：policies/<运行名>/exported/policy.pt，参数全部来自同目录的 deploy.json（训练时导出）。
模型：MuJoCo Menagerie 2024-07 版 G1（menagerie_g1_2024/），37 个关节与 Isaac Lab 的 g1_minimal 一一对应。
场地：terrain.py 的 6 条跑道（坡 5/10/15°、台阶 10/15/18 cm）。

运行（Windows 端 Python）：
    python mujoco/g1_rough_walk.py                              # 站在场地中间，方向键自己开
    python mujoco/g1_rough_walk.py --course stairs15            # 站在该跑道前自动往前走
    python mujoco/g1_rough_walk.py --course stairs15 --check    # 无窗口判定能否走完
    python mujoco/g1_rough_walk.py --check-all                  # 无窗口跑全部 6 条跑道

窗口里的按键：↑/↓ 前进速度 ±0.1 m/s（0～1），←/→ 转向 ±0.2 rad/s，Home 停，End 复位，空格暂停。
"""
import argparse
import json
import os
import time

import mujoco
import mujoco.viewer
import numpy as np
import torch

import terrain
from terrain import BY_NAME, LANE_WIDTH

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = os.path.join(HERE, "menagerie_g1_2024", "scene.xml")
POLICY_DIR = os.path.join(HERE, "policies", "g1_rough_n16384_mb8", "exported")

# Isaac Lab G1_CFG 的力矩上限（isaaclab_assets/robots/unitree.py），比 MuJoCo 模型里的 ctrlrange 宽。
# 按训练时的来，否则策略要的力矩给不出来。
EFFORT_LIMIT = [("ankle", 20.0), ("", 300.0)]
INIT_HEIGHT = 0.74      # G1_CFG.init_state.pos 的 z
SCAN_OFFSET = 0.5       # height_scan 观测里减掉的常数（mdp.height_scan 的 offset）
FALL_HEIGHT = 0.45      # 骨盆离脚下地面低于此高度视为摔倒
FALL_TILT = -0.5        # 机身 z 轴上的重力投影，直立 -1，大于此值（倾斜超过 60°）视为摔倒
CMD_MIN = np.array([0.0, 0.0, -1.0], dtype=np.float32)   # 训练时的指令范围：vx 0～1，不练横移
CMD_MAX = np.array([1.0, 0.0, 1.0], dtype=np.float32)
HEADING_STIFFNESS = 0.5  # Isaac Lab velocity_env_cfg 的 heading_control_stiffness
SCAN_GROUPS = np.array([1, 1, 0, 0, 0, 0], dtype=np.uint8)  # 射线只打 group 0/1（地面和跑道），机器人在 2/3

KEY_RIGHT, KEY_LEFT, KEY_DOWN, KEY_UP = 262, 263, 264, 265
KEY_HOME, KEY_END = 268, 269


def quat_to_mat(q):
    m = np.zeros(9)
    mujoco.mju_quat2Mat(m, q)
    return m.reshape(3, 3)


class RoughWalker:
    def __init__(self, cmd, course=None, policy_dir=POLICY_DIR, dt=None):
        with open(os.path.join(policy_dir, "deploy.json"), encoding="utf-8") as f:
            cfg = json.load(f)
        self.names = cfg["joint_names"]
        self.default = np.array(cfg["default_joint_pos"], dtype=np.float64)
        self.kp = np.array([cfg["stiffness"][j] for j in self.names])
        self.kd = np.array([cfg["damping"][j] for j in self.names])
        self.limit = np.array([next(v for k, v in EFFORT_LIMIT if k in j) for j in self.names])
        self.action_scale = cfg["action"]["scale"]
        self.control_dt = cfg["control_dt"]
        self.dt = dt or cfg["sim_dt"]
        self.substeps = max(1, round(self.control_dt / self.dt))
        self.ray_starts = np.array(cfg["height_scan"]["ray_starts_body"])   # 已含 z=+20 的偏移
        self.n = len(self.names)
        self.obs_dim = cfg["obs_dim"]
        self.policy = torch.jit.load(os.path.join(policy_dir, "policy.pt"))
        self.policy.eval()

        self.course = course
        self.m = terrain.build_spec(SCENE).compile()
        self.m.opt.timestep = self.dt
        # Isaac 里关节没有阻尼和摩擦（PD 全在执行器里），MuJoCo 模型默认有，清零保持一致
        self.m.dof_damping[6:] = 0
        self.m.dof_frictionloss[6:] = 0

        # Isaac 关节顺序 → MuJoCo 的 qpos / qvel / 执行器下标
        jid = [self.m.joint(j).id for j in self.names]
        self.qadr = np.array([self.m.jnt_qposadr[i] for i in jid])
        self.vadr = np.array([self.m.jnt_dofadr[i] for i in jid])
        act_of_joint = {self.m.actuator_trnid[a, 0]: a for a in range(self.m.nu)}
        self.aid = np.array([act_of_joint[i] for i in jid])

        # Isaac 的关节 PD 是隐式积分（ImplicitActuator），阻尼大、手指很轻也不会发散。
        # 显式算力矩在 0.005 s 步长下手指会振荡发散，所以改成 MuJoCo 位置伺服
        # （力 = kp·(ctrl−q) − kd·q̇），配合 implicitfast 积分器隐式处理阻尼。
        for k, a in enumerate(self.aid):
            self.m.actuator_gaintype[a] = mujoco.mjtGain.mjGAIN_FIXED
            self.m.actuator_gainprm[a, :3] = [self.kp[k], 0, 0]
            self.m.actuator_biastype[a] = mujoco.mjtBias.mjBIAS_AFFINE
            self.m.actuator_biasprm[a, :3] = [0, -self.kp[k], -self.kd[k]]
            self.m.actuator_ctrllimited[a] = 0
            self.m.actuator_forcelimited[a] = 1
            self.m.actuator_forcerange[a] = [-self.limit[k], self.limit[k]]
        self.m.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        self.d = mujoco.MjData(self.m)
        self.pelvis = self.m.body("pelvis").id
        self.torso = self.m.body("torso_link").id

        self.cmd = np.clip(np.array(cmd, dtype=np.float32), CMD_MIN, CMD_MAX)
        self.heading = None   # 设成目标朝向（弧度）后，转向指令由朝向误差自动算，和训练时一致
        if course in BY_NAME:
            self.spawn = (0.0, BY_NAME[course].y)
        else:
            self.spawn = (0.0, 0.0)
        self._down = np.array([0.0, 0.0, -1.0])
        self._ray_geom = np.zeros(1, dtype=np.int32)
        self._ray_dist = np.zeros(len(self.ray_starts))
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.m, self.d)
        self.d.qpos[0:2] = self.spawn
        self.d.qpos[2] = INIT_HEIGHT
        self.d.qpos[3:7] = [1, 0, 0, 0]
        self.d.qpos[self.qadr] = self.default
        self.d.ctrl[self.aid] = self.default
        mujoco.mj_forward(self.m, self.d)
        self.action = np.zeros(self.n, dtype=np.float32)
        self.target = self.default.copy()
        self.steps = 0

    def height_scan(self):
        d = self.d
        R = quat_to_mat(d.xquat[self.torso])
        yaw = np.arctan2(R[1, 0], R[0, 0])
        c, s = np.cos(yaw), np.sin(yaw)
        rs = self.ray_starts
        starts = np.empty_like(rs)
        starts[:, 0] = c * rs[:, 0] - s * rs[:, 1]
        starts[:, 1] = s * rs[:, 0] + c * rs[:, 1]
        starts[:, 2] = rs[:, 2]
        starts += d.xpos[self.torso]
        # mj_multiRay 要求所有射线同一起点，这里起点各不相同，只能逐条打
        for i in range(len(rs)):
            self._ray_dist[i] = mujoco.mj_ray(self.m, d, starts[i], self._down, SCAN_GROUPS, 1, -1, self._ray_geom)
        hit_z = starts[:, 2] - np.where(self._ray_dist >= 0, self._ray_dist, np.inf)
        return np.clip(d.xpos[self.torso][2] - hit_z - SCAN_OFFSET, -1.0, 1.0)

    def observe(self):
        d = self.d
        R = quat_to_mat(d.qpos[3:7])
        obs = np.concatenate([
            R.T @ d.qvel[0:3],              # 机身线速度（机身坐标系）
            d.qvel[3:6],                    # 机身角速度（MuJoCo 自由关节已是机身坐标系）
            R.T @ np.array([0, 0, -1.0]),   # 重力方向
            self.cmd,
            d.qpos[self.qadr] - self.default,
            d.qvel[self.vadr],
            self.action,
            self.height_scan(),
        ]).astype(np.float32)
        assert obs.shape[0] == self.obs_dim, obs.shape
        return obs

    def hold_heading(self):
        """训练时所有机器人的转向指令都由朝向误差实时算出（heading_command，系数 0.5，限幅 ±1），
        策略从没见过"转向指令恒为 0、身体却已偏了"的情况。这里按同样方式算 wz。"""
        R = quat_to_mat(self.d.qpos[3:7])
        yaw = np.arctan2(R[1, 0], R[0, 0])
        err = (self.heading - yaw + np.pi) % (2 * np.pi) - np.pi
        self.cmd[2] = np.clip(HEADING_STIFFNESS * err, CMD_MIN[2], CMD_MAX[2])

    def control_step(self):
        """算一次策略，再按训练时的 PD 跑 substeps 个物理步。"""
        if self.heading is not None:
            self.hold_heading()
        with torch.no_grad():
            self.action = self.policy(torch.from_numpy(self.observe()).unsqueeze(0)).numpy().squeeze()
        self.target = self.default + self.action_scale * self.action
        self.d.ctrl[self.aid] = self.target   # 位置伺服，PD 由执行器算
        for _ in range(self.substeps):
            mujoco.mj_step(self.m, self.d)
        self.steps += 1

    @property
    def time(self):
        return self.steps * self.control_dt

    def ground(self):
        x, y = self.d.xpos[self.pelvis][:2]
        for c in terrain.COURSES:
            if abs(y - c.y) < LANE_WIDTH / 2:
                return c.height(x)
        return 0.0

    def fallen(self):
        R = quat_to_mat(self.d.qpos[3:7])
        tilt = (R.T @ np.array([0, 0, -1.0]))[2]
        return tilt > FALL_TILT or self.d.xpos[self.pelvis][2] - self.ground() < FALL_HEIGHT

    def on_key(self, key):
        delta = {KEY_UP: (0, 0.1), KEY_DOWN: (0, -0.1), KEY_LEFT: (2, 0.2), KEY_RIGHT: (2, -0.2)}
        if key in delta:
            i, dv = delta[key]
            self.cmd[i] = np.clip(round(float(self.cmd[i]) + dv, 2), CMD_MIN[i], CMD_MAX[i])
        elif key == KEY_HOME:
            self.cmd[:] = 0
        elif key == KEY_END:
            self.reset()
        else:
            return
        print(f"速度指令 vx={self.cmd[0]:+.1f} m/s  wz={self.cmd[2]:+.1f} rad/s")


def run_viewer(w):
    keys = []
    with mujoco.viewer.launch_passive(w.m, w.d, key_callback=keys.append) as v:
        v.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        v.cam.trackbodyid = w.pelvis
        v.cam.distance, v.cam.elevation, v.cam.azimuth = 5.0, -20, 150
        print("窗口已打开。↑↓ 前进速度，←→ 转向，Home 停，End 复位。")
        print(f"速度指令 vx={w.cmd[0]:+.1f} m/s  wz={w.cmd[2]:+.1f} rad/s")
        while v.is_running():
            t0 = time.time()
            with v.lock():
                while keys:
                    w.on_key(keys.pop(0))
                w.control_step()
                if w.fallen():
                    print(f"第 {w.time:.1f} s 摔倒了，自动复位")
                    w.reset()
            v.sync()
            left = w.control_dt - (time.time() - t0)
            if left > 0:
                time.sleep(left)


def check_course(name, vx, seconds, dt=None, policy_dir=POLICY_DIR):
    """沿跑道直走，走过终点 0.5 m、没摔倒、没走出跑道算通过。"""
    w = RoughWalker([vx, 0, 0], name, policy_dir=policy_dir, dt=dt)
    w.heading = 0.0   # 朝 +x（跑道方向）保持朝向，和 Isaac 评估时一样
    c = BY_NAME[name]
    top = 0.0
    t0 = time.time()
    while w.time < seconds:
        w.control_step()
        x, y = w.d.xpos[w.pelvis][:2]
        top = max(top, w.ground())
        if w.fallen():
            print(f"{name:9s} 不合格：第 {w.time:.1f} s 在 x={x:.2f} m 摔倒，最高爬到 {top:.2f} / {c.rise:.2f} m")
            return False
        # 以前没查这条，机器人横向偏了 5 m 绕过台阶也算"合格"
        if terrain.START < x < c.end and abs(y - c.y) > LANE_WIDTH / 2:
            print(f"{name:9s} 不合格：第 {w.time:.1f} s 在 x={x:.2f} m 走出跑道（横向偏移 {y - c.y:+.2f} m），"
                  f"最高爬到 {top:.2f} / {c.rise:.2f} m")
            return False
        if x > c.end + 0.5:
            print(f"{name:9s} 合格：{w.time:.1f} s 走完全程，爬升 {top:.2f} m，横向偏移 {y - c.y:+.2f} m"
                  f"（耗时 {time.time() - t0:.1f} s）")
            return True
    print(f"{name:9s} 不合格：{seconds:.0f} s 内没走完，停在 x={x:.2f} m，最高爬到 {top:.2f} / {c.rise:.2f} m")
    return False


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--course", choices=[*BY_NAME, "field"], default="field", help="站到哪条跑道前（默认场地中间）")
    p.add_argument("--vx", type=float, help="初始前进速度 m/s（跑道默认 0.5，场地中间默认 0）")
    p.add_argument("--wz", type=float, default=0.0, help="初始转向速度 rad/s")
    p.add_argument("--check", action="store_true", help="无窗口判定能否走完 --course 指定的跑道")
    p.add_argument("--check-all", action="store_true", help="无窗口跑全部 6 条跑道")
    p.add_argument("--seconds", type=float, default=40.0, help="判定时长")
    # 0.005 s（和 Isaac 一样）时 MuJoCo 接触不够准：rand070 策略 6 条跑道过 1 条，0.002 s 过 3 条（2026-09-29）
    p.add_argument("--dt", type=float, default=0.002, help="物理步长，默认 0.002 s（每个控制周期 10 步）")
    p.add_argument("--policy", default="g1_rough_n16384_mb8",
                   help="policies/ 下的策略目录名（g1_rough_n16384_mb8 原版半蹲、g1_rough_height072_n16384_mb8 站直、"
                        "g1_rough_rand070_n16384_mb8 站直+随机化）")
    a = p.parse_args()
    policy_dir = os.path.join(HERE, "policies", a.policy, "exported")
    print(f"策略：{a.policy}")
    vx = a.vx if a.vx is not None else (0.0 if a.course == "field" else 0.5)
    if a.check_all:
        ok = [check_course(c.name, vx or 0.5, a.seconds, a.dt, policy_dir) for c in terrain.COURSES]
        print(f"通过 {sum(ok)} / {len(ok)} 条跑道")
        raise SystemExit(0 if all(ok) else 1)
    if a.check:
        if a.course not in BY_NAME:
            raise SystemExit("--check 需要指定 --course 跑道名")
        raise SystemExit(0 if check_course(a.course, vx, a.seconds, a.dt, policy_dir) else 1)
    run_viewer(RoughWalker([vx, 0, a.wz], a.course, policy_dir=policy_dir, dt=a.dt))


if __name__ == "__main__":
    main()
