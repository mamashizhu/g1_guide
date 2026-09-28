"""MuJoCo 里用宇树官方预训练策略让 G1 走路（12 自由度腿部模型）。

改写自 unitree_rl_gym/deploy/deploy_mujoco/deploy_mujoco.py，去掉对 legged_gym
（需要 Isaac Gym）的依赖，加上键盘改速度、摔倒自动复位和无窗口自检。

运行（Windows 端 Python）：
    python mujoco/g1_walk.py            # 打开窗口
    python mujoco/g1_walk.py --check    # 无窗口跑 10 s，打印走了多远、有没有摔
    python mujoco/g1_walk.py --course stairs15          # 斜坡楼梯场地，站在该跑道前
    python mujoco/g1_walk.py --course stairs15 --check  # 无窗口判定能否走完该跑道
跑道：slope5/10/15（坡度°）、stairs10/15/18（台阶 cm）、field（站在场地中间自己开）。

窗口里的按键（在 MuJoCo 窗口内按）：
    ↑ / ↓        前进速度 +/- 0.1 m/s
    ← / →        转向速度 +/- 0.2 rad/s（← 为左转）
    PgUp / PgDn  横移速度 +/- 0.1 m/s（PgUp 向左）
    Home         速度清零（原地踏步）
    End          复位机器人
    空格          暂停（MuJoCo 自带）
"""
import argparse
import os
import time

import mujoco
import mujoco.viewer
import numpy as np
import torch
import yaml

import terrain
from terrain import BY_NAME, LANE_WIDTH

HERE = os.path.dirname(os.path.abspath(__file__))
RL_GYM = os.path.join(HERE, "unitree_rl_gym")
CONFIG = os.path.join(RL_GYM, "deploy", "deploy_mujoco", "configs", "g1.yaml")

# 与训练时的限幅一致，超出范围策略会走不稳
CMD_MIN = np.array([-0.5, -0.5, -1.0], dtype=np.float32)
CMD_MAX = np.array([1.0, 0.5, 1.0], dtype=np.float32)
FALL_HEIGHT = 0.45  # 骨盆离脚下地面低于此高度视为摔倒
FALL_TILT = -0.5    # 重力在机身 z 轴的投影，直立为 -1，大于此值（倾斜超过 60°）视为摔倒

KEY_RIGHT, KEY_LEFT, KEY_DOWN, KEY_UP = 262, 263, 264, 265
KEY_PAGE_UP, KEY_PAGE_DOWN, KEY_HOME, KEY_END = 266, 267, 268, 269


def gravity_orientation(q):
    qw, qx, qy, qz = q
    return np.array([
        2 * (-qz * qx + qw * qy),
        -2 * (qz * qy + qw * qx),
        1 - 2 * (qw * qw + qz * qz),
    ])


class G1Walker:
    def __init__(self, cmd, course=None):
        if not os.path.isdir(RL_GYM):
            raise SystemExit(f"找不到 {RL_GYM}\n先按 README 的 MuJoCo 一节克隆 unitree_rl_gym。")
        with open(CONFIG, encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        sub = lambda p: p.replace("{LEGGED_GYM_ROOT_DIR}", RL_GYM)
        self.dt = cfg["simulation_dt"]
        self.decimation = cfg["control_decimation"]
        self.kps = np.array(cfg["kps"], dtype=np.float32)
        self.kds = np.array(cfg["kds"], dtype=np.float32)
        self.default = np.array(cfg["default_angles"], dtype=np.float32)
        self.ang_vel_scale = cfg["ang_vel_scale"]
        self.dof_pos_scale = cfg["dof_pos_scale"]
        self.dof_vel_scale = cfg["dof_vel_scale"]
        self.action_scale = cfg["action_scale"]
        self.cmd_scale = np.array(cfg["cmd_scale"], dtype=np.float32)
        self.n_act = cfg["num_actions"]
        self.obs = np.zeros(cfg["num_obs"], dtype=np.float32)
        self.cmd = np.clip(np.array(cmd, dtype=np.float32), CMD_MIN, CMD_MAX)

        self.course = course
        if course is None:
            self.m = mujoco.MjModel.from_xml_path(sub(cfg["xml_path"]))
            self.spawn = (0.0, 0.0)
        else:  # 斜坡楼梯场地，站在所选跑道的起点前
            self.m = terrain.build_spec().compile()
            self.spawn = (0.0, BY_NAME[course].y if course in BY_NAME else 0.0)
        self.m.opt.timestep = self.dt
        self.d = mujoco.MjData(self.m)
        self.policy = torch.jit.load(sub(cfg["policy_path"]))
        self.pelvis = self.m.body("pelvis").id
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.m, self.d)
        self.d.qpos[:2] = self.spawn
        self.d.qpos[7:] = self.default
        mujoco.mj_forward(self.m, self.d)
        self.action = np.zeros(self.n_act, dtype=np.float32)
        self.target = self.default.copy()
        self.counter = 0

    def step(self):
        d = self.d
        d.ctrl[:] = (self.target - d.qpos[7:]) * self.kps - d.qvel[6:] * self.kds
        mujoco.mj_step(self.m, d)
        self.counter += 1
        if self.counter % self.decimation:
            return
        n = self.n_act
        phase = (self.counter * self.dt) % 0.8 / 0.8
        o = self.obs
        o[:3] = d.qvel[3:6] * self.ang_vel_scale
        o[3:6] = gravity_orientation(d.qpos[3:7])
        o[6:9] = self.cmd * self.cmd_scale
        o[9:9 + n] = (d.qpos[7:] - self.default) * self.dof_pos_scale
        o[9 + n:9 + 2 * n] = d.qvel[6:] * self.dof_vel_scale
        o[9 + 2 * n:9 + 3 * n] = self.action
        o[9 + 3 * n:9 + 3 * n + 2] = (np.sin(2 * np.pi * phase), np.cos(2 * np.pi * phase))
        with torch.no_grad():
            self.action = self.policy(torch.from_numpy(o).unsqueeze(0)).numpy().squeeze()
        self.target = self.action * self.action_scale + self.default

    def ground(self):
        """机器人脚下的设计地面高度（平地模型为 0）。"""
        x, y = self.d.xpos[self.pelvis][:2]
        if self.course is not None:
            for c in terrain.COURSES:
                if abs(y - c.y) < LANE_WIDTH / 2:
                    return c.height(x)
        return 0.0

    def fallen(self):
        tilt = gravity_orientation(self.d.qpos[3:7])[2]
        return tilt > FALL_TILT or self.d.xpos[self.pelvis][2] - self.ground() < FALL_HEIGHT

    def on_key(self, key):
        delta = {
            KEY_UP: (0, 0.1), KEY_DOWN: (0, -0.1),
            KEY_PAGE_UP: (1, 0.1), KEY_PAGE_DOWN: (1, -0.1),
            KEY_LEFT: (2, 0.2), KEY_RIGHT: (2, -0.2),
        }
        if key in delta:
            i, dv = delta[key]
            self.cmd[i] = np.clip(round(self.cmd[i] + dv, 2), CMD_MIN[i], CMD_MAX[i])
        elif key == KEY_HOME:
            self.cmd[:] = 0
        elif key == KEY_END:
            self.reset()
        else:
            return
        print(f"速度指令 vx={self.cmd[0]:+.1f} m/s  vy={self.cmd[1]:+.1f} m/s  wz={self.cmd[2]:+.1f} rad/s")


def run_viewer(w):
    keys = []  # 回调在窗口线程里触发，放到主循环处理，避免和仿真步抢数据
    with mujoco.viewer.launch_passive(w.m, w.d, key_callback=keys.append) as v:
        v.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        v.cam.trackbodyid = w.pelvis
        v.cam.distance, v.cam.elevation, v.cam.azimuth = 3.0, -15, 135
        if w.course is not None:  # 场地模式拉远，能看到整条跑道
            v.cam.distance, v.cam.elevation, v.cam.azimuth = 7.0, -25, 150
        print("窗口已打开。↑↓ 前后，←→ 转向，PgUp/PgDn 横移，Home 停，End 复位。")
        print(f"速度指令 vx={w.cmd[0]:+.1f} m/s  vy={w.cmd[1]:+.1f} m/s  wz={w.cmd[2]:+.1f} rad/s")
        while v.is_running():
            t0 = time.time()
            with v.lock():
                while keys:
                    w.on_key(keys.pop(0))
                for _ in range(w.decimation):
                    w.step()
                if w.fallen():
                    print("摔倒了，自动复位")
                    w.reset()
            v.sync()
            left = w.dt * w.decimation - (time.time() - t0)
            if left > 0:
                time.sleep(left)


def run_check(w, seconds):
    # 按控制周期累计路程（转圈时起终点距离很短，不能用位移判断）
    last = w.d.xpos[w.pelvis][:2].copy()
    path = 0.0
    t0 = time.time()
    for _ in range(int(seconds / w.dt)):
        w.step()
        if w.fallen():
            print(f"不合格：第 {w.counter * w.dt:.1f} s 摔倒")
            return 1
        if w.counter % w.decimation == 0:
            cur = w.d.xpos[w.pelvis][:2].copy()
            path += np.linalg.norm(cur - last)
            last = cur
    speed = path / seconds
    want = float(np.hypot(w.cmd[0], w.cmd[1]))
    print(f"仿真 {seconds:.0f} s（耗时 {time.time() - t0:.1f} s）：没有摔倒，"
          f"路程 {path:.2f} m，平均 {speed:.2f} m/s，指令 {want:.2f} m/s，"
          f"骨盆高度 {w.d.xpos[w.pelvis][2]:.2f} m")
    ok = speed > 0.5 * want
    print("合格" if ok else "不合格：几乎没走动")
    return 0 if ok else 1


def run_course_check(w, seconds):
    """沿跑道直走，走过跑道终点 0.5 m 且没摔倒算通过。"""
    c = BY_NAME[w.course]
    top = 0.0
    t0 = time.time()
    for _ in range(int(seconds / w.dt)):
        w.step()
        x, y = w.d.xpos[w.pelvis][:2]
        top = max(top, w.ground())
        if w.fallen():
            print(f"{c.name}: 不合格，第 {w.counter * w.dt:.1f} s 在 x={x:.2f} m 摔倒"
                  f"（跑道 {terrain.START:.1f}～{c.end:.1f} m），最高爬到 {top:.2f} / {c.rise:.2f} m")
            return 1
        if x > c.end + 0.5:
            print(f"{c.name}: 合格，{w.counter * w.dt:.1f} s 走完全程（耗时 {time.time() - t0:.1f} s），"
                  f"爬升 {top:.2f} m，横向偏移 {y - c.y:+.2f} m")
            return 0
    print(f"{c.name}: 不合格，{seconds:.0f} s 内没走完，停在 x={x:.2f} m，最高爬到 {top:.2f} / {c.rise:.2f} m")
    return 1


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--vx", type=float, default=0.5, help="初始前进速度 m/s（默认 0.5）")
    p.add_argument("--vy", type=float, default=0.0, help="初始横移速度 m/s")
    p.add_argument("--wz", type=float, default=0.0, help="初始转向速度 rad/s")
    p.add_argument("--check", action="store_true", help="无窗口自检")
    p.add_argument("--seconds", type=float, help="自检时长（默认平地 10 s，跑道 40 s）")
    p.add_argument("--course", choices=[*BY_NAME, "field"], help="加载斜坡楼梯场地并站到该跑道前")
    a = p.parse_args()
    w = G1Walker([a.vx, a.vy, a.wz], a.course)
    if a.check:
        if a.course in BY_NAME:
            raise SystemExit(run_course_check(w, a.seconds or 40.0))
        raise SystemExit(run_check(w, a.seconds or 10.0))
    run_viewer(w)


if __name__ == "__main__":
    main()
