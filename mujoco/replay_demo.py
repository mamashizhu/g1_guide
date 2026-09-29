"""把 Isaac 里记录的 G1 运动（record_demo.py 导出）在 MuJoCo 里逐帧回放，渲染成 mp4。

不做物理仿真，只按记录的位姿摆放机器人，所以画面就是 Isaac 里真实发生的动作。
每个机器人一格，2×2 拼在一起，镜头跟着各自的机器人。

    python mujoco/replay_demo.py mujoco/policies/g1_rough_n16384_mb8/demo
输出：<目录>/demo.mp4
"""
import argparse
import os

import cv2
import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = os.path.join(HERE, "menagerie_g1_2024", "scene.xml")

LABELS = {"stairs_up": "Stairs up", "stairs_down": "Stairs down", "slope_up": "Slope up", "slope_down": "Slope down"}


def label_text(tag):
    kind, v = tag.rsplit("_", 1)
    v = float(v)
    unit = f"{v * 100:.0f} cm" if kind.startswith("stairs") else f"{v:.0f} deg"
    return f"{LABELS.get(kind, kind)} {unit}"


def build_model(stl):
    spec = mujoco.MjSpec.from_file(SCENE)
    spec.geom("floor").rgba = [0, 0, 0, 0]   # 地形网格自带地面，原来的平面藏起来
    mesh = spec.add_mesh()
    mesh.name = "terrain"
    mesh.file = os.path.abspath(stl)
    mesh.inertia = mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL
    g = spec.worldbody.add_geom()
    g.type = mujoco.mjtGeom.mjGEOM_MESH
    g.meshname = "terrain"
    g.contype = g.conaffinity = 0
    g.rgba = [0.55, 0.57, 0.6, 1]
    return spec.compile()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("demo_dir", help="含 traj.npz 和 terrain.stl 的目录")
    p.add_argument("--width", type=int, default=640, help="每格宽度")
    p.add_argument("--height", type=int, default=400, help="每格高度")
    p.add_argument("--fps", type=int, default=50)
    p.add_argument("--azimuth", type=float, default=120.0, help="镜头方位角（度）")
    a = p.parse_args()

    t = np.load(os.path.join(a.demo_dir, "traj.npz"))
    pos, quat, qj = t["pos"], t["quat"], t["joint_pos"]
    names = [str(n) for n in t["joint_names"]]
    labels = [label_text(str(s)) for s in t["labels"]]
    frames, n_robot = pos.shape[:2]

    m = build_model(os.path.join(a.demo_dir, "terrain.stl"))
    d = mujoco.MjData(m)
    qadr = np.array([m.jnt_qposadr[m.joint(j).id] for j in names])
    r = mujoco.Renderer(m, a.height, a.width)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance, cam.elevation, cam.azimuth = 3.2, -18.0, a.azimuth

    cols = 2 if n_robot > 1 else 1
    rows = (n_robot + cols - 1) // cols
    out = os.path.join(a.demo_dir, "demo.mp4")
    vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), a.fps, (a.width * cols, a.height * rows))
    for f in range(frames):
        canvas = np.zeros((a.height * rows, a.width * cols, 3), np.uint8)
        for i in range(n_robot):
            d.qpos[0:3] = pos[f, i]
            d.qpos[3:7] = quat[f, i]
            d.qpos[qadr] = qj[f, i]
            mujoco.mj_kinematics(m, d)
            cam.lookat[:] = pos[f, i] + np.array([0, 0, -0.2])
            r.update_scene(d, camera=cam)
            img = cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR)
            cv2.putText(img, labels[i], (14, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)
            y, x = divmod(i, cols)
            canvas[y * a.height:(y + 1) * a.height, x * a.width:(x + 1) * a.width] = img
        cv2.putText(canvas, f"t = {f / a.fps:4.1f} s", (a.width * cols - 170, a.height * rows - 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        vw.write(canvas)
    vw.release()
    print(f"已写出 {out}：{frames} 帧，{frames / a.fps:.1f} 秒，{n_robot} 个机器人")


if __name__ == "__main__":
    main()
