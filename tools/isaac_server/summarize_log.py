"""从 rsl_rl 训练日志里每隔 N 轮摘一行关键指标，写成表格。

    python summarize_log.py <训练日志> [--every 100] [--out summary.tsv]
不需要 Isaac 环境，任何 Python 3 都能跑；训练中途跑也行（只统计已有的轮次）。
"""
import argparse
import math
import re

FIELDS = [
    ("reward", "Mean reward"),
    ("ep_len", "Mean episode length"),
    ("terrain", "Curriculum/terrain_levels"),
    ("vel_err", "Metrics/base_velocity/error_vel_xy"),
    ("height_pen", "Episode_Reward/base_height"),
    ("steps_s", "Computation"),
]
ANSI = re.compile(r"\x1b\[[0-9;]*m")


def parse(path):
    rows, cur = [], None
    for line in open(path, encoding="utf-8", errors="replace"):
        line = ANSI.sub("", line).strip()
        m = re.match(r"Learning iteration (\d+)/(\d+)", line)
        if m:
            cur = {"iter": int(m.group(1))}
            rows.append(cur)
            continue
        if cur is None or ":" not in line:
            continue
        key, val = line.split(":", 1)
        for name, label in FIELDS:
            if key.strip() == label:
                num = re.match(r"\s*(-?[\d.]+)", val)
                if num:
                    cur[name] = float(num.group(1))
    return rows


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("log")
    p.add_argument("--every", type=int, default=100)
    p.add_argument("--weight", type=float, default=-5.0, help="身高奖励权重，用来把惩罚换算成高度偏差")
    p.add_argument("--out")
    a = p.parse_args()
    rows = parse(a.log)
    picked = [r for r in rows if r["iter"] % a.every == 0 or r is rows[-1]]
    head = "轮次\t平均奖励\t回合长度\t地形等级\t速度误差\t高度偏差cm\t步/秒"
    lines = [head]
    for r in picked:
        pen = r.get("height_pen")
        dev = f"{math.sqrt(max(pen / a.weight, 0)) * 100:.1f}" if pen is not None and a.weight else "-"
        lines.append("\t".join([
            str(r["iter"]), f"{r.get('reward', float('nan')):.2f}", f"{r.get('ep_len', float('nan')):.0f}",
            f"{r.get('terrain', float('nan')):.2f}", f"{r.get('vel_err', float('nan')):.3f}", dev,
            f"{r.get('steps_s', float('nan')):.0f}",
        ]))
    text = "\n".join(lines)
    print(text)
    if a.out:
        with open(a.out, "w", encoding="utf-8") as f:
            f.write(text + "\n")


if __name__ == "__main__":
    main()
