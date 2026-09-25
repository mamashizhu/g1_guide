"""Pure acceptance calculations, kept independent from ROS message transport."""
import math


def settled_window(rows, phase, targets):
    selected = [r for r in rows if r[0] == phase]
    if not selected:
        raise ValueError("Missing phase samples")
    end = max(r[1] for r in selected)
    window = [r for r in selected if r[1] >= end - 1.0]
    errors, speeds, averages = [], [], {}
    for name, target in targets.items():
        joint = [r for r in window if r[2] == name]
        stamps = sorted(set(r[1] for r in joint))
        if len(stamps) < 50 or stamps[-1] - stamps[0] < 0.95:
            raise ValueError(f"Insufficient one-second settled window: {name}")
        if max(b - a for a, b in zip(stamps, stamps[1:])) > 0.05:
            raise ValueError(f"Discontinuous settled feedback: {name}")
        if any(not all(math.isfinite(v) for v in r[3:]) for r in joint):
            raise ValueError(f"Non-finite settled data: {name}")
        if max(abs(r[3] - target) for r in joint) > 0.005:
            raise ValueError(f"Reference not settled at requested target: {name}")
        errors.extend(target - r[4] for r in joint)
        speeds.extend(abs(r[5]) for r in joint)
        averages[name] = sum(r[4] for r in joint) / len(joint)
    metrics = dict(max_error_rad=max(map(abs, errors)),
                   rms_error_rad=math.sqrt(sum(e * e for e in errors) / len(errors)),
                   max_speed_rad_s=max(speeds), window_sim_s=1.0, samples=len(window),
                   positions=averages, sim_s=end)
    if metrics["max_error_rad"] > 0.12 or metrics["rms_error_rad"] > 0.08 or metrics["max_speed_rad_s"] > 0.1:
        raise ValueError(f"Unsettled tracking window: {metrics}")
    return metrics


def live_group_members(pgid):
    """Inspect only the exact group created by the probe; zombies cannot run."""
    from pathlib import Path
    members = []
    for path in Path("/proc").glob("[0-9]*/stat"):
        try:
            raw = path.read_text()
            tail = raw[raw.rfind(")") + 2:].split()
            if int(tail[2]) == pgid and tail[0] != "Z":
                members.append(int(path.parent.name))
        except (OSError, ValueError, IndexError):
            continue
    return members
