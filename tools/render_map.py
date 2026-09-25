"""把 SLAM 地图画成带标注的 PNG：展品轮廓与名称、展位（位置 + 朝向箭头）、出生点、米刻度。

用法（Windows 或 WSL 均可，需要 Pillow）：
  python tools/render_map.py maps/exhibition_map_mid360.yaml
  python tools/render_map.py maps/exhibition_simple_map_mid360.yaml -o out.png

展馆配置默认取地图旁 *_meta.json 里记录的建图展馆，也可以用 --scene 指定。

展品轮廓取自场景配置，只用来核对：地图里的黑色墙格应该正好落在虚线框上。
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "ws/src/g1_guide"))

from g1_guide.scene_config import load_scene  # noqa: E402
from g1_guide.slam_map import FREE, OCCUPIED, load_map  # noqa: E402

FONT_CANDIDATES = ("C:/Windows/Fonts/msyh.ttc", "/mnt/c/Windows/Fonts/msyh.ttc",
                   "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")


def font(size: int):
    for path in FONT_CANDIDATES:
        if Path(path).is_file():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def render(map_yaml: Path, scene_path: Path, output: Path, px_per_cell: int = 8) -> Path:
    grid = load_map(map_yaml)
    scene = load_scene(scene_path)
    margin = 60
    w, h = grid.width * px_per_cell, grid.height * px_per_cell
    image = Image.new("RGB", (w + 2 * margin, h + 2 * margin + 40), "white")
    draw = ImageDraw.Draw(image)
    ox, oy, res = grid.origin[0], grid.origin[1], grid.resolution

    def to_px(x: float, y: float) -> tuple[float, float]:
        return (margin + (x - ox) / res * px_per_cell,
                margin + h - (y - oy) / res * px_per_cell)

    colors = {FREE: (255, 255, 255), OCCUPIED: (30, 30, 30)}
    for i, cell in enumerate(grid.cells):
        row, col = divmod(i, grid.width)
        x0 = margin + col * px_per_cell
        y0 = margin + h - (row + 1) * px_per_cell
        draw.rectangle([x0, y0, x0 + px_per_cell - 1, y0 + px_per_cell - 1],
                       fill=colors.get(cell, (190, 190, 200)))

    small, label, title = font(14), font(18), font(22)
    # 1 m grid and axis ticks.
    for xm in range(math.ceil(ox), math.floor(ox + grid.width * res) + 1):
        px, _ = to_px(xm, oy)
        draw.line([px, margin, px, margin + h], fill=(215, 225, 240), width=1)
        draw.text((px, margin + h + 4), str(xm), fill=(90, 90, 90), font=small, anchor="mt")
    for ym in range(math.ceil(oy), math.floor(oy + grid.height * res) + 1):
        _, py = to_px(ox, ym)
        draw.line([margin, py, margin + w, py], fill=(215, 225, 240), width=1)
        draw.text((margin - 6, py), str(ym), fill=(90, 90, 90), font=small, anchor="rm")
    # Redraw occupied cells above the grid lines.
    for i, cell in enumerate(grid.cells):
        if cell == OCCUPIED:
            row, col = divmod(i, grid.width)
            x0 = margin + col * px_per_cell
            y0 = margin + h - (row + 1) * px_per_cell
            draw.rectangle([x0, y0, x0 + px_per_cell - 1, y0 + px_per_cell - 1], fill=(30, 30, 30))

    blue, orange = (30, 100, 220), (230, 110, 20)
    for exhibit in scene.get("exhibits", ()):
        x, y = exhibit["position"][:2]
        sx, sy = exhibit["size"][:2]
        yaw = exhibit.get("yaw", 0.0)
        c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
        ex, ey = sx * c + sy * s, sx * s + sy * c
        a, b = to_px(x - ex / 2, y + ey / 2), to_px(x + ex / 2, y - ey / 2)
        draw.rectangle([a, b], outline=blue, width=3)
        name = f"{exhibit.get('exhibit_id', '')} {exhibit.get('title', '')}".strip()
        cx, cy = to_px(x, y)
        draw.text((cx, cy - 18 if y > 0 else cy + 18), name, fill=blue, font=label,
                  anchor="mm", stroke_width=3, stroke_fill="white")

        station = exhibit.get("station")
        if station:
            sxp, syp = to_px(station["x"], station["y"])
            r = 13
            draw.ellipse([sxp - r, syp - r, sxp + r, syp + r], fill=orange, outline="white", width=2)
            draw.text((sxp, syp), str(station["key"]), fill="white", font=label, anchor="mm")
            tip = to_px(station["x"] + 0.8 * math.cos(station["yaw"]),
                        station["y"] + 0.8 * math.sin(station["yaw"]))
            draw.line([sxp + r * math.cos(station["yaw"]), syp - r * math.sin(station["yaw"]), *tip],
                      fill=orange, width=3)
            head = station["yaw"] + math.pi
            for side in (-0.5, 0.5):
                draw.line([*tip, tip[0] + 10 * math.cos(head + side),
                           tip[1] - 10 * math.sin(head + side)], fill=orange, width=3)

    spawn = scene.get("spawn")
    if spawn:
        sx, sy = to_px(spawn["x"], spawn["y"])
        draw.regular_polygon((sx, sy, 12), 3, rotation=90 - math.degrees(spawn["yaw"]) - 90,
                             fill=(40, 160, 70))
        draw.text((sx + 16, sy), "出生点", fill=(40, 160, 70), font=small, anchor="lm",
                  stroke_width=2, stroke_fill="white")

    draw.text((margin, 18), f"SLAM 地图 {map_yaml.name}（{grid.width * res:.1f}×{grid.height * res:.1f} m，"
              f"格子 {res * 100:.0f} cm，刻度单位 m）", fill="black", font=title, anchor="lm")
    legend_y = margin + h + 30
    items = [((30, 30, 30), "黑：扫到的墙/障碍"), ((190, 190, 200), "灰：未探索"),
             (blue, "蓝框：展品真实位置（核对用）"), (orange, "橙圈+箭头：展位与朝向（数字键）")]
    x = margin
    for color, text in items:
        draw.rectangle([x, legend_y - 7, x + 14, legend_y + 7], fill=color)
        draw.text((x + 20, legend_y), text, fill="black", font=small, anchor="lm")
        x += 20 + draw.textlength(text, font=small) + 24
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output)
    return output


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("map_yaml", type=Path)
    parser.add_argument("--scene", type=Path, default=None)
    parser.add_argument("-o", "--output", type=Path)
    args = parser.parse_args(argv)
    scene = args.scene
    if scene is None:
        import json
        from g1_guide.slam_map import meta_path
        scene = PROJECT / "ws/src/g1_guide/config" / json.loads(
            meta_path(args.map_yaml).read_text(encoding="utf-8"))["scene"]
    output = args.output or args.map_yaml.with_name(args.map_yaml.stem + "_annotated.png")
    print(render(args.map_yaml, scene, output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
