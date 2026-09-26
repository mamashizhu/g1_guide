#!/usr/bin/env python3
"""Compose the four gallery wall textures E01–E04 from public-domain paintings.

Each texture is a dark display board with the painting centred at its own
aspect ratio (nothing important is cropped away), a frame in the style of its
tradition, a soft drop shadow and a small museum label.  Chinese scrolls get a
pale silk mounting inside a thin wooden frame; Western/Japanese prints get a
gilt frame.

Sources live in ``assets/exhibition/paintings_src`` (see ``SOURCES.md`` there);
only the composed PNGs are installed with the package.  The world generator
uses them as poster textures, so the hall layout and ``exhibition.json`` asset
names stay unchanged.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

from PIL import Image, ImageDraw, ImageFilter, ImageFont


WIDTH = 1536
HEIGHT = 1024  # 3:2, matching the 2.4 m × 1.6 m poster plane
BOARD = (38, 42, 48)
BOARD_EDGE = (28, 31, 36)
LABEL_BG = (236, 231, 219)
LABEL_INK = (40, 36, 32)
LABEL_MUTED = (110, 102, 92)

# Space reserved around the framed painting: the label sits in the bottom band.
MARGIN_X = 90
MARGIN_TOP = 70
MARGIN_BOTTOM = 190

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_DIR = PACKAGE_ROOT / "assets" / "exhibition" / "paintings_src"
OUTPUT_DIR = PACKAGE_ROOT / "assets" / "exhibition" / "posters"


@dataclass(frozen=True)
class Painting:
    exhibit_id: str
    source: str
    title: str
    byline: str
    medium: str
    style: str  # "scroll" or "gilt"
    crop: tuple[float, float, float, float] = (0.0, 0.0, 1.0, 1.0)  # fractions of the source


PAINTINGS = (
    Painting("E01", "qingming.jpg", "清明上河图（局部）", "张择端　北宋",
             "绢本设色　长卷，此为虹桥一段", "scroll"),
    Painting("E02", "qianli.jpg", "千里江山图（局部）", "王希孟　北宋　一一一三年",
             "绢本青绿设色　长卷，此为开卷主峰一段", "scroll", crop=(0.0, 0.0, 0.44, 1.0)),
    Painting("E03", "starry_night.jpg", "星月夜", "文森特・梵高　荷兰　一八八九年",
             "布面油画", "gilt"),
    Painting("E04", "great_wave.jpg", "神奈川冲浪里", "葛饰北斋　日本　约一八三一年",
             "木版画　《富岳三十六景》之一", "gilt"),
)

FONT_CANDIDATES = (
    "/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
    "C:/Windows/Fonts/msyh.ttc",
    "C:/Windows/Fonts/simhei.ttf",
)


def find_cjk_font() -> Path | None:
    # Label text sticks to CJK characters and full-width punctuation (years
    # are written in Chinese numerals): Droid Sans Fallback, the only CJK
    # font on the stock WSL image, has no ASCII digits or middle dot.
    for candidate in FONT_CANDIDATES:
        path = Path(candidate)
        if path.is_file():
            return path
    return None


def _font(path: Path | None, size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    return ImageFont.truetype(str(path), size) if path else ImageFont.load_default(size=size)


def _crop(image: Image.Image, fractions: tuple[float, float, float, float]) -> Image.Image:
    left, top, right, bottom = fractions
    return image.crop((round(left * image.width), round(top * image.height),
                       round(right * image.width), round(bottom * image.height)))


def _frame_widths(style: str) -> tuple[int, int]:
    """(inner mounting, outer frame) widths in pixels."""
    return (34, 14) if style == "scroll" else (0, 40)


def painting_box(aspect: float, style: str) -> tuple[int, int, int, int]:
    """Pixel box of the painting itself, centred in the space above the label band."""
    mount, frame = _frame_widths(style)
    border = mount + frame
    avail_w = WIDTH - 2 * MARGIN_X - 2 * border
    avail_h = HEIGHT - MARGIN_TOP - MARGIN_BOTTOM - 2 * border
    w = min(avail_w, round(avail_h * aspect))
    h = min(avail_h, round(w / aspect))
    left = (WIDTH - w) // 2
    top = MARGIN_TOP + border + (avail_h - h) // 2
    return left, top, left + w, top + h


def _draw_gilt(draw: ImageDraw.ImageDraw, box: tuple[int, int, int, int], width: int) -> None:
    # Concentric bands from dark outer edge to bright inner lip read as moulding.
    bands = [(92, 66, 22), (150, 112, 42), (198, 158, 72), (226, 190, 102),
             (184, 142, 58), (132, 96, 34), (212, 176, 92), (96, 70, 26)]
    step = width / len(bands)
    left, top, right, bottom = box
    for index, color in enumerate(bands):
        inset = round(index * step)
        draw.rectangle((left - width + inset, top - width + inset,
                        right + width - inset - 1, bottom + width - inset - 1), fill=color)


def _draw_scroll(draw: ImageDraw.ImageDraw, box: tuple[int, int, int, int],
                 mount: int, frame: int) -> None:
    left, top, right, bottom = box
    outer = mount + frame
    draw.rectangle((left - outer, top - outer, right + outer - 1, bottom + outer - 1),
                   fill=(70, 46, 30))                      # dark wooden frame
    draw.rectangle((left - mount, top - mount, right + mount - 1, bottom + mount - 1),
                   fill=(214, 206, 186))                   # pale silk mounting
    draw.rectangle((left - 6, top - 6, right + 5, bottom + 5), fill=(150, 130, 100))  # inner rule


def _draw_label(image: Image.Image, painting: Painting, font_path: Path | None,
                frame_bottom: int, frame_right: int) -> tuple[int, int, int, int]:
    draw = ImageDraw.Draw(image)
    title_font, body_font = _font(font_path, 34), _font(font_path, 22)
    lines = [(painting.title, title_font, LABEL_INK),
             (painting.byline, body_font, LABEL_MUTED),
             (painting.medium, body_font, LABEL_MUTED)]
    widths = [draw.textbbox((0, 0), text, font=font)[2] for text, font, _ in lines]
    pad, gap = 20, 10
    heights = [draw.textbbox((0, 0), text, font=font)[3] for text, font, _ in lines]
    label_w = max(widths) + 2 * pad
    label_h = sum(heights) + gap * (len(lines) - 1) + 2 * pad
    right = min(frame_right, WIDTH - MARGIN_X)
    top = min(frame_bottom + 30, HEIGHT - 30 - label_h)
    box = (right - label_w, top, right, top + label_h)
    draw.rectangle(box, fill=LABEL_BG)
    y = top + pad
    for (text, font, color), height in zip(lines, heights):
        draw.text((box[0] + pad, y), text, font=font, fill=color)
        y += height + gap
    return box


def compose(painting: Painting, source_dir: Path = SOURCE_DIR,
            font_path: Path | None = None) -> Image.Image:
    art = _crop(Image.open(source_dir / painting.source).convert("RGB"), painting.crop)
    left, top, right, bottom = painting_box(art.width / art.height, painting.style)
    mount, frame = _frame_widths(painting.style)
    border = mount + frame

    board = Image.new("RGB", (WIDTH, HEIGHT), BOARD)
    ImageDraw.Draw(board).rectangle((0, 0, WIDTH - 1, HEIGHT - 1), outline=BOARD_EDGE, width=12)

    shadow = Image.new("L", (WIDTH, HEIGHT), 0)
    ImageDraw.Draw(shadow).rectangle((left - border + 10, top - border + 16,
                                      right + border + 10, bottom + border + 16), fill=150)
    shadow = shadow.filter(ImageFilter.GaussianBlur(14))
    board.paste(Image.new("RGB", board.size, (8, 9, 11)), (0, 0), shadow)

    draw = ImageDraw.Draw(board)
    if painting.style == "gilt":
        _draw_gilt(draw, (left, top, right, bottom), frame)
    else:
        _draw_scroll(draw, (left, top, right, bottom), mount, frame)
    board.paste(art.resize((right - left, bottom - top), Image.LANCZOS), (left, top))

    _draw_label(board, painting, font_path, bottom + border, right + border)
    return board


def generate(output_dir: Path = OUTPUT_DIR, source_dir: Path = SOURCE_DIR) -> list[Path]:
    font_path = find_cjk_font()
    output_dir.mkdir(parents=True, exist_ok=True)
    written = []
    for painting in PAINTINGS:
        path = output_dir / f"{painting.exhibit_id}.png"
        compose(painting, source_dir, font_path).save(path, optimize=True)
        written.append(path)
    return written


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR)
    parser.add_argument("--source-dir", type=Path, default=SOURCE_DIR)
    args = parser.parse_args()
    if find_cjk_font() is None:
        print("warning: no CJK font found; museum labels will not render Chinese")
    for path in generate(args.output_dir, args.source_dir):
        print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
