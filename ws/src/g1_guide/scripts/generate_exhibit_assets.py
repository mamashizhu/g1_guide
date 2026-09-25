#!/usr/bin/env python3
"""Generate the four deterministic science posters used by the exhibition world.

The artwork is intentionally made from Pillow primitives only.  Keeping the
source procedural makes it possible to regenerate the PNGs without downloading
icons or image assets. Byte identity requires the same Pillow/font environment.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Callable, Iterable

from PIL import Image, ImageDraw, ImageFont


WIDTH = 768
HEIGHT = 512
BG = (10, 20, 43)
INK = (235, 243, 255)
MUTED = (155, 177, 207)
WHITE = (250, 252, 255)

# E04 layout anchors are kept explicit so the poster's dense coding diagram
# can be checked without relying on a pixel-perfect screenshot.
E04_CODE_BOX = (236, 214, 430, 458)
E04_WAVE_BOUNDS = (456, 262, 586, 289)
E04_DECODE_BOX = (603, 214, 728, 363)
E04_DECODE_COPY_BOX = (618, 328, 710, 352)
E03_OXYGEN_ARROW = ((606, 214), (645, 188))  # from cell to external O2 card


def _font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    """Load a bundled/system DejaVu font, with Pillow's safe fallback."""

    filename = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    candidates = (
        Path("/usr/share/fonts/truetype/dejavu") / filename,
        Path("/usr/local/share/fonts/truetype/dejavu") / filename,
        Path("C:/Windows/Fonts") / filename,
    )
    for candidate in candidates:
        if candidate.exists():
            return ImageFont.truetype(str(candidate), size)
    return ImageFont.load_default(size=size)


def _text(draw: ImageDraw.ImageDraw, xy: tuple[int, int], text: str, size: int,
          fill: tuple[int, int, int] = INK, bold: bool = False) -> None:
    draw.text(xy, text, font=_font(size, bold), fill=fill)


def _center_text(draw: ImageDraw.ImageDraw, box: tuple[int, int, int, int], text: str,
                 size: int, fill: tuple[int, int, int] = INK, bold: bool = False) -> None:
    font = _font(size, bold)
    left, top, right, bottom = box
    bbox = draw.textbbox((0, 0), text, font=font)
    x = left + (right - left - (bbox[2] - bbox[0])) // 2 - bbox[0]
    y = top + (bottom - top - (bbox[3] - bbox[1])) // 2 - bbox[1]
    draw.text((x, y), text, font=font, fill=fill)


def _line(draw: ImageDraw.ImageDraw, points: Iterable[tuple[int, int]], fill, width: int = 2) -> None:
    draw.line(list(points), fill=fill, width=width, joint="curve")


def _arrow(draw: ImageDraw.ImageDraw, start: tuple[int, int], end: tuple[int, int], fill,
           width: int = 3, head: int = 9) -> None:
    """Draw a deterministic line arrow without trigonometry-heavy dependencies."""

    x1, y1 = start
    x2, y2 = end
    _line(draw, [start, end], fill, width)
    dx, dy = x2 - x1, y2 - y1
    length = max((dx * dx + dy * dy) ** 0.5, 1.0)
    ux, uy = dx / length, dy / length
    px, py = -uy, ux
    left = (int(x2 - ux * head + px * head * 0.55), int(y2 - uy * head + py * head * 0.55))
    right = (int(x2 - ux * head - px * head * 0.55), int(y2 - uy * head - py * head * 0.55))
    draw.polygon([end, left, right], fill=fill)


def _header(draw: ImageDraw.ImageDraw, code: str, title: str, subtitle: str, accent) -> None:
    draw.rectangle((0, 0, WIDTH, 92), fill=(12, 27, 54))
    draw.rectangle((0, 88, WIDTH, 92), fill=accent)
    draw.rounded_rectangle((28, 20, 105, 70), radius=12, fill=accent)
    _center_text(draw, (28, 20, 105, 70), code, 25, fill=BG, bold=True)
    _text(draw, (128, 17), title, 29, WHITE, bold=True)
    _text(draw, (130, 57), subtitle, 14, MUTED)


def _footer(draw: ImageDraw.ImageDraw, caption: str, accent) -> None:
    draw.rectangle((0, HEIGHT - 30, WIDTH, HEIGHT), fill=(8, 16, 34))
    draw.rectangle((28, HEIGHT - 30, 154, HEIGHT - 26), fill=accent)
    _text(draw, (170, HEIGHT - 25), caption, 12, MUTED)


def _stars(draw: ImageDraw.ImageDraw) -> None:
    stars = ((52, 132, 2), (91, 214, 3), (145, 120, 2), (231, 164, 2),
             (311, 118, 2), (410, 204, 2), (506, 128, 3), (594, 185, 2),
             (687, 128, 2), (726, 242, 3), (645, 318, 2), (92, 371, 2))
    for x, y, radius in stars:
        draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=(174, 210, 246))


def _poster_e01() -> Image.Image:
    image = Image.new("RGB", (WIDTH, HEIGHT), BG)
    draw = ImageDraw.Draw(image)
    _header(draw, "E01", "SOLAR SYSTEM", "Earth, sunlight, and orbital motion", (247, 193, 69))
    _stars(draw)

    # Orbits are intentionally offset and tilted to make the diagram read at a glance.
    draw.ellipse((142, 150, 628, 454), outline=(67, 112, 166), width=2)
    draw.ellipse((210, 184, 597, 407), outline=(53, 93, 143), width=2)
    draw.ellipse((273, 216, 549, 378), outline=(47, 81, 127), width=2)

    # Sun and rays.
    sun = (180, 304)
    draw.ellipse((106, 230, 254, 378), fill=(248, 170, 48), outline=(255, 224, 126), width=4)
    draw.ellipse((126, 250, 234, 358), fill=(255, 193, 61))
    for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1), (-.7, -.7), (.7, .7)):
        _line(draw, [(int(sun[0] + dx * 83), int(sun[1] + dy * 83)),
                     (int(sun[0] + dx * 107), int(sun[1] + dy * 107))], (255, 221, 113), 3)
    _text(draw, (126, 396), "SUN", 14, (255, 223, 125), bold=True)

    # Earth, moon, and the sunlight direction.
    earth = (499, 296)
    draw.ellipse((466, 263, 532, 329), fill=(44, 139, 205), outline=(178, 231, 255), width=3)
    draw.pieslice((469, 267, 529, 327), 195, 350, fill=(58, 177, 106))
    draw.ellipse((536, 248, 551, 263), fill=(199, 207, 220), outline=(241, 244, 248))
    _text(draw, (464, 344), "EARTH", 14, (150, 224, 255), bold=True)
    _text(draw, (541, 222), "MOON", 12, MUTED)
    _arrow(draw, (272, 304), (445, 304), (255, 213, 91), 3, 11)
    _text(draw, (292, 278), "SUNLIGHT", 13, (255, 224, 126), bold=True)

    # Small fact cards complete the explanatory schematic.
    draw.rounded_rectangle((40, 124, 250, 190), 10, fill=(14, 37, 69), outline=(61, 105, 157), width=2)
    _text(draw, (57, 138), "1 AU", 22, (114, 207, 255), bold=True)
    _text(draw, (57, 166), "Earth–Sun distance", 13, MUTED)
    draw.rounded_rectangle((559, 370, 724, 443), 10, fill=(14, 37, 69), outline=(61, 105, 157), width=2)
    _text(draw, (577, 383), "ORBIT", 14, (247, 193, 69), bold=True)
    _text(draw, (577, 407), "gravity guides motion", 12, MUTED)
    _footer(draw, "A compact map of our neighborhood in space", (247, 193, 69))
    return image


def _poster_e02() -> Image.Image:
    image = Image.new("RGB", (WIDTH, HEIGHT), (18, 25, 36))
    draw = ImageDraw.Draw(image)
    accent = (255, 151, 74)
    _header(draw, "E02", "CIRCUITS & ENERGY", "A closed path turns stored energy into light", accent)

    # PCB-like grid.
    for x in range(40, 730, 32):
        _line(draw, [(x, 124), (x, 465)], (27, 54, 69), 1)
    for y in range(132, 466, 28):
        _line(draw, [(34, y), (734, y)], (27, 54, 69), 1)

    # Battery.
    draw.rounded_rectangle((56, 238, 144, 356), 12, fill=(34, 66, 77), outline=(143, 219, 195), width=3)
    draw.rectangle((81, 225, 119, 239), fill=(143, 219, 195))
    draw.rectangle((75, 275, 125, 319), fill=(249, 191, 77))
    _text(draw, (55, 378), "BATTERY", 13, (143, 219, 195), bold=True)
    _text(draw, (76, 262), "+", 16, BG, bold=True)
    _text(draw, (76, 320), "−", 16, BG, bold=True)

    wire = (105, 220, 191)
    hot = (255, 151, 74)
    # One unbroken loop: battery -> resistor -> lamp -> battery.
    _line(draw, [(144, 267), (207, 267), (207, 213), (296, 213)], wire, 4)
    _line(draw, [(296, 213), (318, 213), (328, 229), (338, 197), (348, 229), (358, 197), (368, 229), (378, 213), (402, 213)], hot, 4)
    _line(draw, [(402, 213), (474, 213), (474, 267)], wire, 4)
    draw.ellipse((464, 257, 484, 277), fill=(255, 213, 105), outline=(255, 151, 74), width=3)
    _line(draw, [(474, 277), (474, 383), (207, 383), (207, 327), (144, 327)], wire, 4)
    for x, y in ((144, 267), (207, 267), (296, 213), (402, 213), (474, 267), (207, 383), (144, 327)):
        draw.ellipse((x - 6, y - 6, x + 6, y + 6), fill=hot, outline=(255, 235, 161))

    # Component callouts and energy arrows.
    draw.rounded_rectangle((292, 256, 405, 316), 10, fill=(42, 39, 42), outline=(255, 151, 74), width=2)
    _center_text(draw, (292, 256, 405, 286), "RESISTOR", 13, WHITE, bold=True)
    _center_text(draw, (292, 286, 405, 316), "controls current", 11, MUTED)
    draw.ellipse((514, 180, 587, 253), fill=(47, 48, 53), outline=(255, 213, 105), width=3)
    draw.ellipse((530, 196, 571, 237), fill=(255, 213, 105))
    _text(draw, (514, 263), "LAMP", 13, (255, 213, 105), bold=True)
    _arrow(draw, (236, 199), (274, 199), hot, 3, 9)
    _arrow(draw, (430, 238), (430, 278), hot, 3, 9)
    _arrow(draw, (328, 405), (290, 405), hot, 3, 9)
    _text(draw, (548, 317), "energy flow", 14, hot, bold=True)
    _text(draw, (548, 340), "chemical → electrical → light", 12, MUTED)
    _footer(draw, "Voltage pushes charge around a complete circuit", accent)
    return image


def _poster_e03() -> Image.Image:
    image = Image.new("RGB", (WIDTH, HEIGHT), (9, 38, 46))
    draw = ImageDraw.Draw(image)
    accent = (105, 220, 155)
    _header(draw, "E03", "PLANT CELL", "A tiny solar-powered factory", accent)

    # Cell wall and membrane.
    draw.ellipse((122, 139, 602, 438), fill=(30, 107, 81), outline=(139, 242, 170), width=6)
    draw.ellipse((143, 157, 580, 422), fill=(43, 145, 100), outline=(72, 205, 133), width=3)
    # Vacuole, nucleus, and chloroplasts.
    draw.ellipse((355, 189, 541, 371), fill=(87, 192, 157), outline=(164, 244, 192), width=3)
    draw.ellipse((254, 228, 380, 352), fill=(102, 122, 218), outline=(190, 207, 255), width=4)
    draw.ellipse((282, 255, 350, 326), fill=(137, 159, 241))
    for x, y in ((186, 220), (204, 343), (454, 176), (507, 347), (526, 244)):
        draw.ellipse((x - 31, y - 15, x + 31, y + 15), fill=(202, 218, 71), outline=(235, 250, 144), width=2)
        _line(draw, [(x - 18, y), (x + 18, y)], (102, 143, 50), 2)
        _line(draw, [(x - 13, y - 6), (x + 13, y + 6)], (102, 143, 50), 2)
    for x, y in ((225, 282), (420, 375), (410, 150)):
        draw.ellipse((x - 19, y - 12, x + 19, y + 12), fill=(228, 125, 87), outline=(255, 198, 153), width=2)
        _line(draw, [(x - 10, y - 4), (x + 10, y + 4)], (139, 59, 55), 2)
    _text(draw, (270, 364), "NUCLEUS", 13, (218, 225, 255), bold=True)
    _text(draw, (400, 378), "VACUOLE", 13, (203, 255, 224), bold=True)

    # Inputs and outputs around the cell.
    draw.rounded_rectangle((36, 145, 111, 211), 10, fill=(15, 71, 76), outline=(105, 220, 155), width=2)
    _text(draw, (51, 157), "CO₂", 20, WHITE, bold=True)
    _text(draw, (51, 185), "in", 12, MUTED)
    _arrow(draw, (112, 188), (153, 213), accent, 3, 9)
    draw.rounded_rectangle((645, 145, 734, 211), 10, fill=(15, 71, 76), outline=(105, 220, 155), width=2)
    _text(draw, (659, 157), "O₂", 20, WHITE, bold=True)
    _text(draw, (659, 185), "out", 12, MUTED)
    _arrow(draw, *E03_OXYGEN_ARROW, accent, 3, 9)
    draw.rounded_rectangle((38, 354, 111, 420), 10, fill=(15, 71, 76), outline=(247, 193, 69), width=2)
    _text(draw, (50, 365), "LIGHT", 14, (247, 221, 128), bold=True)
    _text(draw, (50, 392), "energy", 12, MUTED)
    _arrow(draw, (112, 382), (157, 351), (247, 193, 69), 3, 9)
    _text(draw, (633, 362), "CELL WALL", 12, (139, 242, 170), bold=True)
    _text(draw, (633, 383), "shape + support", 11, MUTED)
    _footer(draw, "Chloroplasts capture light; the cell turns it into stored energy", accent)
    return image


def _poster_e04() -> Image.Image:
    image = Image.new("RGB", (WIDTH, HEIGHT), (27, 18, 57))
    draw = ImageDraw.Draw(image)
    accent = (208, 123, 255)
    _header(draw, "E04", "INFORMATION / CODING", "Bits become patterns that machines can read", accent)

    # Binary stream with deliberate rhythm.
    stream = "10110010 01101001 11001010 00110111"
    for index, bit in enumerate(stream.replace(" ", "")):
        x = 34 + (index % 16) * 28
        y = 126 + (index // 16) * 25
        _center_text(draw, (x, y, x + 22, y + 20), bit, 12,
                     fill=(112, 230, 255) if bit == "1" else (101, 89, 169), bold=True)
    _text(draw, (35, 188), "BINARY SIGNAL", 12, (155, 138, 220), bold=True)

    # Encoder arrow and block.
    draw.rounded_rectangle((35, 236, 159, 343), 13, fill=(50, 34, 87), outline=(208, 123, 255), width=3)
    _center_text(draw, (35, 249, 159, 282), "MESSAGE", 15, WHITE, bold=True)
    _center_text(draw, (35, 290, 159, 331), "HELLO", 23, (250, 196, 110), bold=True)
    _arrow(draw, (172, 290), (231, 290), (250, 196, 110), 4, 11)
    _text(draw, (178, 266), "encode", 12, (250, 196, 110), bold=True)

    # QR-like code, with fixed modules and a white quiet zone.
    quiet = E04_CODE_BOX
    draw.rounded_rectangle(quiet, 10, fill=(242, 245, 255))
    pattern = [
        "1110111011101", "1000101000101", "1011101011101", "1011101010101",
        "1011101011101", "1000101000001", "1110111010101", "0000000011100",
        "1101111010111", "0010100101001", "1110011110110", "1001010011101",
        "1011111010001",
    ]
    module = 12
    for row, bits in enumerate(pattern):
        for column, bit in enumerate(bits):
            if bit == "1":
                x = quiet[0] + 12 + column * module
                y = quiet[1] + 12 + row * module
                draw.rectangle((x, y, x + module - 1, y + module - 1), fill=(31, 20, 63))
    _text(draw, (308, 425), "CODE", 12, (31, 20, 63), bold=True)

    # Transmission wave and a small decoding panel.
    wave = []
    for x in range(456, 586, 4):
        phase = (x - 456) % 48
        offset = int(27 * phase / 24 if phase < 24 else 27 * (48 - phase) / 24)
        wave.append((x, 289 - offset))
    _line(draw, wave, (112, 230, 255), 4)
    _line(draw, [(456, 289), (586, 289)], (75, 63, 127), 1)
    _text(draw, (478, 322), "signal", 13, (112, 230, 255), bold=True)
    draw.rounded_rectangle(E04_DECODE_BOX, 13, fill=(47, 29, 82), outline=(208, 123, 255), width=3)
    _text(draw, (622, 232), "DECODE", 14, (208, 123, 255), bold=True)
    _text(draw, (622, 271), "01001000", 15, (112, 230, 255), bold=True)
    _text(draw, (622, 298), "= H", 17, WHITE, bold=True)
    _text(draw, (618, 328), "pattern →", 10, MUTED)
    _text(draw, (618, 342), "meaning", 10, MUTED)
    _footer(draw, "A code is a shared rule for turning symbols into meaning", accent)
    return image


_POSTERS: tuple[tuple[str, Callable[[], Image.Image]], ...] = (
    ("E01.png", _poster_e01),
    ("E02.png", _poster_e02),
    ("E03.png", _poster_e03),
    ("E04.png", _poster_e04),
)


def generate_assets(output_dir: Path) -> list[Path]:
    """Generate all exhibition posters under ``output_dir`` and return paths."""

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    paths: list[Path] = []
    for filename, factory in _POSTERS:
        image = factory()
        if image.size != (WIDTH, HEIGHT):
            raise RuntimeError(f"poster {filename} has unexpected size {image.size}")
        destination = output_dir / filename
        image.save(destination, format="PNG", optimize=False, compress_level=9)
        paths.append(destination)
    return paths


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True,
                        help="directory for E01.png through E04.png")
    args = parser.parse_args()
    for path in generate_assets(args.output_dir):
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
