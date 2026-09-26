"""Tests for the framed-painting wall textures (E01–E04)."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest
from PIL import Image, ImageChops


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "frame_paintings.py"
SPEC = importlib.util.spec_from_file_location("frame_paintings", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE  # dataclasses look the module up while executing it
SPEC.loader.exec_module(MODULE)

EXPECTED = ["E01.png", "E02.png", "E03.png", "E04.png"]


@pytest.fixture(scope="module")
def generated(tmp_path_factory) -> Path:
    out = tmp_path_factory.mktemp("posters")
    MODULE.generate(out)
    return out


def test_every_painting_has_a_documented_source() -> None:
    sources = (MODULE.SOURCE_DIR / "SOURCES.md").read_text(encoding="utf-8")
    for painting in MODULE.PAINTINGS:
        assert (MODULE.SOURCE_DIR / painting.source).is_file()
        assert painting.source in sources


def test_writes_four_3x2_rgb_textures(generated: Path) -> None:
    assert sorted(p.name for p in generated.glob("*.png")) == EXPECTED
    for name in EXPECTED:
        with Image.open(generated / name) as image:
            assert image.mode == "RGB"
            assert image.size == (MODULE.WIDTH, MODULE.HEIGHT)
            assert MODULE.WIDTH * 2 == MODULE.HEIGHT * 3  # matches the 2.4 m × 1.6 m plane


def test_textures_are_visibly_different(generated: Path) -> None:
    images = [Image.open(generated / name).convert("RGB") for name in EXPECTED]
    for index, first in enumerate(images):
        for second in images[index + 1:]:
            assert ImageChops.difference(first, second).getbbox() is not None


def test_generation_is_reproducible(generated: Path, tmp_path: Path) -> None:
    MODULE.generate(tmp_path)
    for name in EXPECTED:
        assert (tmp_path / name).read_bytes() == (generated / name).read_bytes()


@pytest.mark.parametrize("aspect", [0.6, 1.0, 1.26, 1.5, 2.0, 4.8])
@pytest.mark.parametrize("style", ["scroll", "gilt"])
def test_painting_and_frame_stay_above_the_label_band(aspect: float, style: str) -> None:
    left, top, right, bottom = MODULE.painting_box(aspect, style)
    border = sum(MODULE._frame_widths(style))
    assert left - border >= MODULE.MARGIN_X
    assert right + border <= MODULE.WIDTH - MODULE.MARGIN_X
    assert top - border >= MODULE.MARGIN_TOP
    assert bottom + border <= MODULE.HEIGHT - MODULE.MARGIN_BOTTOM
    assert abs((right - left) / (bottom - top) - aspect) < 0.02  # never stretched


def test_label_fits_on_the_board() -> None:
    font = MODULE.find_cjk_font()
    for painting in MODULE.PAINTINGS:
        board = Image.new("RGB", (MODULE.WIDTH, MODULE.HEIGHT))
        art = MODULE._crop(Image.open(MODULE.SOURCE_DIR / painting.source), painting.crop)
        _, _, right, bottom = MODULE.painting_box(art.width / art.height, painting.style)
        border = sum(MODULE._frame_widths(painting.style))
        box = MODULE._draw_label(board, painting, font, bottom + border, right + border)
        assert 0 <= box[0] and box[2] <= MODULE.WIDTH
        assert bottom + border <= box[1] and box[3] <= MODULE.HEIGHT - 20


def test_label_text_avoids_glyphs_missing_from_droid_fallback() -> None:
    # The stock WSL CJK font has no ASCII digits or U+00B7 middle dot.
    for painting in MODULE.PAINTINGS:
        for text in (painting.title, painting.byline, painting.medium):
            assert not any(ch.isascii() and ch.isalnum() for ch in text), text
            assert "·" not in text
