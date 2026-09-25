"""Regression tests for the deterministic exhibition poster generator."""

from __future__ import annotations

import hashlib
import importlib.util
from pathlib import Path

from PIL import Image, ImageChops


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "generate_exhibit_assets.py"
SPEC = importlib.util.spec_from_file_location("generate_exhibit_assets", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


EXPECTED = ["E01.png", "E02.png", "E03.png", "E04.png"]


def _hashes(paths: list[Path]) -> list[str]:
    return [hashlib.sha256(path.read_bytes()).hexdigest() for path in paths]


def test_generate_assets_writes_four_768x512_pngs(tmp_path: Path) -> None:
    paths = MODULE.generate_assets(tmp_path)

    assert [path.name for path in paths] == EXPECTED
    assert all(path.is_file() for path in paths)
    for path in paths:
        with Image.open(path) as image:
            assert image.format == "PNG"
            assert image.size == (768, 512)
            assert image.mode == "RGB"


def test_posters_are_visibly_different(tmp_path: Path) -> None:
    paths = MODULE.generate_assets(tmp_path)
    images = [Image.open(path).convert("RGB") for path in paths]
    try:
        assert len({image.tobytes() for image in images}) == 4
        assert all(ImageChops.difference(images[0], image).getbbox() is not None for image in images[1:])
    finally:
        for image in images:
            image.close()


def test_generation_is_byte_for_byte_reproducible(tmp_path: Path) -> None:
    first_dir = tmp_path / "first"
    second_dir = tmp_path / "second"
    first = MODULE.generate_assets(first_dir)
    second = MODULE.generate_assets(second_dir)

    assert _hashes(first) == _hashes(second)
    for first_path, second_path in zip(first, second):
        assert first_path.read_bytes() == second_path.read_bytes()


def test_e04_layout_keeps_code_wave_and_decode_copy_in_bounds() -> None:
    code_left, code_top, code_right, code_bottom = MODULE.E04_CODE_BOX
    wave_left, wave_top, wave_right, wave_bottom = MODULE.E04_WAVE_BOUNDS
    panel_left, panel_top, panel_right, panel_bottom = MODULE.E04_DECODE_BOX
    copy_left, copy_top, copy_right, copy_bottom = MODULE.E04_DECODE_COPY_BOX

    assert 92 < wave_top < wave_bottom < 482  # below header and above footer
    assert 0 <= code_left < code_right <= 768
    assert 92 < code_top < code_bottom < 482
    assert 0 <= wave_left < wave_right <= 768
    assert (panel_left, panel_top, panel_right, panel_bottom) == (603, 214, 728, 363)
    assert panel_left < copy_left < copy_right < panel_right
    assert panel_top < copy_top < copy_bottom < panel_bottom


def test_oxygen_arrow_points_out_of_the_cell():
    start, end = MODULE.E03_OXYGEN_ARROW
    assert 602 < start[0] < end[0] == 645


def test_fallback_font_preserves_requested_size(monkeypatch):
    monkeypatch.setattr(MODULE.Path, "exists", lambda self: False)
    small = MODULE._font(12).getbbox("EXHIBIT")
    large = MODULE._font(29).getbbox("EXHIBIT")
    assert large[3] - large[1] > small[3] - small[1]
