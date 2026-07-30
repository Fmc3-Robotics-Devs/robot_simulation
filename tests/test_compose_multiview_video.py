"""Tests for the standalone multi-camera video composition utility."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest
from PIL import Image


SCRIPT = Path(__file__).parents[1] / "scripts" / "compose_multiview_video.py"
SPEC = importlib.util.spec_from_file_location("compose_multiview_video", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _write_frames(directory: Path, count: int) -> None:
    directory.mkdir()
    for index in range(count):
        Image.new("RGB", (64, 36), (index * 30, 20, 50)).save(directory / f"capture_{index:03d}.png")


def test_compose_frame_has_expected_full_hd_layout() -> None:
    images = {name: Image.new("RGB", (160, 90), (10, 20, 30)) for name in MODULE.VIEW_ORDER}
    frame = MODULE.compose_frame(images, frame_number=7)
    assert frame.size == (1920, 1080)
    assert frame.getpixel((100, 100)) == (10, 20, 30)
    assert frame.getpixel((100, 950)) == (10, 20, 30)


def test_collect_sequences_requires_equal_frame_counts(tmp_path: Path) -> None:
    directories = {name: tmp_path / name for name in MODULE.VIEW_ORDER}
    for name, directory in directories.items():
        _write_frames(directory, 2 if name != "right" else 1)
    with pytest.raises(ValueError, match="counts must match"):
        MODULE.collect_sequences(directories)


def test_write_frames_creates_one_full_hd_png_per_input_frame(tmp_path: Path) -> None:
    directories = {name: tmp_path / name for name in MODULE.VIEW_ORDER}
    for directory in directories.values():
        _write_frames(directory, 2)
    output = tmp_path / "composed"
    assert MODULE.write_frames(MODULE.collect_sequences(directories), output) == 2
    with Image.open(output / "frame_00001.png") as image:
        assert image.size == (1920, 1080)


def test_collect_sequences_rejects_equal_counts_with_mismatched_names(tmp_path: Path) -> None:
    directories = {name: tmp_path / name for name in MODULE.VIEW_ORDER}
    for directory in directories.values():
        _write_frames(directory, 2)
    (directories["right"] / "capture_001.png").rename(
        directories["right"] / "stale_001.png"
    )
    with pytest.raises(ValueError, match="filenames do not match"):
        MODULE.collect_sequences(directories)


def test_timeline_metadata_count_must_match_frames(tmp_path: Path) -> None:
    directories = {name: tmp_path / name for name in MODULE.VIEW_ORDER}
    for directory in directories.values():
        _write_frames(directory, 2)
    sequences = MODULE.collect_sequences(directories)
    with pytest.raises(ValueError, match="timeline contains 1 frames"):
        MODULE.write_frames(
            sequences,
            tmp_path / "composed",
            [{"stage": "grasp"}],
        )
