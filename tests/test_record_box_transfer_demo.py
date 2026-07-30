"""Static tests for the Isaac Sim box-transfer recorder contract."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


SCRIPT = Path(__file__).parents[1] / "scripts" / "record_box_transfer_demo.py"
SPEC = importlib.util.spec_from_file_location("record_box_transfer_demo", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_recorder_binds_all_motion_camera_and_scene_sources() -> None:
    """Evidence hashes must cover the authored trajectory and formal USD stack."""

    required = {
        Path("scripts/record_box_transfer_demo.py"),
        Path("scripts/compose_multiview_video.py"),
        Path("source/franzi_sim/franzi_sim/scenarios/box_transfer_demo.py"),
        Path("usd/scenes/warehouse_box_transfer.usda"),
        Path("usd/assets/robots/wheel_bot/configuration/wheel_bot_base.usd"),
        Path("usd/assets/robots/wheel_bot/camera_sensors.usda"),
    }
    assert required.issubset(MODULE.SOURCE_FILES)
    assert all((MODULE.PROJECT_ROOT / path).is_file() for path in MODULE.SOURCE_FILES)


def test_recorder_refuses_nonempty_output_directory(tmp_path: Path) -> None:
    """A failed rerun cannot leave an old successful manifest beside new frames."""

    output = tmp_path / "capture"
    output.mkdir()
    (output / "manifest.json").write_text('{"ok": true}', encoding="utf-8")
    with pytest.raises(ValueError, match="new or empty"):
        MODULE._prepare_output(output)


def test_representative_frames_cover_complete_manipulation() -> None:
    timeline = [
        {"stage": stage, "frame_index": index}
        for index, stage in enumerate(
            (
                "approach",
                "pregrasp",
                "grasp",
                "lift",
                "transport",
                "lower",
                "release",
                "complete",
            )
        )
    ]
    assert [stage for stage, _index in MODULE._representative_frame_indices(timeline)] == [
        "approach",
        "pregrasp",
        "grasp",
        "lift",
        "transport",
        "lower",
        "release",
        "complete",
    ]


def test_recorder_uses_documented_delivery_gate_and_zero_time_capture() -> None:
    """The visual recorder cannot add hidden physics steps or relax past 30 mm."""

    source = SCRIPT.read_text(encoding="utf-8")
    assert MODULE.DELIVERY_XY_TOLERANCE_M == pytest.approx(0.030)
    assert MODULE.GRIP_POINT_TOLERANCE_M == pytest.approx(0.030)
    assert "delta_time=0.0" in source
    assert source.index("manifest_path.write_text") < source.index("app.close()")
