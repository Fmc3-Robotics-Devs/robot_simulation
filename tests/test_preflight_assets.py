"""Tests for asset validation and local mount behavior."""

from pathlib import Path

import pytest

from scripts.preflight_assets import mount_assets, validate_asset_root


def test_validate_asset_root_finds_warehouse(tmp_path: Path) -> None:
    """A complete root returns the official Warehouse USD path."""

    warehouse_directory = tmp_path / "Isaac" / "Environments" / "Simple_Warehouse"
    warehouse_directory.mkdir(parents=True)
    (tmp_path / "NVIDIA").mkdir()
    warehouse = warehouse_directory / "full_warehouse.usd"
    warehouse.touch()

    result = validate_asset_root(tmp_path)

    assert result.warehouse_usd == warehouse


def test_validate_asset_root_rejects_missing_layout(tmp_path: Path) -> None:
    """An incomplete root produces an actionable error."""

    with pytest.raises(ValueError, match="Isaac"):
        validate_asset_root(tmp_path)


def test_mount_assets_is_idempotent(tmp_path: Path) -> None:
    """A matching symlink can be safely reused by repeated preflight runs."""

    assets = tmp_path / "assets"
    assets.mkdir()
    mount = tmp_path / "vendor" / "isaac_assets"

    assert mount_assets(assets, mount) == mount
    assert mount_assets(assets, mount) == mount
    assert mount.resolve() == assets


def test_box_scene_composes_portable_robot_and_project_crate_wrapper() -> None:
    """The formal scene reaches NVIDIA's crate through the metre-scale wrapper."""

    scene = Path("usd/scenes/warehouse_box_transfer.usda").read_text(encoding="utf-8")
    crate_wrapper = Path("usd/assets/props/blue_transport_box.usda").read_text(
        encoding="utf-8"
    )
    assert "wheel_bot_with_cameras.usda" in scene
    assert "blue_transport_box.usda" in scene
    assert "SM_Crate_A08_Blue_01.usd" in crate_wrapper
    assert "SM_Crate_A08_Blue_01_physics.usd" not in crate_wrapper
    assert 'scenario:status = "workcell_visual_validation"' in scene
    assert "/home/" not in scene
    assert "/home/" not in crate_wrapper
