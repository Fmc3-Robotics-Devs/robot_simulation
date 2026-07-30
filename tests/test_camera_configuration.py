"""Static checks for the four required Wheel Bot image streams."""

import tomllib
from pathlib import Path

from franzi_sim.cameras import CAMERAS, camera_by_name, validate_camera_specs
from scripts.verify_camera_configuration import (
    DEFAULT_OVERLAY,
    DEFAULT_ROBOT_ENTRY,
    DEFAULT_URDF,
    validate_files,
)


def test_four_camera_contracts_are_complete_and_side_correct() -> None:
    """Head, chest and both wrists have unique links, frames and ROS topics."""

    validate_camera_specs()
    assert len(CAMERAS) == 4
    assert len({camera.parent_link for camera in CAMERAS}) == 4
    assert len({camera.frame_id for camera in CAMERAS}) == 4
    assert len({camera.topic for camera in CAMERAS}) == 4
    assert len({camera.camera_info_topic for camera in CAMERAS}) == 4
    assert camera_by_name("left_wrist_d405").parent_link == "left_wrist_d405_Link"
    assert camera_by_name("right_wrist_d405").parent_link == "right_wrist_d405_Link"


def test_usd_overlay_covers_every_declared_camera() -> None:
    """The portable overlay contains every link, frame and ROS endpoint once."""

    overlay = Path("usd/assets/robots/wheel_bot/camera_sensors.usda").read_text()
    for camera in CAMERAS:
        assert camera.parent_link in overlay
        assert camera.camera_mount_prim in overlay
        assert camera.optical_frame_prim in overlay
        assert camera.frame_id in overlay
        assert camera.topic in overlay
        assert camera.camera_info_topic in overlay
        assert camera.clipping_range_m == (0.05, 100.0)
    assert overlay.count("float2 clippingRange = (0.05, 100)") == len(CAMERAS)
    assert overlay.count('over "left_wrist_d405_Link"') == 1
    assert overlay.count('over "right_wrist_d405_Link"') == 1


def test_d405_render_axis_matches_the_stl_lens_face() -> None:
    """Both D405s look through physical -Z lens faces along the gripper."""

    for name in ("left_wrist_d405", "right_wrist_d405"):
        camera = camera_by_name(name)
        assert camera.mount_xyz_m == (0.0, 0.0, -0.0235)
        assert camera.camera_mount_rpy_deg == (0.0, 0.0, 90.0)
        assert camera.optical_frame_rpy_deg == (180.0, 0.0, 90.0)


def test_complete_robot_entry_uses_only_relative_layers() -> None:
    """The delivered camera-equipped robot entry composes portable local layers."""

    entry = Path("usd/assets/robots/wheel_bot/wheel_bot_with_cameras.usda").read_text()
    assert "@camera_sensors.usda@" in entry
    assert "@wheel_bot.usd@" in entry
    assert "/home/" not in entry


def test_native_urdf_import_keeps_the_corrected_robot_asset_reproducible() -> None:
    """The generator pins Isaac 5.1 settings and hashes the true camera layers."""

    importer = Path("scripts/import_robot_urdf.py").read_text(encoding="utf-8")
    capture = Path("scripts/capture_workcell_evidence.py").read_text(
        encoding="utf-8"
    )

    assert "Isaac Sim 5.1 native URDF importer 2.4.30" in importer
    assert "import_config.set_convex_decomp(False)" in importer
    assert "import_config.set_merge_fixed_joints(False)" in importer
    assert "isaaclab.sim.converters" not in importer
    for dependency in (
        "configuration/wheel_bot_base.usd",
        "configuration/wheel_bot_physics.usd",
        "configuration/wheel_bot_robot.usd",
        "configuration/wheel_bot_sensor.usd",
    ):
        assert dependency in capture


def test_urdf_and_usd_camera_files_pass_the_cross_format_contract() -> None:
    """The verifier checks real fixed joints, USD prim metadata, and portability."""

    validate_files(DEFAULT_URDF, DEFAULT_OVERLAY, DEFAULT_ROBOT_ENTRY)


def test_franzisim_is_declared_as_an_isaaclab_extension() -> None:
    """The external-project package exposes its Python module to Kit."""

    config_path = Path("source/franzi_sim/config/extension.toml")
    config = tomllib.loads(config_path.read_text(encoding="utf-8"))
    assert config["package"]["category"] == "isaaclab"
    assert config["dependencies"]["isaaclab"] == {}
    assert config["python"]["module"] == [{"name": "franzi_sim"}]
