"""Validate camera contracts against the delivered URDF and USD layers."""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

from franzi_sim.cameras import CAMERAS, validate_camera_specs


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF = PROJECT_ROOT / "Rviz/src/franzi_description/urdf/wheel_robot_4.0.urdf"
DEFAULT_OVERLAY = PROJECT_ROOT / "usd/assets/robots/wheel_bot/camera_sensors.usda"
DEFAULT_ROBOT_ENTRY = PROJECT_ROOT / "usd/assets/robots/wheel_bot/wheel_bot_with_cameras.usda"


def parse_args() -> argparse.Namespace:
    """Allow CI to validate an alternate imported robot without editing code."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--overlay", type=Path, default=DEFAULT_OVERLAY)
    parser.add_argument("--robot-entry", type=Path, default=DEFAULT_ROBOT_ENTRY)
    return parser.parse_args()


def validate_files(urdf_path: Path, overlay_path: Path, robot_entry_path: Path) -> None:
    """Require real URDF housing links and matching portable USD camera prims."""

    validate_camera_specs()
    root = ET.parse(urdf_path).getroot()
    urdf_links = {link.attrib["name"] for link in root.findall("link")}
    fixed_joint_children = {
        child.attrib["link"]
        for joint in root.findall("joint")
        if joint.attrib.get("type") == "fixed"
        if (child := joint.find("child")) is not None
    }
    overlay = overlay_path.read_text(encoding="utf-8")
    robot_entry = robot_entry_path.read_text(encoding="utf-8")

    for camera in CAMERAS:
        if camera.parent_link not in urdf_links:
            raise ValueError(f"URDF camera housing link is absent: {camera.parent_link}")
        if camera.parent_link not in fixed_joint_children:
            raise ValueError(f"URDF camera housing is not attached by a fixed joint: {camera.parent_link}")
        for expected in (
            camera.parent_link,
            camera.sensor_prim,
            camera.frame_id,
            camera.topic,
            camera.camera_info_topic,
        ):
            if expected not in overlay:
                raise ValueError(f"USD camera overlay is missing {camera.name} value: {expected}")
    expected_clipping = "float2 clippingRange = (0.05, 100)"
    if overlay.count(expected_clipping) != len(CAMERAS):
        raise ValueError(
            "every USD camera must author the shared 0.05-100 m clipping range"
        )

    if "@/" in overlay or "@/" in robot_entry or "/home/" in overlay or "/home/" in robot_entry:
        raise ValueError("camera USD layers contain a machine-specific absolute reference")
    for relative_reference in ("@camera_sensors.usda@", "@wheel_bot.usd@"):
        if relative_reference not in robot_entry:
            raise ValueError(f"complete robot entry is missing {relative_reference}")


def main() -> None:
    """Validate all source files and print their stable ROS endpoints."""

    args = parse_args()
    validate_files(args.urdf, args.overlay, args.robot_entry)
    for camera in CAMERAS:
        print(
            f"{camera.name}: {camera.parent_link} -> {camera.topic} "
            f"({camera.width_px}x{camera.height_px}, "
            f"clip={camera.clipping_range_m[0]}-{camera.clipping_range_m[1]} m)"
        )


if __name__ == "__main__":
    main()
