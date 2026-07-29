#!/usr/bin/env python3
"""Import the selected franzi Wheel Bot URDF into a versioned Isaac USD asset."""

from __future__ import annotations

import argparse
import os
import shutil
import tempfile
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF = PROJECT_ROOT / "Rviz/src/franzi_description/urdf/wheel_robot_4.0.urdf"
DEFAULT_OUTPUT = PROJECT_ROOT / "usd/assets/robots/wheel_bot/wheel_bot.usd"


def parse_args() -> argparse.Namespace:
    """Parse import paths and launch mode."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--headless", action="store_true", help="run the converter without an Isaac Sim window")
    return parser.parse_args()


def prepare_import_package(source_urdf: Path, work_dir: Path) -> Path:
    """Copy the ROS package and sanitize one USD-invalid leading-digit name.

    The original URDF remains byte-for-byte identical to ``origin/franzi``.
    Isaac's importer cannot create a prim named ``2Dlidar_Link``, so only the
    disposable import copy uses ``lidar_2d_Link``.
    """

    package_root = source_urdf.resolve().parents[1]
    prepared_package = work_dir / package_root.name
    shutil.copytree(package_root, prepared_package)
    prepared_urdf = prepared_package / "urdf" / source_urdf.name
    invalid_mesh = prepared_package / "meshes/2Dlidar_Link.STL"
    valid_mesh = prepared_package / "meshes/lidar_2d_Link.STL"
    invalid_mesh.rename(valid_mesh)
    source = prepared_urdf.read_text(encoding="utf-8")
    prepared_urdf.write_text(
        source.replace("2Dlidar_Link", "lidar_2d_Link").replace("2Dlidar_joint", "lidar_2d_joint"),
        encoding="utf-8",
    )
    return prepared_urdf


def write_portable_config(output: Path, source_urdf: Path) -> None:
    """Replace converter host paths with the stable project-relative source."""

    relative_source = os.path.relpath(source_urdf, output.parent)
    source_commit = (
        "7457a17ead2a7ab199c84a281f0ab90824abc7ef"
        if source_urdf == DEFAULT_URDF.resolve()
        else "custom-source"
    )
    config = f"""# Portable provenance for the generated Wheel Bot USD.
asset_path: {relative_source}
usd_dir: .
usd_file_name: {output.name}
source_branch: origin/franzi
source_commit: {source_commit}
importer: Isaac Sim 5.1 / Isaac Lab 2.3.2 UrdfConverter
fix_base: true
merge_fixed_joints: false
joint_drive:
  target_type: position
  stiffness: 100.0
  damping: 1.0
"""
    (output.parent / "config.yaml").write_text(config, encoding="utf-8")


def main() -> int:
    """Prepare the URDF, run NVIDIA's converter, and record portable provenance."""

    args = parse_args()
    source_urdf = args.urdf.expanduser().resolve()
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix="wheelbot-urdf-") as temp_name:
        prepared_urdf = prepare_import_package(source_urdf, Path(temp_name))
        os.environ["ROS_PACKAGE_PATH"] = str(prepared_urdf.parents[2])

        from isaacsim import SimulationApp

        app = SimulationApp({"headless": args.headless})
        try:
            from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg

            config = UrdfConverterCfg(
                asset_path=str(prepared_urdf),
                usd_dir=str(output.parent),
                usd_file_name=output.name,
                fix_base=True,
                merge_fixed_joints=False,
                force_usd_conversion=True,
                joint_drive=UrdfConverterCfg.JointDriveCfg(
                    gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=100.0, damping=1.0),
                    target_type="position",
                ),
            )
            converter = UrdfConverter(config)
            if Path(converter.usd_path).resolve() != output:
                raise RuntimeError(f"converter wrote an unexpected path: {converter.usd_path}")
        finally:
            app.close()

    write_portable_config(output, source_urdf)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
