#!/usr/bin/env python3
"""Import the selected franzi Wheel Bot URDF into a versioned Isaac USD asset."""

from __future__ import annotations

import argparse
import hashlib
import math
import os
import shutil
import sys
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

    The project URDF retains the selected ``origin/franzi`` geometry and joint
    data plus the documented D405 fixed-mount correction.  Isaac's importer
    cannot create a prim named ``2Dlidar_Link``, so only the disposable import
    copy uses ``lidar_2d_Link``.
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
source_patch: align_d405_housing_minus_z_with_gripper_minus_z
importer: Isaac Sim 5.1 native URDF importer 2.4.30
fix_base: true
merge_fixed_joints: false
collision_from_visuals: false
collider_type: convex_hull
self_collision: false
joint_drive:
  target_type: position
  stiffness: 100.0
  damping: 1.0
"""
    (output.parent / "config.yaml").write_text(config, encoding="utf-8")
    fingerprint = hashlib.sha256()
    fingerprint.update(source_urdf.read_bytes())
    fingerprint.update(config.encode("utf-8"))
    (output.parent / ".asset_hash").write_text(
        fingerprint.hexdigest() + "\n",
        encoding="utf-8",
    )


def import_with_native_isaacsim(prepared_urdf: Path, output: Path) -> None:
    """Convert one URDF with the importer shipped by Isaac Sim 5.1.

    Isaac Lab 2.3.2 requests URDF importer 2.4.31, while the pinned Isaac Sim
    5.1 wheel supplies 2.4.30.  Calling the official Isaac Sim commands
    directly keeps the project lock reproducible and preserves the same
    convex-hull, fixed-base, position-drive settings used by the Lab wrapper.
    """

    import omni.kit.app
    import omni.kit.commands
    from isaacsim.asset.importer.urdf._urdf import (
        UrdfJointDriveType,
        UrdfJointTargetType,
        UrdfJointType,
    )

    extension_manager = omni.kit.app.get_app().get_extension_manager()
    extension_manager.set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)

    _, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.set_distance_scale(1.0)
    import_config.set_make_default_prim(True)
    import_config.set_create_physics_scene(False)
    import_config.set_density(0.0)
    # False selects one convex hull per collision mesh, not convex decomposition.
    import_config.set_convex_decomp(False)
    import_config.set_collision_from_visuals(False)
    import_config.set_merge_fixed_joints(False)
    import_config.set_fix_base(True)
    import_config.set_self_collision(False)
    import_config.set_parse_mimic(False)
    import_config.set_replace_cylinders_with_capsules(False)

    parsed, robot_model = omni.kit.commands.execute(
        "URDFParseFile",
        urdf_path=str(prepared_urdf),
        import_config=import_config,
    )
    if not parsed:
        raise ValueError(f"failed to parse URDF: {prepared_urdf}")

    for joint in robot_model.joints.values():
        joint.drive.set_drive_type(UrdfJointDriveType.JOINT_DRIVE_FORCE)
        joint.drive.set_target_type(UrdfJointTargetType.JOINT_DRIVE_POSITION)
        if joint.type == UrdfJointType.JOINT_PRISMATIC:
            joint.drive.set_strength(100.0)
            joint.drive.set_damping(1.0)
        else:
            joint.drive.set_strength(math.pi / 180.0 * 100.0)
            joint.drive.set_damping(math.pi / 180.0 * 1.0)

    imported, _ = omni.kit.commands.execute(
        "URDFImportRobot",
        urdf_path=str(prepared_urdf),
        urdf_robot=robot_model,
        import_config=import_config,
        dest_path=str(output),
    )
    if not imported or not output.is_file():
        raise RuntimeError(f"URDF importer did not create the requested USD: {output}")


def main() -> int:
    """Prepare the URDF, run NVIDIA's converter, and record portable provenance."""

    args = parse_args()
    source_urdf = args.urdf.expanduser().resolve()
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    temp_package = tempfile.TemporaryDirectory(prefix="wheelbot-urdf-")
    app = None
    try:
        temp_name = temp_package.name
        prepared_urdf = prepare_import_package(source_urdf, Path(temp_name))
        os.environ["ROS_PACKAGE_PATH"] = str(prepared_urdf.parents[2])

        # Kit also parses ``sys.argv``.  Hide this script's path flags so
        # ``--output`` cannot be consumed as an application setting.
        script_argv = sys.argv
        sys.argv = [sys.argv[0]]
        try:
            from isaacsim import SimulationApp

            app = SimulationApp(
                {
                    "headless": args.headless,
                    "fast_shutdown": True,
                }
            )
        finally:
            sys.argv = script_argv
        import_with_native_isaacsim(prepared_urdf, output)
        # Write provenance before fast shutdown so Kit cannot leave the
        # generated asset with host-specific temporary paths.
        write_portable_config(output, source_urdf)
        temp_package.cleanup()
        sys.__stdout__.write(f"{output}\n")
        sys.__stdout__.flush()
    except Exception as error:
        temp_package.cleanup()
        sys.__stderr__.write(
            f"import_robot_urdf failed: {type(error).__name__}: {error}\n"
        )
        sys.__stderr__.flush()
        # Kit's fast-shutdown path otherwise converts an in-flight Python
        # exception into exit code 0.  Exit explicitly so CI can trust it.
        os._exit(1)

    if app is not None:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
