#!/usr/bin/env python3
"""Convert the Franzi URDF to USD for Isaac Sim.

Two things have to be got right here or the result is quietly useless:

* ``package://`` mesh paths do not resolve - Isaac runs in its own conda
  environment with no ROS package index - so they are rewritten to absolute
  paths in a generated copy of the URDF. The original is left untouched.
* ``merge_fixed_joints`` defaults to True, which folds every fixed-joint link
  into its parent. All four cameras hang off fixed joints, so the default would
  delete exactly the frames this whole exercise needs.
* USD prim names cannot begin with a digit, and this URDF has ``2Dlidar_Link``.
  The importer half-renames it, produces the ill-formed path
  ``/visuals/a_Dlidar_Link/2Dlidar_Link``, and fails the entire import with
  "Used null prim" - an error that says nothing about the cause.

Run with the Isaac environment, not the ROS one:

    conda run -n env_isaaclab python IssacSim/convert_urdf.py
"""

import argparse
import re
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
DESCRIPTION = REPO / "Rviz" / "src" / "franzi_description"
CAMERA_LINKS = (
    "head_d435_Link",
    "body_d435_Link",
    "left_wrist_d405_Link",
    "right_wrist_d405_Link",
)


def sanitise_names(text: str) -> tuple[str, dict]:
    """Prefix any link or joint name that starts with a digit.

    USD prim paths must begin with a letter or underscore. Renaming here rather
    than in the source URDF keeps the ROS side - where the name is legal and
    already referenced by the SRDF - untouched.
    """
    # Attributes sit on their own lines in this URDF, so the tag and the name
    # are not adjacent.
    names = set(re.findall(r'<(?:link|joint)\s+name="([^"]+)"', text))
    renamed = {name: f"lidar_{name}" if name[0].isdigit() else name for name in names}
    renamed = {old: new for old, new in renamed.items() if old != new}
    for old, new in renamed.items():
        text = re.sub(rf'(name|link)="{re.escape(old)}"', rf'\1="{new}"', text)
    return text, renamed


def sanitise_meshes(text: str, staging: Path) -> dict:
    """Alias mesh files whose names start with a digit.

    The importer names the mesh prim after the file, so a legal link name is
    not enough on its own - ``2Dlidar_Link.STL`` still produces the prim
    ``.../2Dlidar_Link`` and kills the import.
    """
    aliases = {}
    for reference in sorted(set(re.findall(r'filename="([^"]+)"', text))):
        name = Path(reference).name
        if not name[0].isdigit():
            continue
        staging.mkdir(parents=True, exist_ok=True)
        alias = staging / f"lidar_{name}"
        if not alias.exists():
            alias.symlink_to(Path(reference))
        aliases[reference] = alias.as_posix()
    return aliases


def prepare_urdf(source: Path, destination: Path) -> tuple[int, dict]:
    """Copy the URDF with mesh paths absolute and prim-safe names."""
    text = source.read_text()
    text, count = re.subn(
        r"package://franzi_description/", f"{DESCRIPTION.as_posix()}/", text
    )
    text, renamed = sanitise_names(text)
    for reference, alias in sanitise_meshes(text, destination.parent / "meshes").items():
        text = text.replace(f'filename="{reference}"', f'filename="{alias}"')
        renamed[Path(reference).name] = Path(alias).name
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(text)
    return count, renamed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        default=str(REPO / "IssacSim" / "usd" / "franzi.usd"),
        help="Where to write the converted USD.",
    )
    parser.add_argument(
        "--fix-base",
        action="store_true",
        help="Weld the chassis to the world. Off by default so the base can drive.",
    )
    parser.add_argument("--headless", action="store_true", default=True)
    args = parser.parse_args()

    from isaaclab.app import AppLauncher

    launcher = AppLauncher(headless=args.headless)
    simulation_app = launcher.app

    from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg

    staged = REPO / "IssacSim" / "usd" / "wheel_robot_4.0.isaac.urdf"
    replaced, renamed = prepare_urdf(
        DESCRIPTION / "urdf" / "wheel_robot_4.0.urdf", staged
    )
    print(f"rewrote {replaced} mesh paths -> {staged}")
    for old, new in renamed.items():
        print(f"  renamed for USD: {old} -> {new}")

    output = Path(args.output)
    config = UrdfConverterCfg(
        asset_path=str(staged),
        usd_dir=str(output.parent),
        usd_file_name=output.name,
        fix_base=args.fix_base,
        # Keep every fixed-joint link: the cameras are all fixed joints.
        merge_fixed_joints=False,
        # STL files carry no inertia; the URDF supplies it for real links.
        link_density=0.0,
        force_usd_conversion=True,
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            target_type="position",
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                stiffness=1000.0, damping=100.0
            ),
        ),
    )

    converter = UrdfConverter(config)
    print(f"converted -> {converter.usd_path}")

    from pxr import Usd

    stage = Usd.Stage.Open(converter.usd_path)
    prims = {prim.GetName() for prim in stage.Traverse()}
    missing = [link for link in CAMERA_LINKS if link not in prims]
    print(f"prims: {len(prims)}")
    for link in CAMERA_LINKS:
        print(f"  camera frame {link}: {'present' if link in prims else 'MISSING'}")
    if missing:
        raise SystemExit(
            f"camera frames were dropped: {missing}; merge_fixed_joints must stay False"
        )

    simulation_app.close()


if __name__ == "__main__":
    main()
