"""Strictly validate the Wheel Bot URDF, MoveIt, and converted USD contract."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
from xml.etree import ElementTree as ET

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_MAP = Path(__file__).with_name("joint_map.json")


def movable_urdf_joints(path: Path) -> set[str]:
    """Return non-fixed joints; these are the only joints an articulation can drive."""
    root = ET.parse(path).getroot()
    return {joint.attrib["name"] for joint in root.findall("joint") if joint.attrib["type"] != "fixed"}


def moveit_joint_owners(path: Path) -> dict[str, str]:
    """Read controller ownership directly from MoveIt's controller configuration."""
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    manager = document["moveit_simple_controller_manager"]
    return {joint: controller for controller in manager["controller_names"] for joint in manager[controller]["joints"]}


def project_moveit_joint_owners(root: Path) -> dict[str, str]:
    """Load the checked-out MoveIt file, or its recorded ``origin/franzi`` source.

    The scene branch deliberately carries only the description package.  Falling
    back to the tracked source branch keeps this contract verifiable without
    copying a second ROS package into the Isaac project.
    """
    relative = Path("Rviz/src/franzi_moveit_config/config/moveit_controllers.yaml")
    local = root / relative
    if local.exists():
        return moveit_joint_owners(local)
    result = subprocess.run(
        ["git", "show", f"origin/franzi:{relative.as_posix()}"],
        cwd=root, text=True, capture_output=True, check=False,
    )
    if result.returncode:
        raise RuntimeError(f"MoveIt controllers unavailable locally or in origin/franzi: {result.stderr.strip()}")
    document = yaml.safe_load(result.stdout)
    manager = document["moveit_simple_controller_manager"]
    return {joint: controller for controller in manager["controller_names"] for joint in manager[controller]["joints"]}


def usd_joint_leaf_names(path: Path) -> set[str]:
    """Read USD joint prim names using the runtime's USD bindings.

    A binary USD cannot be safely validated with text search.  Absence of the
    USD bindings is therefore an error, not a skipped check.
    """
    try:
        from pxr import Usd, UsdPhysics
    except ImportError as error:  # pragma: no cover - depends on Isaac install
        raise RuntimeError("USD validation requires Isaac Sim or a Python environment with pxr") from error
    stage = Usd.Stage.Open(str(path))
    if stage is None:
        raise RuntimeError(f"could not open USD stage: {path}")
    return {prim.GetName() for prim in stage.Traverse() if prim.IsA(UsdPhysics.Joint)}


def validate(mapping: dict, urdf_names: set[str], owners: dict[str, str], usd_names: set[str] | None) -> list[str]:
    """Return every disagreement so CI reports the full broken control contract."""
    errors: list[str] = []
    entries = mapping["joints"]
    names = [entry["name"] for entry in entries]
    if len(names) != len(set(names)):
        errors.append("mapping has duplicate joint names")
    if set(names) != urdf_names:
        errors.append(f"URDF/map mismatch: missing={sorted(urdf_names-set(names))}, extra={sorted(set(names)-urdf_names)}")
    for entry in entries:
        name, expected_owner, sdk_dof = entry["name"], entry["moveit_controller"], entry["sdk_dof"]
        if sdk_dof != name:
            errors.append(f"{name}: SDK DOF must equal its USD joint leaf name")
        if owners.get(name) != expected_owner:
            errors.append(f"{name}: MoveIt owner is {owners.get(name)!r}, mapping says {expected_owner!r}")
        if usd_names is not None and name not in usd_names:
            errors.append(f"{name}: absent from USD joint prims")
    unmanaged = set(owners) - set(names)
    if unmanaged:
        errors.append(f"MoveIt references joints absent from mapping: {sorted(unmanaged)}")
    return errors


def emit_result(result: dict[str, object], output: Path | None) -> None:
    """Print and optionally persist the complete cross-format validation result."""

    serialized = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if output is not None:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(serialized, encoding="utf-8")
    # Kit can replace normal stdout during shutdown, so write to the original
    # stream after the result is already durable on disk.
    sys.__stdout__.write(serialized)
    sys.__stdout__.flush()


def main() -> int:
    """Validate the mapping and return a shell status without hiding errors."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mapping", type=Path, default=DEFAULT_MAP)
    parser.add_argument("--skip-usd", action="store_true", help="only permit this for source-only development")
    parser.add_argument("--output", type=Path, help="optional JSON evidence path")
    args = parser.parse_args()
    mapping = json.loads(args.mapping.read_text(encoding="utf-8"))
    urdf = PROJECT_ROOT / mapping["urdf"]
    usd = PROJECT_ROOT / mapping["usd"]
    usd_names = None if args.skip_usd else usd_joint_leaf_names(usd)
    errors = validate(mapping, movable_urdf_joints(urdf), project_moveit_joint_owners(PROJECT_ROOT), usd_names)
    result = {
        "ok": not errors,
        "mapping": str(args.mapping.resolve().relative_to(PROJECT_ROOT)),
        "urdf": mapping["urdf"],
        "usd": mapping["usd"],
        "mapped_joint_count": len(mapping["joints"]),
        "urdf_joint_count": len(movable_urdf_joints(urdf)),
        "moveit_joint_count": len(project_moveit_joint_owners(PROJECT_ROOT)),
        "usd_joint_count": None if usd_names is None else len(usd_names),
        "usd_validation": "skipped" if usd_names is None else "passed",
        "errors": errors,
    }
    emit_result(result, args.output)
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
