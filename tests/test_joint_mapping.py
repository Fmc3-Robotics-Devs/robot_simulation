"""Source-only tests for the portable part of the control contract."""

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "source/franzi_sim"))

from franzi_sim.control.validate_joint_map import movable_urdf_joints, project_moveit_joint_owners, validate


def test_joint_mapping_matches_urdf_and_moveit() -> None:
    mapping = json.loads((ROOT / "source/franzi_sim/franzi_sim/control/joint_map.json").read_text())
    names = movable_urdf_joints(ROOT / mapping["urdf"])
    owners = project_moveit_joint_owners(ROOT)
    # The mock is intentionally exact: unit tests cover cross-format naming;
    # the CLI validates the binary USD prims in an Isaac/PXR environment.
    assert validate(mapping, names, owners, set(names)) == []


def test_head_smoke_trajectory_is_small_and_ordered() -> None:
    sys.path.insert(0, str(ROOT / "scripts"))
    from control_smoke import trajectory

    data = trajectory()
    assert data["joint_names"] == ["head_yaw_joint"]
    assert data["points"][1]["positions"] == [0.05]
    assert data["points"][1]["time_from_start"] > data["points"][0]["time_from_start"]


def test_control_smoke_uses_project_robot_and_strict_tolerance() -> None:
    """The executable smoke test must target the formal asset and P2 tolerance."""

    sys.path.insert(0, str(ROOT / "scripts"))
    import control_smoke

    assert control_smoke.DEFAULT_ROBOT_USD == (
        ROOT / "usd/assets/robots/wheel_bot/wheel_bot_with_cameras.usda"
    )
    assert control_smoke.SMOKE_JOINT == "head_yaw_joint"
    assert control_smoke.TRACKING_TOLERANCE_RAD == 0.01
