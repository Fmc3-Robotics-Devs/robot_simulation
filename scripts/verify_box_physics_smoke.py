#!/usr/bin/env python3
"""Run a headless gravity, contact, tag-follow, and reset smoke test."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENE = ROOT / "usd/scenes/warehouse_box_transfer.usda"
BOX_PATH = "/WheelBotBoxTransfer/BlueTransportBox"
TAG_PATH = f"{BOX_PATH}/AprilTag_0"
PHYSICS_SOURCE_FILES = (
    Path("usd/scenes/warehouse_box_transfer.usda"),
    Path("usd/scenes/warehouse_box_transfer_physics.usda"),
    Path("usd/scenes/warehouse_box_transfer_apriltags.usda"),
    Path("usd/assets/props/blue_transport_box.usda"),
    Path("usd/assets/tags/apriltag_36h11.usda"),
)
RAISE_HEIGHT_M = 0.05
POSITION_TOLERANCE_M = 0.02
RESET_TOLERANCE_M = 1e-4
VELOCITY_TOLERANCE_M_S = 0.03
TAG_DISTANCE_TOLERANCE_M = 0.002


def parse_args() -> argparse.Namespace:
    """Parse the formal scene, simulation duration, and evidence path."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--steps", type=int, default=480)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def portable_path(path: Path) -> str:
    """Return a repository-relative path whenever possible."""

    resolved = path.expanduser().resolve()
    try:
        return str(resolved.relative_to(ROOT))
    except ValueError:
        return str(resolved)


def sha256(path: Path) -> str:
    """Hash the exact formal stage used by the physics smoke."""

    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def physics_source_hashes() -> tuple[dict[str, str], str]:
    """Hash the project layers that define box contact and tag attachment."""

    hashes: dict[str, str] = {}
    aggregate = hashlib.sha256()
    for relative_path in PHYSICS_SOURCE_FILES:
        source_path = ROOT / relative_path
        source_hash = sha256(source_path)
        hashes[relative_path.as_posix()] = source_hash
        aggregate.update(relative_path.as_posix().encode("utf-8"))
        aggregate.update(b"\0")
        aggregate.update(source_hash.encode("ascii"))
        aggregate.update(b"\n")
    return hashes, aggregate.hexdigest()


def emit(result: dict[str, object], output: Path) -> None:
    """Persist and flush evidence before the Kit process shuts down."""

    serialized = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(serialized, encoding="utf-8")
    sys.__stdout__.write(serialized)
    sys.__stdout__.flush()


def finite_vector(values: object) -> bool:
    """Return whether every numeric component is finite."""

    return all(math.isfinite(float(value)) for value in values)


def run_smoke(scene: Path, *, steps: int) -> dict[str, object]:
    """Drop the dynamic box onto its table, then verify tag follow and reset."""

    import numpy as np
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleRigidPrim
    from isaacsim.core.utils.xforms import get_world_pose

    if not scene.is_file():
        raise FileNotFoundError(f"formal scene does not exist: {scene}")
    if steps < 240:
        raise ValueError("--steps must be at least 240 for a stable landing check")

    context = omni.usd.get_context()
    if not context.open_stage(str(scene)):
        raise RuntimeError(f"Isaac Sim could not open formal scene: {scene}")
    stage = context.get_stage()
    if not stage.GetPrimAtPath(TAG_PATH).IsValid():
        raise RuntimeError(f"box-attached AprilTag is missing: {TAG_PATH}")

    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / 120.0,
        rendering_dt=1.0 / 60.0,
    )
    box = world.scene.add(
        SingleRigidPrim(
            prim_path=BOX_PATH,
            name="blue_transport_box_physics_smoke",
            reset_xform_properties=False,
        )
    )
    world.reset()
    try:
        initial_position, initial_orientation = box.get_world_pose()
        # Read the child transform without wrapping it as a mutable scene
        # object.  The tag deliberately uses translate + rotateY ops, whereas
        # SingleXFormPrim reset expects an orient op and would rewrite it.
        initial_tag_position, _ = get_world_pose(TAG_PATH)
        initial_tag_distance = float(
            np.linalg.norm(initial_tag_position - initial_position)
        )

        raised_position = np.asarray(initial_position, dtype=float).copy()
        raised_position[2] += RAISE_HEIGHT_M
        box.set_world_pose(
            position=raised_position,
            orientation=np.asarray(initial_orientation, dtype=float),
        )
        box.set_linear_velocity(np.zeros(3, dtype=float))
        box.set_angular_velocity(np.zeros(3, dtype=float))
        world.step(render=False)
        observed_raised_position, _ = box.get_world_pose()

        sampled_z_m: list[float] = []
        for step in range(steps):
            world.step(render=False)
            if step % 20 == 0 or step == steps - 1:
                position, _ = box.get_world_pose()
                sampled_z_m.append(float(position[2]))

        landing_position, landing_orientation = box.get_world_pose()
        landing_linear_velocity = box.get_linear_velocity()
        landing_angular_velocity = box.get_angular_velocity()
        landing_tag_position, _ = get_world_pose(TAG_PATH)
        landing_tag_distance = float(
            np.linalg.norm(landing_tag_position - landing_position)
        )

        world.reset()
        reset_position, reset_orientation = box.get_world_pose()
        reset_error_m = float(np.linalg.norm(reset_position - initial_position))
        horizontal_drift_m = float(
            np.linalg.norm(landing_position[:2] - initial_position[:2])
        )
        landing_height_error_m = abs(
            float(landing_position[2] - initial_position[2])
        )
        downward_travel_m = float(
            observed_raised_position[2] - landing_position[2]
        )
        linear_speed_m_s = float(np.linalg.norm(landing_linear_velocity))
        angular_speed_rad_s = float(np.linalg.norm(landing_angular_velocity))
        tag_distance_error_m = abs(landing_tag_distance - initial_tag_distance)

        checks = {
            "finite_state": all(
                finite_vector(values)
                for values in (
                    initial_position,
                    observed_raised_position,
                    landing_position,
                    landing_orientation,
                    landing_linear_velocity,
                    landing_angular_velocity,
                    reset_position,
                    reset_orientation,
                )
            ),
            "raised_by_command": (
                float(observed_raised_position[2] - initial_position[2])
                >= RAISE_HEIGHT_M - 0.005
            ),
            "gravity_motion_observed": downward_travel_m >= RAISE_HEIGHT_M - 0.01,
            "landed_at_table_height": landing_height_error_m <= POSITION_TOLERANCE_M,
            "horizontal_drift_bounded": horizontal_drift_m <= POSITION_TOLERANCE_M,
            "linear_velocity_settled": linear_speed_m_s <= VELOCITY_TOLERANCE_M_S,
            "angular_velocity_settled": angular_speed_rad_s <= 0.05,
            "apriltag_followed_box": tag_distance_error_m <= TAG_DISTANCE_TOLERANCE_M,
            "reset_restored_initial_pose": reset_error_m <= RESET_TOLERANCE_M,
        }
        source_hashes, source_bundle_hash = physics_source_hashes()
        return {
            "ok": all(checks.values()),
            "scene": portable_path(scene),
            "scene_sha256": sha256(scene),
            "project_source_sha256": source_hashes,
            "project_source_bundle_sha256": source_bundle_hash,
            "box_prim": BOX_PATH,
            "apriltag_prim": TAG_PATH,
            "physics_steps": steps,
            "physics_dt_s": 1.0 / 120.0,
            "checks": checks,
            "measurements": {
                "initial_position_m": initial_position.tolist(),
                "raised_position_m": observed_raised_position.tolist(),
                "landing_position_m": landing_position.tolist(),
                "reset_position_m": reset_position.tolist(),
                "sampled_box_z_m": sampled_z_m,
                "downward_travel_m": downward_travel_m,
                "landing_height_error_m": landing_height_error_m,
                "horizontal_drift_m": horizontal_drift_m,
                "linear_velocity_m_s": landing_linear_velocity.tolist(),
                "linear_speed_m_s": linear_speed_m_s,
                "angular_velocity_rad_s": landing_angular_velocity.tolist(),
                "angular_speed_rad_s": angular_speed_rad_s,
                "initial_tag_to_box_distance_m": initial_tag_distance,
                "landing_tag_to_box_distance_m": landing_tag_distance,
                "tag_distance_error_m": tag_distance_error_m,
                "reset_error_m": reset_error_m,
            },
        }
    finally:
        world.stop()


def main() -> int:
    """Launch Isaac Sim, execute the smoke, and persist truthful evidence."""

    args = parse_args()
    scene = args.scene.expanduser().resolve()
    output = args.output.expanduser().resolve()
    from isaacsim import SimulationApp

    script_argv = sys.argv
    sys.argv = [sys.argv[0]]
    try:
        app = SimulationApp({"headless": True, "fast_shutdown": True})
    finally:
        sys.argv = script_argv

    try:
        result = run_smoke(scene, steps=args.steps)
    except Exception as error:
        result = {
            "ok": False,
            "scene": portable_path(scene),
            "error": str(error),
        }
    emit(result, output)
    app.close()
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
