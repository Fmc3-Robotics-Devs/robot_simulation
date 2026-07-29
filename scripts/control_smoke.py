#!/usr/bin/env python3
"""Run a no-GUI Wheel Bot articulation smoke test on one safe head joint."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
MAPPING = ROOT / "source/franzi_sim/franzi_sim/control/joint_map.json"
DEFAULT_ROBOT_USD = ROOT / "usd/assets/robots/wheel_bot/wheel_bot_with_cameras.usda"
SMOKE_JOINT = "head_yaw_joint"
TARGET_DELTA_RAD = 0.05
TRACKING_TOLERANCE_RAD = 0.01


def trajectory() -> dict:
    """Provide ROS FollowJointTrajectory-compatible data for transport adapters."""

    return {
        "joint_names": [SMOKE_JOINT],
        "points": [
            {"positions": [0.0], "time_from_start": 0.0},
            {"positions": [TARGET_DELTA_RAD], "time_from_start": 1.0},
        ],
    }


def portable_path(path: Path) -> str:
    """Prefer a repository-relative path in persisted evidence."""

    resolved = path.expanduser().resolve()
    try:
        return str(resolved.relative_to(ROOT))
    except ValueError:
        return str(resolved)


def emit_result(result: dict[str, object], output: Path | None) -> None:
    """Print and optionally persist evidence before Kit shuts down."""

    serialized = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if output is not None:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(serialized, encoding="utf-8")
    sys.__stdout__.write(serialized)
    sys.__stdout__.flush()


def run_isaac_smoke(
    robot_usd: Path,
    *,
    steps: int,
) -> dict[str, object]:
    """Load the formal robot USD, command head yaw, and measure tracking."""

    import numpy as np
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.types import ArticulationAction

    if not robot_usd.is_file():
        raise FileNotFoundError(f"robot USD does not exist: {robot_usd}")
    if steps <= 0:
        raise ValueError("--steps must be positive")

    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 120.0, rendering_dt=1.0 / 60.0)
    add_reference_to_stage(str(robot_usd.resolve()), "/World/WheelBot")
    robot = world.scene.add(
        SingleArticulation(
            prim_path="/World/WheelBot",
            name="wheel_bot_control_smoke",
            reset_xform_properties=False,
        )
    )
    world.reset()
    try:
        dof_names = list(robot.dof_names)
        if SMOKE_JOINT not in dof_names:
            raise RuntimeError(f"{SMOKE_JOINT} is absent from articulation DOFs")
        joint_index = robot.get_dof_index(SMOKE_JOINT)
        initial_positions = np.asarray(robot.get_joint_positions(), dtype=float)
        initial = float(initial_positions[joint_index])
        target = initial + TARGET_DELTA_RAD
        if not math.isfinite(initial):
            raise RuntimeError(f"{SMOKE_JOINT} returned a non-finite initial position")

        # Apply only one indexed target; all other DOFs keep their current drive
        # targets and cannot move because this smoke payload omitted them.
        robot.get_articulation_controller().apply_action(
            ArticulationAction(
                joint_positions=np.asarray([target], dtype=float),
                joint_indices=np.asarray([joint_index], dtype=int),
            )
        )
        for _ in range(steps):
            world.step(render=False)

        final = float(robot.get_joint_positions()[joint_index])
        tracking_error = abs(final - target)
        passed = math.isfinite(final) and tracking_error <= TRACKING_TOLERANCE_RAD
        return {
            "ok": passed,
            "robot_usd": portable_path(robot_usd),
            "articulation_prim": robot.prim_path,
            "dof_count": len(dof_names),
            "dof_names": dof_names,
            "joint": SMOKE_JOINT,
            "joint_index": int(joint_index),
            "initial_position_rad": initial,
            "target_position_rad": target,
            "final_position_rad": final,
            "tracking_error_rad": tracking_error,
            "tracking_tolerance_rad": TRACKING_TOLERANCE_RAD,
            "physics_steps": steps,
            "physics_dt_s": 1.0 / 120.0,
        }
    finally:
        world.stop()


def main() -> int:
    """Print the portable command or execute the real Isaac articulation test."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--print-trajectory", action="store_true")
    parser.add_argument("--run-isaac", action="store_true", help="run the formal robot USD in headless Isaac Sim")
    parser.add_argument("--robot-usd", type=Path, default=DEFAULT_ROBOT_USD)
    parser.add_argument("--steps", type=int, default=180, help="physics steps allowed for the 0.05 rad command")
    parser.add_argument("--output", type=Path, help="optional JSON evidence path")
    args = parser.parse_args()
    if args.print_trajectory:
        print(json.dumps(trajectory(), indent=2))
        return 0
    if not args.run_isaac:
        parser.error("choose --print-trajectory or --run-isaac")
    try:
        from isaacsim import SimulationApp
    except ImportError as error:
        raise SystemExit("Isaac Sim Python runtime is unavailable; use --print-trajectory for the portable smoke check") from error
    script_argv = sys.argv
    sys.argv = [sys.argv[0]]
    try:
        app = SimulationApp({"headless": True})
    finally:
        sys.argv = script_argv
    try:
        result = run_isaac_smoke(args.robot_usd, steps=args.steps)
        emit_result(result, args.output)
        return 0 if result["ok"] else 1
    except Exception as error:
        result = {
            "ok": False,
            "robot_usd": portable_path(args.robot_usd),
            "error": str(error),
        }
        emit_result(result, args.output)
        return 1
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
