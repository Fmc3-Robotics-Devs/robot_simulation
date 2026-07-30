#!/usr/bin/env python3
"""Record a deterministic whole-body Wheel Bot box-transfer demonstration.

The robot approaches the existing blue box, closes both grippers, lifts it,
swerves along the PickTable, lowers it at the opposite corner, releases it,
and returns home.  An upright controlled attachment is used while carrying;
the box is returned to dynamic PhysX contact before the grippers retreat.

Five frame-synchronous RTX streams are written: a workcell overview plus the
head, chest, left-wrist, and right-wrist robot cameras.  The standalone
``compose_multiview_video.py`` utility then creates the review board video and
the individual camera MP4 files.
"""

from __future__ import annotations

import argparse
import hashlib
from importlib.metadata import version as package_version
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import Mapping, Sequence

import numpy as np
from PIL import Image


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PROJECT_ROOT / "source/franzi_sim"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from franzi_sim.scenarios.box_transfer_demo import (  # noqa: E402
    BOX_INITIAL_POSITION_M,
    BOX_TARGET_POSITION_M,
    DEMO_SEGMENTS,
    demo_duration_s,
    sample_demo,
)


DEFAULT_SCENE = PROJECT_ROOT / "usd/scenes/warehouse_box_transfer.usda"
ROBOT_PATH = "/WheelBotBoxTransfer/WheelBot"
BOX_PATH = "/WheelBotBoxTransfer/BlueTransportBox"
PICK_TABLE_TOP_Z_M = 0.9940513610839844
BOX_HALF_EXTENTS_M = np.asarray((0.30256665, 0.2025666, 0.0851469))
DELIVERY_XY_TOLERANCE_M = 0.030
GRIP_POINT_TOLERANCE_M = 0.030
GRIP_POINT_IN_WRIST_M = (-0.03045, 0.0, -0.22904)
PHYSICS_HZ = 120
CAMERA_RESOLUTION = (1280, 720)
CAMERA_PATHS = {
    "head": (
        f"{ROBOT_PATH}/head_d435_Link/"
        "head_d435_camera_mount/camera"
    ),
    "chest": (
        f"{ROBOT_PATH}/body_d435_Link/"
        "chest_d435_camera_mount/camera"
    ),
    "left": (
        f"{ROBOT_PATH}/left_wrist_d405_Link/"
        "left_wrist_d405_camera_mount/camera"
    ),
    "right": (
        f"{ROBOT_PATH}/right_wrist_d405_Link/"
        "right_wrist_d405_camera_mount/camera"
    ),
}
WRIST_PATHS = {
    "left": f"{ROBOT_PATH}/left_wrist_roll_Link",
    "right": f"{ROBOT_PATH}/right_wrist_roll_Link",
}
SOURCE_FILES = (
    Path("Rviz/src/franzi_description/urdf/wheel_robot_4.0.urdf"),
    Path("scripts/record_box_transfer_demo.py"),
    Path("scripts/compose_multiview_video.py"),
    Path("source/franzi_sim/franzi_sim/cameras.py"),
    Path("source/franzi_sim/franzi_sim/scenarios/box_transfer_demo.py"),
    Path("usd/scenes/warehouse_box_transfer.usda"),
    Path("usd/scenes/warehouse_box_transfer_workcell.usda"),
    Path("usd/scenes/warehouse_box_transfer_apriltags.usda"),
    Path("usd/scenes/warehouse_box_transfer_table_apriltags.usda"),
    Path("usd/scenes/warehouse_box_transfer_physics.usda"),
    Path("usd/assets/props/blue_transport_box.usda"),
    Path("usd/assets/robots/wheel_bot/.asset_hash"),
    Path("usd/assets/robots/wheel_bot/wheel_bot.usd"),
    Path("usd/assets/robots/wheel_bot/configuration/wheel_bot_base.usd"),
    Path("usd/assets/robots/wheel_bot/configuration/wheel_bot_physics.usd"),
    Path("usd/assets/robots/wheel_bot/configuration/wheel_bot_robot.usd"),
    Path("usd/assets/robots/wheel_bot/configuration/wheel_bot_sensor.usd"),
    Path("usd/assets/robots/wheel_bot/wheel_bot_with_cameras.usda"),
    Path("usd/assets/robots/wheel_bot/camera_sensors.usda"),
)
VIEW_NAMES = ("overview", "head", "chest", "left", "right")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse the formal scene, recording quality, and new output directory."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--rt-subframes", type=int, default=4)
    parser.add_argument(
        "--skip-encode",
        action="store_true",
        help="capture PNG sequences and metadata without running ffmpeg",
    )
    arguments = parser.parse_args(argv)
    if arguments.fps <= 0 or PHYSICS_HZ % arguments.fps:
        parser.error(f"--fps must be a positive divisor of {PHYSICS_HZ}")
    if arguments.rt_subframes <= 0:
        parser.error("--rt-subframes must be positive")
    return arguments


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _portable_path(path: Path) -> str:
    resolved = path.expanduser().resolve()
    try:
        return str(resolved.relative_to(PROJECT_ROOT))
    except ValueError:
        return str(resolved)


def _project_source_hashes() -> tuple[dict[str, str], str]:
    """Bind the recording to its exact trajectory, robot, scene, and cameras."""

    hashes: dict[str, str] = {}
    aggregate = hashlib.sha256()
    for relative_path in SOURCE_FILES:
        source_path = PROJECT_ROOT / relative_path
        if not source_path.is_file():
            raise FileNotFoundError(f"recording source is missing: {relative_path}")
        source_hash = _sha256(source_path)
        hashes[relative_path.as_posix()] = source_hash
        aggregate.update(relative_path.as_posix().encode("utf-8"))
        aggregate.update(b"\0")
        aggregate.update(source_hash.encode("ascii"))
        aggregate.update(b"\n")
    return hashes, aggregate.hexdigest()


def _git_state() -> dict[str, object]:
    revision = subprocess.check_output(
        ["git", "rev-parse", "HEAD"],
        cwd=PROJECT_ROOT,
        text=True,
    ).strip()
    status = subprocess.run(
        ["git", "status", "--porcelain", "--untracked-files=normal"],
        cwd=PROJECT_ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    return {
        "revision": revision,
        "working_tree_dirty_at_capture": bool(status),
    }


def _prepare_output(output_dir: Path) -> None:
    """Require a new/empty destination so a failed run cannot reuse old frames."""

    if output_dir.exists() and any(output_dir.iterdir()):
        raise ValueError(
            f"output directory must be new or empty: {output_dir}"
        )
    output_dir.mkdir(parents=True, exist_ok=True)
    for name in VIEW_NAMES:
        (output_dir / "frames" / name).mkdir(parents=True, exist_ok=False)


def _save_rgb(
    data: object,
    output_path: Path,
    expected_resolution: tuple[int, int],
) -> dict[str, float | int]:
    """Validate one rendered RGB frame before adding it to the sequence."""

    rgba = np.asarray(data)
    width, height = expected_resolution
    if rgba.ndim != 3 or rgba.shape[:2] != (height, width) or rgba.shape[2] < 3:
        raise RuntimeError(
            f"{output_path.name} returned {rgba.shape}; expected "
            f"({height}, {width}, >=3)"
        )
    rgb = rgba[:, :, :3].astype(np.uint8).copy()
    metrics = {
        "minimum": int(rgb.min()),
        "maximum": int(rgb.max()),
        "mean": float(rgb.mean()),
        "standard_deviation": float(rgb.std()),
    }
    if (
        metrics["maximum"] - metrics["minimum"] < 24
        or metrics["standard_deviation"] < 4.0
        or metrics["mean"] < 2.0
    ):
        raise RuntimeError(f"{output_path}: blank/low-contrast frame {metrics}")
    Image.fromarray(rgb).save(output_path, compress_level=1)
    return metrics


def _quaternion_error_rad(first: np.ndarray, second: np.ndarray) -> float:
    """Return the shortest angular difference between scalar-first quaternions."""

    first_normalized = first / np.linalg.norm(first)
    second_normalized = second / np.linalg.norm(second)
    dot = min(1.0, abs(float(np.dot(first_normalized, second_normalized))))
    return 2.0 * math.acos(dot)


def _grasp_geometry(stage: object) -> dict[str, object]:
    """Measure both gripper centers and tool/D405 forward axes in world space."""

    from pxr import Gf, Usd, UsdGeom

    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    measurements: dict[str, object] = {}
    points: dict[str, np.ndarray] = {}
    for side, path in WRIST_PATHS.items():
        transform = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))
        point = transform.Transform(Gf.Vec3d(*GRIP_POINT_IN_WRIST_M))
        tool_forward = transform.TransformDir(Gf.Vec3d(0.0, 0.0, -1.0))
        tool_forward.Normalize()
        points[side] = np.asarray(tuple(point), dtype=float)
        measurements[side] = {
            "grip_point_m": points[side].tolist(),
            "tool_forward_world": list(tool_forward),
            "tool_forward_to_box_front_dot": float(
                Gf.Dot(tool_forward, Gf.Vec3d(1.0, 0.0, 0.0))
            ),
        }
    anchor = 0.5 * (points["left"] + points["right"])
    measurements["anchor_position_m"] = anchor.tolist()
    measurements["hand_separation_m"] = float(
        np.linalg.norm(points["left"] - points["right"])
    )
    return measurements


def _grasp_gate(
    geometry: Mapping[str, object],
    box_position: np.ndarray,
    commanded_joints: Mapping[str, float],
) -> dict[str, object]:
    """Prove both closed hands are aligned before enabling attachment."""

    expected = {
        "left": box_position
        + np.asarray(
            (
                -BOX_HALF_EXTENTS_M[0],
                BOX_HALF_EXTENTS_M[1],
                0.025,
            )
        ),
        "right": box_position
        + np.asarray(
            (
                -BOX_HALF_EXTENTS_M[0],
                -BOX_HALF_EXTENTS_M[1],
                0.025,
            )
        ),
    }
    errors: dict[str, float] = {}
    axis_dots: dict[str, float] = {}
    for side in ("left", "right"):
        measurement = geometry[side]
        assert isinstance(measurement, dict)
        grip_point = np.asarray(measurement["grip_point_m"], dtype=float)
        errors[side] = float(np.linalg.norm(grip_point - expected[side]))
        axis_dots[side] = float(
            measurement["tool_forward_to_box_front_dot"]
        )
    finger_error = max(
        abs(commanded_joints["leftfinger1_joint"] - 0.032),
        abs(commanded_joints["leftfinger2_joint"] + 0.032),
        abs(commanded_joints["rightfinger1_joint"] - 0.032),
        abs(commanded_joints["rightfinger2_joint"] + 0.032),
    )
    valid = (
        max(errors.values()) <= GRIP_POINT_TOLERANCE_M
        and min(axis_dots.values()) >= 0.995
        and finger_error <= 1e-6
    )
    result = {
        "valid": valid,
        "grip_point_error_m": errors,
        "tool_forward_to_box_front_dot": axis_dots,
        "finger_command_error_m": finger_error,
        "thresholds": {
            "maximum_grip_point_error_m": GRIP_POINT_TOLERANCE_M,
            "minimum_tool_axis_dot": 0.995,
            "maximum_finger_command_error_m": 1e-6,
        },
    }
    if not valid:
        raise RuntimeError(f"controlled-attachment grasp gate failed: {result}")
    return result


class TransferRuntime:
    """Own simulator objects and attachment metrics during one capture."""

    def __init__(
        self,
        *,
        stage: object,
        world: object,
        robot: object,
        box: object,
        joint_names: list[str],
        joint_indices: np.ndarray,
        initial_box_position: np.ndarray,
        initial_box_orientation: np.ndarray,
    ) -> None:
        self.stage = stage
        self.world = world
        self.robot = robot
        self.box = box
        self.joint_names = joint_names
        self.joint_indices = joint_indices
        self.initial_box_position = initial_box_position
        self.initial_box_orientation = initial_box_orientation
        self.attached = False
        self.attachment_offset_m: np.ndarray | None = None
        self.grasp_gate: dict[str, object] | None = None
        self.attach_time_s: float | None = None
        self.release_time_s: float | None = None
        self.maximum_attachment_error_m = 0.0
        self.minimum_transport_box_z_m = math.inf
        self.maximum_joint_tracking_error = 0.0
        self.pre_attach_box_drift_m = 0.0

    def _command_robot(self, sample: object) -> dict[str, float]:
        from isaacsim.core.utils.types import ArticulationAction

        commands = sample.joint_map()
        positions = np.asarray(
            [commands[name] for name in self.joint_names],
            dtype=float,
        )
        self.robot.set_world_pose(
            position=np.asarray(sample.base_position_m, dtype=float),
            orientation=np.asarray((1.0, 0.0, 0.0, 0.0), dtype=float),
        )
        self.robot.set_joint_positions(positions, self.joint_indices)
        self.robot.get_articulation_controller().apply_action(
            ArticulationAction(
                joint_positions=positions,
                joint_indices=self.joint_indices,
            )
        )
        return commands

    def _place_box(self, position: np.ndarray) -> None:
        """Set an exact controlled pose while the box remains a dynamic body."""

        self.box.set_world_pose(
            position=position,
            orientation=self.initial_box_orientation,
        )
        self.box.set_linear_velocity(np.zeros(3, dtype=float))
        self.box.set_angular_velocity(np.zeros(3, dtype=float))

    def step(self, sample: object) -> None:
        """Advance one 120 Hz step and maintain/release controlled attachment."""

        commands = self._command_robot(sample)
        if (
            not self.attached
            and self.release_time_s is None
            and not sample.carries_box
        ):
            # Hold the pick object at its authored start until both hands pass
            # the grasp gate.  It remains a dynamic rigid body, so no live
            # kinematic-attribute transition can desynchronize PhysX/Fabric.
            self._place_box(self.initial_box_position)
        self.world.step(render=False)
        # PhysX advances contacts for the released box, then the visual
        # software-in-the-loop pose is restored exactly before measurement and
        # capture.  This is the same deterministic set-after-step pattern used
        # by the static camera evidence script.
        self._command_robot(sample)
        geometry = _grasp_geometry(self.stage)
        anchor = np.asarray(geometry["anchor_position_m"], dtype=float)
        box_position, _box_orientation = self.box.get_world_pose()
        box_position = np.asarray(box_position, dtype=float)

        if sample.carries_box and not self.attached:
            self.pre_attach_box_drift_m = float(
                np.linalg.norm(
                    box_position - self.initial_box_position
                )
            )
            if self.pre_attach_box_drift_m > 0.02:
                raise RuntimeError(
                    "box drifted before grasp by "
                    f"{self.pre_attach_box_drift_m:.4f} m: "
                    f"initial={self.initial_box_position.tolist()}, "
                    f"observed={box_position.tolist()}"
                )
            self._place_box(self.initial_box_position)
            box_position = self.initial_box_position.copy()
            self.grasp_gate = _grasp_gate(geometry, box_position, commands)
            self.attachment_offset_m = box_position - anchor
            self.attached = True
            self.attach_time_s = float(sample.time_s)

        if self.attached and sample.carries_box:
            assert self.attachment_offset_m is not None
            target_position = anchor + self.attachment_offset_m
            self._place_box(target_position)
            observed_position, _ = self.box.get_world_pose()
            self.maximum_attachment_error_m = max(
                self.maximum_attachment_error_m,
                float(
                    np.linalg.norm(
                        np.asarray(observed_position, dtype=float)
                        - target_position
                    )
                ),
            )
            if sample.segment_name == "transport":
                self.minimum_transport_box_z_m = min(
                    self.minimum_transport_box_z_m,
                    float(target_position[2]),
                )

        if self.attached and not sample.carries_box:
            assert self.attachment_offset_m is not None
            final_position = anchor + self.attachment_offset_m
            self._place_box(final_position)
            self.attached = False
            self.release_time_s = float(sample.time_s)

        observed = np.asarray(self.robot.get_joint_positions(), dtype=float)[
            self.joint_indices
        ]
        commanded = np.asarray(
            [commands[name] for name in self.joint_names],
            dtype=float,
        )
        self.maximum_joint_tracking_error = max(
            self.maximum_joint_tracking_error,
            float(np.max(np.abs(observed - commanded))),
        )


def _capture_sequences(
    *,
    app: object,
    scene: Path,
    output_dir: Path,
    fps: int,
    rt_subframes: int,
) -> tuple[dict[str, object], list[dict[str, object]]]:
    """Run the timeline and save the five synchronized PNG sequences."""

    import omni.replicator.core as rep
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
    from pxr import UsdGeom

    context = omni.usd.get_context()
    if not context.open_stage(str(scene)):
        raise RuntimeError(f"Isaac Sim could not open formal scene: {scene}")
    for _ in range(40):
        app.update()
    stage = context.get_stage()

    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / PHYSICS_HZ,
        rendering_dt=1.0 / fps,
    )
    robot = world.scene.add(
        SingleArticulation(
            prim_path=ROBOT_PATH,
            name="wheel_bot_box_transfer_demo",
            reset_xform_properties=False,
        )
    )
    box = world.scene.add(
        SingleRigidPrim(
            prim_path=BOX_PATH,
            name="blue_transport_box_demo",
            reset_xform_properties=False,
        )
    )
    world.reset()
    initial_box_position, initial_box_orientation = box.get_world_pose()
    initial_box_position = np.asarray(initial_box_position, dtype=float)
    initial_box_orientation = np.asarray(initial_box_orientation, dtype=float)
    if np.linalg.norm(
        initial_box_position - np.asarray(BOX_INITIAL_POSITION_M)
    ) > 0.005:
        raise RuntimeError(
            "formal box start pose drifted: "
            f"{initial_box_position.tolist()}"
        )

    first_sample = sample_demo(0.0)
    joint_names = list(first_sample.joint_map())
    missing = sorted(set(joint_names) - set(robot.dof_names))
    if missing:
        raise RuntimeError(f"demo joints are absent from articulation: {missing}")
    joint_indices = np.asarray(
        [robot.get_dof_index(name) for name in joint_names],
        dtype=int,
    )
    dof_properties = robot.dof_properties
    lower = np.asarray(dof_properties["lower"][joint_indices], dtype=float)
    upper = np.asarray(dof_properties["upper"][joint_indices], dtype=float)
    finite_limits = np.isfinite(lower) & np.isfinite(upper)
    minimum_limit_margin = math.inf
    for segment in DEMO_SEGMENTS:
        for pose in (segment.start, segment.end):
            positions = pose.joint_map()
            values = np.asarray(
                [positions[name] for name in joint_names if name in positions],
                dtype=float,
            )
            pose_indices = np.asarray(
                [joint_names.index(name) for name in joint_names if name in positions],
                dtype=int,
            )
            finite = finite_limits[pose_indices]
            margins = np.minimum(
                values[finite] - lower[pose_indices][finite],
                upper[pose_indices][finite] - values[finite],
            )
            if len(margins):
                minimum_limit_margin = min(
                    minimum_limit_margin,
                    float(np.min(margins)),
                )

    runtime = TransferRuntime(
        stage=stage,
        world=world,
        robot=robot,
        box=box,
        joint_names=joint_names,
        joint_indices=joint_indices,
        initial_box_position=initial_box_position,
        initial_box_orientation=initial_box_orientation,
    )
    runtime.step(first_sample)

    overview_camera = rep.create.camera(
        position=(-3.6, -4.8, 3.7),
        look_at=(1.25, -0.10, 0.92),
        focal_length=20.0,
        horizontal_aperture=20.0,
        clipping_range=(0.05, 1000.0),
    )
    view_specs: dict[str, object] = {"overview": overview_camera}
    for name, camera_path in CAMERA_PATHS.items():
        camera_prim = stage.GetPrimAtPath(camera_path)
        if not camera_prim.IsValid() or not camera_prim.IsA(UsdGeom.Camera):
            raise RuntimeError(f"{name} camera is missing: {camera_path}")
        view_specs[name] = camera_path

    annotators: dict[str, object] = {}
    for name in VIEW_NAMES:
        render_product = rep.create.render_product(
            view_specs[name],
            CAMERA_RESOLUTION,
        )
        annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        annotator.attach([render_product])
        annotators[name] = annotator

    # Complete the first-time MDL/mesh load before frame zero so no sequence
    # can begin with a stale or low-resolution renderer warm-up image.
    rep.orchestrator.step(
        rt_subframes=max(16, rt_subframes),
        pause_timeline=False,
        delta_time=0.0,
    )

    frame_count = round(demo_duration_s() * fps) + 1
    physics_substeps = PHYSICS_HZ // fps
    timeline_frames: list[dict[str, object]] = []
    pixel_summaries = {
        name: {
            "minimum_standard_deviation": math.inf,
            "minimum_mean": math.inf,
            "maximum_mean": -math.inf,
        }
        for name in VIEW_NAMES
    }
    last_time_s = 0.0
    for frame_index in range(frame_count):
        time_s = frame_index / fps
        if frame_index:
            for substep in range(1, physics_substeps + 1):
                substep_time = last_time_s + (
                    (time_s - last_time_s) * substep / physics_substeps
                )
                runtime.step(sample_demo(substep_time))
        sample = sample_demo(time_s)
        rep.orchestrator.step(
            rt_subframes=rt_subframes,
            pause_timeline=False,
            delta_time=0.0,
        )
        for name in VIEW_NAMES:
            frame_path = (
                output_dir
                / "frames"
                / name
                / f"frame_{frame_index:05d}.png"
            )
            metrics = _save_rgb(
                annotators[name].get_data(),
                frame_path,
                CAMERA_RESOLUTION,
            )
            summary = pixel_summaries[name]
            summary["minimum_standard_deviation"] = min(
                summary["minimum_standard_deviation"],
                metrics["standard_deviation"],
            )
            summary["minimum_mean"] = min(
                summary["minimum_mean"],
                metrics["mean"],
            )
            summary["maximum_mean"] = max(
                summary["maximum_mean"],
                metrics["mean"],
            )
        box_position, _ = box.get_world_pose()
        root_position, _ = robot.get_world_pose()
        timeline_frames.append(
            {
                "frame_index": frame_index,
                "time_s": round(time_s, 6),
                "stage": sample.segment_name,
                "stage_progress": round(sample.segment_progress, 6),
                "progress": round(sample.overall_progress, 6),
                "box_attached": runtime.attached,
                "box_position_m": [
                    round(float(value), 6) for value in box_position
                ],
                "robot_base_position_m": [
                    round(float(value), 6) for value in root_position
                ],
            }
        )
        if frame_index % max(1, fps * 2) == 0:
            sys.__stdout__.write(
                f"captured {frame_index + 1}/{frame_count} "
                f"frames · {sample.segment_name}\n"
            )
            sys.__stdout__.flush()
        last_time_s = time_s

    # Give the released box one additional second of real PhysX contact before
    # measuring the delivered result; this does not alter the recorded frames.
    for _ in range(PHYSICS_HZ):
        runtime.step(sample_demo(demo_duration_s()))
    final_box_position, final_box_orientation = box.get_world_pose()
    final_linear_velocity = np.asarray(box.get_linear_velocity(), dtype=float)
    final_angular_velocity = np.asarray(box.get_angular_velocity(), dtype=float)
    final_box_position = np.asarray(final_box_position, dtype=float)
    final_box_orientation = np.asarray(final_box_orientation, dtype=float)

    final_position_error = float(
        np.linalg.norm(
            final_box_position - np.asarray(BOX_TARGET_POSITION_M)
        )
    )
    final_xy_error = float(
        np.linalg.norm(
            final_box_position[:2]
            - np.asarray(BOX_TARGET_POSITION_M[:2])
        )
    )
    final_orientation_error = _quaternion_error_rad(
        final_box_orientation,
        initial_box_orientation,
    )
    checks = {
        "grasp_gate_passed": bool(runtime.grasp_gate and runtime.grasp_gate["valid"]),
        "pre_attach_box_drift_bounded": runtime.pre_attach_box_drift_m <= 0.02,
        "attachment_follow_error_bounded": runtime.maximum_attachment_error_m <= 0.002,
        "transport_clearance_reached": (
            runtime.minimum_transport_box_z_m
            >= PICK_TABLE_TOP_Z_M + 0.25
        ),
        "released_before_retreat": runtime.release_time_s is not None,
        "delivered_to_opposite_corner": (
            final_xy_error <= DELIVERY_XY_TOLERANCE_M
        ),
        "final_height_on_table": (
            abs(final_box_position[2] - PICK_TABLE_TOP_Z_M) <= 0.003
        ),
        "final_orientation_upright": final_orientation_error <= math.radians(2.0),
        "final_linear_velocity_settled": np.linalg.norm(final_linear_velocity) <= 0.02,
        "final_angular_velocity_settled": np.linalg.norm(final_angular_velocity) <= 0.02,
        "joint_tracking_error_bounded": runtime.maximum_joint_tracking_error <= 0.01,
        "all_five_frame_sequences_valid": all(
            summary["minimum_standard_deviation"] >= 4.0
            and summary["minimum_mean"] >= 2.0
            for summary in pixel_summaries.values()
        ),
    }
    checks = {name: bool(value) for name, value in checks.items()}
    if not all(checks.values()):
        raise RuntimeError(
            "box-transfer runtime checks failed: "
            f"{json.dumps(checks, sort_keys=True)}; "
            f"final_box_position_m={final_box_position.tolist()}; "
            f"target_box_position_m={list(BOX_TARGET_POSITION_M)}; "
            f"final_xy_error_m={final_xy_error:.6f}; "
            f"final_orientation_error_rad={final_orientation_error:.6f}; "
            "maximum_joint_tracking_error_rad="
            f"{runtime.maximum_joint_tracking_error:.6f}"
        )
    world.stop()
    return (
        {
            "runtime_checks": checks,
            "frame_count": frame_count,
            "physics_substeps_per_frame": physics_substeps,
            "minimum_joint_limit_margin": minimum_limit_margin,
            "maximum_joint_tracking_error": runtime.maximum_joint_tracking_error,
            "pixel_summaries": pixel_summaries,
            "grasp_gate": runtime.grasp_gate,
            "attachment": {
                "method": (
                    "controlled dynamic-rigid-body pose constraint to "
                    "dual-gripper midpoint"
                ),
                "attach_time_s": runtime.attach_time_s,
                "release_time_s": runtime.release_time_s,
                "pre_attach_box_drift_m": runtime.pre_attach_box_drift_m,
                "maximum_follow_error_m": runtime.maximum_attachment_error_m,
                "minimum_transport_box_z_m": runtime.minimum_transport_box_z_m,
            },
            "result": {
                "initial_box_position_m": initial_box_position.tolist(),
                "target_box_position_m": list(BOX_TARGET_POSITION_M),
                "final_box_position_m": final_box_position.tolist(),
                "final_position_error_m": final_position_error,
                "final_xy_error_m": final_xy_error,
                "final_orientation_error_rad": final_orientation_error,
                "final_linear_velocity_m_s": final_linear_velocity.tolist(),
                "final_angular_velocity_rad_s": final_angular_velocity.tolist(),
            },
        },
        timeline_frames,
    )


def _representative_frame_indices(
    timeline_frames: Sequence[Mapping[str, object]],
) -> list[tuple[str, int]]:
    """Choose one readable frame for every important manipulation phase."""

    stage_order = (
        "approach",
        "pregrasp",
        "grasp",
        "lift",
        "transport",
        "lower",
        "release",
        "complete",
    )
    representatives: list[tuple[str, int]] = []
    for stage in stage_order:
        matching = [
            frame for frame in timeline_frames if frame["stage"] == stage
        ]
        if not matching:
            raise RuntimeError(f"timeline has no frames for stage {stage}")
        chosen = (
            matching[len(matching) // 2]
            if stage == "transport"
            else matching[-1]
        )
        representatives.append((stage, int(chosen["frame_index"])))
    return representatives


def _finalize_media(
    output_dir: Path,
    *,
    fps: int,
    timeline_frames: list[dict[str, object]],
) -> dict[str, dict[str, object]]:
    """Encode the composite and raw videos, then save review keyframes."""

    from compose_multiview_video import (
        collect_sequences,
        compose_frame,
        encode_mp4,
        write_frames,
    )

    frame_root = output_dir / "frames"
    sequences = collect_sequences(
        {name: frame_root / name for name in VIEW_NAMES}
    )
    video_dir = output_dir / "videos"
    video_dir.mkdir(parents=True, exist_ok=False)
    with tempfile.TemporaryDirectory(prefix="wheelbot_box_transfer_board_") as temporary:
        composite_frames = Path(temporary)
        write_frames(sequences, composite_frames, timeline_frames)
        encode_mp4(
            composite_frames,
            video_dir / "wheelbot_box_transfer_multiview.mp4",
            fps,
        )
    for name in VIEW_NAMES:
        encode_mp4(
            frame_root / name,
            video_dir / f"{name}.mp4",
            fps,
        )

    keyframe_dir = output_dir / "keyframes"
    keyframe_dir.mkdir(parents=True, exist_ok=False)
    representatives = _representative_frame_indices(timeline_frames)
    thumbnail: Image.Image | None = None
    for order, (stage, frame_index) in enumerate(representatives, start=1):
        opened = {
            name: Image.open(sequences[name][frame_index]).convert("RGB")
            for name in VIEW_NAMES
        }
        try:
            composite = compose_frame(
                opened,
                frame_index,
                timeline_frames[frame_index],
            )
            composite.save(
                keyframe_dir / f"{order:02d}_{stage}.png",
                compress_level=1,
            )
            if stage == "transport":
                thumbnail = composite.copy()
        finally:
            for image in opened.values():
                image.close()
    if thumbnail is None:
        raise RuntimeError("transport thumbnail frame was not selected")
    thumbnail.save(output_dir / "thumbnail.png", compress_level=1)
    thumbnail.close()

    artifacts: dict[str, dict[str, object]] = {}
    for path in sorted(
        [
            *(video_dir.glob("*.mp4")),
            *(keyframe_dir.glob("*.png")),
            output_dir / "thumbnail.png",
            output_dir / "timeline.json",
        ]
    ):
        artifacts[path.relative_to(output_dir).as_posix()] = {
            "sha256": _sha256(path),
            "size_bytes": path.stat().st_size,
        }
    return artifacts


def main(argv: Sequence[str] | None = None) -> int:
    """Run the simulator first, then encode and publish a truthful manifest."""

    args = parse_args(argv)
    scene = args.scene.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    if not scene.is_file():
        raise FileNotFoundError(f"formal scene does not exist: {scene}")
    _prepare_output(output_dir)
    source_hashes, source_bundle_hash = _project_source_hashes()
    git_state = _git_state()

    script_argv = sys.argv
    sys.argv = [sys.argv[0]]
    try:
        from isaacsim import SimulationApp

        app = SimulationApp(
            {
                "headless": args.headless,
                "width": CAMERA_RESOLUTION[0],
                "height": CAMERA_RESOLUTION[1],
                "renderer": "RaytracedLighting",
                "fast_shutdown": True,
            }
        )
    finally:
        sys.argv = script_argv

    try:
        runtime_result, timeline_frames = _capture_sequences(
            app=app,
            scene=scene,
            output_dir=output_dir,
            fps=args.fps,
            rt_subframes=args.rt_subframes,
        )
    except Exception as error:
        sys.__stderr__.write(
            "record_box_transfer_demo failed: "
            f"{type(error).__name__}: {error}\n"
        )
        sys.__stderr__.flush()
        os._exit(1)

    timeline_path = output_dir / "timeline.json"
    timeline_path.write_text(
        json.dumps({"frames": timeline_frames}, indent=2) + "\n",
        encoding="utf-8",
    )
    artifacts: dict[str, dict[str, object]] = {
        timeline_path.relative_to(output_dir).as_posix(): {
            "sha256": _sha256(timeline_path),
            "size_bytes": timeline_path.stat().st_size,
        }
    }
    if not args.skip_encode:
        artifacts = _finalize_media(
            output_dir,
            fps=args.fps,
            timeline_frames=timeline_frames,
        )

    manifest = {
        "ok": True,
        "implementation_level": "controlled-attachment software-in-the-loop demo",
        "formal_scene": _portable_path(scene),
        "formal_scene_sha256": _sha256(scene),
        "renderer": {
            "isaac_sim_version": package_version("isaacsim"),
            "mode": "RaytracedLighting",
            "rt_subframes": args.rt_subframes,
        },
        "capture": {
            "fps": args.fps,
            "duration_s": demo_duration_s(),
            "frame_count": runtime_result["frame_count"],
            "resolution": list(CAMERA_RESOLUTION),
            "views": list(VIEW_NAMES),
            "frame_synchronous": True,
        },
        "motion": {
            "segments": [
                {
                    "name": segment.name,
                    "duration_s": segment.duration_s,
                    "carries_box": segment.carries_box,
                }
                for segment in DEMO_SEGMENTS
            ],
            "whole_body_components": [
                "kinematic mobile base",
                "three steering pods and wheel roll",
                "dual 7-DOF arms",
                "four gripper fingers",
                "head pitch",
            ],
        },
        **runtime_result,
        "git": git_state,
        "project_source_sha256": source_hashes,
        "project_source_bundle_sha256": source_bundle_hash,
        "artifacts": artifacts,
    }
    manifest_path = output_dir / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    sys.__stdout__.write(
        json.dumps(
            {
                "ok": True,
                "manifest": str(manifest_path),
                "frame_count": runtime_result["frame_count"],
                "runtime_checks": runtime_result["runtime_checks"],
                "result": runtime_result["result"],
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n"
    )
    sys.__stdout__.flush()
    # With fast_shutdown enabled, app.close() terminates Kit immediately.
    # Close only after every video, hash, timeline, and manifest is durable.
    app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
