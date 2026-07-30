#!/usr/bin/env python3
"""Render the formal warehouse workcell and all four WheelBot RGB cameras.

The capture is deliberately made from ``warehouse_box_transfer.usda`` rather
than a synthetic calibration scene.  It therefore checks the exact USD entry
that users open in Isaac Sim, including the box-attached AprilTag.
"""

from __future__ import annotations

import argparse
import hashlib
from importlib.metadata import version as package_version
import json
from datetime import datetime
from pathlib import Path
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENE = PROJECT_ROOT / "usd/scenes/warehouse_box_transfer.usda"
CAMERAS = (
    (
        "head_d435",
        "/WheelBotBoxTransfer/WheelBot/head_d435_Link/"
        "head_d435_camera_mount/camera",
    ),
    (
        "chest_d435",
        "/WheelBotBoxTransfer/WheelBot/body_d435_Link/"
        "chest_d435_camera_mount/camera",
    ),
    (
        "left_wrist_d405",
        "/WheelBotBoxTransfer/WheelBot/left_wrist_d405_Link/"
        "left_wrist_d405_camera_mount/camera",
    ),
    (
        "right_wrist_d405",
        "/WheelBotBoxTransfer/WheelBot/right_wrist_d405_Link/"
        "right_wrist_d405_camera_mount/camera",
    ),
)
PROJECT_SOURCE_FILES = (
    Path("usd/scenes/warehouse_box_transfer.usda"),
    Path("usd/scenes/warehouse_box_transfer_workcell.usda"),
    Path("usd/scenes/warehouse_box_transfer_apriltags.usda"),
    Path("usd/scenes/warehouse_box_transfer_physics.usda"),
    Path("usd/assets/robots/wheel_bot/wheel_bot.usd"),
    Path("usd/assets/robots/wheel_bot/wheel_bot_with_cameras.usda"),
    Path("usd/assets/robots/wheel_bot/camera_sensors.usda"),
    Path("usd/assets/props/packing_table.usda"),
    Path("usd/assets/props/blue_transport_box.usda"),
    Path("usd/assets/tags/apriltag_36h11.usda"),
    Path("source/franzi_sim/franzi_sim/cameras.py"),
    Path("source/franzi_sim/franzi_sim/scenarios/box_transfer.py"),
    Path("scripts/capture_workcell_evidence.py"),
)


def parse_args() -> argparse.Namespace:
    """Parse the formal scene and evidence destination."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--rt-subframes", type=int, default=32)
    return parser.parse_args()


def _sha256(path: Path) -> str:
    """Return the SHA-256 digest of an evidence or source file."""

    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _project_source_hashes() -> tuple[dict[str, str], str]:
    """Hash every project-owned layer that composes the formal scene.

    Hashing only the top USDA misses changes in sublayers and referenced robot
    overlays.  The aggregate digest keeps rendered evidence traceable to the
    complete project-authored scene bundle.
    """

    source_hashes: dict[str, str] = {}
    aggregate = hashlib.sha256()
    for relative_path in PROJECT_SOURCE_FILES:
        source_path = PROJECT_ROOT / relative_path
        source_hash = _sha256(source_path)
        source_hashes[relative_path.as_posix()] = source_hash
        aggregate.update(relative_path.as_posix().encode("utf-8"))
        aggregate.update(b"\0")
        aggregate.update(source_hash.encode("ascii"))
        aggregate.update(b"\n")
    return source_hashes, aggregate.hexdigest()


def _git_state() -> dict[str, object]:
    """Record the source revision and whether tracked project files were dirty."""

    revision = subprocess.check_output(
        ["git", "rev-parse", "HEAD"],
        cwd=PROJECT_ROOT,
        text=True,
    ).strip()
    dirty_result = subprocess.run(
        ["git", "status", "--porcelain", "--untracked-files=no"],
        cwd=PROJECT_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    return {
        "revision": revision,
        "tracked_files_dirty_at_capture": bool(dirty_result.stdout.strip()),
    }


def _detect_apriltag_ids(image_path: Path, cv2: object) -> list[int]:
    """Return sorted tag36h11 IDs decoded from one saved RTX frame."""

    image = cv2.imread(str(image_path))
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
        cv2.aruco.DetectorParameters(),
    )
    _corners, detected_ids, _rejected = detector.detectMarkers(image)
    return (
        sorted(int(value) for value in detected_ids.reshape(-1))
        if detected_ids is not None
        else []
    )


def _font(size: int, *, bold: bool = False) -> ImageFont.ImageFont:
    """Load a predictable system font and fall back safely."""

    family = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    try:
        return ImageFont.truetype(family, size)
    except OSError:
        return ImageFont.load_default()


def _save_rgb(
    data: object,
    output_path: Path,
    expected_resolution: tuple[int, int],
) -> dict[str, float | int]:
    """Validate one RTX frame, write it, and return objective pixel metrics."""

    rgba = np.asarray(data)
    width, height = expected_resolution
    if rgba.shape[:2] != (height, width) or rgba.shape[2] < 3:
        raise RuntimeError(
            f"{output_path.name} returned {rgba.shape}; expected "
            f"({height}, {width}, >=3)"
        )
    rgb = rgba[:, :, :3].astype(np.uint8).copy()
    minimum = int(rgb.min())
    maximum = int(rgb.max())
    mean = float(rgb.mean())
    standard_deviation = float(rgb.std())
    # A shape-only check previously accepted an all-black AprilTag image.  The
    # contrast gate makes that failure impossible to record as valid evidence.
    if maximum - minimum < 24 or standard_deviation < 4.0 or mean < 2.0:
        raise RuntimeError(
            f"{output_path.name} is blank or low-contrast: "
            f"min={minimum}, max={maximum}, mean={mean:.2f}, "
            f"std={standard_deviation:.2f}"
        )
    Image.fromarray(rgb).save(output_path)
    return {
        "minimum": minimum,
        "maximum": maximum,
        "mean": round(mean, 3),
        "standard_deviation": round(standard_deviation, 3),
    }


def _fit_cover(image: Image.Image, size: tuple[int, int]) -> Image.Image:
    """Resize and centre-crop an image to fill a presentation cell."""

    target_width, target_height = size
    scale = max(target_width / image.width, target_height / image.height)
    resized = image.resize(
        (round(image.width * scale), round(image.height * scale)),
        Image.Resampling.LANCZOS,
    )
    left = max(0, (resized.width - target_width) // 2)
    top = max(0, (resized.height - target_height) // 2)
    return resized.crop((left, top, left + target_width, top + target_height))


def _save_four_camera_sheet(
    captures: list[tuple[str, Path]],
    output_path: Path,
) -> None:
    """Create a Unitree-style 2×2 sheet for the robot-mounted cameras."""

    cell_width, cell_height = 640, 360
    header_height, gap = 42, 10
    canvas = Image.new(
        "RGB",
        (
            cell_width * 2 + gap * 3,
            (cell_height + header_height) * 2 + gap * 3,
        ),
        (13, 17, 22),
    )
    draw = ImageDraw.Draw(canvas)
    font = _font(24, bold=True)
    for index, (label, path) in enumerate(captures):
        row, column = divmod(index, 2)
        x = gap + column * (cell_width + gap)
        y = gap + row * (cell_height + header_height + gap)
        canvas.paste(
            _fit_cover(Image.open(path).convert("RGB"), (cell_width, cell_height)),
            (x, y),
        )
        draw.rectangle(
            (x, y + cell_height, x + cell_width, y + cell_height + header_height),
            fill=(23, 30, 38),
        )
        draw.text(
            (x + 14, y + cell_height + 7),
            label,
            font=font,
            fill=(242, 246, 250),
        )
    canvas.save(output_path)


def _save_review_board(
    overview_path: Path,
    closeup_path: Path,
    camera_paths: list[tuple[str, Path]],
    output_path: Path,
) -> None:
    """Create one review image containing overview, tag inset, and four cameras."""

    width, overview_height, camera_height = 1600, 850, 250
    header_height, footer_height, gap = 80, 58, 10
    canvas_height = header_height + overview_height + gap + camera_height + footer_height
    canvas = Image.new("RGB", (width, canvas_height), (11, 16, 22))
    draw = ImageDraw.Draw(canvas)
    title_font = _font(31, bold=True)
    body_font = _font(19)
    camera_font = _font(19, bold=True)

    draw.text(
        (28, 18),
        "WHEEL BOT · WAREHOUSE BOX TRANSFER WORKCELL",
        font=title_font,
        fill=(244, 247, 250),
    )
    draw.text(
        (28, 53),
        "Official Isaac assets · AprilTag 0 attached to blue box · formal USD",
        font=body_font,
        fill=(146, 190, 226),
    )

    overview = _fit_cover(
        Image.open(overview_path).convert("RGB"),
        (width, overview_height),
    )
    canvas.paste(overview, (0, header_height))

    # The inset makes the physical tag placement reviewable even when the full
    # warehouse is framed wide enough to show the mobile delivery station.
    inset_size = (420, 236)
    inset = _fit_cover(Image.open(closeup_path).convert("RGB"), inset_size)
    inset_x = width - inset_size[0] - 24
    inset_y = header_height + overview_height - inset_size[1] - 24
    draw.rectangle(
        (
            inset_x - 5,
            inset_y - 34,
            inset_x + inset_size[0] + 5,
            inset_y + inset_size[1] + 5,
        ),
        fill=(10, 15, 21),
    )
    draw.text(
        (inset_x + 8, inset_y - 29),
        "APRILTAG 0 · BOX FRONT FACE",
        font=camera_font,
        fill=(255, 255, 255),
    )
    canvas.paste(inset, (inset_x, inset_y))

    cell_width = width // 4
    camera_y = header_height + overview_height + gap
    for index, (label, path) in enumerate(camera_paths):
        x = index * cell_width
        image = _fit_cover(
            Image.open(path).convert("RGB"),
            (cell_width - 2, camera_height),
        )
        canvas.paste(image, (x, camera_y))
        draw.rectangle(
            (x, camera_y, x + cell_width - 2, camera_y + 34),
            fill=(14, 20, 27),
        )
        draw.text(
            (x + 10, camera_y + 6),
            label,
            font=camera_font,
            fill=(255, 255, 255),
        )
    draw.text(
        (26, camera_y + camera_height + 17),
        "RTX REAL-TIME · 1280×720 SENSOR FRAMES · VISUAL QA EVIDENCE",
        font=body_font,
        fill=(160, 176, 191),
    )
    canvas.save(output_path)


def _apply_robot_review_pose() -> tuple[object, dict[str, object]]:
    """Initialize the articulation and apply the deterministic pre-grasp pose."""

    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.types import ArticulationAction

    from franzi_sim.scenarios import BOX_TRANSFER_CAMERA_REVIEW_POSE_RAD

    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / 120.0,
        rendering_dt=1.0 / 60.0,
    )
    robot = world.scene.add(
        SingleArticulation(
            prim_path="/WheelBotBoxTransfer/WheelBot",
            name="wheel_bot_camera_review_pose",
            reset_xform_properties=False,
        )
    )
    world.reset()
    joint_names = [name for name, _position in BOX_TRANSFER_CAMERA_REVIEW_POSE_RAD]
    joint_positions = np.asarray(
        [position for _name, position in BOX_TRANSFER_CAMERA_REVIEW_POSE_RAD],
        dtype=float,
    )
    joint_indices = np.asarray(
        [robot.get_dof_index(name) for name in joint_names],
        dtype=int,
    )
    dof_properties = robot.dof_properties
    joint_limits = np.column_stack(
        (
            dof_properties["lower"][joint_indices],
            dof_properties["upper"][joint_indices],
        )
    )
    lower_violation = joint_positions < joint_limits[:, 0] - 1e-6
    upper_violation = joint_positions > joint_limits[:, 1] + 1e-6
    if np.any(lower_violation | upper_violation):
        invalid = {
            name: {
                "position_rad": float(position),
                "limits_rad": [float(lower), float(upper)],
            }
            for name, position, (lower, upper), violated in zip(
                joint_names,
                joint_positions,
                joint_limits,
                lower_violation | upper_violation,
                strict=True,
            )
            if violated
        }
        raise RuntimeError(f"camera review pose exceeds joint limits: {invalid}")
    limit_margins = np.minimum(
        joint_positions - joint_limits[:, 0],
        joint_limits[:, 1] - joint_positions,
    )
    robot.set_joint_positions(joint_positions, joint_indices)
    robot.get_articulation_controller().apply_action(
        ArticulationAction(
            joint_positions=joint_positions,
            joint_indices=joint_indices,
        )
    )
    world.step(render=False)
    # The first step initializes Fabric and drive targets.  Re-apply the exact
    # review state so the evidence pose is deterministic rather than one
    # integration step along the controller trajectory.
    robot.set_joint_positions(joint_positions, joint_indices)
    observed = np.asarray(robot.get_joint_positions(), dtype=float)[joint_indices]
    maximum_error = float(np.max(np.abs(observed - joint_positions)))
    if maximum_error > 1e-5:
        raise RuntimeError(
            f"camera review pose error is {maximum_error:.6f} rad"
        )
    return world, {
        "name": "box_transfer_camera_review_seed",
        "joint_positions_rad": dict(BOX_TRANSFER_CAMERA_REVIEW_POSE_RAD),
        "minimum_joint_limit_margin_rad": float(np.min(limit_margins)),
        "maximum_set_position_error_rad": maximum_error,
    }


def _measure_d405_review_alignment(stage: object) -> dict[str, object]:
    """Measure task aim, roll, lens axis, and the distal task ray.

    The imported wrist/bracket convex hull encloses the camera origin, so the
    physics ray intentionally begins beyond that known hull.  It is not proof
    that the complete optical ray is unobstructed; successful RTX Tag 0 decode
    is the end-to-end visibility gate.
    """

    import carb
    from omni.physx import get_physx_scene_query_interface
    from pxr import Gf, Usd, UsdGeom

    xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    tag_prim = stage.GetPrimAtPath(
        "/WheelBotBoxTransfer/BlueTransportBox/AprilTag_0/Face"
    )
    tag_position = xform_cache.GetLocalToWorldTransform(tag_prim).Transform(
        Gf.Vec3d(0, 0, 0)
    )
    world_up = Gf.Vec3d(0, 0, 1)
    measurements: dict[str, dict[str, object]] = {}
    for name, camera_path in CAMERAS:
        if not name.endswith("wrist_d405"):
            continue
        camera_prim = stage.GetPrimAtPath(camera_path)
        housing_prim = camera_prim.GetParent().GetParent()
        camera_matrix = xform_cache.GetLocalToWorldTransform(camera_prim)
        housing_matrix = xform_cache.GetLocalToWorldTransform(housing_prim)
        camera_position = camera_matrix.Transform(Gf.Vec3d(0, 0, 0))
        forward = camera_matrix.TransformDir(Gf.Vec3d(0, 0, -1)).GetNormalized()
        image_up = camera_matrix.TransformDir(Gf.Vec3d(0, 1, 0)).GetNormalized()
        physical_lens_axis = housing_matrix.TransformDir(
            Gf.Vec3d(0, 0, -1)
        ).GetNormalized()
        to_tag = (tag_position - camera_position).GetNormalized()
        projected_world_up = (
            world_up - forward * (forward * world_up)
        ).GetNormalized()
        # Imported wrist-roll/yaw collision hulls enclose the 134 mm fixed
        # sensor bracket.  Start beyond that known local assembly so this ray
        # measures task-line occlusion instead of a sensor-origin overlap.
        distal_raycast_start_offset_m = 0.18
        ray_origin = camera_position + forward * distal_raycast_start_offset_m
        ray_to_tag = tag_position - ray_origin
        distance_to_tag = float(ray_to_tag.GetLength())
        ray_direction = ray_to_tag.GetNormalized()
        raycast_hit = get_physx_scene_query_interface().raycast_closest(
            carb.Float3(*[float(value) for value in ray_origin]),
            carb.Float3(*[float(value) for value in ray_direction]),
            distance_to_tag + 0.02,
            True,
        )
        first_hit_collision = (
            str(raycast_hit.get("collision", "")) if raycast_hit.get("hit") else ""
        )
        first_hit_distance_m = (
            float(raycast_hit.get("distance", 0.0)) if raycast_hit.get("hit") else None
        )
        distal_raycast_reaches_box = first_hit_collision.startswith(
            "/WheelBotBoxTransfer/BlueTransportBox/"
        )
        measurements[name] = {
            "camera_position_m": [float(value) for value in camera_position],
            "forward_world": [float(value) for value in forward],
            "image_up_world": [float(value) for value in image_up],
            "aim_to_apriltag_dot": float(forward * to_tag),
            "image_up_to_projected_gravity_dot": float(
                image_up * projected_world_up
            ),
            "forward_to_housing_minus_z_dot": float(
                forward * physical_lens_axis
            ),
            "distal_raycast_start_offset_m": distal_raycast_start_offset_m,
            "distal_raycast_first_hit": first_hit_collision,
            "distal_raycast_first_hit_distance_m": first_hit_distance_m,
            "distal_raycast_reaches_box_after_mount_hull": (
                distal_raycast_reaches_box
            ),
        }

    valid = all(
        float(values["aim_to_apriltag_dot"]) >= 0.85
        and float(values["image_up_to_projected_gravity_dot"]) >= 0.999
        and float(values["forward_to_housing_minus_z_dot"]) >= 0.999
        and bool(values["distal_raycast_reaches_box_after_mount_hull"])
        for values in measurements.values()
    )
    if not valid or len(measurements) != 2:
        raise RuntimeError(f"D405 task-pose alignment failed: {measurements}")
    return {"valid": True, "cameras": measurements}


def main() -> int:
    """Open the formal stage, render all review views, and write a manifest."""

    args = parse_args()
    scene = args.scene.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    if not scene.is_file():
        raise FileNotFoundError(f"formal scene does not exist: {scene}")
    if args.rt_subframes <= 0:
        raise ValueError("--rt-subframes must be positive")
    output_dir.mkdir(parents=True, exist_ok=True)

    # Kit also parses sys.argv.  Hide this script's flags until the application
    # is running so paths such as --output-dir are not treated as Kit options.
    script_argv = sys.argv
    sys.argv = [sys.argv[0]]
    world = None
    try:
        from isaacsim import SimulationApp

        app = SimulationApp(
            {
                "headless": args.headless,
                "width": 1600,
                "height": 900,
                "renderer": "RaytracedLighting",
                "fast_shutdown": True,
            }
        )
    finally:
        sys.argv = script_argv

    try:
        import omni.replicator.core as rep
        import omni.usd

        context = omni.usd.get_context()
        if not context.open_stage(str(scene)):
            raise RuntimeError(f"Isaac Sim could not open formal scene: {scene}")
        # Advance Kit while binary USD references and MDL textures finish
        # loading.  No timeline playback is used, so this remains a pose check.
        for _ in range(40):
            app.update()
        stage = context.get_stage()
        world, review_pose = _apply_robot_review_pose()
        app.update()
        d405_alignment = _measure_d405_review_alignment(stage)
        from pxr import UsdGeom

        stage_meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(stage))
        stage_up_axis = str(UsdGeom.GetStageUpAxis(stage))
        camera_runtime_contract: dict[str, dict[str, object]] = {}
        for camera_name, camera_path in CAMERAS:
            camera_prim = stage.GetPrimAtPath(camera_path)
            clipping = UsdGeom.Camera(camera_prim).GetClippingRangeAttr().Get()
            clipping_values = [float(value) for value in clipping]
            resolution = camera_prim.GetAttribute("render:resolution").Get()
            resolution_values = [int(value) for value in resolution]
            if (
                not np.allclose(clipping_values, [0.05, 100.0], atol=1e-6)
                or resolution_values != [1280, 720]
            ):
                raise RuntimeError(
                    f"{camera_name} runtime camera contract drifted: "
                    f"clipping={clipping_values}, resolution={resolution_values}"
                )
            camera_runtime_contract[camera_name] = {
                "prim_path": camera_path,
                "resolution": resolution_values,
                "clipping_range_m": clipping_values,
            }
        tag_path = (
            "/WheelBotBoxTransfer/BlueTransportBox/AprilTag_0/Face"
        )
        if not stage.GetPrimAtPath(tag_path).IsValid():
            raise RuntimeError(f"box-attached tag face is missing: {tag_path}")

        overview_camera = rep.create.camera(
            position=(-3.6, -4.6, 3.8),
            look_at=(1.45, 0.25, 0.8),
            focal_length=20.0,
            horizontal_aperture=20.0,
            clipping_range=(0.05, 1000.0),
        )
        closeup_camera = rep.create.camera(
            position=(0.32, 0.38, 1.10),
            look_at=(0.75, 0.38, 1.08),
            focal_length=28.0,
            horizontal_aperture=20.0,
            clipping_range=(0.01, 100.0),
        )

        view_specs: list[tuple[str, object, tuple[int, int]]] = [
            ("overview", overview_camera, (1600, 900)),
            ("apriltag_box_closeup", closeup_camera, (1280, 720)),
        ]
        view_specs.extend((name, path, (1280, 720)) for name, path in CAMERAS)

        annotators: list[object] = []
        for _name, camera, resolution in view_specs:
            render_product = rep.create.render_product(camera, resolution)
            annotator = rep.AnnotatorRegistry.get_annotator("rgb")
            annotator.attach([render_product])
            annotators.append(annotator)

        rep.orchestrator.step(rt_subframes=args.rt_subframes)

        manifest_views: list[dict[str, object]] = []
        saved: dict[str, Path] = {}
        for (name, _camera, resolution), annotator in zip(
            view_specs,
            annotators,
            strict=True,
        ):
            output_path = output_dir / f"{name}.png"
            pixel_metrics = _save_rgb(
                annotator.get_data(),
                output_path,
                resolution,
            )
            saved[name] = output_path
            manifest_views.append(
                {
                    "name": name,
                    "file": output_path.name,
                    "resolution": list(resolution),
                    "sha256": _sha256(output_path),
                    "pixel_metrics": pixel_metrics,
                }
            )

        # Decode both the independent close-up and the four real robot-mounted
        # camera frames.  A temporary review camera alone is not sufficient
        # evidence for the perception path used by the task.
        import cv2

        decoded_by_view = {
            name: _detect_apriltag_ids(saved[name], cv2)
            for name in ("apriltag_box_closeup", *(name for name, _path in CAMERAS))
        }
        if 0 not in decoded_by_view["apriltag_box_closeup"]:
            raise RuntimeError(
                "formal box close-up does not contain a decodable AprilTag 0; "
                f"decoded IDs: {decoded_by_view['apriltag_box_closeup']}"
            )
        robot_tag_views = [
            name for name, _path in CAMERAS if 0 in decoded_by_view[name]
        ]
        if not robot_tag_views:
            raise RuntimeError(
                "no robot-mounted camera decodes box-attached AprilTag 0; "
                f"decoded IDs by view: {decoded_by_view}"
            )
        d405_names = {
            name for name, _path in CAMERAS if name.endswith("wrist_d405")
        }
        d405_tag_views = d405_names.intersection(robot_tag_views)
        if d405_tag_views != d405_names:
            raise RuntimeError(
                "both D405 task-pose frames must decode box-attached AprilTag 0; "
                f"decoded IDs by view: {decoded_by_view}"
            )

        camera_paths = [
            (
                name.replace("_", " ").upper()
                + (" · TAG 0" if name in robot_tag_views else ""),
                saved[name],
            )
            for name, _path in CAMERAS
        ]
        four_sheet = output_dir / "four_camera_contact_sheet.png"
        _save_four_camera_sheet(camera_paths, four_sheet)
        review_board = output_dir / "workcell_review_board.png"
        _save_review_board(
            saved["overview"],
            saved["apriltag_box_closeup"],
            camera_paths,
            review_board,
        )

        source_hashes, source_bundle_hash = _project_source_hashes()
        for view in manifest_views:
            view_name = str(view["name"])
            if view_name in decoded_by_view:
                view["decoded_apriltag_ids"] = decoded_by_view[view_name]

        manifest = {
            "ok": True,
            "captured_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "renderer": {
                "isaac_sim_version": package_version("isaacsim"),
                "mode": app.config["renderer"],
                "rt_subframes": args.rt_subframes,
            },
            "git": _git_state(),
            "formal_scene": str(scene.relative_to(PROJECT_ROOT)),
            "formal_scene_sha256": _sha256(scene),
            "project_source_sha256": source_hashes,
            "project_source_bundle_sha256": source_bundle_hash,
            "stage_meters_per_unit": stage_meters_per_unit,
            "stage_up_axis": stage_up_axis,
            "simulation_time_s": float(world.current_time),
            "simulation_time_step_index": int(world.current_time_step_index),
            "apriltag": {
                "family": "tag36h11",
                "id": 0,
                "decoded_ids_in_box_closeup": decoded_by_view["apriltag_box_closeup"],
                "robot_camera_detection_required": True,
                "robot_cameras_decoding_tag_0": robot_tag_views,
                "size_m": 0.1,
                "parent_prim": "/WheelBotBoxTransfer/BlueTransportBox",
                "face_prim": tag_path,
            },
            "views": manifest_views,
            "camera_contract": {
                "required_cameras": [name for name, _path in CAMERAS],
                "all_four_rgb_frames_valid": True,
                "tag_0_detected_by_robot_camera": bool(robot_tag_views),
                "both_d405_cameras_decode_tag_0": True,
                "cameras": camera_runtime_contract,
            },
            "robot_review_pose": review_pose,
            "d405_alignment": d405_alignment,
            "review_board": review_board.name,
            "review_board_sha256": _sha256(review_board),
            "four_camera_contact_sheet": four_sheet.name,
            "four_camera_contact_sheet_sha256": _sha256(four_sheet),
        }
        (output_dir / "manifest.json").write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        sys.__stdout__.write(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
        sys.__stdout__.flush()
        return 0
    except Exception as error:
        # SimulationApp shutdown can suppress the normal Python traceback on
        # some Kit builds, so flush the actionable failure first.
        sys.__stderr__.write(
            f"capture_workcell_evidence failed: "
            f"{type(error).__name__}: {error}\n"
        )
        sys.__stderr__.flush()
        raise
    finally:
        if world is not None:
            world.stop()
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
