#!/usr/bin/env python3
"""Render the four Wheel Bot RGB cameras and save review evidence.

Run this script with the project's Isaac Sim Python environment.  It creates a
small calibration scene around the imported robot, captures every declared
camera, and writes both the images and a machine-readable manifest.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
from datetime import datetime
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from isaacsim import SimulationApp


def parse_args() -> argparse.Namespace:
    """Parse paths without importing Kit modules before SimulationApp starts."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-usd", type=Path, required=True, help="Imported Wheel Bot USD")
    parser.add_argument("--output-dir", type=Path, required=True, help="Evidence output directory")
    parser.add_argument("--headless", action="store_true", help="Run without an Isaac Sim window")
    parser.add_argument("--with-ros2", action="store_true", help="Author and tick the ROS 2 camera publisher graph")
    parser.add_argument(
        "--ros2-frames",
        type=int,
        default=30,
        help="Simulation updates to publish when --with-ros2 is enabled",
    )
    return parser.parse_args()


def _add_box(stage: object, path: str, size: tuple[float, float, float], position: tuple[float, float, float],
             color: tuple[float, float, float], usd_geom: object, gf: object) -> None:
    """Add one colored calibration object using only core USD schemas."""

    cube = usd_geom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([gf.Vec3f(*color)])
    xform = usd_geom.Xformable(cube)
    xform.AddTranslateOp().Set(gf.Vec3d(*position))
    xform.AddScaleOp().Set(gf.Vec3f(*size))


def _build_calibration_scene(stage: object, robot_reference: str) -> None:
    """Reference the robot and create recognizable floor, box, and target geometry."""

    from pxr import Gf, UsdGeom, UsdLux

    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    robot = stage.DefinePrim("/World/WheelBot", "Xform")
    robot.GetReferences().AddReference(robot_reference)

    # The colored objects make incorrect optical axes obvious in a single frame.
    _add_box(stage, "/World/Floor", (5.0, 5.0, 0.05), (0.0, 0.0, -0.05), (0.18, 0.21, 0.24), UsdGeom, Gf)
    _add_box(stage, "/World/BlueBox", (0.32, 0.46, 0.24), (1.15, 0.0, 0.38), (0.02, 0.24, 0.72), UsdGeom, Gf)
    _add_box(stage, "/World/LeftMarker", (0.08, 0.08, 0.60), (1.45, 0.70, 0.60), (0.05, 0.75, 0.24), UsdGeom, Gf)
    _add_box(stage, "/World/RightMarker", (0.08, 0.08, 0.60), (1.45, -0.70, 0.60), (0.85, 0.08, 0.06), UsdGeom, Gf)
    _add_box(stage, "/World/FrontWall", (0.05, 1.8, 1.2), (2.2, 0.0, 1.15), (0.55, 0.58, 0.62), UsdGeom, Gf)

    distant = UsdLux.DistantLight.Define(stage, "/World/KeyLight")
    distant.CreateIntensityAttr(3000.0)
    distant.CreateAngleAttr(1.0)
    sphere = UsdLux.SphereLight.Define(stage, "/World/FillLight")
    sphere.CreateIntensityAttr(45000.0)
    sphere.CreateRadiusAttr(0.4)
    UsdGeom.Xformable(sphere).AddTranslateOp().Set(Gf.Vec3d(0.5, -1.5, 3.2))


def _git_state(project_root: Path) -> tuple[str, bool]:
    """Return the current commit and whether evidence came from a dirty tree."""

    revision = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=project_root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    dirty = bool(
        subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=project_root,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    )
    return revision, dirty


def _sha256(path: Path) -> str:
    """Return a content hash for the exact robot entry used by the capture."""

    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _save_contact_sheet(images: list[tuple[str, Path]], output_path: Path) -> None:
    """Write a labeled 2×2 review sheet from the four captured RGB images."""

    cell_width, cell_height, label_height, gap = 640, 360, 48, 12
    canvas = Image.new(
        "RGB",
        (2 * cell_width + 3 * gap, 2 * (cell_height + label_height) + 3 * gap),
        (17, 17, 17),
    )
    try:
        font = ImageFont.truetype("DejaVuSans-Oblique.ttf", 26)
    except OSError:
        font = ImageFont.load_default()
    draw = ImageDraw.Draw(canvas)
    for index, (label, image_path) in enumerate(images):
        row, column = divmod(index, 2)
        left = gap + column * (cell_width + gap)
        top = gap + row * (cell_height + label_height + gap)
        image = Image.open(image_path).convert("RGB")
        image.thumbnail((cell_width, cell_height), Image.Resampling.LANCZOS)
        image_left = left + (cell_width - image.width) // 2
        image_top = top + (cell_height - image.height) // 2
        canvas.paste(image, (image_left, image_top))
        label_box = draw.textbbox((0, 0), label, font=font)
        label_width = label_box[2] - label_box[0]
        draw.text(
            (left + (cell_width - label_width) / 2, top + cell_height + 6),
            label,
            font=font,
            fill="white",
        )
    canvas.save(output_path)


def main() -> int:
    """Launch Isaac Sim, render four RGB frames, and write the evidence manifest."""

    args = parse_args()
    app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})

    import omni.replicator.core as rep
    import omni.timeline
    import omni.usd
    from isaacsim.core.utils.extensions import enable_extension
    from pxr import Usd

    from franzi_sim.camera_runtime import add_camera_sensors
    from franzi_sim.cameras import CAMERAS
    from franzi_sim.ros2_camera_bridge import add_ros2_camera_publishers

    project_root = Path(__file__).resolve().parents[1]
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    robot_usd = args.robot_usd.expanduser().resolve()
    if args.ros2_frames <= 0:
        raise ValueError("--ros2-frames must be positive")

    try:
        context = omni.usd.get_context()
        context.new_stage()
        stage = context.get_stage()
        # The exported evidence stage must remain openable after the repository
        # moves, so author the robot reference relative to the evidence folder.
        robot_reference = Path(os.path.relpath(robot_usd, output_dir)).as_posix()
        _build_calibration_scene(stage, robot_reference)
        add_camera_sensors(stage, "/World/WheelBot")
        if args.with_ros2:
            enable_extension("isaacsim.ros2.bridge")
            app.update()
            add_ros2_camera_publishers()

        render_products: list[object] = []
        annotators: list[object] = []
        for camera in CAMERAS:
            camera_path = (
                f"/World/WheelBot/{camera.parent_link}/"
                f"{camera.camera_mount_prim}/camera"
            )
            render_product = rep.create.render_product(camera_path, (camera.width_px, camera.height_px))
            annotator = rep.AnnotatorRegistry.get_annotator("rgb")
            annotator.attach([render_product])
            render_products.append(render_product)
            annotators.append(annotator)

        # Multiple subframes let RTX finish materials and referenced meshes before capture.
        # Keep this visual snapshot before timeline playback: the imported mobile
        # articulation is intentionally not constrained in this calibration-only
        # stage, while ROS publishing below needs a running timeline.
        rep.orchestrator.step(rt_subframes=16)
        rgb_snapshots: list[np.ndarray] = []
        for camera, annotator in zip(CAMERAS, annotators, strict=True):
            rgba = np.asarray(annotator.get_data())
            if rgba.shape[:2] != (camera.height_px, camera.width_px):
                raise RuntimeError(f"{camera.name} returned unexpected image shape {rgba.shape}")
            rgb_snapshots.append(rgba[:, :, :3].astype(np.uint8).copy())

        if args.with_ros2:
            timeline = omni.timeline.get_timeline_interface()
            timeline.play()
            for _ in range(args.ros2_frames):
                app.update()
            simulation_time_s = timeline.get_current_time()
            timeline.stop()
        else:
            simulation_time_s = 0.0
        manifest_cameras: list[dict[str, object]] = []
        contact_sheet_images: list[tuple[str, Path]] = []
        for camera, rgb in zip(CAMERAS, rgb_snapshots, strict=True):
            image_path = output_dir / f"{camera.name}.png"
            Image.fromarray(rgb).save(image_path)
            contact_sheet_images.append((camera.name.replace("_", " ").upper(), image_path))
            manifest_cameras.append(
                {
                    "name": camera.name,
                    "model": camera.model,
                    "parent_link": camera.parent_link,
                    "prim_path": (
                        f"/World/WheelBot/{camera.parent_link}/"
                        f"{camera.camera_mount_prim}/camera"
                    ),
                    "frame_id": camera.frame_id,
                    "topic": camera.topic,
                    "camera_info_topic": camera.camera_info_topic,
                    "resolution": [camera.width_px, camera.height_px],
                    "file": image_path.name,
                }
            )

        _save_contact_sheet(contact_sheet_images, output_dir / "four_camera_contact_sheet.png")
        stage_path = output_dir / "camera_validation.usda"
        stage.GetRootLayer().Export(str(stage_path))
        git_revision, git_dirty = _git_state(project_root)
        manifest = {
            "captured_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "git_revision": git_revision,
            "git_dirty": git_dirty,
            "robot_usd": str(robot_usd.relative_to(project_root)),
            "robot_usd_sha256": _sha256(robot_usd),
            "stage": stage_path.name,
            "ros2_graph_authored": args.with_ros2,
            "ros2_message_validation": "not_run",
            "ros2_frames": args.ros2_frames if args.with_ros2 else 0,
            "simulation_time_s": simulation_time_s,
            "cameras": manifest_cameras,
        }
        (output_dir / "manifest.json").write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
