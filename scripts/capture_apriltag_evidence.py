#!/usr/bin/env python3
"""Render Isaac Sim 5.1's official AprilTag material for visual review."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TAG_USD = ROOT / "usd/assets/tags/apriltag_36h11.usda"


def parse_args() -> argparse.Namespace:
    """Parse the project tag asset and evidence destination."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tag-usd", type=Path, default=DEFAULT_TAG_USD)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args()


def _add_tag(
    stage: object,
    *,
    prim_path: str,
    tag_usd: Path,
    tag_id: int,
    y_m: float,
) -> None:
    """Reference one board, place it front-facing, and select its mosaic ID."""

    from pxr import Gf, Sdf, UsdGeom

    board = UsdGeom.Xform.Define(stage, prim_path)
    board.GetPrim().GetReferences().AddReference(str(tag_usd.resolve()))
    board.AddTranslateOp().Set(Gf.Vec3d(1.0, y_m, 0.5))
    board.AddRotateYOp().Set(-90.0)
    shader = stage.OverridePrim(f"{prim_path}/AprilTagMaterial/Shader")
    shader.CreateAttribute("inputs:tag_id", Sdf.ValueTypeNames.Int).Set(tag_id)


def _sha256(path: Path) -> str:
    """Return the content hash of one evidence file."""

    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    """Create a two-tag calibration view and save image plus manifest."""

    args = parse_args()
    tag_usd = args.tag_usd.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    if not tag_usd.is_file():
        raise FileNotFoundError(f"AprilTag USD does not exist: {tag_usd}")
    output_dir.mkdir(parents=True, exist_ok=True)

    from isaacsim import SimulationApp

    script_argv = sys.argv
    sys.argv = [sys.argv[0]]
    try:
        app = SimulationApp(
            {
                "headless": args.headless,
                "width": 1280,
                "height": 720,
                "fast_shutdown": True,
            }
        )
    finally:
        sys.argv = script_argv

    try:
        import omni.replicator.core as rep
        import omni.usd
        from pxr import Gf, UsdGeom, UsdLux

        context = omni.usd.get_context()
        context.new_stage()
        stage = context.get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, "/World")

        # White wall and neutral light make missing textures or reversed board
        # normals obvious in the saved image.
        wall = UsdGeom.Cube.Define(stage, "/World/Backdrop")
        wall.CreateDisplayColorAttr([Gf.Vec3f(0.65, 0.67, 0.70)])
        wall_xform = UsdGeom.Xformable(wall)
        wall_xform.AddTranslateOp().Set(Gf.Vec3d(1.03, 0.0, 0.5))
        wall_xform.AddScaleOp().Set(Gf.Vec3f(0.01, 0.45, 0.32))

        _add_tag(stage, prim_path="/World/Tag0", tag_usd=tag_usd, tag_id=0, y_m=0.075)
        _add_tag(stage, prim_path="/World/Tag1", tag_usd=tag_usd, tag_id=1, y_m=-0.075)

        distant = UsdLux.DistantLight.Define(stage, "/World/KeyLight")
        distant.CreateIntensityAttr(2500.0)
        fill = UsdLux.SphereLight.Define(stage, "/World/FrontFill")
        fill.CreateIntensityAttr(50000.0)
        fill.CreateRadiusAttr(0.15)
        UsdGeom.Xformable(fill).AddTranslateOp().Set(Gf.Vec3d(0.55, 0.0, 0.8))
        dome = UsdLux.DomeLight.Define(stage, "/World/AmbientFill")
        dome.CreateIntensityAttr(1200.0)
        camera = rep.create.camera(
            position=(0.45, 0.0, 0.5),
            look_at=(1.0, 0.0, 0.5),
            focal_length=24.0,
            clipping_range=(0.01, 100.0),
        )
        render_product = rep.create.render_product(camera, (1280, 720))
        annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        annotator.attach([render_product])
        # Let MDL compilation, the mosaic texture, and the new stage finish
        # hydrating before requesting the evidence frame.
        for _ in range(20):
            app.update()
        bounds_cache = UsdGeom.BBoxCache(
            0.0,
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
        )
        tag_bounds = {}
        for tag_name in ("Tag0", "Tag1"):
            aligned = bounds_cache.ComputeWorldBound(
                stage.GetPrimAtPath(f"/World/{tag_name}")
            ).ComputeAlignedBox()
            tag_bounds[tag_name] = {
                "minimum": list(aligned.GetMin()),
                "maximum": list(aligned.GetMax()),
            }
        sys.__stdout__.write(
            json.dumps({"tag_bounds": tag_bounds}, ensure_ascii=False) + "\n"
        )
        sys.__stdout__.flush()
        rep.orchestrator.step(rt_subframes=32)
        rgba = np.asarray(annotator.get_data())
        if rgba.shape[:2] != (720, 1280):
            raise RuntimeError(f"AprilTag render returned unexpected shape {rgba.shape}")
        rgb = rgba[:, :, :3].astype(np.uint8)
        pixel_metrics = {
            "minimum": int(rgb.min()),
            "maximum": int(rgb.max()),
            "mean": round(float(rgb.mean()), 3),
            "standard_deviation": round(float(rgb.std()), 3),
        }
        # Flush the metrics before the acceptance gate.  Kit may terminate
        # during renderer shutdown after a failed capture, so this diagnostic
        # must not depend on Python's normal exception reporting.
        sys.__stdout__.write(
            json.dumps({"pixel_metrics": pixel_metrics}, ensure_ascii=False) + "\n"
        )
        sys.__stdout__.flush()
        if (
            pixel_metrics["maximum"] - pixel_metrics["minimum"] < 24
            or pixel_metrics["standard_deviation"] < 4.0
            or pixel_metrics["mean"] < 2.0
        ):
            Image.fromarray(rgb).save(output_dir / "rejected_blank_frame.png")
            raise RuntimeError(f"AprilTag calibration image is blank: {pixel_metrics}")

        image_path = output_dir / "tag36h11_ids_0_1.png"
        Image.fromarray(rgb).save(image_path)
        # Decode the rendered pixels, not merely the authored USD metadata.
        # OpenCV's AprilTag dictionary gives us a deterministic visual gate
        # that catches missing textures, reversed faces, and incorrect IDs.
        import cv2

        detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
            cv2.aruco.DetectorParameters(),
        )
        _corners, detected_ids, _rejected = detector.detectMarkers(rgb)
        decoded_ids = (
            sorted(int(value) for value in detected_ids.reshape(-1))
            if detected_ids is not None
            else []
        )
        if decoded_ids != [0, 1]:
            raise RuntimeError(
                f"rendered AprilTag IDs did not decode as [0, 1]: {decoded_ids}"
            )
        manifest = {
            "ok": True,
            "tag_family": "tag36h11",
            "tag_ids": [0, 1],
            "decoded_tag_ids": decoded_ids,
            "tag_size_m": 0.1,
            "tag_usd": str(tag_usd.relative_to(ROOT)),
            "image": image_path.name,
            "image_sha256": _sha256(image_path),
            "resolution": [1280, 720],
            "renderer": "Isaac Sim 5.1 RTX",
            "pixel_metrics": pixel_metrics,
        }
        (output_dir / "manifest.json").write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        sys.__stdout__.write(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
        sys.__stdout__.flush()
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
