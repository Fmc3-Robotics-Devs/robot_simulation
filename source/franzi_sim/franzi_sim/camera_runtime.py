"""Small adapter for creating the declared sensors in a loaded Isaac USD stage."""

from __future__ import annotations

from franzi_sim.cameras import CAMERAS, CameraSpec, validate_camera_specs


def add_camera_sensors(stage: object, robot_prim_path: str) -> None:
    """Add four USD cameras below existing URDF camera housing links.

    Imports are deliberately local: static contract tests run without Isaac Sim,
    while a runtime caller gets the normal pxr API when Isaac is installed.
    """

    from pxr import Gf, Sdf, UsdGeom

    validate_camera_specs()
    for spec in CAMERAS:
        camera_path = f"{robot_prim_path}/{spec.parent_link}/{spec.sensor_prim}/camera"
        if stage.GetPrimAtPath(camera_path).IsValid():
            # The complete robot asset already authors these sensors; avoid
            # duplicate xform ops when the runtime helper is called defensively.
            continue
        _add_one_camera(stage, robot_prim_path, spec, Gf, Sdf, UsdGeom)


def _add_one_camera(
    stage: object,
    robot_prim_path: str,
    spec: CameraSpec,
    gf: object,
    sdf: object,
    usd_geom: object,
) -> None:
    """Create one optical-frame transform and camera below its physical housing link."""

    parent_path = f"{robot_prim_path}/{spec.parent_link}"
    if not stage.GetPrimAtPath(parent_path).IsValid():
        raise ValueError(f"URDF camera housing link is absent: {parent_path}")
    frame = usd_geom.Xform.Define(stage, f"{parent_path}/{spec.sensor_prim}")
    frame.AddTranslateOp().Set(gf.Vec3d(*spec.mount_xyz_m))
    frame.AddRotateXYZOp().Set(gf.Vec3f(*spec.mount_rpy_deg))
    camera = usd_geom.Camera.Define(stage, f"{frame.GetPath()}/camera")
    camera.CreateFocalLengthAttr(spec.focal_length_mm)
    camera.CreateHorizontalApertureAttr(spec.horizontal_aperture_mm)
    camera.CreateVerticalApertureAttr(spec.horizontal_aperture_mm * spec.height_px / spec.width_px)
    # USD's default near plane is 1 m, which clips the table and box at the
    # Wheel Bot's normal manipulation distance.  Keep the explicit range in
    # the shared specification so static USD and runtime-authored cameras agree.
    camera.CreateClippingRangeAttr(gf.Vec2f(*spec.clipping_range_m))
    # Keep runtime-authored metadata identical to camera_sensors.usda.  Resolution
    # is custom because UsdGeomCamera's aperture is optical, not raster output.
    prim = camera.GetPrim()
    prim.CreateAttribute("ros:frame_id", sdf.ValueTypeNames.String, custom=True).Set(spec.frame_id)
    prim.CreateAttribute("ros:image_topic", sdf.ValueTypeNames.String, custom=True).Set(spec.topic)
    prim.CreateAttribute(
        "ros:camera_info_topic",
        sdf.ValueTypeNames.String,
        custom=True,
    ).Set(spec.camera_info_topic)
    prim.CreateAttribute(
        "render:resolution",
        sdf.ValueTypeNames.IntArray,
        custom=True,
    ).Set([spec.width_px, spec.height_px])
