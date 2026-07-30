"""Central, runtime-neutral configuration for Wheel Bot's four RGB cameras.

The existing robot URDF already provides the physical D435/D405 housing links and
their fixed joints.  These specifications add only Isaac camera sensors below
those links, so the visual mesh is never instantiated a second time.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class CameraSpec:
    """Describe one camera sensor mounted on an existing Wheel Bot URDF link."""

    name: str
    model: str
    parent_link: str
    camera_mount_prim: str
    optical_frame_prim: str
    frame_id: str
    topic: str
    camera_info_topic: str
    width_px: int
    height_px: int
    focal_length_mm: float
    horizontal_aperture_mm: float
    clipping_range_m: tuple[float, float]
    mount_xyz_m: tuple[float, float, float]
    camera_mount_rpy_deg: tuple[float, float, float]
    optical_frame_rpy_deg: tuple[float, float, float]

    @property
    def is_left_wrist(self) -> bool:
        """Return whether this specification belongs to the robot's left wrist."""

        return self.name == "left_wrist_d405"

    @property
    def is_right_wrist(self) -> bool:
        """Return whether this specification belongs to the robot's right wrist."""

        return self.name == "right_wrist_d405"


# USD cameras look along local -Z with local +Y as image-up.  ROS optical
# frames use +Z forward and +Y down, so they are deliberately separate sibling
# transforms: R(housing, optical) = R(housing, USD camera) * Rx(180 deg).
#
# Both D405 STL files place their two lenses on the housing local -Z face at
# z=-23 mm.  The current single RGB sensor is an explicitly virtual stereo
# midpoint at x=y=0, sitting 0.5 mm outside that face; it is not claimed as a
# calibrated physical RGB-lens centre.  The USD mount rotation is identity, so
# rays stay out of the mesh and follow its physical lens face instead of +X.
CAMERAS: tuple[CameraSpec, ...] = (
    CameraSpec(
        name="head_d435",
        model="Intel RealSense D435",
        parent_link="head_d435_Link",
        camera_mount_prim="head_d435_camera_mount",
        optical_frame_prim="head_d435_optical_frame",
        frame_id="head_d435_optical_frame",
        topic="/franzi/camera/head_d435/color/image_raw",
        camera_info_topic="/franzi/camera/head_d435/color/camera_info",
        width_px=1280,
        height_px=720,
        focal_length_mm=3.0,
        horizontal_aperture_mm=4.8,
        clipping_range_m=(0.05, 100.0),
        mount_xyz_m=(0.0, 0.0, 0.0),
        camera_mount_rpy_deg=(180.0, 0.0, -90.0),
        optical_frame_rpy_deg=(0.0, 0.0, -90.0),
    ),
    CameraSpec(
        name="chest_d435",
        model="Intel RealSense D435",
        parent_link="body_d435_Link",
        camera_mount_prim="chest_d435_camera_mount",
        optical_frame_prim="chest_d435_optical_frame",
        frame_id="chest_d435_optical_frame",
        topic="/franzi/camera/chest_d435/color/image_raw",
        camera_info_topic="/franzi/camera/chest_d435/color/camera_info",
        width_px=1280,
        height_px=720,
        focal_length_mm=3.0,
        horizontal_aperture_mm=4.8,
        clipping_range_m=(0.05, 100.0),
        mount_xyz_m=(0.0, 0.0, 0.0),
        camera_mount_rpy_deg=(180.0, 0.0, -90.0),
        optical_frame_rpy_deg=(0.0, 0.0, -90.0),
    ),
    CameraSpec(
        name="left_wrist_d405",
        model="Intel RealSense D405",
        parent_link="left_wrist_d405_Link",
        camera_mount_prim="left_wrist_d405_camera_mount",
        optical_frame_prim="left_wrist_d405_optical_frame",
        frame_id="left_wrist_d405_optical_frame",
        topic="/franzi/camera/left_wrist_d405/color/image_raw",
        camera_info_topic="/franzi/camera/left_wrist_d405/color/camera_info",
        width_px=1280,
        height_px=720,
        focal_length_mm=2.8,
        horizontal_aperture_mm=4.8,
        clipping_range_m=(0.05, 100.0),
        mount_xyz_m=(0.0, 0.0, -0.0235),
        camera_mount_rpy_deg=(0.0, 0.0, 0.0),
        optical_frame_rpy_deg=(180.0, 0.0, 0.0),
    ),
    CameraSpec(
        name="right_wrist_d405",
        model="Intel RealSense D405",
        parent_link="right_wrist_d405_Link",
        camera_mount_prim="right_wrist_d405_camera_mount",
        optical_frame_prim="right_wrist_d405_optical_frame",
        frame_id="right_wrist_d405_optical_frame",
        topic="/franzi/camera/right_wrist_d405/color/image_raw",
        camera_info_topic="/franzi/camera/right_wrist_d405/color/camera_info",
        width_px=1280,
        height_px=720,
        focal_length_mm=2.8,
        horizontal_aperture_mm=4.8,
        clipping_range_m=(0.05, 100.0),
        mount_xyz_m=(0.0, 0.0, -0.0235),
        camera_mount_rpy_deg=(0.0, 0.0, 0.0),
        optical_frame_rpy_deg=(180.0, 0.0, 0.0),
    ),
)


def validate_camera_specs(cameras: tuple[CameraSpec, ...] = CAMERAS) -> None:
    """Raise ``ValueError`` when a sensor mapping is incomplete or ambiguous."""

    expected_names = {"head_d435", "chest_d435", "left_wrist_d405", "right_wrist_d405"}
    if {camera.name for camera in cameras} != expected_names or len(cameras) != 4:
        raise ValueError("exactly head, chest, left-wrist, and right-wrist cameras are required")
    for camera in cameras:
        if (
            not camera.parent_link
            or not camera.camera_mount_prim.endswith("_camera_mount")
            or camera.optical_frame_prim != camera.frame_id
            or not camera.frame_id
            or not camera.topic.startswith("/franzi/")
            or not camera.camera_info_topic.startswith("/franzi/")
        ):
            raise ValueError(f"{camera.name} is missing a parent link, frame, or ROS topic")
        if camera.width_px <= 0 or camera.height_px <= 0 or camera.focal_length_mm <= 0:
            raise ValueError(f"{camera.name} has invalid image dimensions or focal length")
        near_clip_m, far_clip_m = camera.clipping_range_m
        if near_clip_m <= 0 or far_clip_m <= near_clip_m:
            raise ValueError(f"{camera.name} has an invalid clipping range")
    left, right = cameras[2], cameras[3]
    if not (left.is_left_wrist and right.is_right_wrist):
        raise ValueError("wrist camera order must remain left then right")
    if "left" not in left.parent_link or "right" not in right.parent_link:
        raise ValueError("wrist camera parent links are side-inconsistent")


def camera_by_name(name: str) -> CameraSpec:
    """Return one camera configuration by its stable logical name."""

    for camera in CAMERAS:
        if camera.name == name:
            return camera
    raise KeyError(f"unknown camera: {name}")
