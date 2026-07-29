"""Static ROS TF definitions for the four USD camera optical frames.

The transforms are intentionally relative to the existing URDF housing links.
Robot-state publishing remains responsible for the moving link chain; this
module only fills the fixed housing-to-optical-frame edges on ``/tf_static``.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, radians, sin, sqrt
from typing import Callable

from franzi_sim.cameras import CAMERAS, CameraSpec, validate_camera_specs


@dataclass(frozen=True)
class StaticTransformSpec:
    """Describe one fixed parent-link to ROS optical-frame transform."""

    parent_frame: str
    child_frame: str
    translation_m: tuple[float, float, float]
    rotation_rpy_deg: tuple[float, float, float]

    @property
    def rotation_xyzw(self) -> tuple[float, float, float, float]:
        """Convert the USD-authored XYZ Euler rotation into a ROS quaternion."""

        roll, pitch, yaw = (radians(value) for value in self.rotation_rpy_deg)
        cr, sr = cos(roll / 2), sin(roll / 2)
        cp, sp = cos(pitch / 2), sin(pitch / 2)
        cy, sy = cos(yaw / 2), sin(yaw / 2)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )


def optical_frame_transforms(cameras: tuple[CameraSpec, ...] = CAMERAS) -> tuple[StaticTransformSpec, ...]:
    """Return the fixed TF edges that exactly mirror the camera USD layer."""

    validate_camera_specs(cameras)
    return tuple(
        StaticTransformSpec(
            parent_frame=camera.parent_link,
            child_frame=camera.frame_id,
            translation_m=camera.mount_xyz_m,
            rotation_rpy_deg=camera.mount_rpy_deg,
        )
        for camera in cameras
    )


def validate_optical_frame_transforms(
    transforms: tuple[StaticTransformSpec, ...] | None = None,
) -> None:
    """Reject incomplete, duplicate, or non-normalized static TF definitions."""

    if transforms is None:
        transforms = optical_frame_transforms()
    if len(transforms) != len(CAMERAS):
        raise ValueError("every declared camera needs exactly one optical-frame TF edge")
    parents = {transform.parent_frame for transform in transforms}
    children = {transform.child_frame for transform in transforms}
    if len(parents) != len(transforms) or len(children) != len(transforms):
        raise ValueError("optical-frame TF parent and child names must be unique")
    for transform in transforms:
        if not transform.parent_frame or not transform.child_frame.endswith("_optical_frame"):
            raise ValueError("optical-frame TF edge has an invalid parent or child name")
        if len(transform.translation_m) != 3 or len(transform.rotation_rpy_deg) != 3:
            raise ValueError("optical-frame TF transforms must be three-dimensional")
        quaternion = transform.rotation_xyzw
        if abs(sqrt(sum(component * component for component in quaternion)) - 1.0) > 1e-9:
            raise ValueError("optical-frame TF quaternion is not normalized")


def publish_optical_frame_static_transforms(
    node: object,
    *,
    broadcaster_factory: Callable[[object], object] | None = None,
    transform_factory: Callable[[], object] | None = None,
) -> object:
    """Publish all four fixed edges with ROS 2's transient-local static broadcaster.

    Local imports keep this module usable by offline tests.  Factory injection
    lets those tests check the published graph without importing ROS messages.
    """

    validate_optical_frame_transforms()
    if broadcaster_factory is None or transform_factory is None:
        from geometry_msgs.msg import TransformStamped
        from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

        broadcaster_factory = broadcaster_factory or StaticTransformBroadcaster
        transform_factory = transform_factory or TransformStamped

    messages = []
    stamp = node.get_clock().now().to_msg()
    for transform in optical_frame_transforms():
        message = transform_factory()
        message.header.stamp = stamp
        message.header.frame_id = transform.parent_frame
        message.child_frame_id = transform.child_frame
        message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = transform.translation_m
        (
            message.transform.rotation.x,
            message.transform.rotation.y,
            message.transform.rotation.z,
            message.transform.rotation.w,
        ) = transform.rotation_xyzw
        messages.append(message)

    broadcaster = broadcaster_factory(node)
    broadcaster.sendTransform(messages)
    return broadcaster
