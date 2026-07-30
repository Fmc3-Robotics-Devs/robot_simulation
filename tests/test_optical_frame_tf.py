"""Offline tests for the static camera optical-frame TF publisher contract."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from franzi_sim.cameras import CAMERAS
from franzi_sim.optical_frame_tf import (
    StaticTransformSpec,
    optical_frame_transforms,
    publish_optical_frame_static_transforms,
    validate_optical_frame_transforms,
)


def _message() -> SimpleNamespace:
    return SimpleNamespace(
        header=SimpleNamespace(stamp=None, frame_id=""),
        child_frame_id="",
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=None, y=None, z=None),
            rotation=SimpleNamespace(x=None, y=None, z=None, w=None),
        ),
    )


def test_static_optical_frames_use_ros_axes_not_usd_camera_axes() -> None:
    """Each CameraInfo frame gets the separate ROS-convention fixed edge."""

    validate_optical_frame_transforms()
    transforms = optical_frame_transforms()
    assert [(item.parent_frame, item.child_frame) for item in transforms] == [
        (camera.parent_link, camera.frame_id) for camera in CAMERAS
    ]
    assert [item.translation_m for item in transforms] == [camera.mount_xyz_m for camera in CAMERAS]
    assert [item.rotation_rpy_deg for item in transforms] == [
        camera.optical_frame_rpy_deg for camera in CAMERAS
    ]
    assert [item.rotation_rpy_deg for item in transforms] != [
        camera.camera_mount_rpy_deg for camera in CAMERAS
    ]
    for item in transforms:
        assert sum(component * component for component in item.rotation_xyzw) == pytest.approx(1.0)


def test_static_publisher_emits_all_edges_without_a_ros_runtime() -> None:
    """Factory injection verifies the production publisher's ROS message contract."""

    class FakeBroadcaster:
        def __init__(self, node: object) -> None:
            self.node = node
            self.messages: list[SimpleNamespace] = []

        def sendTransform(self, messages: list[SimpleNamespace]) -> None:
            self.messages = messages

    node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: "stamp")))
    broadcaster = publish_optical_frame_static_transforms(
        node,
        broadcaster_factory=FakeBroadcaster,
        transform_factory=_message,
    )
    assert broadcaster.node is node
    assert [message.header.frame_id for message in broadcaster.messages] == [camera.parent_link for camera in CAMERAS]
    assert [message.child_frame_id for message in broadcaster.messages] == [camera.frame_id for camera in CAMERAS]
    assert all(message.header.stamp == "stamp" for message in broadcaster.messages)


def test_static_tf_validation_rejects_an_incomplete_graph() -> None:
    """An empty supplied graph must not silently fall back to default transforms."""

    with pytest.raises(ValueError, match="every declared camera"):
        validate_optical_frame_transforms(())

    duplicate_child = StaticTransformSpec("a", "camera_optical_frame", (0, 0, 0), (0, 0, 0))
    with pytest.raises(ValueError, match="unique"):
        validate_optical_frame_transforms((duplicate_child,) * 4)
