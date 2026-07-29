"""Deterministic synchronization checks for ROS 2 camera topic evidence."""

from types import SimpleNamespace

from franzi_sim.cameras import CAMERAS, CameraSpec
from scripts.verify_ros2_camera_topics import _SynchronizedCameraMessages


def _message(spec: CameraSpec, stamp: tuple[int, int], *, image: bool) -> SimpleNamespace:
    """Create a contract-valid Image or CameraInfo shaped test message."""

    header = SimpleNamespace(
        frame_id=spec.frame_id,
        stamp=SimpleNamespace(sec=stamp[0], nanosec=stamp[1]),
    )
    if image:
        return SimpleNamespace(
            header=header,
            width=spec.width_px,
            height=spec.height_px,
            encoding="rgb8",
            step=spec.width_px * 3,
            data=b"rgb",
        )
    return SimpleNamespace(
        header=header,
        width=spec.width_px,
        height=spec.height_px,
        distortion_model="plumb_bob",
        k=[1.0] * 9,
        d=[0.0] * 5,
        r=[1.0] * 9,
        p=[1.0] * 12,
    )


def test_out_of_order_frames_wait_for_an_exact_timestamp_match() -> None:
    """Older unmatched packets cannot be paired with newer packets on either topic."""

    spec = CAMERAS[0]
    collector = _SynchronizedCameraMessages()
    collector.record_image(_message(spec, (3, 10), image=True), spec)
    collector.record_camera_info(_message(spec, (3, 20), image=False), spec)
    collector.record_camera_info(_message(spec, (3, 10), image=False), spec)

    assert set(collector.received) == {spec.topic, spec.camera_info_topic}
    assert collector.matched_pairs[spec.name]["stamp"] == {"sec": 3, "nanosec": 10}
    assert collector.received[spec.topic]["stamp"] == collector.received[spec.camera_info_topic]["stamp"]


def test_unmatched_messages_never_count_as_received_topics() -> None:
    """A camera with no identical timestamps remains incomplete at timeout."""

    spec = CAMERAS[1]
    collector = _SynchronizedCameraMessages()
    collector.record_image(_message(spec, (4, 1), image=True), spec)
    collector.record_camera_info(_message(spec, (4, 2), image=False), spec)

    assert collector.received == {}
    assert collector.matched_pairs == {}
    assert collector.observed_topics == {spec.topic, spec.camera_info_topic}


def test_exact_pairs_are_recorded_for_every_camera() -> None:
    """Every declared camera contributes both endpoints only after an exact pair."""

    collector = _SynchronizedCameraMessages()
    for index, spec in enumerate(CAMERAS):
        stamp = (5, index)
        collector.record_camera_info(_message(spec, stamp, image=False), spec)
        collector.record_image(_message(spec, stamp, image=True), spec)

    assert len(collector.received) == 2 * len(CAMERAS)
    assert set(collector.matched_pairs) == {spec.name for spec in CAMERAS}
    for spec in CAMERAS:
        assert collector.received[spec.topic]["stamp"] == collector.received[spec.camera_info_topic]["stamp"]
