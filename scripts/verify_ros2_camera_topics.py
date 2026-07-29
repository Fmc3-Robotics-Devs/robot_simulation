#!/usr/bin/env python3
"""Verify all Wheel Bot RGB and CameraInfo topics from a system ROS 2 terminal."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "source" / "franzi_sim"))

from franzi_sim.cameras import CAMERAS, CameraSpec  # noqa: E402


def parse_args() -> argparse.Namespace:
    """Parse the timeout and evidence output path."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=45.0, help="Maximum wait in seconds")
    parser.add_argument("--output", type=Path, required=True, help="JSON evidence path")
    parser.add_argument(
        "--manifest",
        type=Path,
        help="Optional camera capture manifest to mark passed after all topics validate",
    )
    return parser.parse_args()


def _stamp(message: Any) -> dict[str, int]:
    """Return one ROS message timestamp as JSON-safe integers."""

    return {
        "sec": int(message.header.stamp.sec),
        "nanosec": int(message.header.stamp.nanosec),
    }


def _stamp_key(message: Any) -> tuple[int, int]:
    """Return a timestamp tuple suitable for exact Image/CameraInfo matching."""

    stamp = _stamp(message)
    return stamp["sec"], stamp["nanosec"]


def _image_result(message: Any, spec: CameraSpec) -> dict[str, object]:
    """Serialize one Image and evaluate its per-topic camera contract."""

    return {
        "type": "sensor_msgs/msg/Image",
        "frame_id": message.header.frame_id,
        "stamp": _stamp(message),
        "width": int(message.width),
        "height": int(message.height),
        "encoding": message.encoding,
        "step": int(message.step),
        "valid": (
            message.header.frame_id == spec.frame_id
            and message.width == spec.width_px
            and message.height == spec.height_px
            and bool(message.data)
        ),
    }


def _camera_info_result(message: Any, spec: CameraSpec) -> dict[str, object]:
    """Serialize one CameraInfo and evaluate its per-topic camera contract."""

    return {
        "type": "sensor_msgs/msg/CameraInfo",
        "frame_id": message.header.frame_id,
        "stamp": _stamp(message),
        "width": int(message.width),
        "height": int(message.height),
        "distortion_model": message.distortion_model,
        "k": list(message.k),
        "d": list(message.d),
        "r": list(message.r),
        "p": list(message.p),
        "valid": (
            message.header.frame_id == spec.frame_id
            and message.width == spec.width_px
            and message.height == spec.height_px
            and len(message.k) == 9
            and len(message.r) == 9
            and len(message.p) == 12
        ),
    }


class _SynchronizedCameraMessages:
    """Keep pending messages until an Image and CameraInfo share one exact stamp."""

    def __init__(self) -> None:
        """Initialize per-camera pending queues and finalized evidence."""

        self.received: dict[str, dict[str, object]] = {}
        self.matched_pairs: dict[str, dict[str, object]] = {}
        self.observed_topics: set[str] = set()
        self._images: dict[str, dict[tuple[int, int], dict[str, object]]] = {}
        self._camera_infos: dict[str, dict[tuple[int, int], dict[str, object]]] = {}

    def record_image(self, message: Any, spec: CameraSpec) -> None:
        """Queue an Image, finalizing this camera only if its info already matches."""

        if spec.topic in self.received:
            return
        self.observed_topics.add(spec.topic)
        stamp = _stamp_key(message)
        self._images.setdefault(spec.name, {})[stamp] = _image_result(message, spec)
        self._finalize_if_matched(spec, stamp)

    def record_camera_info(self, message: Any, spec: CameraSpec) -> None:
        """Queue CameraInfo, finalizing this camera only if its image already matches."""

        if spec.camera_info_topic in self.received:
            return
        self.observed_topics.add(spec.camera_info_topic)
        stamp = _stamp_key(message)
        self._camera_infos.setdefault(spec.name, {})[stamp] = _camera_info_result(message, spec)
        self._finalize_if_matched(spec, stamp)

    def _finalize_if_matched(self, spec: CameraSpec, stamp: tuple[int, int]) -> None:
        """Publish evidence atomically for an exact Image/CameraInfo timestamp pair."""

        image = self._images.get(spec.name, {}).get(stamp)
        camera_info = self._camera_infos.get(spec.name, {}).get(stamp)
        if image is None or camera_info is None:
            return
        self.received[spec.topic] = image
        self.received[spec.camera_info_topic] = camera_info
        self.matched_pairs[spec.name] = {
            "image_topic": spec.topic,
            "camera_info_topic": spec.camera_info_topic,
            "stamp": _stamp_from_key(stamp),
        }
        del self._images[spec.name][stamp]
        del self._camera_infos[spec.name][stamp]


def _stamp_from_key(stamp: tuple[int, int]) -> dict[str, int]:
    """Convert an internal timestamp key back to the JSON evidence representation."""

    return {"sec": stamp[0], "nanosec": stamp[1]}


def main() -> int:
    """Subscribe through system Jazzy and require one synchronized pair per camera."""

    args = parse_args()
    if args.timeout <= 0:
        raise ValueError("--timeout must be positive")

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    rclpy.init()
    node = Node("wheelbot_camera_topic_verifier")
    synchronized = _SynchronizedCameraMessages()
    subscriptions: list[object] = []

    for camera in CAMERAS:
        def on_image(message: Image, *, spec: CameraSpec = camera) -> None:
            """Queue an RGB message until its matching CameraInfo arrives."""

            synchronized.record_image(message, spec)

        def on_camera_info(message: CameraInfo, *, spec: CameraSpec = camera) -> None:
            """Queue CameraInfo until its matching RGB message arrives."""

            synchronized.record_camera_info(message, spec)

        subscriptions.append(
            node.create_subscription(Image, camera.topic, on_image, qos_profile_sensor_data)
        )
        subscriptions.append(
            node.create_subscription(
                CameraInfo,
                camera.camera_info_topic,
                on_camera_info,
                qos_profile_sensor_data,
            )
        )

    expected_topics = {
        endpoint
        for camera in CAMERAS
        for endpoint in (camera.topic, camera.camera_info_topic)
    }
    deadline = time.monotonic() + args.timeout
    while (
        rclpy.ok()
        and time.monotonic() < deadline
        and synchronized.received.keys() != expected_topics
    ):
        rclpy.spin_once(node, timeout_sec=0.2)

    missing = sorted(expected_topics - synchronized.received.keys())
    unmatched = sorted(synchronized.observed_topics - synchronized.received.keys())
    invalid = sorted(
        topic for topic, result in synchronized.received.items() if not result["valid"]
    )
    result = {
        "validated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "ros_distro": "jazzy",
        "expected_topic_count": len(expected_topics),
        "received_topic_count": len(synchronized.received),
        "matched_camera_count": len(synchronized.matched_pairs),
        "missing_topics": missing,
        "unmatched_topics": unmatched,
        "invalid_topics": invalid,
        "matched_pairs": dict(sorted(synchronized.matched_pairs.items())),
        "topics": dict(sorted(synchronized.received.items())),
        "passed": not missing and not invalid,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    if result["passed"] and args.manifest is not None:
        manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
        manifest["ros2_message_validation"] = "passed"
        manifest["ros2_evidence"] = str(args.output.name)
        args.manifest.write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
