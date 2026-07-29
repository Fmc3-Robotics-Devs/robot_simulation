#!/usr/bin/env python3
"""Record a ROS 2 image topic to an MP4, timed by the wall clock.

Isaac runs headless here, so "watching the run" means recording what its
cameras publish. Frames are written at most at the requested rate; if the
renderer delivers slower, the video simply plays faster than reality, which
for a documentation clip is preferable to freezing.

    python3 record_topic.py /observer/color/image_raw out.mp4 --fps 20

Runs on the ROS side (system Python), not inside Isaac.
"""

import argparse
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class Recorder(Node):
    def __init__(self, topic, path, fps, clocked=False):
        super().__init__("run_recorder")
        self._path = path
        self._fps = fps
        self._clocked = clocked
        self._latest = None
        self._writer = None
        self._last_write = 0.0
        self._frames = 0
        self.create_subscription(Image, topic, self._on_image, 10)
        mode = "clocked" if clocked else "<="
        self.get_logger().info(f"recording {topic} -> {path} at {mode} {fps} fps")

    def _on_image(self, message):
        image = np.frombuffer(message.data, np.uint8)
        image = image.reshape(message.height, message.width, -1)[..., :3]
        frame = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        if self._clocked:
            # Written by the clock in write_tick, not here.
            self._latest = frame
            return

        now = time.monotonic()
        if now - self._last_write < 1.0 / self._fps:
            return
        self._last_write = now
        self._write(frame)

    def _write(self, frame):
        if self._writer is None:
            height, width = frame.shape[:2]
            self._writer = cv2.VideoWriter(
                self._path,
                cv2.VideoWriter_fourcc(*"mp4v"),
                self._fps,
                (width, height),
            )
        self._writer.write(frame)
        self._frames += 1

    def write_tick(self):
        """Clocked mode: write the latest frame, repeating it when the camera
        is slower than the video. Duration then matches wall time, so files
        from parallel recorders stay in sync for side-by-side composition."""
        if self._latest is not None:
            self._write(self._latest)

    def close(self):
        if self._writer is not None:
            self._writer.release()
        self.get_logger().info(f"wrote {self._frames} frames to {self._path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("topic")
    parser.add_argument("output")
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument(
        "--clocked",
        action="store_true",
        help="Write on a fixed clock (repeating frames) so parallel "
        "recordings share a timeline and can be composed side by side.",
    )
    args = parser.parse_args()

    rclpy.init()
    recorder = Recorder(args.topic, args.output, args.fps, clocked=args.clocked)
    try:
        if args.clocked:
            period = 1.0 / args.fps
            next_write = time.monotonic() + period
            while rclpy.ok():
                rclpy.spin_once(recorder, timeout_sec=0.02)
                if time.monotonic() >= next_write:
                    recorder.write_tick()
                    next_write += period
        else:
            rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.close()
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
