#!/usr/bin/env python3
"""Republish camera_info with the matching image's timestamp.

apriltag_ros pairs image and camera_info by *exact* stamp equality. Isaac's
bridge publishes the two from separate graph nodes, each sampling the system
clock on its own, so their stamps never match and the pairing silently starves
("Topics ... do not appear to be synchronized").

This relay caches the latest camera_info and, for every image, republishes it
stamped with that image's own timestamp. Intrinsics are static here, so the
only thing the stamp asserts - "this calibration belongs to this frame" - is
exactly what the copy makes true. A real camera driver stamps both messages
together and does not need this.

    ros2 run franzi_skills camera_stamp_relay --ros-args -p camera:=head_d435
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class CameraStampRelay(Node):
    def __init__(self):
        super().__init__("camera_stamp_relay")
        self.declare_parameter("camera", "head_d435")
        camera = self.get_parameter("camera").value

        self._info = None
        self.create_subscription(
            CameraInfo, f"{camera}/color/camera_info", self._on_info, 10
        )
        self.create_subscription(
            Image, f"{camera}/color/image_raw", self._on_image, 10
        )
        self._publisher = self.create_publisher(
            CameraInfo, f"{camera}/color/synced_camera_info", 10
        )
        self.get_logger().info(f"restamping {camera} camera_info to image time")

    def _on_info(self, message):
        self._info = message

    def _on_image(self, message):
        if self._info is None:
            return
        info = self._info
        info.header.stamp = message.header.stamp
        self._publisher.publish(info)


def main():
    rclpy.init()
    node = CameraStampRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
