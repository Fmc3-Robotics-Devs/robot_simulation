"""Locating stations by AprilTag instead of by prior knowledge.

On hardware the robot does not know where a bench is. It knows what a tag on
that bench looks like, and everything it needs - where the part rests, where
the pocket is - is a fixed offset from that tag. This module is that path, and
the only difference between simulation and hardware is who publishes the
detection:

    <camera>  --(URDF nominal)-->  <camera>_calibrated  --(detection)-->  tag_N
                    ^                        ^
                    |                        |
          hand-eye correction        apriltag_ros on hardware,
          (static, from calibration)  MockTagDetector in simulation

Because the correction is a real link in the TF chain, a wrong hand-eye
calibration produces a wrong grasp here exactly as it would on the robot -
which is the point of simulating it at all.
"""

import math
import random

import numpy as np
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .transforms import (
    from_pose,
    from_rpy,
    from_transform_msg,
    invert,
    to_pose,
    transform,
)


def tag_frame(tag_id):
    return f"tag_{tag_id}"


def _stamped(stamp, parent, child, matrix):
    pose = to_pose(matrix)
    message = TransformStamped()
    message.header.stamp = stamp
    message.header.frame_id = parent
    message.child_frame_id = child
    message.transform.translation.x = pose.position.x
    message.transform.translation.y = pose.position.y
    message.transform.translation.z = pose.position.z
    message.transform.rotation = pose.orientation
    return message


class HandEyeCorrection:
    """The calibration result, as a static frame on top of the URDF nominal.

    Publishing it separately rather than editing the URDF keeps the nominal
    extrinsic and the measured correction distinguishable, and makes it the one
    thing to replace when a real calibration is run.
    """

    def __init__(self, node, camera_frame, translation, rpy):
        self._frame = f"{camera_frame}_calibrated"
        broadcaster = StaticTransformBroadcaster(node)
        broadcaster.sendTransform(
            _stamped(
                node.get_clock().now().to_msg(),
                camera_frame,
                self._frame,
                from_rpy(translation, rpy),
            )
        )
        # Held so the latched publication is not garbage collected.
        self._broadcaster = broadcaster
        if any(translation) or any(rpy):
            node.get_logger().warning(
                f"hand-eye correction is non-zero: {translation} / {rpy}; "
                "grasps will be off by that much"
            )

    @property
    def frame(self):
        return self._frame


class MockTagDetector:
    """Stands in for apriltag_ros. The only ground truth in the system.

    It publishes a detection only when the tag is actually in front of the
    camera, within range and not too oblique, so the task cannot quietly rely
    on knowing where a station is while looking somewhere else.
    """

    def __init__(
        self,
        node,
        tf_buffer,
        camera_frame,
        detection_parent,
        tag_poses,
        planning_frame="odom",
        max_range=3.0,
        min_range=0.25,
        fov=math.radians(60.0),
        max_view_angle=math.radians(70.0),
        position_noise=0.0,
        rotation_noise=0.0,
        rate=10.0,
    ):
        self._node = node
        self._logger = node.get_logger()
        self._buffer = tf_buffer
        self._camera_frame = camera_frame
        self._detection_parent = detection_parent
        self._tag_poses = tag_poses
        self._planning_frame = planning_frame
        self._max_range = max_range
        self._min_range = min_range
        # Approximated as a symmetric cone. The camera *link* frames in this
        # URDF are not optical frames - their axes do not line up with image
        # rows and columns - so a per-axis rectangular FOV test would encode an
        # assumption that is not true here.
        self._half_fov = fov / 2.0
        self._max_view_angle = max_view_angle
        self._position_noise = position_noise
        self._rotation_noise = rotation_noise
        self._random = random.Random(0)

        self._tf = TransformBroadcaster(node)
        self._timer = node.create_timer(1.0 / rate, self._publish)
        self._visible = set()

    def _camera_in_world(self):
        try:
            message = self._buffer.lookup_transform(
                self._planning_frame, self._camera_frame, Time()
            )
        except TransformException:
            return None
        return from_transform_msg(message.transform)

    def _observation(self, world_to_camera, world_to_tag):
        """What the camera sees, or None when the tag is not observable."""
        camera_to_tag = invert(world_to_camera) @ world_to_tag
        offset = camera_to_tag[:3, 3]

        distance = float(np.linalg.norm(offset))
        if not self._min_range <= distance <= self._max_range:
            return None
        # +z points out of the lens.
        if math.acos(np.clip(offset[2] / distance, -1.0, 1.0)) > self._half_fov:
            return None

        # A tag seen edge-on does not decode. Its face normal is its own +z.
        normal = camera_to_tag[:3, 2]
        towards_camera = -camera_to_tag[:3, 3] / distance
        if math.acos(np.clip(np.dot(normal, towards_camera), -1.0, 1.0)) > self._max_view_angle:
            return None

        return camera_to_tag @ self._noise()

    def _noise(self):
        if not self._position_noise and not self._rotation_noise:
            return np.eye(4)
        gauss = self._random.gauss
        return from_rpy(
            [gauss(0.0, self._position_noise) for _ in range(3)],
            [gauss(0.0, self._rotation_noise) for _ in range(3)],
        )

    def _publish(self):
        world_to_camera = self._camera_in_world()
        if world_to_camera is None:
            return

        stamp = self._node.get_clock().now().to_msg()
        for tag_id, world_to_tag in self._tag_poses.items():
            observation = self._observation(world_to_camera, world_to_tag)
            if observation is None:
                if tag_id in self._visible:
                    self._visible.discard(tag_id)
                    self._logger.info(f"tag {tag_id} out of view")
                continue
            if tag_id not in self._visible:
                self._visible.add(tag_id)
                self._logger.info(f"tag {tag_id} detected")
            self._tf.sendTransform(
                _stamped(stamp, self._detection_parent, tag_frame(tag_id), observation)
            )

    @property
    def visible(self):
        return set(self._visible)


class TagObserver:
    """Reads tag detections back out of TF, in the planning frame."""

    def __init__(self, node, tf_buffer, planning_frame="odom", settle=0.5):
        self._node = node
        self._buffer = tf_buffer
        self._planning_frame = planning_frame
        self._settle = settle

    def wait_for(self, tag_id, timeout=5.0):
        """Return the tag pose in the planning frame once it has been seen.

        Detections are only read while the robot is standing still. tf2
        resolves each link at its own latest stamp, so a detection computed
        before the head stopped moving gets composed with the camera pose from
        after it stopped; that showed up as a centimetre of phantom error even
        with a perfect calibration and no noise. Waiting out one detection
        cycle after the motion ends removes it at the source, and is what you
        would do on hardware anyway.
        """
        self._node.get_clock().sleep_for(Duration(seconds=self._settle))

        deadline = self._node.get_clock().now() + Duration(seconds=timeout)
        frame = tag_frame(tag_id)
        last_error = "no detection"
        while self._node.get_clock().now() < deadline:
            try:
                message = self._buffer.lookup_transform(
                    self._planning_frame, frame, Time()
                )
                return from_transform_msg(message.transform)
            except TransformException as error:
                last_error = str(error)
            self._node.get_clock().sleep_for(Duration(seconds=0.05))
        raise RuntimeError(f"tag {tag_id} was never observed: {last_error}")


def pose_from_tag(tag_matrix, offset_xyz, offset_rpy=(0.0, 0.0, 0.0)):
    """Compose a station feature pose from a tag pose and a fixed offset."""
    return to_pose(tag_matrix @ from_rpy(offset_xyz, offset_rpy))
