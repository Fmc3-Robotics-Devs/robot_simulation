#!/usr/bin/env python3
"""Hand-eye calibration for the head camera, and a self-test of it.

What this measures is the difference between where the URDF says the camera is
bolted and where it actually is. The camera rides on the head, so this is the
eye-in-hand formulation with the head's pitch link as the "gripper": observe
one fixed tag from many robot poses, and solve AX = XB for the mount.

Simulation cannot calibrate anything - the URDF *is* the truth there, so the
answer is zero by construction. What it can do, and what is worth far more than
a number, is prove the pipeline right: inject a known mounting error, and check
this recovers it. Frame conventions and inverted transforms are where hand-eye
code goes wrong, and finding that out here costs minutes instead of days.

Pose diversity is the whole game, and this robot makes it hard. Head yaw and
base yaw turn about the same vertical axis, so using only those gives an
effectively single-axis rotation set - measured against synthetic data, that
alone costs 212 mm of translation error while the identical solver call on
well-spread rotations lands within 0.001 mm. The second axis has to come from
pitch: the head pitches +-30 deg and the waist adds another +-30 deg, and the
tag has to stay in view throughout.

If the result still looks unstable on hardware, the textbook fix is to put the
target on the gripper and move the seven-axis arm instead. That is an
eye-to-hand solve rather than this one, but the rotations are unconstrained.

STATUS: this does not yet pass its own self-test. Injecting a known mounting
error and solving for it comes back ~170 mm out. What has been ruled out:

* the solver call - synthetic data with this robot's exact rotation range and
  planar translations recovers the answer to 0.000 mm;
* rotation degeneracy - two non-parallel axes are sufficient, and adding waist
  pitch to the head motion supplies them;
* a head that never moved, which is what the first version actually did:
  move_to_state defaulted to the arm group, so setting head joints on a goal
  state moved nothing.

What is left is the samples themselves. `spread()` reconstructs the target from
every sample using the *known* answer, and it should collapse to zero; it sits
at ~89 mm, so the recorded mount pose and the detection paired with it do not
describe the same instant. Longer settling moves it (108 mm to 89 mm at 1.5 s)
but does not close it, so staleness is a contributor and not the cause.
"""

import math
import os
import sys
import threading

import cv2
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from tf2_ros import TransformListener, TransformException
from tf2_ros.buffer import Buffer

from .base import MobileBase
from .motion import ArmMotion, PlanningFailure
from .pick_place_node import PickPlaceTask, build_moveit
from .scene import FEEDER
from .tags import HandEyeCorrection, MockTagDetector, tag_frame
from .transforms import from_rpy, from_transform_msg, from_pose, invert


def rpy_of(rotation):
    """Extract roll-pitch-yaw for reporting only."""
    pitch = math.asin(-max(-1.0, min(1.0, rotation[2][0])))
    if abs(rotation[2][0]) < 0.9999:
        roll = math.atan2(rotation[2][1], rotation[2][2])
        yaw = math.atan2(rotation[1][0], rotation[0][0])
    else:
        roll = math.atan2(-rotation[1][2], rotation[1][1])
        yaw = 0.0
    return roll, pitch, yaw


class HandEyeSession:
    """Collects (mount pose, tag observation) pairs and solves for the mount."""

    def __init__(self, node, tf_buffer, base, arm, mount_frame, camera_frame, detection_frame):
        self._node = node
        self._buffer = tf_buffer
        self._base = base
        self._arm = arm
        self._mount_frame = mount_frame
        self._camera_frame = camera_frame
        self._detection_frame = detection_frame
        self._frame_id = node.layout().frame_id
        self.samples = []

    def _lookup(self, parent, child):
        try:
            message = self._buffer.lookup_transform(parent, child, Time())
        except TransformException:
            return None
        return from_transform_msg(message.transform)

    def capture(self, tag_id):
        """Record one pose pair, or return False if the tag was not seen."""
        observation = self._lookup(self._detection_frame, tag_frame(tag_id))
        mount = self._lookup(self._frame_id, self._mount_frame)
        if observation is None or mount is None:
            return False
        self.samples.append((mount, observation))
        return True

    def solve(self, method=cv2.CALIB_HAND_EYE_TSAI):
        """Return the measured mount -> camera transform."""
        if len(self.samples) < 3:
            raise RuntimeError(f"only {len(self.samples)} usable views; need at least 3")

        base_to_mount = [sample[0] for sample in self.samples]
        camera_to_tag = [sample[1] for sample in self.samples]

        rotation, translation = cv2.calibrateHandEye(
            R_gripper2base=[matrix[:3, :3] for matrix in base_to_mount],
            t_gripper2base=[matrix[:3, 3].reshape(3, 1) for matrix in base_to_mount],
            R_target2cam=[matrix[:3, :3] for matrix in camera_to_tag],
            t_target2cam=[matrix[:3, 3].reshape(3, 1) for matrix in camera_to_tag],
            method=method,
        )
        solved = np.eye(4)
        solved[:3, :3] = rotation
        solved[:3, 3] = translation.reshape(3)
        return solved

    def spread(self, mount_to_camera):
        """Scatter of the reconstructed target position, in metres.

        Every sample should put the target in the same place: A @ X @ B is the
        base-to-target transform, and the target does not move. How far apart
        those reconstructions land is the honest quality measure for a
        calibration, and it works on hardware where the answer is unknown.
        """
        points = np.array(
            [
                (mount @ mount_to_camera @ observation)[:3, 3]
                for mount, observation in self.samples
            ]
        )
        return float(np.linalg.norm(points.std(axis=0)))

    def nominal(self):
        """What the URDF claims the mount -> camera transform is."""
        mount = self._lookup(self._frame_id, self._mount_frame)
        camera = self._lookup(self._frame_id, self._camera_frame)
        if mount is None or camera is None:
            raise RuntimeError("could not read the nominal camera mount from TF")
        return invert(mount) @ camera


def main():
    rclpy.init()
    node = PickPlaceTask()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    exit_code = 0
    try:
        layout = node.layout()
        correction_xyz = node.get("handeye_correction.xyz")
        correction_rpy = node.get("handeye_correction.rpy")
        if any(correction_xyz) or any(correction_rpy):
            raise RuntimeError(
                "run calibration with handeye_correction at zero - it is the "
                "quantity being measured, not an input"
            )

        truth = from_rpy(
            node.get("handeye_truth.xyz"), node.get("handeye_truth.rpy")
        )

        base = MobileBase(node, frame_id=layout.frame_id)
        dock = layout.dock_pose(FEEDER)
        base.set_pose(*dock)

        moveit = build_moveit()
        tf_buffer = Buffer()
        listener = TransformListener(tf_buffer, node)
        correction = HandEyeCorrection(
            node, camera_frame=node.get("camera_frame"), translation=[0.0] * 3, rpy=[0.0] * 3
        )
        MockTagDetector(
            node,
            tf_buffer,
            camera_frame=node.get("camera_frame"),
            detection_parent=correction.frame,
            tag_poses={layout.tag_ids[FEEDER]: from_pose(layout.tag_pose(FEEDER))},
            planning_frame=layout.frame_id,
            max_range=node.get("tag_detection.max_range"),
            min_range=node.get("tag_detection.min_range"),
            fov=node.get("tag_detection.fov"),
            mounting_error=truth,
        )
        arm = ArmMotion(
            moveit,
            node.get_logger(),
            arm_group=node.get("arm_group"),
            tip_link=node.get("tip_link"),
            ik_attempts=node.get("ik_attempts"),
            ik_timeout=node.get("ik_timeout"),
        )
        arm.move_named(node.get("body_group"), node.get("body_posture"))

        session = HandEyeSession(
            node,
            tf_buffer,
            base,
            arm,
            mount_frame=node.get("handeye_mount_frame"),
            camera_frame=node.get("camera_frame"),
            detection_frame=correction.frame,
        )

        head = moveit.get_planning_component(node.get("head_group"))
        skipped = 0
        base_poses = list(
            zip(
                node.get("handeye_base_dx"),
                node.get("handeye_base_dy"),
                node.get("handeye_base_dyaw"),
            )
        )
        for dx, dy, dyaw in base_poses:
            base.drive_to(dock[0] + dx, dock[1] + dy, dock[2] + dyaw, label="calibration pose")
            if not arm.wait_for_base(dock[0] + dx, dock[1] + dy):
                raise RuntimeError("base pose never reached MoveIt")

            for yaw, pitch, waist in zip(
                node.get("handeye_head_yaw"),
                node.get("handeye_head_pitch"),
                node.get("handeye_waist_pitch"),
            ):
                state = arm.current_state()
                state.set_joint_group_positions(node.get("head_group"), [yaw, pitch])
                # Tilting the waist swings the camera about an axis the head
                # cannot reach on its own, which is what keeps AX = XB solvable.
                body = list(state.get_joint_group_positions(node.get("body_group")))
                body[2] = waist
                state.set_joint_group_positions(node.get("body_group"), body)
                state.update()
                try:
                    # Each group has to be commanded on its own: a goal state is
                    # only read for the joints of the group being planned.
                    arm.move_to_state(
                        state, "calibration waist", group=node.get("body_group")
                    )
                    arm.move_to_state(
                        state, "calibration look", group=node.get("head_group")
                    )
                except PlanningFailure:
                    skipped += 1
                    continue
                node.get_clock().sleep_for(
                    rclpy.duration.Duration(seconds=node.get("handeye_settle"))
                )
                if not session.capture(layout.tag_ids[FEEDER]):
                    skipped += 1

        nominal = session.nominal()
        solved = session.solve()
        measured = invert(nominal) @ solved

        expected_xyz = np.array(node.get("handeye_truth.xyz"))
        error = np.linalg.norm(measured[:3, 3] - expected_xyz)
        roll, pitch, yaw = rpy_of(measured[:3, :3])

        print(f"\nviews used: {len(session.samples)} ({skipped} skipped)")
        print(f"target scatter, solved answer : {session.spread(solved) * 1000:8.2f} mm")
        print(f"target scatter, true answer   : "
              f"{session.spread(nominal @ truth) * 1000:8.2f} mm   "
              "<- if this is not ~0, the samples are inconsistent, not the solver")
        print(f"injected mounting error : xyz={list(expected_xyz)} "
              f"rpy={node.get('handeye_truth.rpy')}")
        print(f"recovered correction    : xyz={[round(v, 5) for v in measured[:3, 3]]} "
              f"rpy={[round(v, 5) for v in (roll, pitch, yaw)]}")
        print(f"translation residual    : {error * 1000:.2f} mm")
        print(
            "\nWrite the recovered values into handeye_correction in task.yaml.\n"
            "On hardware the injected error does not exist - the residual line is\n"
            "meaningless there, and the recovered correction is the whole output.",
            flush=True,
        )
    except (RuntimeError, PlanningFailure) as error:
        node.get_logger().error(f"hand-eye calibration failed: {error}")
        exit_code = 1
    finally:
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(exit_code)


if __name__ == "__main__":
    main()
