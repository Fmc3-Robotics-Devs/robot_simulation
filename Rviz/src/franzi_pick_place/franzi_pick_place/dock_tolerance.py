#!/usr/bin/env python3
"""How badly can the base park and still work the station?

This is the number to hand Nav2. Coarse navigation only has to get the robot
close enough that two things still hold at the parked pose:

1. the tag is detectable - in range, inside the camera cone, not edge-on;
2. the part it implies is reachable, collision-free, and the straight-line
   descent onto it can actually be planned.

Whichever of the two fails first is the binding constraint, and the tool says
which, because they want opposite things: backing off helps the camera and
hurts the arm.

Needs a running move_group (`ros2 launch franzi_moveit_config demo.launch.py`).
"""

import os
import sys
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer

from .base import MobileBase
from .geometry import make_pose, offset_pose, quaternion_from_rpy, tcp_to_tip
from .motion import ArmMotion, PlanningFailure
from .pick_place_node import PickPlaceTask, build_moveit
from .planning_scene import PlanningSceneClient
from .scene import COLORS, FEEDER, build_ground, build_station, build_workpiece
from .tags import HandEyeCorrection, MockTagDetector, TagObserver, pose_from_tag
from .transforms import from_pose

OK = "  +  "
NO_TAG = "  t  "
NO_ARM = "  a  "


def _samples(low, high, step):
    count = int(round((high - low) / step))
    return [round(low + index * step, 4) for index in range(count + 1)]


class DockSweep:
    def __init__(self, node, moveit, base, scene, tags, arm):
        self._node = node
        self._layout = node.layout()
        self._base = base
        self._scene = scene
        self._tags = tags
        self._arm = arm
        self._tcp_offset = tuple(node.get("tcp_offset"))
        self._orientation = quaternion_from_rpy(0.0, 0.0, node.get("grasp_yaw"))
        self._approach = (0.0, 0.0, node.get("approach_height"))

    def _tip(self, tcp):
        return tcp_to_tip(tcp, self._tcp_offset)

    def probe(self, dx, dy, dyaw):
        """Park with the given error and report what still works."""
        nominal_x, nominal_y, nominal_yaw = self._layout.dock_pose(FEEDER)
        self._base.set_pose(nominal_x + dx, nominal_y + dy, nominal_yaw + dyaw)
        if not self._arm.wait_for_base(nominal_x + dx, nominal_y + dy):
            return NO_TAG

        try:
            tag = self._tags.wait_for(self._layout.tag_ids[FEEDER], timeout=1.5)
        except RuntimeError:
            return NO_TAG

        part = pose_from_tag(tag, self._layout.part_offset_in_tag)
        objects = build_station(self._layout, FEEDER, part)
        workpiece = build_workpiece(self._layout)
        workpiece.pose = part
        self._scene.add_objects(objects + [workpiece], colors=COLORS)

        grasp = make_pose(
            (
                part.position.x,
                part.position.y,
                part.position.z + self._node.get("grasp_height_offset"),
            ),
            self._orientation,
        )
        try:
            self._arm.solve_approach_pair(
                self._tip(grasp), self._tip(offset_pose(grasp, self._approach)), "probe", branches=3
            )
            return OK
        except PlanningFailure:
            return NO_ARM

    def sweep(self, xs, ys, dyaw, label):
        print(f"\n=== dock error sweep, yaw error {dyaw:+.1f} deg ({label}) ===")
        print("  dy \\ dx" + "".join(f"{x:+6.2f}" for x in xs))
        rows = list(reversed(ys))
        results = {}
        for dy in rows:
            cells = []
            for dx in xs:
                outcome = self.probe(dx, dy, dyaw)
                results[(dx, dy)] = outcome
                cells.append(outcome)
            print(f"{dy:+8.2f}" + "".join(cells))
        return results


def main():
    rclpy.init()
    node = PickPlaceTask()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    exit_code = 0
    try:
        layout = node.layout()
        base = MobileBase(node, frame_id=layout.frame_id)
        base.set_pose(*layout.dock_pose(FEEDER))

        moveit = build_moveit()
        scene = PlanningSceneClient(node, monitor=moveit.get_planning_scene_monitor())
        scene.wait_for_services()
        scene.add_objects([build_ground(layout)], colors=COLORS)

        tf_buffer = Buffer()
        listener = TransformListener(tf_buffer, node)
        correction = HandEyeCorrection(
            node,
            camera_frame=node.get("camera_frame"),
            translation=node.get("handeye_correction.xyz"),
            rpy=node.get("handeye_correction.rpy"),
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
        )
        tags = TagObserver(node, tf_buffer, layout.frame_id, settle=0.3)
        arm = ArmMotion(
            moveit,
            node.get_logger(),
            arm_group=node.get("arm_group"),
            tip_link=node.get("tip_link"),
            ik_attempts=node.get("ik_attempts"),
            ik_timeout=node.get("ik_timeout"),
        )

        # The body posture decides the camera height and the arm envelope, so
        # probe in the posture the task actually docks in.
        arm.move_named(node.get("body_group"), node.get("body_posture"))
        arm.move_named(node.get("head_group"), node.get("look_posture"))

        sweep = DockSweep(node, moveit, base, scene, tags, arm)
        span = node.get("dock_sweep.span")
        step = node.get("dock_sweep.step")
        xs = _samples(-span, span, step)
        ys = _samples(-span, span, step)

        for dyaw_deg in node.get("dock_sweep.yaw_errors"):
            sweep.sweep(xs, ys, dyaw_deg * 3.14159265 / 180.0, f"{dyaw_deg:+.0f} deg")

        print(
            f"\n{OK.strip()} usable   {NO_TAG.strip()} tag not detectable   "
            f"{NO_ARM.strip()} tag seen but the part is out of reach"
        )
        print("The usable box is the docking accuracy Nav2 has to deliver.", flush=True)
    except (RuntimeError, PlanningFailure) as error:
        node.get_logger().error(str(error))
        exit_code = 1
    finally:
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(exit_code)


if __name__ == "__main__":
    main()
