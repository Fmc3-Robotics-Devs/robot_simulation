#!/usr/bin/env python3
"""Record a docking pose per station, and check each one is actually usable.

On hardware this is a human driving the robot to a station and saying "here".
In simulation there is nobody at the joystick, so the sweep positions the base
itself - that is the one place a simulated pose is allowed to come from ground
truth, because on the real robot a person's eyes play the same role.

What is *not* taken on faith is whether the taught pose works: each one is
checked for a tag detection and a reachable, planable grasp before it is
written. A dock that fails here would fail every cycle.

Needs a running move_group (`ros2 launch franzi_moveit_config demo.launch.py \\
  publish_virtual_joint_tf:=false publish_chassis_joints:=false`).
"""

import os
import sys
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer

from .base import MobileBase
from .docks import save
from .geometry import make_pose, offset_pose, quaternion_from_rpy, tcp_to_tip
from .motion import ArmMotion, PlanningFailure
from .pick_place_node import PickPlaceTask, build_moveit
from .planning_scene import PlanningSceneClient
from .scene import COLORS, STATIONS, build_ground, build_station, build_workpiece
from .tags import HandEyeCorrection, MockTagDetector, TagObserver, pose_from_tag
from .transforms import from_pose


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
        base.set_pose(*layout.dock_pose(STATIONS[0]))

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
            tag_poses={
                layout.tag_ids[station]: from_pose(layout.tag_pose(station))
                for station in STATIONS
            },
            planning_frame=layout.frame_id,
            max_range=node.get("tag_detection.max_range"),
            min_range=node.get("tag_detection.min_range"),
            fov=node.get("tag_detection.fov"),
        )
        tags = TagObserver(node, tf_buffer, layout.frame_id, settle=0.4)
        arm = ArmMotion(
            moveit,
            node.get_logger(),
            arm_group=node.get("arm_group"),
            tip_link=node.get("tip_link"),
            ik_attempts=node.get("ik_attempts"),
            ik_timeout=node.get("ik_timeout"),
        )
        arm.move_named(node.get("body_group"), node.get("body_posture"))
        arm.move_named(node.get("head_group"), node.get("look_posture"))

        tcp_offset = tuple(node.get("tcp_offset"))
        orientation = quaternion_from_rpy(0.0, 0.0, node.get("grasp_yaw"))
        approach = (0.0, 0.0, node.get("approach_height"))

        taught = {}
        for station in STATIONS:
            target = layout.dock_pose(station)
            base.drive_to(*target, label=f"teach {station}")
            if not arm.wait_for_base(target[0], target[1]):
                raise RuntimeError(f"{station}: base pose never reached MoveIt")

            tag = tags.wait_for(layout.tag_ids[station], timeout=node.get("tag_timeout"))
            part = pose_from_tag(tag, layout.part_offset_in_tag_for(station))

            workpiece = build_workpiece(layout)
            workpiece.pose = part
            scene.add_objects(
                build_station(layout, station, part) + [workpiece], colors=COLORS
            )

            grasp = make_pose(
                (
                    part.position.x,
                    part.position.y,
                    part.position.z + node.get("grasp_height_offset"),
                ),
                orientation,
            )
            arm.solve_approach_pair(
                tcp_to_tip(grasp, tcp_offset),
                tcp_to_tip(offset_pose(grasp, approach), tcp_offset),
                f"teach {station}",
            )

            taught[station] = base.pose
            node.get_logger().info(
                f"{station}: dock ({target[0]:.3f}, {target[1]:.3f}, "
                f"{target[2]:.3f}) verified - tag seen and grasp planable"
            )
            scene.remove_objects([workpiece.id])

        path = save(node.get("dock_poses_file"), taught, layout.frame_id)
        node.get_logger().info(f"wrote {len(taught)} docking poses to {path}")
    except (RuntimeError, PlanningFailure) as error:
        node.get_logger().error(f"teaching failed: {error}")
        exit_code = 1
    finally:
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(exit_code)


if __name__ == "__main__":
    main()
