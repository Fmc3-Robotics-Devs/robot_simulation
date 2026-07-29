#!/usr/bin/env python3
"""Print where the arm can reach a top-down grasp, in the base frame.

The grid is relative to the chassis, so it is the envelope the robot carries
around with it. What has to land inside it is `dock.offset` - the position a
station occupies in the base frame once the robot has parked. Station world
poses themselves are irrelevant here; only the offset is.

Endpoint reachability alone is not enough - a pose near the edge of the
envelope is reachable but the straight approach to it often is not - so keep
`dock.offset` away from the boundary.

Needs a running move_group (`ros2 launch franzi_moveit_config demo.launch.py`).
"""

import os
import random
import sys
import threading
from dataclasses import replace

import rclpy
from moveit.core.robot_state import RobotState
from rclpy.executors import MultiThreadedExecutor

from .geometry import make_pose, quaternion_from_rpy, tcp_to_tip
from .pick_place_node import PickPlaceTask, build_moveit
from .planning_scene import PlanningSceneClient
from .scene import (
    COLORS,
    FEEDER,
    FIXTURE,
    STATIONS,
    WORKPIECE,
    bench_id,
    build_bench,
    build_ground,
)


class ReachMap:
    def __init__(self, node: PickPlaceTask, moveit):
        self._node = node
        self._moveit = moveit
        self._model = moveit.get_robot_model()
        self._psm = moveit.get_planning_scene_monitor()
        self._arm_group = node.get("arm_group")
        self._tip_link = node.get("tip_link")
        self._tcp_offset = tuple(node.get("tcp_offset"))
        self._orientation = quaternion_from_rpy(0.0, 0.0, node.get("grasp_yaw"))
        self._attempts = node.get("ik_attempts")
        self._timeout = node.get("ik_timeout")
        self._random = random.Random(0)

        # The torso pose decides the whole envelope, so probe with the body in
        # the posture the task actually uses.
        body = moveit.get_planning_component(node.get("body_group"))
        self._body_pose = body.get_named_target_state_values(node.get("body_posture"))

    def _fresh_state(self):
        state = RobotState(self._model)
        state.set_to_default_values()
        positions = dict(state.joint_positions)
        positions.update(self._body_pose)
        state.joint_positions = positions
        state.update()
        return state

    def reachable(self, x, y, z):
        state = self._fresh_state()
        seed = list(state.get_joint_group_positions(self._arm_group))
        tip = tcp_to_tip(make_pose((x, y, z), self._orientation), self._tcp_offset)

        for attempt in range(self._attempts):
            if attempt:
                state.set_joint_group_positions(
                    self._arm_group,
                    [value + self._random.uniform(-0.8, 0.8) for value in seed],
                )
                state.update()
            if not state.set_from_ik(self._arm_group, tip, self._tip_link, self._timeout):
                continue
            state.update()
            with self._psm.read_only() as scene:
                if scene.is_state_valid(state, self._arm_group, False):
                    return True
        return False

    def print_map(self, heights):
        x_min, x_max = self._node.get("reach_map.x_range")
        y_min, y_max = self._node.get("reach_map.y_range")
        step = self._node.get("reach_map.step")

        xs = _samples(x_min, x_max, step)
        ys = list(reversed(_samples(y_min, y_max, step)))

        for label, z in heights:
            print(f"\n=== {label} (z = {z:.3f}) ===")
            print("  y \\ x " + "".join(f"{x:6.2f}" for x in xs))
            for y in ys:
                cells = "".join(
                    "     +" if self.reachable(x, y, z) else "     ." for x in xs
                )
                print(f"{y:7.2f}" + cells)
        print("\n+ reachable and collision-free   . no valid IK", flush=True)


def _samples(low, high, step):
    count = int(round((high - low) / step))
    return [round(low + index * step, 3) for index in range(count + 1)]


def main():
    rclpy.init()
    node = PickPlaceTask()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    exit_code = 0
    try:
        layout = node.layout()
        # Probing happens with the chassis at the origin, so the bench has to be
        # placed where a station sits in the base frame - that is dock.offset -
        # rather than at its world pose.
        probe = replace(layout, station_xy={FEEDER: layout.dock_offset})

        # MoveItPy first: its monitor is the one the probes plan against, and
        # the scene client waits for diffs to land there.
        moveit = build_moveit()
        scene = PlanningSceneClient(node, monitor=moveit.get_planning_scene_monitor())
        scene.wait_for_services()
        # A previous demo run may have left the real cell in the scene at its
        # world poses, which would sit nowhere near the probe.
        scene.remove_objects(
            [bench_id(station) for station in STATIONS] + [FIXTURE, WORKPIECE]
        )
        scene.add_objects(
            [build_ground(probe), build_bench(probe, FEEDER)], colors=COLORS
        )

        grasp_z = layout.workpiece_centre_z + node.get("grasp_height_offset")
        print(f"\nbase-frame reachability; dock.offset = {tuple(layout.dock_offset)}")
        ReachMap(node, moveit).print_map(
            [
                ("grasp height", grasp_z),
                ("approach height", grasp_z + node.get("approach_height")),
            ]
        )
    except RuntimeError as error:
        node.get_logger().error(str(error))
        exit_code = 1
    finally:
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(exit_code)


if __name__ == "__main__":
    main()
