#!/usr/bin/env python3
"""Machine-tending demo: load a workpiece into a fixture, wait, unload it.

The whole cycle is kinematic. Grasping is modelled by attaching the workpiece
to the wrist in the planning scene, which is enough to validate reachability,
approach directions, postures and the station handshake. Contact forces and
insertion tolerance are out of scope here - that is what the physics-based
stage is for.
"""

import os
import sys
import threading
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .geometry import make_pose, offset_pose, quaternion_from_rpy, tcp_to_tip
from .gripper import Gripper
from .machining import MachiningStation
from .motion import ArmMotion, PlanningFailure
from .planning_scene import PlanningSceneClient
from .scene import COLORS, FIXTURE, WORK_TABLE, WORKPIECE, CellLayout, build_cell

DEFAULTS = {
    "frame_id": "world",
    "arm_group": "left_arm",
    "body_group": "body",
    "head_group": "head",
    "park_group": "both_arms",
    "body_posture": "work",
    "head_posture": "home",
    "park_posture": "off_arms",
    "tip_link": "left_wrist_roll_Link",
    "attach_link": "left_wrist_roll_Link",
    "touch_links": [
        "leftfinger1_Link",
        "leftfinger2_Link",
        "left_wrist_roll_Link",
        "left_wrist_d405_Link",
    ],
    "gripper_controller": "left_gripper_controller",
    "gripper_joints": ["leftfinger1_joint", "leftfinger2_joint"],
    "gripper_gap_at_zero": 0.095,
    "gripper_max_stroke": 0.0475,
    "gripper_open_gap": 0.085,
    "grasp_squeeze": 0.002,
    "tcp_offset": [-0.0305, -0.0018, -0.2414],
    "grasp_height_offset": 0.02,
    "grasp_yaw": 0.0,
    "approach_height": 0.12,
    "approach_velocity_scaling": 0.15,
    "ik_attempts": 25,
    "ik_timeout": 0.05,
    "repeat": 1,
    "ground_z": -0.0873,
    "table.center_xy": [0.48, 0.16],
    "table.size_xy": [0.40, 0.60],
    "table.top_z": 0.80,
    "table.thickness": 0.03,
    "table.leg_size": 0.05,
    "table.leg_inset": 0.02,
    "workpiece.size": [0.05, 0.05, 0.09],
    "workpiece.pick_xy": [0.38, 0.34],
    "pocket.center_xy": [0.42, 0.16],
    "pocket.clearance": 0.012,
    "pocket.wall_thickness": 0.015,
    "pocket.wall_height": 0.03,
    "unload.center_xy": [0.38, -0.02],
    "machining.duration": 8.0,
    "machining.wait_for_trigger": False,
    "reach_map.x_range": [0.28, 0.60],
    "reach_map.y_range": [-0.24, 0.42],
    "reach_map.step": 0.04,
}


class PickPlaceTask(Node):
    def __init__(self):
        super().__init__("pick_place_task")
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)

    def get(self, name):
        return self.get_parameter(name).value

    def layout(self):
        return CellLayout(
            frame_id=self.get("frame_id"),
            ground_z=self.get("ground_z"),
            table_center_xy=tuple(self.get("table.center_xy")),
            table_size_xy=tuple(self.get("table.size_xy")),
            table_top_z=self.get("table.top_z"),
            table_thickness=self.get("table.thickness"),
            leg_size=self.get("table.leg_size"),
            leg_inset=self.get("table.leg_inset"),
            workpiece_size=tuple(self.get("workpiece.size")),
            pick_xy=tuple(self.get("workpiece.pick_xy")),
            pocket_xy=tuple(self.get("pocket.center_xy")),
            pocket_clearance=self.get("pocket.clearance"),
            pocket_wall_thickness=self.get("pocket.wall_thickness"),
            pocket_wall_height=self.get("pocket.wall_height"),
            unload_xy=tuple(self.get("unload.center_xy")),
        )


class MachineTendingDemo:
    def __init__(self, node: PickPlaceTask, moveit: MoveItPy):
        self._node = node
        self._logger = node.get_logger()
        self._layout = node.layout()
        self._scene = PlanningSceneClient(node)
        self._station = MachiningStation(
            node,
            self._layout,
            duration=node.get("machining.duration"),
            wait_for_trigger=node.get("machining.wait_for_trigger"),
        )
        self._gripper = Gripper(
            node,
            controller=node.get("gripper_controller"),
            joints=node.get("gripper_joints"),
            gap_at_zero=node.get("gripper_gap_at_zero"),
            max_stroke=node.get("gripper_max_stroke"),
        )
        self._arm = ArmMotion(
            moveit,
            self._logger,
            arm_group=node.get("arm_group"),
            tip_link=node.get("tip_link"),
            ik_attempts=node.get("ik_attempts"),
            ik_timeout=node.get("ik_timeout"),
        )
        self._tcp_offset = tuple(node.get("tcp_offset"))
        self._grasp_orientation = quaternion_from_rpy(0.0, 0.0, node.get("grasp_yaw"))
        self._approach = (0.0, 0.0, node.get("approach_height"))
        self._approach_scaling = node.get("approach_velocity_scaling")

    # -- helpers -----------------------------------------------------------

    def _grasp_tcp(self, resting_pose):
        """TCP pose for grasping a workpiece resting at ``resting_pose``."""
        return make_pose(
            (
                resting_pose.position.x,
                resting_pose.position.y,
                resting_pose.position.z + self._node.get("grasp_height_offset"),
            ),
            self._grasp_orientation,
        )

    def _tip_pose(self, tcp_pose):
        return tcp_to_tip(tcp_pose, self._tcp_offset)

    def _open_gripper(self):
        self._gripper.set_gap(self._node.get("gripper_open_gap"))

    def _close_on_workpiece(self):
        width = self._layout.workpiece_size[0]
        self._gripper.set_gap(width - self._node.get("grasp_squeeze"))

    # -- scene -------------------------------------------------------------

    def setup_scene(self):
        self._scene.wait_for_services()
        self._gripper.wait_for_controller()
        self._scene.add_objects(build_cell(self._layout), colors=COLORS)
        # The workpiece rests on the table and is inserted between the fixture
        # walls, so those contacts are expected rather than faults. Robot links
        # are still checked against both.
        self._scene.allow_collisions(
            [(WORKPIECE, WORK_TABLE), (WORKPIECE, FIXTURE)]
        )
        self._logger.info("planning scene ready")

    def reset_workpiece(self):
        self._scene.move_object(WORKPIECE, self._layout.frame_id, self._layout.pick_pose)
        self._station.reset()

    # -- task steps --------------------------------------------------------

    def prepare(self):
        self._arm.move_named(self._node.get("body_group"), self._node.get("body_posture"))
        self._arm.move_named(self._node.get("head_group"), self._node.get("head_posture"))
        self._open_gripper()

    def park(self):
        self._arm.move_named(self._node.get("park_group"), self._node.get("park_posture"))

    def _approach_states(self, resting_pose, label):
        """IK for the contact pose and the pose straight above it."""
        contact = self._grasp_tcp(resting_pose)
        return self._arm.solve_approach_pair(
            self._tip_pose(contact),
            self._tip_pose(offset_pose(contact, self._approach)),
            label,
        )

    def _descend(self, state, label):
        self._arm.move_to_state(
            state, label, linear=True, velocity_scaling=self._approach_scaling
        )

    def pick(self, resting_pose, label):
        grasp, pregrasp = self._approach_states(resting_pose, label)

        self._logger.info(f"{label}: approaching")
        self._arm.move_to_state(pregrasp, f"{label} pre-grasp")
        self._descend(grasp, f"{label} grasp")

        self._close_on_workpiece()
        self._scene.attach(
            WORKPIECE, self._node.get("attach_link"), self._node.get("touch_links")
        )
        self._logger.info(f"{label}: workpiece attached")

        self._descend(pregrasp, f"{label} lift")

    def place(self, resting_pose, label):
        release, prerelease = self._approach_states(resting_pose, label)

        self._logger.info(f"{label}: approaching")
        self._arm.move_to_state(prerelease, f"{label} pre-place")
        self._descend(release, f"{label} insert")

        self._open_gripper()
        self._scene.detach(WORKPIECE, self._node.get("attach_link"))
        self._logger.info(f"{label}: workpiece released")

        self._descend(prerelease, f"{label} retreat")

    def run_once(self):
        self.prepare()
        self.pick(self._layout.pick_pose, "load: pick from feeder")
        self.place(self._layout.place_pose, "load: insert into pocket")
        self.park()

        self._station.run_cycle()

        self.pick(self._layout.place_pose, "unload: pick from pocket")
        self.place(self._layout.unload_pose, "unload: put down finished part")
        self.park()

    def run(self):
        self.setup_scene()
        cycles = max(1, self._node.get("repeat"))
        for cycle in range(cycles):
            if cycle:
                self._logger.info("resetting cell for the next cycle")
                self.reset_workpiece()
            self._logger.info(f"--- cycle {cycle + 1}/{cycles} ---")
            self.run_once()
        self._logger.info("machine tending demo finished")


def build_moveit():
    """Start an embedded MoveIt instance that shares move_group's scene."""
    moveit_config = (
        MoveItConfigsBuilder("wheel_robot_4.0", package_name="franzi_moveit_config")
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"],
        )
        .pilz_cartesian_limits()
        .to_moveit_configs()
    )

    config = moveit_config.to_dict()
    planning_params = (
        Path(get_package_share_directory("franzi_pick_place")) / "config" / "moveit_py.yaml"
    )
    config.update(yaml.safe_load(planning_params.read_text()))

    return MoveItPy(node_name="pick_place_moveit", config_dict=config)


def main():
    rclpy.init()
    node = PickPlaceTask()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    exit_code = 0
    try:
        # Keep a strong reference: MoveItCpp crashes if it is collected while
        # the process is still running.
        moveit = build_moveit()
        MachineTendingDemo(node, moveit).run()
    except (PlanningFailure, RuntimeError) as error:
        node.get_logger().error(str(error))
        exit_code = 1
    except KeyboardInterrupt:
        pass
    finally:
        # MoveItCpp runs its own rclcpp node and executor thread and segfaults
        # when it is unwound together with the interpreter, so the process is
        # handed straight back to the OS instead of being torn down.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(exit_code)


if __name__ == "__main__":
    main()
