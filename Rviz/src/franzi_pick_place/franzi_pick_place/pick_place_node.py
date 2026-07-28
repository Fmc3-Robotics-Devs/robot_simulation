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
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .base import MobileBase
from .geometry import make_pose, offset_pose, quaternion_from_rpy, tcp_to_tip
from .gripper import Gripper
from .machining import MachiningStation
from .motion import ArmMotion, PlanningFailure
from .planning_scene import PlanningSceneClient
from .scene import (
    COLORS,
    FEEDER,
    FIXTURE,
    MACHINE,
    OUTFEED,
    STATIONS,
    WORKPIECE,
    CellLayout,
    bench_id,
    build_cell,
)

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
    "station.feeder_xy": [1.00, -1.50],
    "station.machine_xy": [1.00, 0.00],
    "station.outfeed_xy": [1.00, 1.50],
    "bench.size_xy": [0.50, 0.70],
    "bench.top_z": 0.80,
    "bench.thickness": 0.03,
    "bench.part_inset": 0.12,
    "bench.leg_size": 0.05,
    "bench.leg_inset": 0.03,
    "workpiece.size": [0.05, 0.05, 0.09],
    "pocket.clearance": 0.012,
    "pocket.wall_thickness": 0.015,
    "pocket.wall_height": 0.03,
    "dock.offset": [0.44, 0.16],
    "dock.yaw": 0.0,
    "dock.standby_retreat": 0.9,
    "base.linear_speed": 0.5,
    "base.angular_speed": 0.8,
    "base.wheel_radius": 0.0827,
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
            station_xy={
                FEEDER: tuple(self.get("station.feeder_xy")),
                MACHINE: tuple(self.get("station.machine_xy")),
                OUTFEED: tuple(self.get("station.outfeed_xy")),
            },
            bench_size_xy=tuple(self.get("bench.size_xy")),
            bench_top_z=self.get("bench.top_z"),
            bench_thickness=self.get("bench.thickness"),
            bench_part_inset=self.get("bench.part_inset"),
            leg_size=self.get("bench.leg_size"),
            leg_inset=self.get("bench.leg_inset"),
            workpiece_size=tuple(self.get("workpiece.size")),
            pocket_clearance=self.get("pocket.clearance"),
            pocket_wall_thickness=self.get("pocket.wall_thickness"),
            pocket_wall_height=self.get("pocket.wall_height"),
            dock_offset=tuple(self.get("dock.offset")),
            dock_yaw=self.get("dock.yaw"),
            standby_retreat=self.get("dock.standby_retreat"),
        )


class MachineTendingDemo:
    def __init__(self, node: PickPlaceTask, moveit: MoveItPy, base: MobileBase):
        self._node = node
        self._logger = node.get_logger()
        self._layout = node.layout()
        self._base = base
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
        # The workpiece rests on a bench and is inserted between the fixture
        # walls, so those contacts are expected rather than faults. Robot links
        # are still checked against both.
        self._scene.allow_collisions(
            [(WORKPIECE, bench_id(station)) for station in STATIONS]
            + [(WORKPIECE, FIXTURE)]
        )
        self._logger.info("planning scene ready")

    def reset_workpiece(self):
        self._scene.move_object(
            WORKPIECE, self._layout.frame_id, self._layout.part_pose(FEEDER)
        )
        self._station.reset()

    # -- task steps --------------------------------------------------------

    def prepare(self):
        self._arm.move_named(self._node.get("body_group"), self._node.get("body_posture"))
        self._arm.move_named(self._node.get("head_group"), self._node.get("head_posture"))
        self._open_gripper()

    def park_arms(self):
        self._arm.move_named(self._node.get("park_group"), self._node.get("park_posture"))

    def drive_to(self, pose, label):
        """Fold the arms away, drive, then confirm nothing ended up in collision.

        Base motion is not planned - there is no navigation here - so this check
        is the only thing standing between a bad dock offset and an arm plan
        that starts inside a bench.
        """
        self.park_arms()
        self._base.drive_to(*pose, label=label)
        # Give the state monitor a moment to catch up with the new transform.
        time.sleep(0.5)
        if self._arm.in_collision():
            raise PlanningFailure(
                f"{label}: robot is in collision after docking at "
                f"({pose[0]:.2f}, {pose[1]:.2f}); check dock.offset against the bench"
            )

    def drive_to_station(self, station):
        self.drive_to(self._layout.dock_pose(station), f"drive to {station}")

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

        self.drive_to_station(FEEDER)
        self.pick(self._layout.part_pose(FEEDER), "load: pick from feeder")

        self.drive_to_station(MACHINE)
        self.place(self._layout.part_pose(MACHINE), "load: insert into pocket")

        self.drive_to(self._layout.standby_pose(), "back off from machine")
        self._station.run_cycle()

        self.drive_to_station(MACHINE)
        self.pick(self._layout.part_pose(MACHINE), "unload: pick from pocket")

        self.drive_to_station(OUTFEED)
        self.place(self._layout.part_pose(OUTFEED), "unload: put down finished part")
        self.park_arms()

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
        # The base has to publish world -> moveit_root before MoveIt starts
        # monitoring state, otherwise the planar virtual joint has no value and
        # the current state never completes.
        base = MobileBase(
            node,
            frame_id=node.get("frame_id"),
            wheel_radius=node.get("base.wheel_radius"),
            linear_speed=node.get("base.linear_speed"),
            angular_speed=node.get("base.angular_speed"),
        )
        base.set_pose(*node.layout().dock_pose(FEEDER))

        # Keep a strong reference: MoveItCpp crashes if it is collected while
        # the process is still running.
        moveit = build_moveit()
        MachineTendingDemo(node, moveit, base).run()
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
