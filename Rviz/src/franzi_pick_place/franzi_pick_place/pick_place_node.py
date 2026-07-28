#!/usr/bin/env python3
"""Machine-tending demo: load a workpiece into a fixture, wait, unload it.

The whole cycle is kinematic. Grasping is modelled by attaching the workpiece
to the wrist in the planning scene, which is enough to validate reachability,
approach directions, postures and the station handshake. Contact forces and
insertion tolerance are out of scope here - that is what the physics-based
stage is for.
"""

import math
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
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer

from .base import MobileBase
from .geometry import (
    make_pose,
    offset_pose,
    quaternion_from_rpy,
    quaternion_product,
    tcp_to_tip,
)
from .gripper import Gripper
from .machining import MachiningStation
from .motion import ArmMotion, PlanningFailure
from .planning_scene import PlanningSceneClient
from .tags import HandEyeCorrection, MockTagDetector, TagObserver, pose_from_tag
from .transforms import from_pose
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
    "carry_tcp": [0.36, 0.24, 0.985],
    "approach_height": 0.12,
    "approach_velocity_scaling": 0.15,
    "ik_attempts": 25,
    "ik_timeout": 0.05,
    "repeat": 1,
    "ground_z": -0.0873,
    "station.feeder_xy": [1.00, -1.50],
    "station.machine_xy": [1.00, 0.00],
    "station.outfeed_xy": [1.00, 1.50],
    "tag.feeder_id": 0,
    "tag.machine_id": 1,
    "tag.outfeed_id": 2,
    "tag.size": 0.08,
    "tag.thickness": 0.002,
    "tag.to_part_xy": [-0.16, 0.0],
    "tag_timeout": 5.0,
    "camera_frame": "head_d435_Link",
    "look_posture": "look_down",
    "handeye_correction.xyz": [0.0, 0.0, 0.0],
    "handeye_correction.rpy": [0.0, 0.0, 0.0],
    "tag_detection.max_range": 3.0,
    "tag_detection.min_range": 0.25,
    "tag_detection.fov": 1.047,
    "tag_detection.position_noise": 0.0,
    "tag_detection.rotation_noise": 0.0,
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
            tag_ids={
                FEEDER: self.get("tag.feeder_id"),
                MACHINE: self.get("tag.machine_id"),
                OUTFEED: self.get("tag.outfeed_id"),
            },
            tag_size=self.get("tag.size"),
            tag_thickness=self.get("tag.thickness"),
            tag_to_part_xy=tuple(self.get("tag.to_part_xy")),
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
        # The scene client has to know the monitor that plans, so it can wait
        # for its own diffs to land there rather than in move_group only.
        self._scene = PlanningSceneClient(node, monitor=moveit.get_planning_scene_monitor())
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
        # The tag pipeline: a hand-eye correction frame on top of the URDF
        # nominal extrinsic, a detector publishing into it, and a reader that
        # lets tf2 chain the whole thing back to the planning frame.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)
        correction = HandEyeCorrection(
            node,
            camera_frame=node.get("camera_frame"),
            translation=node.get("handeye_correction.xyz"),
            rpy=node.get("handeye_correction.rpy"),
        )
        self._detector = MockTagDetector(
            node,
            self._tf_buffer,
            camera_frame=node.get("camera_frame"),
            detection_parent=correction.frame,
            tag_poses={
                self._layout.tag_ids[station]: from_pose(self._layout.tag_pose(station))
                for station in STATIONS
            },
            planning_frame=self._layout.frame_id,
            max_range=node.get("tag_detection.max_range"),
            min_range=node.get("tag_detection.min_range"),
            fov=node.get("tag_detection.fov"),
            position_noise=node.get("tag_detection.position_noise"),
            rotation_noise=node.get("tag_detection.rotation_noise"),
        )
        self._tags = TagObserver(node, self._tf_buffer, self._layout.frame_id)

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
        self._logger.info("planning scene ready")

    def _contact_pairs(self, station):
        """What the held workpiece is allowed to touch while working a station.

        Resting a part on a bench, or seating it in the pocket, is contact by
        definition. Whitelisting it permanently is what let an earlier version
        sweep the carried part straight through a bench top, so the allowance
        only exists between the descent and the retreat.
        """
        pairs = [(WORKPIECE, bench_id(station))]
        if station == MACHINE:
            pairs.append((WORKPIECE, FIXTURE))
        return pairs

    def reset_workpiece(self):
        self._scene.move_object(
            WORKPIECE, self._layout.frame_id, self._layout.part_pose(FEEDER)
        )
        self._station.reset()

    # -- task steps --------------------------------------------------------

    def prepare(self):
        base_x, base_y, _ = self._base.pose
        if not self._arm.wait_for_base(base_x, base_y):
            raise PlanningFailure("MoveIt never picked up the initial base pose from TF")

        self._arm.move_named(self._node.get("body_group"), self._node.get("body_posture"))
        self._arm.move_named(self._node.get("head_group"), self._node.get("head_posture"))
        self._open_gripper()

        # A posture is a joint configuration, so solving it once against the
        # current base pose makes it valid wherever the robot later stands.
        carry = make_pose(self._node.get("carry_tcp"), self._grasp_orientation)
        self._carry_state = self._arm.solve_ik(self._base_to_world(self._tip_pose(carry)))
        if self._carry_state is None:
            self._logger.warning(
                "carry_tcp is not reachable, transiting with the arms down instead; "
                "check it against reach_map"
            )

    def _base_to_world(self, pose):
        """Re-express a base-frame pose in the world, at the current base pose."""
        x, y, yaw = self._base.pose
        px, py, pz = pose.position.x, pose.position.y, pose.position.z
        turn = quaternion_from_rpy(0.0, 0.0, yaw)
        return make_pose(
            (
                x + math.cos(yaw) * px - math.sin(yaw) * py,
                y + math.sin(yaw) * px + math.cos(yaw) * py,
                pz,
            ),
            quaternion_product(turn, pose.orientation),
        )

    def park_arms(self):
        self._arm.move_named(self._node.get("park_group"), self._node.get("park_posture"))

    def transit_posture(self, carrying):
        """Arms down when empty, held up in front when loaded.

        Letting the arm hang while holding a part swings it down past the bench
        edge, which is both wrong-looking and the shape of a real collision.
        """
        if carrying and self._carry_state is not None:
            self._arm.move_to_state(self._carry_state, "carry")
        else:
            self.park_arms()

    def drive_to(self, pose, label, carrying=False):
        """Tuck the arms, drive, then confirm nothing ended up in collision.

        Base motion is not planned - there is no navigation here - so this check
        is the only thing standing between a bad dock offset and an arm plan
        that starts inside a bench.
        """
        self.transit_posture(carrying)
        self._base.drive_to(*pose, label=label)
        if not self._arm.wait_for_base(pose[0], pose[1]):
            raise PlanningFailure(
                f"{label}: MoveIt never saw the base reach the dock pose; "
                "is anything else publishing world -> moveit_root?"
            )
        if self._arm.in_collision():
            raise PlanningFailure(
                f"{label}: robot is in collision after docking at "
                f"({pose[0]:.2f}, {pose[1]:.2f}); check dock.offset against the bench"
            )

    def drive_to_station(self, station, carrying=False):
        self.drive_to(
            self._layout.dock_pose(station), f"drive to {station}", carrying=carrying
        )

    def observe_part_pose(self, station, label):
        """Where the part is, according to the camera - never according to the map.

        This is the whole point of the tag: on hardware the station's world pose
        is unknown, and the only thing tying the arm to the bench is a detection
        plus the surveyed tag-to-part offset.
        """
        self._arm.move_named(self._node.get("head_group"), self._node.get("look_posture"))
        tag = self._tags.wait_for(
            self._layout.tag_ids[station], timeout=self._node.get("tag_timeout")
        )
        pose = pose_from_tag(tag, self._layout.part_offset_in_tag)
        truth = self._layout.part_pose(station)
        error = math.dist(
            (pose.position.x, pose.position.y, pose.position.z),
            (truth.position.x, truth.position.y, truth.position.z),
        )
        self._logger.info(
            f"{label}: tag {self._layout.tag_ids[station]} gives the part at "
            f"({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}), "
            f"{error * 1000:.1f} mm from truth"
        )
        return pose

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

    def pick(self, station, label):
        grasp, pregrasp = self._approach_states(
            self.observe_part_pose(station, label), label
        )
        pairs = self._contact_pairs(station)

        self._logger.info(f"{label}: approaching")
        self._arm.move_to_state(pregrasp, f"{label} pre-grasp")
        self._scene.set_collisions(pairs, True)
        self._descend(grasp, f"{label} grasp")

        self._close_on_workpiece()
        self._scene.attach(
            WORKPIECE, self._node.get("attach_link"), self._node.get("touch_links")
        )
        self._logger.info(f"{label}: workpiece attached")

        self._descend(pregrasp, f"{label} lift")
        self._scene.set_collisions(pairs, False)

    def place(self, station, label):
        release, prerelease = self._approach_states(
            self.observe_part_pose(station, label), label
        )
        pairs = self._contact_pairs(station)

        self._logger.info(f"{label}: approaching")
        self._arm.move_to_state(prerelease, f"{label} pre-place")
        self._scene.set_collisions(pairs, True)
        self._descend(release, f"{label} insert")

        self._open_gripper()
        self._scene.detach(WORKPIECE, self._node.get("attach_link"))
        self._logger.info(f"{label}: workpiece released")

        self._descend(prerelease, f"{label} retreat")
        self._scene.set_collisions(pairs, False)

    def run_once(self):
        self.prepare()

        self.drive_to_station(FEEDER)
        self.pick(FEEDER, "load: pick from feeder")

        self.drive_to_station(MACHINE, carrying=True)
        self.place(MACHINE, "load: insert into pocket")

        self.drive_to(self._layout.standby_pose(), "back off from machine")
        self._station.run_cycle()

        self.drive_to_station(MACHINE)
        self.pick(MACHINE, "unload: pick from pocket")

        self.drive_to_station(OUTFEED, carrying=True)
        self.place(OUTFEED, "unload: put down finished part")
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
