"""The cell as the skills see it: observations, grasps, placements, postures.

This is the manipulation core of `franzi_pick_place`'s demo, re-cut so each
piece can be called on its own by an action server instead of only in one
scripted order. Nothing here decides *when* anything happens - that is the
task manager's job - and nothing here talks to the engraving machine - that is
the bridge's. What remains is the part that would survive a move to hardware
unchanged: TF, MoveIt, the gripper, the planning scene, and the tag pipeline
that ties them to the world.

The one piece of state it keeps is whether the gripper holds the workpiece,
because a transport posture is a different shape when carrying (plan section
9.1) and because a pick that never closed on anything must be reportable.
"""

import math

from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import TransformException, TransformListener
from tf2_ros.buffer import Buffer

from franzi_pick_place.docks import DockBook
from franzi_pick_place.geometry import (
    make_pose,
    offset_pose,
    quaternion_from_rpy,
    quaternion_product,
    tcp_to_tip,
)
from franzi_pick_place.gripper import Gripper
from franzi_pick_place.motion import ArmMotion, PlanningFailure
from franzi_pick_place.params import default_dock_poses_file, layout_from
from franzi_pick_place.planning_scene import PlanningSceneClient
from franzi_pick_place.scene import (
    COLORS,
    FIXTURE,
    MACHINE,
    STATIONS,
    WORKPIECE,
    bench_id,
    build_ground,
    build_station,
    build_workpiece,
)
from franzi_pick_place.tags import (
    HandEyeCorrection,
    MockTagDetector,
    TagObserver,
    pose_from_tag,
)
from franzi_pick_place.transforms import (
    from_pose,
    from_rpy,
    from_transform_msg,
    invert,
    to_pose,
    yaw_of,
)


class SkillFailure(RuntimeError):
    """A skill step that failed with a reportable error code."""

    def __init__(self, error_code, detail=""):
        super().__init__(detail or error_code)
        self.error_code = error_code


class TendingCell:
    def __init__(self, node, moveit, base):
        self._node = node
        self._logger = node.get_logger()
        self._layout = layout_from(node)
        self._base = base
        self._scene = PlanningSceneClient(node, monitor=moveit.get_planning_scene_monitor())
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
        self._tf_buffer = Buffer()
        # The TF stream gets its own node *and* its own spin thread. On the
        # main node it queues behind skill callbacks until lookups "require
        # extrapolation into the past"; with spin_thread on the main node,
        # tf2 adds that node to a second executor and the two executors then
        # steal each other's callbacks (Nav2 goal responses vanished that
        # way). A dedicated node is the only arrangement with neither race.
        import rclpy

        self._tf_node = rclpy.create_node("skill_tf_listener")
        self._tf_listener = TransformListener(
            self._tf_buffer, self._tf_node, spin_thread=True
        )
        correction = HandEyeCorrection(
            node,
            camera_frame=node.get("camera_frame"),
            translation=node.get("handeye_correction.xyz"),
            rpy=node.get("handeye_correction.rpy"),
        )
        # In Isaac or on hardware an apriltag node publishes the detections and
        # this mock simply never sees a tag of its own to publish; enabled
        # false removes even that possibility.
        self._detector = None
        if node.get("mock_tags.enabled"):
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
        self._tags = TagObserver(
            node,
            self._tf_buffer,
            self._layout.frame_id,
            settle=node.get("tag_settle"),
        )
        self._docks = DockBook.load(default_dock_poses_file(node))

        self._holding = False
        self._workpiece_in_scene = False
        # Contact allowances left open after a placement, closed once the arm
        # has retreated to a named posture.
        self._lingering_pairs = []
        # Where the real part rests when it is not in the gripper. This is what
        # the Isaac stage mirrors, so the prop moves when the robot moves it.
        self._workpiece_rest = None
        self._carry_state = None
        self._tcp_offset = tuple(node.get("tcp_offset"))
        self._grasp_orientation = quaternion_from_rpy(0.0, 0.0, node.get("grasp_yaw"))
        self._approach = (0.0, 0.0, node.get("approach_height"))
        self._approach_scaling = node.get("approach_velocity_scaling")

    # -- properties --------------------------------------------------------

    @property
    def layout(self):
        return self._layout

    @property
    def docks(self):
        return self._docks

    @property
    def base(self):
        return self._base

    @property
    def holding(self):
        return self._holding

    def tag_id(self, station):
        if station not in self._layout.tag_ids:
            raise SkillFailure("UNKNOWN_STATION", f"no station '{station}' in the cell")
        return self._layout.tag_ids[station]

    def taught_dock(self, station):
        return self._docks.pose(station)

    # -- bring-up ----------------------------------------------------------

    def setup(self):
        """Scene furniture, postures, and the carry configuration."""
        self._scene.wait_for_services()
        self._gripper.wait_for_controller()

        environment = [build_ground(self._layout)]
        for station in STATIONS:
            environment += build_station(
                self._layout, station, self._layout.part_pose(station)
            )
        self._scene.add_objects(environment, colors=COLORS)
        self._logger.info(f"planning scene ready ({len(environment)} surveyed objects)")

        base_x, base_y, _ = self._base.pose
        if not self._arm.wait_for_base(base_x, base_y, timeout=20.0):
            raise PlanningFailure("MoveIt never picked up the initial base pose from TF")

        self._arm.move_named(self._node.get("body_group"), self._node.get("body_posture"))
        self._arm.move_named(self._node.get("head_group"), self._node.get("head_posture"))
        self._gripper.set_gap(self._node.get("gripper_open_gap"))

        # A posture is a joint configuration, so solving it once against the
        # current base pose makes it valid wherever the robot later stands.
        carry = make_pose(self._node.get("carry_tcp"), self._grasp_orientation)
        self._carry_state = self._arm.solve_ik(self._base_to_world(self._tip_pose(carry)))
        if self._carry_state is None:
            self._logger.warning(
                "carry_tcp is not reachable, transiting with the arms down instead"
            )

    # -- geometry helpers --------------------------------------------------

    def _tip_pose(self, tcp_pose):
        return tcp_to_tip(tcp_pose, self._tcp_offset)

    def _grasp_tcp(self, resting_pose):
        return make_pose(
            (
                resting_pose.position.x,
                resting_pose.position.y,
                resting_pose.position.z + self._node.get("grasp_height_offset"),
            ),
            self._grasp_orientation,
        )

    def _base_to_world(self, pose):
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

    def odom_pose_in_frame(self, pose, frame):
        """A taught (x, y, yaw) pose, re-expressed in ``frame`` right now.

        Taught docks live in odom, which in this cell does not drift. A
        navigation goal has to be said in the map frame, so the current
        localization correction is applied at send time - then however far
        AMCL has wandered, the base still physically arrives at the taught
        spot, which is what the tag pipeline and the arm actually need.
        """
        if frame == self._layout.frame_id:
            return pose
        try:
            # Waits out localization coming up: AMCL only publishes map->odom
            # once it has seen a scan, which can lose a race with the first
            # navigation goal after launch.
            message = self._tf_buffer.lookup_transform(
                frame, self._layout.frame_id, Time(), timeout=Duration(seconds=15.0)
            )
        except TransformException as error:
            raise SkillFailure(
                "NAV_NO_TF", f"no {frame} <- {self._layout.frame_id}: {error}"
            ) from error
        x, y, yaw = pose
        matrix = from_transform_msg(message.transform) @ from_rpy(
            (x, y, 0.0), (0.0, 0.0, yaw)
        )
        return (float(matrix[0, 3]), float(matrix[1, 3]), yaw_of(matrix))

    def sync_base(self):
        """Block until MoveIt has caught up with where the base now stands.

        Planning before the virtual joint lands means planning for a robot
        standing somewhere else; and a base that parked itself into collision
        must fail here, before an arm plan is allowed to start inside a bench.
        """
        x, y, _ = self._base.pose
        if not self._arm.wait_for_base(x, y):
            raise SkillFailure(
                "BASE_TF_LOST", "MoveIt never saw the base reach its pose"
            )
        if self._arm.in_collision():
            raise SkillFailure(
                "DOCKED_IN_COLLISION",
                f"robot stands in collision at ({x:.2f}, {y:.2f}); "
                "check dock.offset against the bench",
            )

    # -- observation -------------------------------------------------------

    def look_at_bench(self):
        self._arm.move_named(self._node.get("head_group"), self._node.get("look_posture"))

    def observe_tag(self, station, timeout=None):
        """The station tag's pose in the planning frame, or SkillFailure."""
        try:
            return self._tags.wait_for(
                self.tag_id(station),
                timeout=self._node.get("tag_timeout") if timeout is None else timeout,
            )
        except RuntimeError as error:
            raise SkillFailure("TAG_NOT_VISIBLE", str(error)) from error

    def observe_part(self, station):
        """Part rest pose from the tag, with its deviation from the survey.

        The deviation is diagnostic gold and a guard in one number: it is how
        far the calibration chain disagrees with the surveyed cell, and past a
        threshold the right response is to stop, not to grasp through it.
        """
        self.look_at_bench()
        tag = self.observe_tag(station)
        pose = pose_from_tag(tag, self._layout.part_offset_in_tag_for(station))
        if not self._holding:
            self._workpiece_rest = pose
        truth = self._layout.part_pose(station)
        deviation = math.dist(
            (pose.position.x, pose.position.y, pose.position.z),
            (truth.position.x, truth.position.y, truth.position.z),
        )
        self._logger.info(
            f"{station}: part at ({pose.position.x:.3f}, {pose.position.y:.3f}, "
            f"{pose.position.z:.3f}), {deviation * 1000:.1f} mm from the survey"
        )
        limit = self._node.get("max_survey_deviation")
        if deviation > limit:
            raise SkillFailure(
                "SURVEY_MISMATCH",
                f"tag places the part {deviation * 1000:.0f} mm from the survey "
                f"(limit {limit * 1000:.0f} mm); check the calibration",
            )
        return pose, deviation

    def track_workpiece(self, pose):
        """Put the scene's workpiece where the camera says the real one is."""
        if self._holding:
            raise SkillFailure(
                "ALREADY_HOLDING", "refusing to re-detect while carrying the part"
            )
        if not self._workpiece_in_scene:
            workpiece = build_workpiece(self._layout)
            workpiece.pose = pose
            self._scene.add_objects([workpiece], colors=COLORS)
            self._workpiece_in_scene = True
        else:
            self._scene.move_object(WORKPIECE, self._layout.frame_id, pose)

    def _freeze_grasp_offset(self):
        """Fix the part's pose relative to the wrist at the attach moment.

        Publishing the held part as a world pose recomputed from TF made it
        flicker on camera: that chain updates at 10 Hz with TF latency while
        the arm renders from physics, so the prop jittered around the
        fingers. A rigid wrist-frame offset lets the stage compose the part
        from the same physics pose as the hand - one source, no lag.
        """
        try:
            message = self._tf_buffer.lookup_transform(
                self._layout.frame_id, self._node.get("tip_link"), Time()
            )
        except TransformException:
            self._held_in_tip = None
            return
        tip = from_transform_msg(message.transform)
        tcp = tip @ from_rpy(self._tcp_offset, (0.0, 0.0, 0.0))
        world = from_rpy(
            (
                float(tcp[0, 3]),
                float(tcp[1, 3]),
                float(tcp[2, 3]) - self._node.get("grasp_height_offset"),
            ),
            (0.0, 0.0, 0.0),
        )
        self._held_in_tip = invert(tip) @ world

    def workpiece_world_pose(self):
        """(frame, pose) of the real part, for the Isaac stage to mirror.

        Held: a rigid pose in the wrist frame, composed by the stage from the
        wrist's physics pose. At rest: a world pose where it was last
        observed or released. None before the first observation.
        """
        if self._holding and getattr(self, "_held_in_tip", None) is not None:
            return self._node.get("tip_link"), to_pose(self._held_in_tip)
        if self._workpiece_rest is not None:
            return self._layout.frame_id, self._workpiece_rest
        return None, None

    # -- arm work ----------------------------------------------------------

    def _contact_pairs(self, station):
        """Everything legitimately in contact during a pick or place there.

        The fingers straddle the part between the fixture walls, so gripper
        links against part, fixture and bench top are all real, intended
        contacts for the duration of the approach-release-retreat window -
        and only for that window (plan section 11).
        """
        pairs = [(WORKPIECE, bench_id(station))]
        touch = self._node.get("touch_links")
        for link in touch:
            pairs.append((WORKPIECE, link))
            pairs.append((bench_id(station), link))
        if station == MACHINE:
            pairs.append((WORKPIECE, FIXTURE))
            for link in touch:
                pairs.append((FIXTURE, link))
        return pairs

    def _approach_states(self, resting_pose, label):
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

    def pick(self, station, resting_pose, phase=lambda text: None):
        """Approach from above, close, attach, lift (plan section 9.2)."""
        if self._holding:
            raise SkillFailure("ALREADY_HOLDING", "gripper already carries the part")

        pairs = self._contact_pairs(station)
        # The allowance opens before IK: a grasp between the fixture walls is
        # a colliding state to the checker until these contacts are legal.
        self._scene.set_collisions(pairs, True)
        phase("planning approach")
        try:
            grasp, pregrasp = self._approach_states(resting_pose, f"pick at {station}")
            phase("pre-grasp")
            self._arm.move_to_state(pregrasp, f"{station} pre-grasp")
            phase("descend")
            self._descend(grasp, f"{station} grasp")
            phase("close gripper")
            width = self._layout.workpiece_size[0]
            self._gripper.set_gap(width - self._node.get("grasp_squeeze"))
            self._scene.attach(
                WORKPIECE, self._node.get("attach_link"), self._node.get("touch_links")
            )
            self._holding = True
            self._workpiece_rest = None
            self._freeze_grasp_offset()
            phase("lift")
            self._descend(pregrasp, f"{station} lift")
        finally:
            self._scene.set_collisions(pairs, False)

    def place(self, station, resting_pose, phase=lambda text: None):
        """Approach, open, detach, retreat (plan sections 9.3 and 9.5)."""
        if not self._holding:
            raise SkillFailure("NOT_HOLDING", "no part in the gripper to place")

        pairs = self._contact_pairs(station)
        self._scene.set_collisions(pairs, True)
        phase("planning approach")
        try:
            release, prerelease = self._approach_states(resting_pose, f"place at {station}")
            phase("pre-place")
            self._arm.move_to_state(prerelease, f"{station} pre-place")
            phase("insert")
            self._descend(release, f"{station} insert")
            phase("open gripper")
            self._gripper.set_gap(self._node.get("gripper_open_gap"))
            self._scene.detach(WORKPIECE, self._node.get("attach_link"))
            self._holding = False
            self._workpiece_rest = resting_pose
            phase("retreat")
            try:
                self._descend(prerelease, f"{station} retreat")
            except PlanningFailure as error:
                # The part is placed; failing the skill now would make the
                # retry re-place a part that is no longer in the gripper. The
                # next safe-pose state hauls the arm out with a free-space
                # plan instead.
                self._logger.warning(f"{station} retreat blocked ({error}); "
                                     "leaving the exit to the safe-pose move")
        finally:
            # Keep the placement contacts allowed until the arm has actually
            # left: with the fingers still straddling the placed part, closing
            # them now makes the current state "colliding" and every later
            # plan - including the safe-pose exit - refuses to start.
            self._lingering_pairs = pairs

    def assume_posture(self, posture, phase=lambda text: None):
        """A named travel shape: arms parked when empty, carry when loaded.

        `transport` and `machine_safe` are the same joint shape today; they
        stay distinct postures because on the real cell "safely outside the
        machine envelope" and "safe to drive" are separate certifications with
        separate poses, and the state table already tells them apart.
        """
        if posture not in ("transport", "machine_safe", "home"):
            raise SkillFailure("UNKNOWN_POSTURE", f"no posture named '{posture}'")

        phase(f"assume {posture}")
        if self._holding and self._carry_state is not None:
            self._arm.move_to_state(self._carry_state, "carry")
        else:
            self._arm.move_named(
                self._node.get("park_group"), self._node.get("park_posture")
            )
        if self._lingering_pairs:
            self._scene.set_collisions(self._lingering_pairs, False)
            self._lingering_pairs = []
