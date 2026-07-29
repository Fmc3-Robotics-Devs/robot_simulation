#!/usr/bin/env python3
"""The skill layer: every arm and base capability, each behind a ROS 2 action.

The task manager sequences these; it never touches MoveIt, TF or the base
directly (plan sections 3 and 4.2). Each skill reports progress, honours
cancellation between motion segments, and fails with an error code instead of
a stack trace, because the caller's recovery table routes on codes.

One skill runs at a time. The servers share one arm, one base and one planning
scene, so a second goal is rejected outright rather than queued behind state
it would invalidate.

Machine safety is checked here as well as in the bridge (plan section 12.5):
the bridge refuses to actuate into an unsafe machine, and this layer refuses
to move the arm into one. Neither trusts the other to have been consulted.
"""

import math
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool

from franzi_engraving_interfaces.action import (
    DetectMaterial,
    LoadMachine,
    MoveToSafePose,
    PickMaterial,
    PlaceProduct,
    PreciseDock,
    UnloadMachine,
)
from franzi_engraving_interfaces.msg import DockStatus, MachineStatus
from franzi_machine_bridge.interlocks import may_enter_machine
from franzi_pick_place.base import MobileBase
from franzi_pick_place.gripper import GripperError
from franzi_pick_place.motion import PlanningFailure
from franzi_pick_place.params import build_moveit, declare_cell_parameters
from franzi_pick_place.transforms import from_pose, from_rpy, invert, yaw_of

from .dock_controller import DockGains, DockLoop
from .slots import SlotBook, TrayFull
from .tending import SkillFailure, TendingCell

SKILL_DEFAULTS = {
    # Coarse navigation through Nav2's NavigateToPose instead of the kinematic
    # base's teleport-drive. Needs the localization + navigation stack up.
    "use_nav2": False,
    # The frame navigation goals are expressed in. Taught docks are re-taught
    # in this frame once a map exists; while map and odom coincide (fresh map,
    # no drift) the same numbers serve both.
    "nav_frame": "map",
    # The mock detector stands in for apriltag_ros; Isaac and hardware turn it
    # off and publish real detections into the same TF frames.
    "mock_tags.enabled": True,
    "tag_settle": 0.3,
    "max_survey_deviation": 0.10,
    # How long the loaded part may take to register on the machine's presence
    # sensor (plan section 12.6).
    "material_wait": 3.0,
    "dock.position_tolerance": 0.008,
    "dock.yaw_tolerance_deg": 0.8,
    # Each observation costs a settle period, so "frames" are slow here; the
    # hardware profile raises this back towards the plan's fifteen.
    "dock.stable_frames": 3,
    "dock.max_linear_step": 0.15,
    "dock.max_yaw_step_deg": 12.0,
    "dock.dead_band": 0.002,
    "dock.lost_tolerance": 2,
    "dock.retreat_step": 0.08,
    "dock.max_retreats": 3,
    "dock.max_iterations": 60,
    "dock.approach_speed": 0.08,
    "dock.observe_timeout": 1.5,
    "tray.rows": 1,
    "tray.columns": 2,
    "tray.pitch": [0.12, 0.12],
    # Where the robot wakes up: backed off the named station's taught dock.
    "initial_station": "machine",
}


class SkillCancelled(RuntimeError):
    pass


class SkillServer(Node):
    def __init__(self):
        super().__init__("skill_server")
        declare_cell_parameters(self)
        for name, value in SKILL_DEFAULTS.items():
            self.declare_parameter(name, value)

    def get(self, name):
        return self.get_parameter(name).value


class MachineWatch:
    """The bridge's status stream, held for interlock checks.

    Starts from a default message, whose ``link_ok`` is False: a machine that
    has never spoken is treated exactly like one that stopped speaking.
    """

    def __init__(self, node, callback_group):
        self._lock = threading.Lock()
        self._status = MachineStatus()
        node.create_subscription(
            MachineStatus,
            "machine/status",
            self._on_status,
            10,
            callback_group=callback_group,
        )

    def _on_status(self, message):
        with self._lock:
            self._status = message

    @property
    def status(self):
        with self._lock:
            return self._status

    def require_entry_permission(self):
        """Plan section 12.5, enforced at the arm as well as at the machine."""
        verdict = may_enter_machine(self.status)
        if not verdict:
            raise SkillFailure(
                "INTERLOCK",
                f"machine forbids entry: {verdict.blocked_by}",
            )

    def wait_material(self, present, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.status.material_detected == present:
                return True
            time.sleep(0.05)
        return False


class Skills:
    """The action servers, sharing one cell and one busy flag."""

    def __init__(self, node, cell, base):
        self._node = node
        self._logger = node.get_logger()
        self._cell = cell
        self._base = base
        self._busy = threading.Lock()
        self._group = ReentrantCallbackGroup()
        self._watch = MachineWatch(node, self._group)
        self._tray = SlotBook(
            rows=node.get("tray.rows"),
            columns=node.get("tray.columns"),
            pitch=tuple(node.get("tray.pitch")),
        )
        # The seam to the simulated machine's presence sensor. On hardware
        # nothing publishes this; the machine's own sensor reports through the
        # bridge's MachineIO instead.
        self._material = node.create_publisher(Bool, "machine/sim/material_present", 1)

        # Where the real workpiece is, for the Isaac stage to move its prop.
        # Simulation-only, like the material seam: hardware has a real part.
        self._workpiece_publisher = node.create_publisher(PoseStamped, "workpiece/pose", 10)
        node.create_timer(0.1, self._publish_workpiece, callback_group=self._group)

        self._nav2 = None
        if node.get("use_nav2"):
            from nav2_msgs.action import NavigateToPose

            self._nav2 = ActionClient(
                node, NavigateToPose, "navigate_to_pose", callback_group=self._group
            )

        self._servers = [
            self._serve("navigate_to_station", PreciseDock, self._navigate),
            self._serve("precise_dock", PreciseDock, self._dock),
            self._serve("detect_material", DetectMaterial, self._detect),
            self._serve("pick_material", PickMaterial, self._pick),
            self._serve("load_machine", LoadMachine, self._load),
            self._serve("unload_machine", UnloadMachine, self._unload),
            self._serve("place_product", PlaceProduct, self._place),
            self._serve("move_to_safe_pose", MoveToSafePose, self._move_safe),
        ]

    # -- server plumbing ---------------------------------------------------

    def _serve(self, name, action_type, execute):
        def wrapped(handle):
            with self._busy:
                result = action_type.Result()
                label = f"{name}({self._goal_summary(handle.request)})"
                self._logger.info(f"skill {label} started")
                try:
                    execute(handle, result)
                    result.success = True
                    handle.succeed()
                    self._logger.info(f"skill {label} succeeded")
                except SkillCancelled:
                    result.success = False
                    result.error_code = "CANCELLED"
                    handle.canceled()
                    self._logger.warning(f"skill {label} cancelled")
                except SkillFailure as error:
                    result.success = False
                    result.error_code = error.error_code
                    handle.abort()
                    self._logger.error(f"skill {label} failed: {error}")
                except (PlanningFailure, GripperError, TrayFull) as error:
                    result.success = False
                    result.error_code = {
                        PlanningFailure: "PLANNING_FAILED",
                        GripperError: "GRIPPER_FAULT",
                        TrayFull: "TRAY_FULL",
                    }[type(error)]
                    handle.abort()
                    self._logger.error(f"skill {label} failed: {error}")
                return result

        return ActionServer(
            self._node,
            action_type,
            name,
            execute_callback=wrapped,
            goal_callback=self._on_goal,
            cancel_callback=lambda _handle: CancelResponse.ACCEPT,
            callback_group=self._group,
        )

    @staticmethod
    def _goal_summary(request):
        for field in ("station", "posture"):
            value = getattr(request, field, "")
            if value:
                return value
        return ""

    def _on_goal(self, _request):
        # One arm, one base: a concurrent skill would fight over both.
        if self._busy.locked():
            self._logger.warning("skill rejected: another skill is running")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _phase_reporter(self, handle, feedback):
        def report(text):
            if handle.is_cancel_requested:
                raise SkillCancelled()
            feedback.phase = text
            handle.publish_feedback(feedback)

        return report

    def _set_material(self, present):
        self._material.publish(Bool(data=present))

    def _publish_workpiece(self):
        frame, pose = self._cell.workpiece_world_pose()
        if pose is None:
            return
        message = PoseStamped()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.header.frame_id = frame
        message.pose = pose
        self._workpiece_publisher.publish(message)

    # -- navigation and docking -------------------------------------------

    def _navigate(self, handle, result):
        """Coarse navigation to a station's taught dock.

        With `use_nav2` this is a Nav2 ``NavigateToPose`` goal - map, costmaps,
        recovery behaviours, the real thing. Without it the kinematic base
        drives there directly, arrival scatter and all. Either way the precise
        dock afterwards owes nothing to how the base arrived."""
        station = handle.request.station
        pose = self._cell.taught_dock(station)
        self._report_dock(handle, DockStatus.SEARCH_TAG, station)
        if handle.is_cancel_requested:
            raise SkillCancelled()
        if self._nav2 is not None:
            self._navigate_nav2(handle, station, pose)
        else:
            self._base.drive_to(*pose, label=f"navigate to {station}")
        self._cell.sync_base()
        self._report_dock(handle, DockStatus.DOCKED, station)

    def _navigate_nav2(self, handle, station, pose):
        # A goal can bounce off a stack that is technically active but still
        # warming up - a planner mid-activation, a TF tree one frame short.
        # Those are seconds-scale conditions, so absorb them here instead of
        # failing the whole cycle over a launch race.
        last = None
        for attempt in range(4):
            if handle.is_cancel_requested:
                raise SkillCancelled()
            try:
                return self._navigate_nav2_once(handle, station, pose)
            except SkillFailure as error:
                last = error
                self._logger.warning(
                    f"navigate to {station} attempt {attempt + 1} failed "
                    f"({error.error_code}); retrying"
                )
                time.sleep(3.0)
        raise last

    def _navigate_nav2_once(self, handle, station, pose):
        from action_msgs.msg import GoalStatus
        from nav2_msgs.action import NavigateToPose

        if not self._nav2.wait_for_server(timeout_sec=10.0):
            raise SkillFailure("NAV_UNAVAILABLE", "Nav2 is not answering")

        x, y, yaw = self._cell.odom_pose_in_frame(pose, self._node.get("nav_frame"))
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._node.get("nav_frame")
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        send = self._nav2.send_goal_async(goal)
        deadline = time.monotonic() + 10.0
        while not send.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        nav_handle = send.result() if send.done() else None
        if nav_handle is None or not nav_handle.accepted:
            raise SkillFailure("NAV_REJECTED", f"Nav2 refused the goal for {station}")

        result_future = nav_handle.get_result_async()
        while not result_future.done():
            if handle.is_cancel_requested:
                nav_handle.cancel_goal_async()
                raise SkillCancelled()
            time.sleep(0.1)
        status = result_future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            raise SkillFailure(
                "NAV_FAILED", f"NavigateToPose to {station} ended with status {status}"
            )

    def _dock_gains(self, request):
        get = self._node.get
        return DockGains(
            position_tolerance=request.position_tolerance or get("dock.position_tolerance"),
            yaw_tolerance=request.yaw_tolerance
            or math.radians(get("dock.yaw_tolerance_deg")),
            stable_frames=get("dock.stable_frames"),
            max_linear_step=get("dock.max_linear_step"),
            max_yaw_step=math.radians(get("dock.max_yaw_step_deg")),
            dead_band=get("dock.dead_band"),
            lost_tolerance=get("dock.lost_tolerance"),
            retreat_step=get("dock.retreat_step"),
            max_retreats=get("dock.max_retreats"),
            max_iterations=get("dock.max_iterations"),
        )

    def _dock_target_in_tag(self, station):
        """Where the taught dock sits relative to the station's tag.

        Survey data: the nominal tag pose and the taught dock pose, composed
        once. At run time the observed tag replaces the nominal one and the
        product is the dock pose the real world implies (plan section 6.4).
        """
        tag_nominal = from_pose(self._cell.layout.tag_pose(station))
        x, y, yaw = self._cell.taught_dock(station)
        dock = from_rpy((x, y, 0.0), (0.0, 0.0, yaw))
        return invert(tag_nominal) @ dock

    def _report_dock(
        self, handle, phase, station, error=(0.0, 0.0, 0.0), visible=False, stable=0
    ):
        status = DockStatus()
        status.stamp = self._node.get_clock().now().to_msg()
        status.phase = phase
        status.station = station
        try:
            status.tag_id = self._cell.tag_id(station)
        except SkillFailure:
            status.tag_id = -1  # taught pose without a tag, e.g. home
        status.tag_visible = visible
        status.error_x, status.error_y, status.error_yaw = error
        status.frames_in_tolerance = stable
        feedback = PreciseDock.Feedback()
        feedback.status = status
        handle.publish_feedback(feedback)

    def _dock(self, handle, result):
        """Close the base position loop on the station tag (plan section 8)."""
        station = handle.request.station
        gains = self._dock_gains(handle.request)
        loop = DockLoop(gains)
        dock_in_tag = self._dock_target_in_tag(station)
        observe_timeout = self._node.get("dock.observe_timeout")
        speed = self._node.get("dock.approach_speed")

        # Coarse navigation parks within its goal tolerance - a quarter metre
        # and a dozen degrees off - which can leave the tag too small to
        # decode. Close that last stretch on odometry to the taught dock
        # first; the visual loop then only has the odometry residual to fix,
        # which is its actual job.
        x, y, yaw = self._cell.taught_dock(station)
        self._base.drive_to(x, y, yaw, label=f"{station} final approach")

        self._cell.look_at_bench()
        while True:
            if handle.is_cancel_requested:
                raise SkillCancelled()

            target = None
            visible = False
            try:
                tag = self._cell.observe_tag(station, timeout=observe_timeout)
                implied = tag @ dock_in_tag
                target = (implied[0, 3], implied[1, 3], yaw_of(implied))
                visible = True
            except SkillFailure:
                pass

            decision = loop.step(self._base.pose, target)
            self._report_dock(
                handle,
                decision.phase,
                station,
                error=decision.error,
                visible=visible,
                stable=decision.stable_count,
            )
            base_x, base_y, base_yaw = self._base.pose
            error_x, error_y, error_yaw = decision.error
            self._logger.info(
                f"dock {station} {decision.phase}: "
                + (
                    f"target ({target[0]:.3f}, {target[1]:.3f}, "
                    f"{math.degrees(target[2]):.1f}deg) "
                    if target
                    else "no tag "
                )
                + f"base ({base_x:.3f}, {base_y:.3f}, {math.degrees(base_yaw):.1f}deg) "
                f"error ({error_x * 1000:.0f}, {error_y * 1000:.0f}) mm "
                f"{math.degrees(error_yaw):.1f}deg [{decision.stable_count}]"
            )

            if decision.docked:
                error_x, error_y, error_yaw = decision.error
                result.final_error_x = error_x
                result.final_error_y = error_y
                result.final_error_yaw = error_yaw
                self._logger.info(
                    f"docked at {station}: ({error_x * 1000:.1f}, {error_y * 1000:.1f}) mm, "
                    f"{math.degrees(error_yaw):.2f} deg"
                )
                self._cell.sync_base()
                return
            if decision.failed:
                raise SkillFailure(decision.error_code, f"docking at {station} failed")

            if decision.retreat:
                x, y, yaw = self._base.pose
                self._base.drive_to(
                    x - math.cos(yaw) * decision.retreat,
                    y - math.sin(yaw) * decision.retreat,
                    yaw,
                    label=f"{station} dock retreat",
                    exact=True,
                    speed=speed,
                )
            elif decision.step:
                x, y, yaw = self._base.pose
                step_x, step_y, step_yaw = decision.step
                self._base.drive_to(
                    x + step_x,
                    y + step_y,
                    yaw + step_yaw,
                    label=None,
                    exact=True,
                    speed=speed,
                )

    # -- perception --------------------------------------------------------

    def _detect(self, handle, result):
        report = self._phase_reporter(handle, DetectMaterial.Feedback())
        station = handle.request.station or "feeder"
        report(f"observing {station}")
        pose, deviation = self._cell.observe_part(station)
        self._cell.track_workpiece(pose)
        result.grasp_pose.header.frame_id = self._cell.layout.frame_id
        result.grasp_pose.header.stamp = self._node.get_clock().now().to_msg()
        result.grasp_pose.pose = pose
        result.deviation = deviation

    # -- manipulation ------------------------------------------------------

    def _pick(self, handle, result):
        report = self._phase_reporter(handle, PickMaterial.Feedback())
        station = handle.request.station or "feeder"
        given = handle.request.grasp_pose
        if given.header.frame_id:
            if given.header.frame_id != self._cell.layout.frame_id:
                raise SkillFailure(
                    "WRONG_FRAME",
                    f"grasp pose is in '{given.header.frame_id}', "
                    f"cell works in '{self._cell.layout.frame_id}'",
                )
            pose = given.pose
        else:
            report(f"re-observing {station}")
            pose, _ = self._cell.observe_part(station)
            self._cell.track_workpiece(pose)
        self._cell.pick(station, pose, phase=report)
        result.holding = self._cell.holding

    def _load(self, handle, result):
        report = self._phase_reporter(handle, LoadMachine.Feedback())
        station = handle.request.station or "machine"

        report("checking interlocks")
        self._watch.require_entry_permission()
        report("locating fixture")
        pose, _ = self._cell.observe_part(station)
        # Re-check right before the arm crosses the envelope: the door state
        # may have changed while the head was busy looking at the bench.
        self._watch.require_entry_permission()
        self._cell.place(station, pose, phase=report)
        self._set_material(True)

        report("waiting for the presence sensor")
        wait = self._node.get("material_wait")
        result.material_detected = self._watch.wait_material(True, wait)
        if not result.material_detected:
            raise SkillFailure(
                "MATERIAL_NOT_DETECTED",
                f"machine never confirmed the part within {wait:.1f}s",
            )

    def _unload(self, handle, result):
        report = self._phase_reporter(handle, UnloadMachine.Feedback())
        station = handle.request.station or "machine"

        report("checking interlocks")
        self._watch.require_entry_permission()
        report("locating finished part")
        pose, _ = self._cell.observe_part(station)
        self._cell.track_workpiece(pose)
        self._watch.require_entry_permission()
        self._cell.pick(station, pose, phase=report)
        self._set_material(False)
        result.holding = self._cell.holding

    def _place(self, handle, result):
        report = self._phase_reporter(handle, PlaceProduct.Feedback())
        station = handle.request.station or "outfeed"

        report(f"observing {station}")
        base_pose, _ = self._cell.observe_part(station)
        slot, (offset_x, offset_y) = self._tray.claim(handle.request.slot)
        target = type(base_pose)()
        target.position.x = base_pose.position.x + offset_x
        target.position.y = base_pose.position.y + offset_y
        target.position.z = base_pose.position.z
        target.orientation = base_pose.orientation
        try:
            self._cell.place(station, target, phase=report)
        except Exception:
            self._tray.release(slot)
            raise
        result.slot_used = slot
        self._logger.info(f"product placed in slot {slot}; free: {self._tray.free}")

    def _move_safe(self, handle, result):
        report = self._phase_reporter(handle, MoveToSafePose.Feedback())
        self._cell.assume_posture(handle.request.posture or "transport", phase=report)


def main():
    rclpy.init()
    node = SkillServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # The base has to publish odom -> moveit_root before MoveIt starts
    # monitoring state, otherwise the planar virtual joint never resolves.
    base = MobileBase(
        node,
        frame_id=node.get("frame_id"),
        wheel_radius=node.get("base.wheel_radius"),
        linear_speed=node.get("base.linear_speed"),
        angular_speed=node.get("base.angular_speed"),
        position_error=node.get("base.arrival_position_error"),
        yaw_error=node.get("base.arrival_yaw_error"),
    )

    exit_code = 0
    try:
        moveit = build_moveit()
        cell = TendingCell(node, moveit, base)

        station = node.get("initial_station")
        x, y, yaw = cell.taught_dock(station)
        # Stations get a stand-off so the cycle can approach them; home *is*
        # the stand-off.
        if station != "home":
            retreat = node.get("dock.standby_retreat")
            x -= math.cos(yaw) * retreat
            y -= math.sin(yaw) * retreat
        base.set_pose(x, y, yaw)

        cell.setup()
        skills = Skills(node, cell, base)  # noqa: F841 - servers live here
        node.get_logger().info("skill server ready")
        spin_thread.join()
    except KeyboardInterrupt:
        pass
    except Exception as error:  # noqa: BLE001 - report bring-up failures
        node.get_logger().error(f"skill server failed to start: {error}")
        exit_code = 1
    finally:
        # MoveItCpp segfaults when unwound with the interpreter; hand the
        # process back to the OS instead.
        import os
        import sys

        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(exit_code)


if __name__ == "__main__":
    main()
