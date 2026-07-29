"""Gripper control.

The gripper is driven straight through its FollowJointTrajectory controller
instead of the planner: closing onto a workpiece is intentionally a "collision"
and there is nothing for a motion planner to figure out on a 2-DoF parallel jaw.

Geometry note - the two fingers sit 95 mm apart at joint zero and travel
*towards* each other, so joint zero is the fully open pose:

    gap = gap_at_zero - 2 * stroke
"""

import threading

from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class GripperError(RuntimeError):
    pass


def _wait_for(future, timeout, what):
    """Block on a future while the node keeps being spun by its executor."""
    done = threading.Event()
    future.add_done_callback(lambda _future: done.set())
    if not done.wait(timeout):
        raise GripperError(f"timed out waiting for {what}")
    return future.result()


class Gripper:
    def __init__(self, node, controller, joints, gap_at_zero, max_stroke):
        if len(joints) != 2:
            raise ValueError("expected exactly two finger joints")
        self._node = node
        self._logger = node.get_logger()
        self._joints = list(joints)
        self._gap_at_zero = gap_at_zero
        self._max_stroke = max_stroke
        self._client = ActionClient(
            node, FollowJointTrajectory, f"{controller}/follow_joint_trajectory"
        )

    def wait_for_controller(self, timeout=30.0):
        if not self._client.wait_for_server(timeout_sec=timeout):
            raise GripperError(f"gripper controller did not come up within {timeout}s")

    def stroke_for_gap(self, gap):
        stroke = (self._gap_at_zero - gap) / 2.0
        return max(0.0, min(self._max_stroke, stroke))

    def set_gap(self, gap, duration=1.0):
        """Command a finger opening in metres, measured between the two pads."""
        stroke = self.stroke_for_gap(gap)

        point = JointTrajectoryPoint()
        point.positions = [stroke, -stroke]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(joint_names=self._joints, points=[point])

        handle = _wait_for(
            self._client.send_goal_async(goal), duration + 10.0, "the gripper goal response"
        )
        if not handle.accepted:
            raise GripperError(f"gripper goal for gap={gap:.3f} m was rejected")

        result = _wait_for(
            handle.get_result_async(), duration + 10.0, "the gripper to finish"
        ).result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise GripperError(f"gripper motion failed with code {result.error_code}")
        self._logger.info(f"gripper gap -> {gap * 1000:.0f} mm (stroke {stroke * 1000:.1f} mm)")
