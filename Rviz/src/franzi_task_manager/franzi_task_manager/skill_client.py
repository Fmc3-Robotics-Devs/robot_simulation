"""Calling the skill layer from the task manager.

Each skill is a ROS 2 action so it can report progress, be cancelled and be
timed out (plan section 3). This wrapper hides the goal-handle dance and gives
the state machine one shape of answer: succeeded, or failed with a code.

A skill that is not running at all is reported as SKILL_UNAVAILABLE rather than
waited on forever - a missing server is a deployment fault and should look like
one immediately.
"""

import time

from franzi_engraving_interfaces.action import (
    DetectMaterial,
    LoadMachine,
    MoveToSafePose,
    PickMaterial,
    PlaceProduct,
    PreciseDock,
    UnloadMachine,
)
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient

# Skill name in the state table -> action type and server name.
SKILLS = {
    "precise_dock": (PreciseDock, "precise_dock"),
    "detect_material": (DetectMaterial, "detect_material"),
    "pick_material": (PickMaterial, "pick_material"),
    "load_machine": (LoadMachine, "load_machine"),
    "unload_machine": (UnloadMachine, "unload_machine"),
    "place_product": (PlaceProduct, "place_product"),
    "move_safe": (MoveToSafePose, "move_to_safe_pose"),
    # Coarse navigation is Nav2's NavigateToPose on hardware. Until that is
    # wired, the docking skill is asked to drive to the station outright.
    "navigate": (PreciseDock, "navigate_to_station"),
}


class SkillClient:
    def __init__(self, node, callback_group=None):
        self._node = node
        self._logger = node.get_logger()
        self._clients = {
            name: ActionClient(node, action_type, server, callback_group=callback_group)
            for name, (action_type, server) in SKILLS.items()
        }
        # Results that later states need: the grasp pose DetectMaterial found.
        self.last_grasp_pose = PoseStamped()

    def _goal_for(self, name, action):
        action_type, _ = SKILLS[name]
        goal = action_type.Goal()
        if hasattr(goal, "station"):
            goal.station = action.get("station", "")
        if hasattr(goal, "posture"):
            goal.posture = action.get("posture", "transport")
        if hasattr(goal, "slot"):
            goal.slot = action.get("slot", "")
        if hasattr(goal, "grasp_pose"):
            goal.grasp_pose = self.last_grasp_pose
        return goal

    def run(self, action, timeout, abort):
        name = action.get("skill", "")
        if name not in self._clients:
            return False, f"UNKNOWN_SKILL:{name}"

        client = self._clients[name]
        if not client.wait_for_server(timeout_sec=5.0):
            return False, "SKILL_UNAVAILABLE"

        send = client.send_goal_async(self._goal_for(name, action))
        handle = self._await(send, timeout, abort)
        if handle is None:
            return False, "TIMEOUT"
        if not handle.accepted:
            return False, "GOAL_REJECTED"

        result = self._await(handle.get_result_async(), timeout, abort)
        if result is None:
            # Ask the skill to stop, and wait for it to actually finish:
            # retrying while the old goal still holds the arm gets the retry
            # rejected, which turns one timeout into a phantom second failure.
            handle.cancel_goal_async()
            if self._await(handle.get_result_async(), 10.0, abort) is None:
                self._logger.error("skill ignored cancellation for 10s")
            return False, "TIMEOUT"

        outcome = result.result
        if getattr(outcome, "success", False):
            if hasattr(outcome, "grasp_pose"):
                self.last_grasp_pose = outcome.grasp_pose
            return True, ""
        return False, getattr(outcome, "error_code", "") or "SKILL_FAILED"

    def _await(self, future, timeout, abort):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            if abort.is_set():
                return None
            time.sleep(0.02)
        return future.result() if future.done() else None
