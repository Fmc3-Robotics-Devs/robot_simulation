"""Arm motion built on the MoveItPy API.

Goals are always reduced to joint-space targets: the arm is redundant (7 DoF)
and letting the sampling planner chase a Cartesian goal makes the demo
non-deterministic. IK is solved here, checked against the planning scene, and
only then handed to the planner.
"""

import random

from moveit.core.robot_state import RobotState
from moveit.planning import PlanRequestParameters

TRANSFER = "ompl_transfer"
LINEAR = "pilz_lin"


class PlanningFailure(RuntimeError):
    pass


class ArmMotion:
    def __init__(self, moveit, logger, arm_group, tip_link, ik_attempts=25, ik_timeout=0.05):
        self._moveit = moveit
        self._logger = logger
        self._arm_group = arm_group
        self._tip_link = tip_link
        self._ik_attempts = ik_attempts
        self._ik_timeout = ik_timeout
        self._robot_model = moveit.get_robot_model()
        self._psm = moveit.get_planning_scene_monitor()
        self._random = random.Random(0)

    # -- state -------------------------------------------------------------

    def current_state(self):
        """Snapshot of the monitored robot state, detached from the scene lock."""
        with self._psm.read_only() as scene:
            positions = dict(scene.current_state.joint_positions)
        state = RobotState(self._robot_model)
        state.set_to_default_values()
        state.joint_positions = positions
        state.update()
        return state

    def solve_ik(self, pose, seed_state=None, seed_noise=0.8):
        """Return a collision-free RobotState reaching ``pose`` with the tip link.

        ``seed_state`` matters more than it looks: a 7-DoF arm has a continuum
        of solutions, and seeding a pose from its neighbour keeps both on the
        same IK branch. Without that, two poses 12 cm apart can end up in
        postures the arm cannot travel between in a straight line.

        Later attempts perturb the arm joints only, leaving the torso and the
        other arm untouched.
        """
        state = self.current_state()
        if seed_state is not None:
            state.set_joint_group_positions(
                self._arm_group, seed_state.get_joint_group_positions(self._arm_group)
            )
            state.update()
        seed = list(state.get_joint_group_positions(self._arm_group))

        for attempt in range(self._ik_attempts):
            if attempt:
                state.set_joint_group_positions(
                    self._arm_group,
                    [value + self._random.uniform(-seed_noise, seed_noise) for value in seed],
                )
                state.update()

            if not state.set_from_ik(
                self._arm_group, pose, self._tip_link, self._ik_timeout
            ):
                continue

            state.update()
            if self._state_is_valid(state):
                return state

        return None

    def _state_is_valid(self, state):
        with self._psm.read_only() as scene:
            return scene.is_state_valid(state, self._arm_group, False)

    def in_collision(self):
        """Whether the robot as it stands right now is touching anything.

        Collision checking is whole-robot; the group only selects which
        constraints are evaluated, so this covers the chassis too.
        """
        return not self._state_is_valid(self.current_state())

    # -- planning ----------------------------------------------------------

    def _parameters(self, namespace, velocity_scaling=None):
        parameters = PlanRequestParameters(self._moveit, namespace)
        if velocity_scaling is not None:
            parameters.max_velocity_scaling_factor = velocity_scaling
            parameters.max_acceleration_scaling_factor = velocity_scaling
        return parameters

    def _plan_and_execute(self, component, namespace, velocity_scaling=None):
        result = component.plan(
            single_plan_parameters=self._parameters(namespace, velocity_scaling)
        )
        if not result:
            return False
        self._moveit.execute(result.trajectory, controllers=[])
        return True

    def move_to_state(self, goal_state, label, linear=False, velocity_scaling=None):
        """Move the arm to a joint configuration.

        ``linear`` asks Pilz for a straight-line Cartesian motion, which is what
        approach and retreat segments need. It is allowed to fail (the pose may
        be unreachable in a straight line) and then falls back to a free-space
        plan, because a demo that stops moving is harder to debug than one that
        takes a detour.
        """
        component = self._moveit.get_planning_component(self._arm_group)
        component.set_start_state_to_current_state()
        component.set_goal_state(robot_state=goal_state)

        if linear:
            if self._plan_and_execute(component, LINEAR, velocity_scaling):
                return
            self._logger.warning(f"{label}: no straight-line plan, falling back to free space")
            component.set_start_state_to_current_state()
            component.set_goal_state(robot_state=goal_state)

        if not self._plan_and_execute(component, TRANSFER, velocity_scaling):
            raise PlanningFailure(f"{label}: planning failed")

    def solve_approach_pair(self, target_pose, approach_pose, label):
        """Solve a target and its approach pose onto a common IK branch."""
        target = self.solve_ik(target_pose)
        if target is None:
            raise PlanningFailure(f"{label}: no collision-free IK solution for the target")
        approach = self.solve_ik(approach_pose, seed_state=target)
        if approach is None:
            raise PlanningFailure(f"{label}: no collision-free IK solution for the approach")
        return target, approach

    def move_named(self, group, configuration, label=None):
        """Move any planning group to one of its SRDF named states."""
        component = self._moveit.get_planning_component(group)
        component.set_start_state_to_current_state()
        component.set_goal_state(configuration_name=configuration)
        if not self._plan_and_execute(component, TRANSFER):
            raise PlanningFailure(f"{label or group}: cannot reach '{configuration}'")
