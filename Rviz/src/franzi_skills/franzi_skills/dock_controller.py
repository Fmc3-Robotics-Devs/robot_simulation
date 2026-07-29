"""The precise-docking control loop, as decisions rather than motion.

Plan sections 8.3 to 8.8: drive the base towards the pose the station tag
implies until the remaining error stays inside tolerance for enough consecutive
observations. This module owns every decision in that loop - error computation,
step clamping, the dead band, the settle count, and what to do when the tag
disappears - and none of the motion. The caller applies the returned step with
whatever base it has: the kinematic simulator here, a velocity controller on
hardware.

Keeping it free of ROS is what makes the tolerances testable: every branch in
this file runs in a plain pytest, including the ones that would need a robot to
misbehave on demand.
"""

import math
from dataclasses import dataclass


def shortest_angle(target, current):
    """Plan section 16.1: the one correct way to subtract angles."""
    return math.atan2(math.sin(target - current), math.cos(target - current))


@dataclass(frozen=True)
class DockGains:
    """Tolerances and limits, plan section 8.7 and the appendix defaults."""

    position_tolerance: float = 0.008
    yaw_tolerance: float = math.radians(0.8)
    # Consecutive in-tolerance observations before DOCKED. The plan suggests 15
    # frames of a 30 FPS stream; each observation here costs a settle period,
    # so the simulation profile lowers it rather than the meaning changing.
    stable_frames: int = 15
    # Longest correction applied from one observation. Small steps keep the
    # loop convergent even when an observation is bad: no single frame can
    # send the base far.
    max_linear_step: float = 0.15
    max_yaw_step: float = math.radians(12.0)
    # Corrections below the dead band are not worth exciting the base for.
    dead_band: float = 0.002
    # How many consecutive lost observations before retreating, and how many
    # retreats before giving up (plan section 14.1).
    lost_tolerance: int = 3
    retreat_step: float = 0.08
    max_retreats: int = 3
    max_iterations: int = 200


# Phases, matching DockStatus.msg.
SEARCH_TAG = "SEARCH_TAG"
ALIGN_YAW = "ALIGN_YAW"
ALIGN_POSITION = "ALIGN_POSITION"
FINE_ADJUST = "FINE_ADJUST"
SETTLE = "SETTLE"
DOCKED = "DOCKED"
LOST = "LOST"


@dataclass(frozen=True)
class Decision:
    phase: str
    # World-frame correction to apply now, or None to stay put.
    step: "tuple[float, float, float] | None" = None
    # Straight-back retreat distance to reacquire a lost tag.
    retreat: float = 0.0
    docked: bool = False
    failed: bool = False
    error_code: str = ""
    # Remaining error in the dock frame, for feedback and the result.
    error: "tuple[float, float, float]" = (0.0, 0.0, 0.0)
    stable_count: int = 0


def dock_frame_error(base_pose, target_pose):
    """Remaining error expressed in the dock's own frame (plan section 8.3).

    In the dock frame "x" is how far short of the station the base stopped and
    "y" is how far off to the side, which is what the tolerances mean. World
    axes would smear both into whichever direction the station happens to face.
    """
    base_x, base_y, base_yaw = base_pose
    target_x, target_y, target_yaw = target_pose
    world_dx = target_x - base_x
    world_dy = target_y - base_y
    cos_yaw = math.cos(target_yaw)
    sin_yaw = math.sin(target_yaw)
    return (
        cos_yaw * world_dx + sin_yaw * world_dy,
        -sin_yaw * world_dx + cos_yaw * world_dy,
        shortest_angle(target_yaw, base_yaw),
    )


class DockLoop:
    """One docking attempt. Feed it observations, apply the steps it returns."""

    def __init__(self, gains: DockGains = None):
        self._gains = gains or DockGains()
        self._stable = 0
        self._lost = 0
        self._retreats = 0
        self._iterations = 0

    def step(self, base_pose, target_pose):
        """Decide from one observation cycle.

        ``target_pose`` is the dock pose the tag currently implies, in the same
        frame as ``base_pose``, or None when the tag was not observed.
        """
        gains = self._gains
        self._iterations += 1
        if self._iterations > gains.max_iterations:
            return Decision(LOST, failed=True, error_code="DOCK_DIVERGED")

        if target_pose is None:
            return self._handle_lost()
        self._lost = 0

        error = dock_frame_error(base_pose, target_pose)
        error_x, error_y, error_yaw = error
        in_position = math.hypot(error_x, error_y) <= gains.position_tolerance
        in_yaw = abs(error_yaw) <= gains.yaw_tolerance

        if in_position and in_yaw:
            self._stable += 1
            if self._stable >= gains.stable_frames:
                return Decision(
                    DOCKED, docked=True, error=error, stable_count=self._stable
                )
            return Decision(SETTLE, error=error, stable_count=self._stable)

        # One bad frame restarts the count: fifteen *consecutive* frames is the
        # promise, not fifteen frames eventually.
        self._stable = 0

        step = self._clamped_step(base_pose, target_pose, error)
        phase = self._phase(error_x, error_y, error_yaw)
        return Decision(phase, step=step, error=error)

    def _handle_lost(self):
        gains = self._gains
        self._stable = 0
        self._lost += 1
        if self._lost <= gains.lost_tolerance:
            return Decision(SEARCH_TAG)

        # Plan section 14.1: back away and look again, a bounded number of
        # times. The tag is usually lost by being too close, not too far.
        self._lost = 0
        self._retreats += 1
        if self._retreats > gains.max_retreats:
            return Decision(LOST, failed=True, error_code="TAG_LOST")
        return Decision(LOST, retreat=gains.retreat_step)

    def _phase(self, error_x, error_y, error_yaw):
        if abs(error_yaw) > 4.0 * self._gains.yaw_tolerance:
            return ALIGN_YAW
        if math.hypot(error_x, error_y) > 4.0 * self._gains.position_tolerance:
            return ALIGN_POSITION
        return FINE_ADJUST

    def _clamped_step(self, base_pose, target_pose, error):
        """The world-frame move towards the target, clamped and dead-banded."""
        gains = self._gains
        world_dx = target_pose[0] - base_pose[0]
        world_dy = target_pose[1] - base_pose[1]
        distance = math.hypot(world_dx, world_dy)

        if distance > gains.max_linear_step:
            scale = gains.max_linear_step / distance
            world_dx *= scale
            world_dy *= scale
        elif distance < gains.dead_band:
            world_dx = world_dy = 0.0

        yaw_step = error[2]
        yaw_step = max(-gains.max_yaw_step, min(gains.max_yaw_step, yaw_step))

        if world_dx == 0.0 and world_dy == 0.0 and yaw_step == 0.0:
            return None
        return (world_dx, world_dy, yaw_step)
