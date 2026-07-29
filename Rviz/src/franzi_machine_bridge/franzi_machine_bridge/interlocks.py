"""The conditions that decide what the machine and the robot may do.

Separated from the node so they can be read, reviewed and tested without a ROS
graph. These are transcribed from plan sections 12.4 and 12.5 and are the part
of this system where being conservative costs nothing and being clever costs a
spindle.

Note what is *not* here: this is a software interlock, and software interlocks
do not replace the emergency stop, the safety relay, the light curtain or the
door switch (plan section 12.7). It exists to stop the robot asking for
something impossible, not to make an unsafe machine safe.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Verdict:
    """Allowed, or refused with the specific signal to blame.

    The blocking signal is carried rather than a generic failure, because
    "start refused" sends someone to read code while "start refused:
    clamp_closed" sends them to look at the clamp.
    """

    allowed: bool
    blocked_by: str = ""

    def __bool__(self):
        return self.allowed


def _first_unmet(conditions) -> str:
    for name, satisfied in conditions:
        if not satisfied:
            return name
    return ""


def link_healthy(status) -> Verdict:
    """A stale link is unsafe, not optimistically the last known state."""
    if not status.link_ok:
        return Verdict(False, "link_ok")
    return Verdict(True)


def may_start_machining(status, robot) -> Verdict:
    """Plan section 12.4 - every one of these must hold."""
    healthy = link_healthy(status)
    if not healthy:
        return healthy

    blocker = _first_unmet(
        [
            ("material_detected", status.material_detected),
            ("clamp_closed", status.clamp_closed),
            ("door_closed", status.door_closed),
            ("robot_clear", robot.robot_clear),
            ("alarm", not status.alarm),
            ("emergency_stop", not status.emergency_stop),
        ]
    )
    return Verdict(not blocker, blocker)


def may_enter_machine(status) -> Verdict:
    """Plan section 12.5 - the arm may not cross the envelope otherwise.

    `spindle_stopped` is a positive confirmation from the machine, not the
    absence of `machining`: a spindle coasting down reports neither.
    """
    healthy = link_healthy(status)
    if not healthy:
        return healthy

    blocker = _first_unmet(
        [
            ("spindle_stopped", status.spindle_stopped),
            ("door_open", status.door_open),
            ("clamp_open", status.clamp_open),
            ("alarm", not status.alarm),
            ("emergency_stop", not status.emergency_stop),
        ]
    )
    return Verdict(not blocker, blocker)


def may_move_door(status) -> Verdict:
    """Refuse to move the door while the spindle is still turning."""
    healthy = link_healthy(status)
    if not healthy:
        return healthy

    blocker = _first_unmet(
        [
            ("spindle_stopped", status.spindle_stopped),
            ("emergency_stop", not status.emergency_stop),
        ]
    )
    return Verdict(not blocker, blocker)
