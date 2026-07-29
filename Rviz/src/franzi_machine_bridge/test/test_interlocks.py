"""Interlock tests.

These run without a ROS graph on purpose: the conditions in plan sections 12.4
and 12.5 are the ones that must not quietly drift, and a test that needs a
simulator to run is a test that stops being run.

Each condition gets its own case rather than one happy path and one sad one,
because the failure that matters is a single signal being dropped from the
list - which a combined test still passes.
"""

import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from franzi_machine_bridge.interlocks import (  # noqa: E402
    may_enter_machine,
    may_move_door,
    may_start_machining,
)


def machine(**overrides):
    state = dict(
        link_ok=True,
        machine_ready=True,
        door_open=False,
        door_closed=True,
        clamp_open=False,
        clamp_closed=True,
        material_detected=True,
        machining=False,
        finished=False,
        spindle_stopped=True,
        alarm=False,
        emergency_stop=False,
    )
    state.update(overrides)
    return SimpleNamespace(**state)


def robot(**overrides):
    state = dict(robot_clear=True, robot_busy=False, robot_fault=False)
    state.update(overrides)
    return SimpleNamespace(**state)


class TestStartMachining:
    def test_allowed_when_everything_holds(self):
        assert may_start_machining(machine(), robot())

    def test_refused_without_material(self):
        verdict = may_start_machining(machine(material_detected=False), robot())
        assert not verdict and verdict.blocked_by == "material_detected"

    def test_refused_with_clamp_open(self):
        verdict = may_start_machining(machine(clamp_closed=False), robot())
        assert not verdict and verdict.blocked_by == "clamp_closed"

    def test_refused_with_door_open(self):
        verdict = may_start_machining(machine(door_closed=False), robot())
        assert not verdict and verdict.blocked_by == "door_closed"

    def test_refused_while_the_arm_is_inside(self):
        verdict = may_start_machining(machine(), robot(robot_clear=False))
        assert not verdict and verdict.blocked_by == "robot_clear"

    def test_refused_on_alarm(self):
        verdict = may_start_machining(machine(alarm=True), robot())
        assert not verdict and verdict.blocked_by == "alarm"

    def test_refused_on_emergency_stop(self):
        verdict = may_start_machining(machine(emergency_stop=True), robot())
        assert not verdict and verdict.blocked_by == "emergency_stop"

    def test_refused_when_the_link_is_stale(self):
        # A machine that has stopped answering is not a machine that is fine.
        verdict = may_start_machining(machine(link_ok=False), robot())
        assert not verdict and verdict.blocked_by == "link_ok"


class TestEnterMachine:
    def test_allowed_when_open_and_stopped(self):
        assert may_enter_machine(machine(door_open=True, clamp_open=True))

    def test_refused_while_the_spindle_turns(self):
        verdict = may_enter_machine(
            machine(door_open=True, clamp_open=True, spindle_stopped=False)
        )
        assert not verdict and verdict.blocked_by == "spindle_stopped"

    def test_refused_with_the_door_shut(self):
        verdict = may_enter_machine(machine(door_open=False, clamp_open=True))
        assert not verdict and verdict.blocked_by == "door_open"

    def test_refused_with_the_clamp_shut(self):
        verdict = may_enter_machine(machine(door_open=True, clamp_open=False))
        assert not verdict and verdict.blocked_by == "clamp_open"

    def test_not_machining_is_not_the_same_as_spindle_stopped(self):
        # A spindle coasting down reports neither, and only the positive
        # confirmation may be trusted.
        coasting = machine(
            door_open=True, clamp_open=True, machining=False, spindle_stopped=False
        )
        assert not may_enter_machine(coasting)


class TestDoor:
    def test_refused_while_the_spindle_turns(self):
        verdict = may_move_door(machine(spindle_stopped=False))
        assert not verdict and verdict.blocked_by == "spindle_stopped"

    def test_allowed_when_stopped(self):
        assert may_move_door(machine())
