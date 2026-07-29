"""The docking loop, exercised the way a robot would misbehave."""

import math

import pytest

from franzi_skills.dock_controller import (
    ALIGN_POSITION,
    ALIGN_YAW,
    DOCKED,
    FINE_ADJUST,
    LOST,
    SEARCH_TAG,
    SETTLE,
    Decision,
    DockGains,
    DockLoop,
    dock_frame_error,
    shortest_angle,
)

GAINS = DockGains(stable_frames=3, lost_tolerance=2, max_retreats=2)


def drive(loop, base, target, steps=100):
    """Apply every step the loop asks for until it settles or fails."""
    decision = None
    for _ in range(steps):
        decision = loop.step(base, target)
        if decision.docked or decision.failed:
            return base, decision
        if decision.step:
            base = (
                base[0] + decision.step[0],
                base[1] + decision.step[1],
                base[2] + decision.step[2],
            )
    return base, decision


class TestAngles:
    def test_wraps_at_pi(self):
        # Plan section 8.3: naive subtraction explodes at the +/- pi seam.
        assert shortest_angle(math.pi - 0.1, -math.pi + 0.1) == pytest.approx(-0.2)

    def test_zero(self):
        assert shortest_angle(1.0, 1.0) == 0.0


class TestDockFrameError:
    def test_expressed_in_dock_frame(self):
        # Base one metre short of a dock that faces +y: the shortfall must
        # appear on the dock's x axis, not the world's.
        error = dock_frame_error((0.0, -1.0, math.pi / 2), (0.0, 0.0, math.pi / 2))
        assert error[0] == pytest.approx(1.0)
        assert error[1] == pytest.approx(0.0, abs=1e-9)

    def test_yaw_error_is_shortest(self):
        error = dock_frame_error((0.0, 0.0, math.pi - 0.05), (0.0, 0.0, -math.pi + 0.05))
        assert error[2] == pytest.approx(0.1)


class TestConvergence:
    def test_docks_from_coarse_arrival(self):
        loop = DockLoop(GAINS)
        base, decision = drive(loop, (0.05, -0.04, 0.05), (0.0, 0.0, 0.0))
        assert decision.docked
        assert math.hypot(base[0], base[1]) <= GAINS.position_tolerance

    def test_docked_needs_consecutive_frames(self):
        loop = DockLoop(GAINS)
        on_target = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        assert loop.step(*on_target).phase == SETTLE
        assert loop.step(*on_target).phase == SETTLE
        assert loop.step(*on_target).docked

    def test_bad_frame_restarts_the_count(self):
        loop = DockLoop(GAINS)
        on_target = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        loop.step(*on_target)
        loop.step(*on_target)
        loop.step((0.5, 0.0, 0.0), (0.0, 0.0, 0.0))  # kicked away
        decision = loop.step(*on_target)
        assert decision.phase == SETTLE and decision.stable_count == 1

    def test_step_is_clamped(self):
        loop = DockLoop(GAINS)
        decision = loop.step((2.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        assert math.hypot(*decision.step[:2]) <= GAINS.max_linear_step + 1e-9

    def test_dead_band_keeps_base_still(self):
        gains = DockGains(stable_frames=3, dead_band=0.002, position_tolerance=0.0001)
        loop = DockLoop(gains)
        # Inside the dead band but outside the (absurdly tight) tolerance:
        # the loop must not command a twitch.
        decision = loop.step((0.001, 0.0, 0.0), (0.0, 0.0, 0.0))
        assert decision.step is None

    def test_phases_follow_error_magnitude(self):
        loop = DockLoop(GAINS)
        assert loop.step((0.0, 0.0, 1.0), (0.0, 0.0, 0.0)).phase == ALIGN_YAW
        loop = DockLoop(GAINS)
        assert loop.step((0.5, 0.0, 0.0), (0.0, 0.0, 0.0)).phase == ALIGN_POSITION
        loop = DockLoop(GAINS)
        assert loop.step((0.02, 0.0, 0.0), (0.0, 0.0, 0.0)).phase == FINE_ADJUST


class TestTagLoss:
    def test_short_dropout_waits(self):
        loop = DockLoop(GAINS)
        assert loop.step((0.0, 0.0, 0.0), None).phase == SEARCH_TAG
        assert loop.step((0.0, 0.0, 0.0), None).phase == SEARCH_TAG

    def test_long_loss_retreats_then_fails(self):
        loop = DockLoop(GAINS)
        retreats = 0
        for _ in range(50):
            decision = loop.step((0.0, 0.0, 0.0), None)
            if decision.failed:
                assert decision.error_code == "TAG_LOST"
                assert retreats == GAINS.max_retreats
                return
            if decision.retreat:
                retreats += 1
        raise AssertionError("loss never escalated to failure")

    def test_reacquisition_resets_the_clock(self):
        loop = DockLoop(GAINS)
        loop.step((0.0, 0.0, 0.0), None)
        loop.step((0.0, 0.0, 0.0), None)
        # Seen again: the loss count starts over, no retreat yet.
        decision = loop.step((0.1, 0.0, 0.0), (0.0, 0.0, 0.0))
        assert not decision.retreat
        assert loop.step((0.0, 0.0, 0.0), None).phase == SEARCH_TAG

    def test_runaway_iterations_fail(self):
        loop = DockLoop(DockGains(max_iterations=5))
        decision = Decision(SEARCH_TAG)
        for _ in range(10):
            decision = loop.step((1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            if decision.failed:
                break
        assert decision.failed and decision.error_code == "DOCK_DIVERGED"
