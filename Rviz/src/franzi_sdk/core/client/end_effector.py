# -*- coding: utf-8 -*-
"""Direct serial RS485 end effector API (gripper / suction)."""

from __future__ import annotations

from typing import Any, Dict

from core import types
from core.client._base import ArmClientBase


class ArmClientEndEffector(ArmClientBase):
    def GripperInitialize(
        self,
        side: types.SideLike,
        *,
        full_calibration: bool = False,
        timeout_ms: int = 500,
    ) -> int:
        """Initialize a direct-serial gripper on the configured side channel."""
        return self.status_of(self.rpc("EeGripperInitialize", {
            "side": types.side_value(side),
            "full_calibration": bool(full_calibration),
            "timeout_ms": int(timeout_ms),
        }))

    def GripperOpen(
        self,
        side: types.SideLike,
        *,
        timeout_ms: int = 500,
    ) -> int:
        """Open gripper to position 1000."""
        return self.status_of(self.rpc("EeGripperOpen", self._side_req(side, timeout_ms)))

    def GripperClose(
        self,
        side: types.SideLike,
        *,
        timeout_ms: int = 500,
    ) -> int:
        """Close gripper to position 0."""
        return self.status_of(self.rpc("EeGripperClose", self._side_req(side, timeout_ms)))

    def GripperSetPosition(
        self,
        side: types.SideLike,
        position: int,
        *,
        timeout_ms: int = 500,
    ) -> int:
        """Set gripper position, range 0..1000."""
        print(f"Set gripper position to {position}")
        return self.status_of(self.rpc("EeGripperSetPosition", self._ee_value_req(side, position, timeout_ms)))

    def GripperSetForce(
        self,
        side: types.SideLike,
        force: int,
        *,
        timeout_ms: int = 500,
    ) -> int:
        """Set gripper force percent, range 20..100."""
        return self.status_of(self.rpc("EeGripperSetForce", self._ee_value_req(side, force, timeout_ms)))

    def GripperReadPosition(
        self,
        side: types.SideLike,
        *,
        timeout_ms: int = 500,
    ) -> Dict[str, Any]:
        """Read gripper position feedback."""
        return dict(self.rpc("EeGripperReadPosition", self._side_req(side, timeout_ms)))

    def GripperReadForce(
        self,
        side: types.SideLike,
        *,
        timeout_ms: int = 500,
    ) -> Dict[str, Any]:
        """Read gripper force register."""
        return dict(self.rpc("EeGripperReadForce", self._side_req(side, timeout_ms)))

    def GripperReadGripState(
        self,
        side: types.SideLike,
        *,
        timeout_ms: int = 500,
    ) -> Dict[str, Any]:
        """Read gripper grip state: 0 moving, 1 arrived, 2 holding, 3 dropped."""
        return dict(self.rpc("EeGripperReadGripState", self._side_req(side, timeout_ms)))

    def GripperReadInitState(
        self,
        side: types.SideLike,
        *,
        timeout_ms: int = 500,
    ) -> Dict[str, Any]:
        """Read gripper init state: 0 not done, 1 done, 2 running."""
        return dict(self.rpc("EeGripperReadInitState", self._side_req(side, timeout_ms)))

    def GripperGetBinaryState(
        self,
        side: types.SideLike,
        *,
        timeout_ms: int = 500,
    ) -> Dict[str, Any]:
        """Read cached binary gripper state: 0 open, 1 closed."""
        return dict(self.rpc("EeGripperGetBinaryState", self._side_req(side, timeout_ms)))

    def SuctionHold(self, side: types.SideLike, *, timeout_ms: int = 500) -> int:
        """Activate suction on a configured suction side channel."""
        return self.status_of(self.rpc("EeSuctionHold", self._side_req(side, timeout_ms)))

    def SuctionRelease(self, side: types.SideLike, *, timeout_ms: int = 500) -> int:
        """Release suction on a configured suction side channel."""
        return self.status_of(self.rpc("EeSuctionRelease", self._side_req(side, timeout_ms)))

    def SuctionReadState(self, side: types.SideLike, *, timeout_ms: int = 500) -> Dict[str, Any]:
        """Read cached suction state: 0 released, 1 holding."""
        return dict(self.rpc("EeSuctionReadState", self._side_req(side, timeout_ms)))

    @staticmethod
    def _side_req(side: types.SideLike, timeout_ms: int) -> Dict[str, Any]:
        return {
            "side": types.side_value(side),
            "timeout_ms": int(timeout_ms),
        }

    @staticmethod
    def _ee_value_req(side: types.SideLike, value: int, timeout_ms: int) -> Dict[str, Any]:
        return {
            "side": types.side_value(side),
            "value": int(value),
            "timeout_ms": int(timeout_ms),
        }
