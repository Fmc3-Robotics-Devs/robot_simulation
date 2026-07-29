# -*- coding: utf-8 -*-
"""TIO RS485 passthrough and AG gripper API."""

from __future__ import annotations

from typing import Any, Dict, Sequence

from core import types
from core.client._base import ArmClientBase


class ArmClientRs485(ArmClientBase):
    def Rs485Write(
        self,
        side: types.SideLike,
        payload: Sequence[int],
        *,
        frame_id: int = 0,
        timeout_ms: int = 100,
        hold_ms: int = 3,
        expect_reply: bool = True,
    ) -> int:
        """Send raw RS485 payload through a side-mapped TIO channel configured as mode=0."""
        req = {
            "side": types.side_value(side),
            "payload": [int(x) & 0xFF for x in payload],
            "frame_id": int(frame_id),
            "timeout_ms": int(timeout_ms),
            "hold_ms": int(hold_ms),
            "expect_reply": bool(expect_reply),
        }
        return self.status_of(self.rpc("485Write", req))

    def Rs485Read(self, side: types.SideLike, *, consume: bool = True) -> Dict[str, Any]:
        """Read the latest matched RS485 reply; returns has_data=false when empty."""
        return dict(self.rpc("485Read", {"side": types.side_value(side), "consume": bool(consume)}))

    def AgInitialize(
        self,
        side: types.SideLike,
        *,
        rs485_slave_id: int = 1,
        full_calibration: bool = False,
        timeout_ms: int = 500,
    ) -> int:
        """Initialize or calibrate an AG gripper on a side-mapped TIO channel configured as mode=1."""
        return self.status_of(self.rpc("AgInitialize", {
            "side": types.side_value(side),
            "rs485_slave_id": int(rs485_slave_id),
            "full_calibration": bool(full_calibration),
            "timeout_ms": int(timeout_ms),
        }))

    def AgOpen(
        self,
        side: types.SideLike,
        *,
        rs485_slave_id: int = 1,
        timeout_ms: int = 500,
    ) -> int:
        """Command AG gripper opening to 1000."""
        return self.status_of(self.rpc("AgOpen", self._read_req(side, rs485_slave_id, timeout_ms)))

    def AgClose(
        self,
        side: types.SideLike,
        *,
        rs485_slave_id: int = 1,
        timeout_ms: int = 500,
    ) -> int:
        """Command AG gripper closing to 0."""
        return self.status_of(self.rpc("AgClose", self._read_req(side, rs485_slave_id, timeout_ms)))

    def AgSetPosition(
        self,
        side: types.SideLike,
        position: int,
        *,
        rs485_slave_id: int = 1,
        timeout_ms: int = 500,
    ) -> int:
        """Set AG gripper opening, range 0..1000."""
        return self.status_of(self.rpc("AgSetPosition", self._value_req(side, rs485_slave_id, position, timeout_ms)))

    def AgSetForce(
        self,
        side: types.SideLike,
        force: int,
        *,
        rs485_slave_id: int = 1,
        timeout_ms: int = 500,
    ) -> int:
        """Set AG gripper force percent, range 20..100."""
        return self.status_of(self.rpc("AgSetForce", self._value_req(side, rs485_slave_id, force, timeout_ms)))

    def AgReadPosition(
        self,
        side: types.SideLike,
        *,
        rs485_slave_id: int = 1,
        timeout_ms: int = 500,
    ) -> Dict[str, Any]:
        """Read AG gripper position feedback."""
        return dict(self.rpc("AgReadPosition", self._read_req(side, rs485_slave_id, timeout_ms)))

    def AgReadForce(
        self,
        side: types.SideLike,
        *,
        rs485_slave_id: int = 1,
        timeout_ms: int = 500,
    ) -> Dict[str, Any]:
        """Read AG gripper force setting/feedback register."""
        return dict(self.rpc("AgReadForce", self._read_req(side, rs485_slave_id, timeout_ms)))

    def AgReadGripState(
        self,
        side: types.SideLike,
        *,
        rs485_slave_id: int = 1,
        timeout_ms: int = 500,
    ) -> Dict[str, Any]:
        """Read AG gripper grip state: 0 moving, 1 arrived, 2 holding, 3 dropped."""
        return dict(self.rpc("AgReadGripState", self._read_req(side, rs485_slave_id, timeout_ms)))

    def AgReadInitState(
        self,
        side: types.SideLike,
        *,
        rs485_slave_id: int = 1,
        timeout_ms: int = 500,
    ) -> Dict[str, Any]:
        """Read AG gripper init state: 0 not done, 1 done, 2 running."""
        return dict(self.rpc("AgReadInitState", self._read_req(side, rs485_slave_id, timeout_ms)))

    @staticmethod
    def _read_req(side: types.SideLike, rs485_slave_id: int, timeout_ms: int) -> Dict[str, Any]:
        return {
            "side": types.side_value(side),
            "rs485_slave_id": int(rs485_slave_id),
            "timeout_ms": int(timeout_ms),
        }

    @staticmethod
    def _value_req(side: types.SideLike, rs485_slave_id: int, value: int, timeout_ms: int) -> Dict[str, Any]:
        return {
            "side": types.side_value(side),
            "rs485_slave_id": int(rs485_slave_id),
            "value": int(value),
            "timeout_ms": int(timeout_ms),
        }
