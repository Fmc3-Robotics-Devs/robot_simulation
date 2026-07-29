"""
feedback 域 API
motion_control_module：反馈/期望/运动参数。
"""

from __future__ import annotations

from core.client._base import ArmClientBase
from typing import List

from core import types


class ArmClientFeedback(ArmClientBase):
    # ==================== 反馈 / 期望 ====================

    def GetCurJointPos(self, side: types.SideLike) -> List[float]:
        """反馈关节位置。

        服务端 RPC: ``arm_sdk.GetCurJointPos``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        list[float] rad
        """
        v = self.rpc("GetCurJointPos", types.side_req(side)).get("value")
        return list(v or [])

    def GetCurJointVel(self, side: types.SideLike) -> List[float]:
        """反馈关节速度。

        服务端 RPC: ``arm_sdk.GetCurJointVel``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        list[float]
        """
        v = self.rpc("GetCurJointVel", types.side_req(side)).get("value")
        return list(v or [])

    def GetDesireJointPos(self, side: types.SideLike) -> List[float]:
        """期望关节位置。

        服务端 RPC: ``arm_sdk.GetDesireJointPos``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        list[float]
        """
        v = self.rpc("GetDesireJointPos", types.side_req(side)).get("value")
        return list(v or [])

    def GetDesireJointVel(self, side: types.SideLike) -> List[float]:
        """期望关节速度。

        服务端 RPC: ``arm_sdk.GetDesireJointVel``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        list[float]
        """
        v = self.rpc("GetDesireJointVel", types.side_req(side)).get("value")
        return list(v or [])

    def GetDesireJointAcc(self, side: types.SideLike) -> List[float]:
        """期望关节加速度。

        服务端 RPC: ``arm_sdk.GetDesireJointAcc``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        list[float]
        """
        v = self.rpc("GetDesireJointAcc", types.side_req(side)).get("value")
        return list(v or [])

    def GetJointMotionPara(self, side: types.SideLike) -> List[float]:
        """关节运动参数 (acc, vel, jerk)。

        服务端 RPC: ``arm_sdk.GetJointMotionPara``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        list[float]
        """
        v = self.rpc("GetJointMotionPara", types.side_req(side)).get("value")
        return list(v or [])

    def GetCartesianMotionPara(self, side: types.SideLike) -> List[float]:
        """笛卡尔运动参数。

        服务端 RPC: ``arm_sdk.GetCartesianMotionPara``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        list[float]
        """
        v = self.rpc("GetCartesianMotionPara", types.side_req(side)).get("value")
        return list(v or [])

    def GetMotionStatus(self, side: types.SideLike) -> int:
        """机械臂运动状态。

        服务端 RPC: ``arm_sdk.GetMotionStatus``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        int: 0 停止且到位, 1 运动中, 2 停止未到位
        """
        v = self.rpc("GetMotionStatus", types.side_req(side)).get("value")
        return int(v or 0)

