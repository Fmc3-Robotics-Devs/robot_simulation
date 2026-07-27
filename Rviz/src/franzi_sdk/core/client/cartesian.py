"""
cartesian 域 API
robot_system：笛卡尔位姿、编码器计数。
"""

from __future__ import annotations

from core.client._base import ArmClientBase
from typing import List, Tuple

from core import types


class ArmClientCartesian(ArmClientBase):
    # ==================== 笛卡尔 / 编码器 ====================

    def ReadCartesianPose(self, side: types.SideLike) -> List[float]:
        """读取末端 6D 位姿。

        服务端 RPC: ``arm_sdk.ReadCartesianPose``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        list[float]
        """
        rsp = dict(self.rpc("ReadCartesianPose", types.side_req(side)))
        return list(rsp.get("position", []))

    def ReadCartesianPoseDual(self) -> List[float]:
        """读取双臂 12D 位姿。

        服务端 RPC: ``arm_sdk.ReadCartesianPoseDual``

        返回:
        list[float]
        """
        rsp = dict(self.rpc("ReadCartesianPoseDual"))
        return list(rsp.get("position", []))

    def ReadJointPositionCnt(self, side: types.SideLike) -> Tuple[int, List[int]]:
        """读取编码器位置计数 (0x6064)。

        服务端 RPC: ``arm_sdk.ReadJointPositionCnt``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        tuple (status, counts)
        """
        rsp = self.rpc("ReadJointPositionCnt", types.side_req(side))
        return self.status_of(rsp), [int(x) for x in rsp.get("value", [])]

