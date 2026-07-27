"""
limits 域 API
conf_parameter_module：安装角、碰撞、速度加速度限位。
"""

from __future__ import annotations

from core.client._base import ArmClientBase
from typing import List, Sequence, Tuple

from core import types


class ArmClientLimits(ArmClientBase):
    # ==================== 安装角 / 碰撞 / 限位 ====================

    def GetInstallAngle(self, side: types.SideLike) -> Tuple[int, float, float]:
        """读取安装角 gamma/beta（rad）。

        服务端 RPC: ``arm_sdk.GetInstallAngle``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        tuple (status, gamma, beta)
        """
        rsp = self.rpc("GetInstallAngle", types.side_req(side))
        return self.status_of(rsp), float(rsp.get("gamma", 0)), float(rsp.get("beta", 0))

    def SetInstallAngle(self, side: types.SideLike, gamma: float, beta: float) -> int:
        """设置安装角。

        服务端 RPC: ``arm_sdk.SetInstallAngle``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(self.rpc("SetInstallAngle", {"side": types.side_value(side), "gamma": gamma, "beta": beta}))

    def GetCollisionThreshold(self, side: types.SideLike) -> List[float]:
        """读取碰撞检测阈值（7 轴）。

        服务端 RPC: ``arm_sdk.GetCollisionThreshold``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        list[float]
        """
        rsp = dict(self.rpc("GetCollisionThreshold", types.side_req(side)))
        return list(rsp.get("collision_threshold", []))

    def SetCollisionThreshold(
        self, side: types.SideLike, threshold: Sequence[float]
    ) -> int:
        """设置碰撞阈值。

        服务端 RPC: ``arm_sdk.SetCollisionThreshold``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right
        ``threshold``: 长度 7

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(self.rpc("SetCollisionThreshold", {
                "side": types.side_value(side),
                "collision_threshold": list(threshold),
            }))

    def GetJointVelLimit(self, side: types.SideLike) -> List[float]:
        """读取关节速度上限。

        服务端 RPC: ``arm_sdk.GetJointVelLimit``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        list[float]
        """
        v = self.rpc("GetJointVelLimit", types.side_req(side)).get("value")
        return list(v or [])

    def GetJointAccLimit(self, side: types.SideLike) -> List[float]:
        """读取关节加速度上限。

        服务端 RPC: ``arm_sdk.GetJointAccLimit``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        list[float]
        """
        v = self.rpc("GetJointAccLimit", types.side_req(side)).get("value")
        return list(v or [])

    def GetCartesianVelLimit(self, side: types.SideLike) -> List[float]:
        """读取笛卡尔速度上限。

        服务端 RPC: ``arm_sdk.GetCartesianVelLimit``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        list[float] —— 长度 6：[vx, vy, vz, wx, wy, wz]（m/s, rad/s），effective 值
        """
        v = self.rpc("GetCartesianVelLimit", types.side_req(side)).get("value")
        return list(v or [])

    def GetCartesianAccLimit(self, side: types.SideLike) -> List[float]:
        """读取笛卡尔加速度上限。

        服务端 RPC: ``arm_sdk.GetCartesianAccLimit``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        list[float] —— 长度 6：[ax, ay, az, alphax, alphay, alphaz]（m/s², rad/s²），effective 值
        """
        v = self.rpc("GetCartesianAccLimit", types.side_req(side)).get("value")
        return list(v or [])

    def GetJointVelocityRatio(self, side: types.SideLike) -> int:
        """关节速度比例 1~100。

        服务端 RPC: ``arm_sdk.GetJointVelocityRatio``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        int
        """
        v = self.rpc("GetJointVelocityRatio", types.side_req(side)).get("value")
        return int(v or 0)

    def SetJointVelocityRatio(self, side: types.SideLike, ratio: int) -> int:
        """设置速度比例。

        服务端 RPC: ``arm_sdk.SetJointVelocityRatio``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right
        ``ratio``: 1~100

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(self.rpc("SetJointVelocityRatio", {"side": types.side_value(side), "ratio": int(ratio)}))
