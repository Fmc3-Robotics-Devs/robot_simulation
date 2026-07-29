"""
motion_traj 域 API
motion_control_module：关节路径、MOVL、回零。
"""

from __future__ import annotations

from core.client._base import ArmClientBase
from core.libs.inbc_rpc import exceptions as inbc_rpc_exceptions
from typing import Optional, Sequence

from core import types


class ArmClientMotionTraj(ArmClientBase):
    # ==================== 轨迹 ====================

    def MoveJ(
        self,
        joint_path: Sequence[Sequence[float]],
        side: Optional[types.SideLike] = None,
    ) -> int:
        """关节空间路径规划并执行。

        服务端 RPC: ``arm_sdk.MoveJ``

        参数:
        ``joint_path``: 路径点序列；``side=None`` 为双臂

        返回:
        int —— SdkStatus，0 表示成功
        """
        s = types.ARM_SIDE_DUAL if side is None else types.side_value(side)
        return self.status_of(self.rpc("MoveJ", {"side": s, "joint_path": [list(p) for p in joint_path]}))

    def MoveStop(self, side: types.SideLike) -> int:
        """急停当前运动（TrajStopper 减速停车）。

        服务端 RPC: ``arm_sdk.MoveStop``

        参数:
        ``side``: int —— SdkArmSide，0=左臂，1=右臂

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(self.rpc("MoveStop", types.side_req(side)))

    def Home(self, side: Optional[types.SideLike] = None) -> int:
        """回零位（阻塞）。

        服务端 RPC: ``arm_sdk.Home``

        参数:
        ``side=None`` 双臂

        返回:
        int —— SdkStatus，0 表示成功
        """
        s = types.ARM_SIDE_DUAL if side is None else types.side_value(side)
        return self.status_of(self.rpc("Home", {"side": s}))

    def MoveL(
        self,
        cartesian_path: Sequence[Sequence[float]],
        side: Optional[types.SideLike] = None,
        is_sync: bool = True,
    ) -> int:
        """笛卡尔直线 MOVL。

        服务端 RPC: ``arm_sdk.MoveL``

        参数:
        ``side``: int —— 0=左臂，1=右臂（与 C++ SdkArmSide 一致）
        ``cartesian_path``: 6D 路点 [x,y,z,rx,ry,rz]（m + rad），不含起点
        ``is_sync``: True 阻塞至完成

        返回:
        int —— SdkStatus，0 表示成功
        """
        s = types.ARM_SIDE_DUAL if side is None else types.side_value(side)
        payload = {
            "side": s,
            "cartesian_path": [list(p) for p in cartesian_path],
            "is_sync": bool(is_sync),
        }
        # 旧版服务端注册名为 arm_sdk.Movl；新版为 arm_sdk.MoveL
        for method in ("MoveL", "Movl"):
            try:
                return self.status_of(self.rpc(method, payload))
            except inbc_rpc_exceptions.InbcRpcRemoteError as ex:
                if method == "MoveL" and "unknown function" in str(ex).lower():
                    continue
                raise
        return int(types.SdkStatus.UNSUPPORTED)

