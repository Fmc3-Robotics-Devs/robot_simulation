"""
teleop 域 API
load_ident、关节/笛卡尔遥操作。
"""

from __future__ import annotations

from core.client._base import ArmClientBase
from typing import Any, Dict, Optional, Sequence, Tuple

from core import types


class ArmClientTeleop(ArmClientBase):
    # ==================== 负载辨识 / 遥操作 ====================

    def RunLoadIdent(
        self,
        side: types.SideLike,
        unload_path: str = "",
        load_path: str = "",
        options: Optional[types.LoadIdentOptions] = None,
    ) -> Tuple[int, Dict[str, Any]]:
        """负载辨识（阻塞）。

        服务端 RPC: ``arm_sdk.RunLoadIdent``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right
        路径与 options 见 LoadIdentOptions

        返回:
        tuple (status, result dict)


        说明:
        仅支持 Left/Right。"""
        req = {
            "side": types.side_value(side),
            "unload_path": unload_path,
            "load_path": load_path,
            "options": options or types.default_load_ident_options(),
        }
        rsp = self.rpc("RunLoadIdent", req)
        return self.status_of(rsp), dict(rsp.get("result", {}))

    def TeleJoint(self, side: types.SideLike) -> int:
        """启动关节遥操作。

        服务端 RPC: ``arm_sdk.TeleJoint``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(self.rpc("TeleJoint", {"side": types.side_value(side)}))

    def PushJointTeleopQueue(
        self, side: types.SideLike, target_joints: Sequence[float]
    ) -> int:
        """遥操作队列推送（高频流式）。

        服务端 RPC: ``arm_sdk.PushJointTeleopQueue``

        参数:
        ``side``: 0=Left, 1=Right（与 C++ SdkArmSide 一致）
        ``target_joints``: 单点 7 维关节角（rad），非路径二维列表

        返回:
        int —— SdkStatus，0 表示成功
        """
        joints = list(target_joints)
        if joints and isinstance(joints[0], (list, tuple)):
            raise ValueError(
                "PushJointTeleopQueue 需要 7 维关节角一维列表，"
                "请勿传入 joint_path 二维路径；应对每个路点单独调用"
            )
        return self.status_of(
            self.rpc("PushJointTeleopQueue", {"side": types.side_value(side), "joints": joints})
        )

    def TeleCartesian(self, side: types.SideLike) -> int:
        """启动笛卡尔遥操作。

        服务端 RPC: ``arm_sdk.TeleCartesian``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(self.rpc("TeleCartesian", {"side": types.side_value(side)}))

    def StopTeleCartesian(self, side: types.SideLike) -> int:
        """停止笛卡尔遥操作。

        服务端 RPC: ``arm_sdk.StopTeleCartesian``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(self.rpc("StopTeleCartesian", {"side": types.side_value(side)}))

    def TeleCartesianDual(self) -> int:
        """启动双臂笛卡尔遥操作。

        服务端 RPC: ``arm_sdk.TeleCartesianDual``

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(self.rpc("TeleCartesianDual"))

    def StopTeleCartesianDual(self) -> int:
        """停止双臂笛卡尔遥操作。

        服务端 RPC: ``arm_sdk.StopTeleCartesianDual``

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(self.rpc("StopTeleCartesianDual"))

    def PushCartesianTeleopQueue(
        self, side: types.SideLike, target_pose: Sequence[float]
    ) -> int:
        """笛卡尔遥操作队列推送（高频流式）。

        服务端 RPC: ``arm_sdk.PushCartesianTeleopQueue``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right
        ``target_pose``: 6D 位姿 [x, y, z, rx, ry, rz]（m + rad）

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(
            self.rpc(
                "PushCartesianTeleopQueue",
                {"side": types.side_value(side), "pose": list(target_pose)},
            )
        )

    def PushCartesianTeleopQueueDual(
        self,
        left_pose: Sequence[float],
        right_pose: Sequence[float],
    ) -> int:
        """双臂笛卡尔遥操作队列推送（高频流式）。

        服务端 RPC: ``arm_sdk.PushCartesianTeleopQueueDual``

        参数:
        ``left_pose`` / ``right_pose``: 各 6D 位姿 [x, y, z, rx, ry, rz]（m + rad）

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(
            self.rpc(
                "PushCartesianTeleopQueueDual",
                {"left": list(left_pose), "right": list(right_pose)},
            )
        )

