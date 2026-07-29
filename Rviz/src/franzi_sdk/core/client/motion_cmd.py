"""
motion_cmd 域 API
motion_control_module：关节位置指令。
"""

from __future__ import annotations

from core.client._base import ArmClientBase
from typing import Optional, Sequence

from core import types


class ArmClientMotionCmd(ArmClientBase):
    # ==================== 关节指令 ====================

    def CommandJointPosition(
        self, joints: Sequence[float], side: Optional[types.SideLike] = None
    ) -> int:
        """下发关节位置指令（瞬时目标）。

        服务端 RPC: ``arm_sdk.CommandJointPosition``

        参数:
        ``joints``: 目标角列表，单臂 7 维、双臂 14 维（rad）
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right；``side=None`` 时使用双臂合并（side=-1）

        返回:
        int —— SdkStatus，0 表示成功
        """
        s = types.ARM_SIDE_DUAL if side is None else types.side_value(side)
        return self.status_of(self.rpc("CommandJointPosition", {"side": s, "joints": list(joints)}))

