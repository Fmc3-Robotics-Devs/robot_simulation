"""
user_loop 域 API
bio_arm_sdk：用户 1ms 控制回路。
"""

from __future__ import annotations

from core.client._base import ArmClientBase

class ArmClientUserLoop(ArmClientBase):
    # ==================== 用户控制回路 ====================

    def EnableUserControlLoop(self, enable: bool) -> int:
        """使能用户 1ms 控制回路。

        服务端 RPC: ``arm_sdk.EnableUserControlLoop``

        参数:
        ``enable``: bool

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(
            self.rpc("EnableUserControlLoop", {"side": 0, "value": bool(enable)})
        )

    def IsUserControlLoopEnabled(self) -> bool:
        """查询用户控制回路是否使能。

        服务端 RPC: ``arm_sdk.IsUserControlLoopEnabled``

        返回:
        bool
        """
        rsp = self.rpc("IsUserControlLoopEnabled")
        return bool(rsp.get("value", False))

