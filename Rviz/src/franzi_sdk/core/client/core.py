"""
core 域 API
SdkServerModule：版本、就绪、轴数。
"""

from __future__ import annotations

from core.client._base import ArmClientBase

class ArmClientCore(ArmClientBase):
    """SdkServerModule 基础能力。"""

    def GetVersion(self) -> str:
        """查询 SDK 服务端版本号。

        服务端 RPC: ``arm_sdk.GetVersion``

        返回:
        str，版本字符串（如 ``1.0.0``）

        说明:
        无业务 status 字段。"""
        return str(self.rpc("GetVersion"))

    def IsReady(self) -> bool:
        """查询执行层是否已完成初始化、可接受运动指令。

        服务端 RPC: ``arm_sdk.IsReady``

        返回:
        bool —— 是否就绪

        说明:
        Mock 下一般为 true；旧桩未就绪时 status 会是 -5。"""
        rsp = self.rpc("IsReady")
        return bool(rsp.get("value", False))

    def AxisCount(self) -> int:
        """查询当前配置的关节轴数。

        服务端 RPC: ``arm_sdk.AxisCount``

        返回:
        int —— 单臂 7，双臂 14
        """
        rsp = self.rpc("AxisCount")
        return int(rsp.get("value", 0))


    def SoftStop(self, active: bool = True) -> bool:
        """设置软急停状态：按下/弹起。

        服务端 RPC: ``arm_sdk.SoftStop``

        参数:
        active --
            True:  按下软急停（SDK 进入 NotReady，运动指令应停止下发）
            False: 弹起软急停（恢复 IsReady）

        返回:
        bool —— 操作是否成功（rsp.status == 0）

        说明:
        服务端会按 10ms 周期持续向 ``emergency_stop/external_trigger`` 发布
        当前软急停状态，订阅者（power_manager_module 等）按最新值响应。
        """
        rsp = self.rpc("SoftStop", {"active": bool(active)})
        return int(rsp.get("status", -1)) == 0
