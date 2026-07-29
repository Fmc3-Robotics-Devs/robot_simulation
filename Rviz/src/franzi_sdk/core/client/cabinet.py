"""
cabinet 域 API
power_manager：机柜急停、上下电。
"""

from __future__ import annotations

from core.client._base import ArmClientBase
from typing import Tuple

class ArmClientCabinet(ArmClientBase):
    # ==================== 机柜 ====================

    def GetCabinetEmergencyStop(self) -> bool:
        """机柜硬件急停（hardmaster）。

        服务端 RPC: ``arm_sdk.GetCabinetEmergencyStop``

        返回:
        bool triggered
        """
        rsp = self.rpc("GetCabinetEmergencyStop")
        return bool(rsp.get("value", False))

    def GetCabinetPowerExceptionCode(self) -> Tuple[int, int]:
        """电柜电源故障码 0x0040。

        服务端 RPC: ``arm_sdk.GetCabinetPowerExceptionCode``

        返回:
        tuple (status, code)
        """
        rsp = self.rpc("GetCabinetPowerExceptionCode")
        return self.status_of(rsp), int(rsp.get("value", 0))

    def GetCabinetFanFailureCode(self) -> Tuple[int, int]:
        """风扇故障码 0x0048。

        服务端 RPC: ``arm_sdk.GetCabinetFanFailureCode``

        返回:
        tuple (status, code)
        """
        rsp = self.rpc("GetCabinetFanFailureCode")
        return self.status_of(rsp), int(rsp.get("value", 0))

    def GetCabinetPowerStateRaw(self) -> Tuple[int, int]:
        """电源状态寄存器 0x003C。

        服务端 RPC: ``arm_sdk.GetCabinetPowerStateRaw``

        返回:
        tuple (status, raw)
        """
        rsp = self.rpc("GetCabinetPowerStateRaw")
        return self.status_of(rsp), int(rsp.get("value", 0))

    def PowerOnArmBody(self) -> int:
        """本体 48V 上电。

        服务端 RPC: ``arm_sdk.PowerOnArmBody``

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(self.rpc("PowerOnArmBody"))

    def PowerOffArmBody(self) -> int:
        """本体下电。

        服务端 RPC: ``arm_sdk.PowerOffArmBody``

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(self.rpc("PowerOffArmBody"))


