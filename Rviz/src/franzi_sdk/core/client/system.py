"""
system 域 API
robot_system：使能、运动模式、急停、系统状态。
"""

from __future__ import annotations

from core.client._base import ArmClientBase
from typing import Any, Dict, Tuple, Union

from core import types


class ArmClientSystem(ArmClientBase):
    # ==================== 系统状态 / 使能 / 模式 / 急停 ====================

    def GetSystemState(self, side: types.SideLike) -> types.ArmSystemState:
        """读取指定臂的系统运行状态。

        服务端 RPC: ``arm_sdk.GetSystemState``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        :class:`types.ArmSystemState`
        """
        v = self.rpc("GetSystemState", types.side_req(side)).get("value")
        return types.ArmSystemState(int(v))

    def GetEnableState(self, side: types.SideLike) -> bool:
        """查询指定臂是否处于使能（非 NotEnabled 即视为已使能）。

        服务端 RPC: ``arm_sdk.GetEnableState``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        bool
        """
        v = self.rpc("GetEnableState", types.side_req(side)).get("value")
        return bool(v)

    def SetEnableState(self, robot_type: int, lifecycle_cmd: int) -> int:
        """设置指定机械单元 lifecycle 指令（Enable/Disable/Reset）。

        服务端 RPC: ``arm_sdk.SetEnableState``

        参数:
        ``robot_type``: MechUnitType，如 0=LEFTARM, 10=RIGHTARM, 20=WAIST, 30=HEAD
        ``lifecycle_cmd``: 0=None, 1=Enable, 2=Disable, 3=Reset

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(
            self.rpc("SetEnableState", types.robot_type_req(robot_type, lifecycle_cmd))
        )

    def SetMechUnitLifecycle(
        self,
        component_type: int,
        lifecycle_cmd: Union[types.MechUnitLifecycleCmd, int],
    ) -> int:
        """经 SystemModule 下发机械单元 NR 生命周期命令。

        服务端 RPC: ``arm_sdk.SetMechUnitLifecycle``

        参数:
        ``component_type``: MechUnitType，如 0=LEFTARM, 10=RIGHTARM, 20=WAIST, 30=HEAD
        ``lifecycle_cmd``: :class:`types.MechUnitLifecycleCmd` 或 int —— 1=Enable, 2=Disable, 3=Reset

        返回:
        int —— SdkStatus，0 表示 MechUnit 接收成功
        """
        cmd = (
            int(lifecycle_cmd.value)
            if isinstance(lifecycle_cmd, types.MechUnitLifecycleCmd)
            else int(lifecycle_cmd)
        )
        return self.status_of(
            self.rpc(
                "SetMechUnitLifecycle",
                types.component_lifecycle_req(component_type, cmd),
            )
        )

    def GetMechUnitState(self, component_type: int) -> Tuple[int, int]:
        """查询指定机械单元当前聚合状态（经 SystemModule）。

        服务端 RPC: ``arm_sdk.GetMechUnitState``

        参数:
        ``component_type``: MechUnitType，如 0=LEFTARM, 10=RIGHTARM

        返回:
        ``(status, unit_state)`` —— status 为 SdkStatus；unit_state 见 :class:`types.MechUnitStateType`
        """
        rsp = self.rpc("GetMechUnitState", types.component_type_req(component_type))
        return self.status_of(rsp), int(rsp.get("value", 0))

    def GetMechUnitErrorInfo(self, component_type: int) -> Dict[str, Any]:
        """获取指定机械单元报错摘要（经 SystemModule GetRobotState 聚合）。

        服务端 RPC: ``arm_sdk.GetMechUnitErrorInfo``

        参数:
        ``component_type``: MechUnitType，如 0=LEFTARM, 10=RIGHTARM

        返回:
        dict —— 含 status、mech_state、unit_has_fault、fault_joint_count、joints 等字段；
        ``primary_error_code`` / ``joints[].error_code`` 为整型，展示时可用
        :func:`types.format_mechunit_error_code` 格式化为十六进制。
        """
        return dict(self.rpc("GetMechUnitErrorInfo", types.component_type_req(component_type)))

    def GetEmergencyStopState(self) -> Dict[str, Any]:
        """查询 SystemModule 聚合的软急停状态（PowerManager ``/SoftEmergencyStop``）。

        服务端 RPC: ``arm_sdk.GetEmergencyStopState``

        返回:
        dict —— 含 status、estop_active、estop_reg、timestamp_us
        """
        return dict(self.rpc("GetEmergencyStopState"))

    def IsEmergencyStopActive(self) -> bool:
        """软急停是否有效（``estop_active==true`` 表示急停中）。

        服务端 RPC: ``arm_sdk.GetEmergencyStopState``

        返回:
        bool
        """
        rsp = self.rpc("GetEmergencyStopState")
        if self.status_of(rsp) != types.SdkStatus.OK:
            return False
        return bool(rsp.get("estop_active", False))

    def SetMotionMode(self, side: types.SideLike, mode: Union[types.ArmMotionMode, int]) -> int:
        """设置运动控制模式。

        服务端 RPC: ``arm_sdk.SetMotionMode``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right
        ``mode``: :class:`types.ArmMotionMode` 或 int

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(self.rpc("SetMotionMode", types.side_mode_req(side, mode)))

    def GetMotionMode(self, side: types.SideLike) -> types.ArmMotionMode:
        """读取当前运动模式。

        服务端 RPC: ``arm_sdk.GetMotionMode``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        :class:`types.ArmMotionMode`
        """
        v = self.rpc("GetMotionMode", types.side_req(side)).get("value")
        return types.ArmMotionMode(int(v))

    def SetEmergencyStopType(
        self, side: types.SideLike, estop: Union[types.EmergencyStopType, int]
    ) -> int:
        """设置软件急停类型。

        服务端 RPC: ``arm_sdk.SetEmergencyStopType``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right
        ``estop``: :class:`types.EmergencyStopType`

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(self.rpc("SetEmergencyStopType", types.side_estop_req(side, estop)))

    def GetEmergencyStopType(self, side: types.SideLike) -> types.EmergencyStopType:
        """读取当前急停类型。

        服务端 RPC: ``arm_sdk.GetEmergencyStopType``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        :class:`types.EmergencyStopType`
        """
        v = self.rpc("GetEmergencyStopType", types.side_req(side)).get("value")
        return types.EmergencyStopType(int(v))

