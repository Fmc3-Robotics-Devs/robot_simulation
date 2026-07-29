"""
force 域 API
力控/阻抗/拖动示教。
"""

from __future__ import annotations

from core.client._base import ArmClientBase
from typing import List, Sequence, Tuple

from core import types


class ArmClientForce(ArmClientBase):
    # ==================== 力控 / 阻抗 / 示教 ====================

    def AdmittanceControl(
        self,
        side: types.SideLike,
        M_cartesian: Sequence[float],
        D_cartesian: Sequence[float],
        K_cartesian: Sequence[float],
    ) -> int:
        """笛卡尔导纳柔顺控制（异步进入 AdmittanceJPos）。

        服务端 RPC: ``arm_sdk.AdmittanceControl``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right
        ``M_cartesian``/``D_cartesian``/``K_cartesian``: 各 6 维

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(self.rpc("AdmittanceControl", {
                "side": types.side_value(side),
                "M_cartesian": list(M_cartesian),
                "D_cartesian": list(D_cartesian),
                "K_cartesian": list(K_cartesian),
            }))

    def ImpedanceTrace(
        self,
        side: types.SideLike,
        target_joint_position: Sequence[float],
        kd: Sequence[float],
        bd: Sequence[float],
        ratio: Sequence[float],
    ) -> int:
        """关节阻抗位置跟踪 ImpedanceTrace。

        服务端 RPC: ``arm_sdk.ImpedanceTrace``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right
        目标角、kd、bd、ratio(长度3)

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(self.rpc("ImpedanceTrace", {
                "side": types.side_value(side),
                "target_joint_position": list(target_joint_position),
                "kd": list(kd),
                "bd": list(bd),
                "ratio": list(ratio),
            }))

    def DragTeachDragMode(self, side: types.SideLike) -> int:
        """拖动示教-拖动模式 DragJTorq（同步）。

        服务端 RPC: ``arm_sdk.DragTeachDragMode``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(self.rpc("DragTeachDragMode", types.side_req(side)))

    def DragTeachPlayMode(self, side: types.SideLike) -> int:
        """拖动示教-回放 TeachJPos（同步）。

        服务端 RPC: ``arm_sdk.DragTeachPlayMode``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(self.rpc("DragTeachPlayMode", types.side_req(side)))

    def ReadCartesianWrench(self, side: types.SideLike) -> List[float]:
        """末端外力/力矩估计。

        服务端 RPC: ``arm_sdk.ReadCartesianWrench``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        list[float] [Fx,Fy,Fz,Tx,Ty,Tz] N, N·m
        """
        v = self.rpc("ReadCartesianWrench", types.side_req(side)).get("value")
        return list(v or [])

    def ReadJointTorqueCmd(self, side: types.SideLike) -> List[float]:
        """关节力矩指令反馈。

        服务端 RPC: ``arm_sdk.ReadJointTorqueCmd``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        list[float]
        """
        v = self.rpc("ReadJointTorqueCmd", types.side_req(side)).get("value")
        return list(v or [])

    def ReadImpedanceTraceParams(
        self, side: types.SideLike
    ) -> Tuple[int, List[float], List[float]]:
        """查询阻抗跟踪 kd/bd。

        服务端 RPC: ``arm_sdk.ReadImpedanceTraceParams``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        tuple (status, kd, bd)
        """
        rsp = self.rpc("ReadImpedanceTraceParams", types.side_req(side))
        return (
            self.status_of(rsp),
            list(rsp.get("kd", [])),
            list(rsp.get("bd", [])),
        )
    def StartDragMode(self, side: types.SideLike, record_enable: bool = False) -> int:
        """启动拖动模式 DragMode。

        服务端 RPC: ``arm_sdk.StartDragMode``

        参数:
        ``side``: :class:`types.ArmSide` 
        ``record_enable``: bool —— 是否记录拖动轨迹

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(self.rpc("StartDragMode", {
                "side": types.side_value(side),
                "record_enable": bool(record_enable),
            }))
    
    def ExitDragMode(self, side: types.SideLike) -> int:
        """退出拖动模式 ExitDragMode。

        服务端 RPC: ``arm_sdk.ExitDragMode``

        参数:
        ``side``: :class:`types.ArmSide` 

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(self.rpc("ExitDragMode", types.side_req(side)))
    
    def StartImpedance(
        self,
        side: types.SideLike,
        D_joint=None,
        K_joint=None,
        Deadzone_joint=None,
    ) -> int:
        """启动阻抗控制 ImpedanceControl。

        服务端 RPC: ``arm_sdk.StartImpedance``

        参数:
        ``side``: :class:`types.ArmSide`
        ``D_joint``: 关节阻尼参数，长度为 7；None 或 [] 表示使用默认参数
        ``K_joint``: 关节刚度参数，长度为 7；None 或 [] 表示使用默认参数
        ``Deadzone_joint``: 关节外力矩死区参数，长度为 7；None 或 [] 表示使用默认参数

        返回:
        int —— SdkStatus，0 表示成功
        """
        if D_joint is None:
            D_joint = []
        if K_joint is None:
            K_joint = []
        if Deadzone_joint is None:
            Deadzone_joint = []

        req = types.side_req(side)
        req["D_joint"] = list(D_joint)
        req["K_joint"] = list(K_joint)
        req["Deadzone_joint"] = list(Deadzone_joint)

        return self.status_of(self.rpc("StartImpedance", req))
    
    def ExitImpedance(self, side: types.SideLike) -> int:
        """退出阻抗控制 ImpedanceControl。

        服务端 RPC: ``arm_sdk.ExitImpedance``

        参数:
        ``side``: :class:`types.ArmSide`

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(self.rpc("ExitImpedance", types.side_req(side)))

    def Playback(self, side: types.SideLike) -> int:
        """回放拖动示教记录的关节轨迹。

        服务端 RPC: ``arm_sdk.Playback``

        参数:
        ``side``: :class:`types.ArmSide`

        说明:
        该接口不从 SDK 侧传入轨迹点，也不传入文件路径。
        服务端会从固定路径 ``/inodata/dragmode_joint_trajectory.txt`` 读取已记录的轨迹，
        并在控制侧缓存轨迹后按实时控制周期回放。

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(self.rpc("Playback", {
            "side": types.side_value(side),
        }))

