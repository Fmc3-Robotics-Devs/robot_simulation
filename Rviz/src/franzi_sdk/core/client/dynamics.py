"""
dynamics 域 API
bioDyn：动力学与参数文件。
"""

from __future__ import annotations

from core.client._base import ArmClientBase
from typing import List, Sequence

from core import types


class ArmClientDynamics(ArmClientBase):
    # ==================== 动力学 ====================

    def ComputeEulerXyzToMatrix(
        self, rx: float, ry: float, rz: float
    ) -> List[float]:
        """欧拉角 XYZ → 旋转矩阵。

        服务端 RPC: ``arm_sdk.ComputeEulerXyzToMatrix``

        参数:
        ``rx, ry, rz`` rad

        返回:
        list[float] 3×3 展平 9 元
        """
        v = self.rpc(
            "ComputeEulerXyzToMatrix",
            {"rx": rx, "ry": ry, "rz": rz},
        ).get("value")
        return list(v or [])

    def ComputeGravB(
        self,
        grav_w: Sequence[float],
        rx: float,
        ry: float,
        rz: float,
    ) -> List[float]:
        """世界系重力 → 基座系。

        服务端 RPC: ``arm_sdk.ComputeGravB``

        参数:
        grav_w, rx, ry, rz

        返回:
        list[float]
        """
        v = self.rpc(
            "ComputeGravB",
            {"grav_w": list(grav_w), "rx": rx, "ry": ry, "rz": rz},
        ).get("value")
        return list(v or [])

    def ComputeInvdynStdPara(
        self,
        side: types.SideLike,
        q: Sequence[float],
        qd: Sequence[float],
        qdd: Sequence[float],
        grav: Sequence[float],
        fext: Sequence[float],
        is_tool: int,
    ) -> List[float]:
        """逆动力学标准参数。

        服务端 RPC: ``arm_sdk.ComputeInvdynStdPara``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right
        q, qd, qdd, grav, fext, is_tool

        返回:
        list[float] 力矩
        """
        req = {
            "side": types.side_value(side),
            "q": list(q),
            "qd": list(qd),
            "qdd": list(qdd),
            "grav": list(grav),
            "fext": list(fext),
            "is_tool": int(is_tool),
        }
        v = self.rpc("ComputeInvdynStdPara", req).get("value")
        return list(v or [])

    def ComputeMassMatrix(
        self, side: types.SideLike, q: Sequence[float], is_tool: int = 0
    ) -> List[float]:
        """关节空间质量矩阵 7×7。

        服务端 RPC: ``arm_sdk.ComputeMassMatrix``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        list[float] 49 元
        """
        req = {
            "side": types.side_value(side),
            "q": list(q),
            "qd": [],
            "is_tool": int(is_tool),
        }
        v = self.rpc("ComputeMassMatrix", req).get("value")
        return list(v or [])

    def ComputeCoriolisMat(
        self,
        side: types.SideLike,
        q: Sequence[float],
        qd: Sequence[float],
        is_tool: int,
    ) -> List[float]:
        """科氏/离心力矩阵。

        服务端 RPC: ``arm_sdk.ComputeCoriolisMat``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        list[float] 49 元
        """
        req = {
            "side": types.side_value(side),
            "q": list(q),
            "qd": list(qd),
            "is_tool": int(is_tool),
        }
        v = self.rpc("ComputeCoriolisMat", req).get("value")
        return list(v or [])

    def ComputeImpedanceJoint(
        self,
        side: types.SideLike,
        M_mat: Sequence[float],
        C_mat: Sequence[float],
        Kd: Sequence[float],
        Bd: Sequence[float],
        q_cmd: Sequence[float],
        qd_cmd: Sequence[float],
        qdd_cmd: Sequence[float],
        q_cur: Sequence[float],
        qd_cur: Sequence[float],
    ) -> List[float]:
        """关节阻抗补偿力矩。

        服务端 RPC: ``arm_sdk.ComputeImpedanceJoint``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right
        M,C,Kd,Bd, q_cmd...

        返回:
        list[float]
        """
        req = {
            "side": types.side_value(side),
            "M_mat": list(M_mat),
            "C_mat": list(C_mat),
            "Kd": list(Kd),
            "Bd": list(Bd),
            "q_cmd": list(q_cmd),
            "qd_cmd": list(qd_cmd),
            "qdd_cmd": list(qdd_cmd),
            "q_cur": list(q_cur),
            "qd_cur": list(qd_cur),
        }
        v = self.rpc("ComputeImpedanceJoint", req).get("value")
        return list(v or [])

    def SetLinksParaFromFile(self, side: types.SideLike) -> int:
        """从 dynPara.txt 加载连杆参数。

        服务端 RPC: ``arm_sdk.SetLinksParaFromFile``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(self.rpc("SetLinksParaFromFile", types.side_req(side)))

    def SetToolParaFromFile(self, side: types.SideLike) -> int:
        """从文件加载工具动力学参数。

        服务端 RPC: ``arm_sdk.SetToolParaFromFile``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(self.rpc("SetToolParaFromFile", types.side_req(side)))

    def GetLinksParaFromFile(self, side: types.SideLike) -> int:
        """打印连杆动力学参数。

        服务端 RPC: ``arm_sdk.GetLinksParaFromFile``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(self.rpc("GetLinksParaFromFile", types.side_req(side)))

    def GetToolParaFromFile(self, side: types.SideLike) -> int:
        """打印工具动力学参数。

        服务端 RPC: ``arm_sdk.GetToolParaFromFile``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(self.rpc("GetToolParaFromFile", types.side_req(side)))

