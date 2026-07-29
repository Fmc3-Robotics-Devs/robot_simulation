"""
kinematics 域 API
bio_arm_sdk：IK/FK、MDH、flan2tool。
"""

from __future__ import annotations

from core.client._base import ArmClientBase
from typing import List, Sequence, Tuple

from core import types


class ArmClientKinematics(ArmClientBase):
    # ==================== 运动学 ====================

    def IK(
        self,
        side: types.SideLike,
        matrix16: Sequence[float],
    ) -> Tuple[int, List[float]]:
        """逆运动学：末端位姿 → 关节角（初值取当前关节角 q）。

        服务端 RPC: ``arm_sdk.IK``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right
        ``matrix16``: 基座→末端 4×4 齐次矩阵，列优先 16 元

        返回:
        tuple (status, out_joint_pos)
        """
        req = {
            "side": types.side_value(side),
            "base2tool": {"value": list(matrix16)},
        }
        rsp = self.rpc("IK", req)
        return self.status_of(rsp), list(rsp.get("out_joint_pos", []))

    def Fk(
        self, side: types.SideLike, cur_joints: Sequence[float]
    ) -> Tuple[int, List[float]]:
        """正运动学：基座 → 末端，4×4 齐次矩阵。

        服务端 RPC: ``arm_sdk.Fk``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right
        ``cur_joints``: 当前关节角

        返回:
        tuple (status, matrix16) 行主序 16 元
        """
        return self._fk_impl("Fk", side, cur_joints)

    def FkTool(
        self, side: types.SideLike, cur_joints: Sequence[float]
    ) -> Tuple[int, List[float]]:
        """正解：基座 → 工具坐标系。

        服务端 RPC: ``arm_sdk.FkTool``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        tuple (status, matrix16)
        """
        return self._fk_impl("FkTool", side, cur_joints)

    def FkLink(
        self, side: types.SideLike, cur_joints: Sequence[float], link_num: int
    ) -> Tuple[int, List[float]]:
        """正解：基座 → 指定连杆。

        服务端 RPC: ``arm_sdk.FkLink``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right
        ``link_num``: 连杆编号

        返回:
        tuple (status, matrix16)
        """
        req = {
            "side": types.side_value(side),
            "cur_joints": list(cur_joints),
            "link_num": int(link_num),
        }
        rsp = self.rpc("FkLink", req)
        tf = rsp.get("out_tf", {})
        return self.status_of(rsp), list(tf.get("value", []))

    def _fk_impl(
        self, suffix: str, side: types.SideLike, cur_joints: Sequence[float]
    ) -> Tuple[int, List[float]]:
        req = {"side": types.side_value(side), "cur_joints": list(cur_joints), "link_num": 0}
        rsp = self.rpc(suffix, req)
        tf = rsp.get("out_tf", {})
        return self.status_of(rsp), list(tf.get("value", []))

    def GetFlan2tool(self, side: types.SideLike) -> Tuple[int, List[float]]:
        """查询法兰→工具变换矩阵。

        服务端 RPC: ``arm_sdk.GetFlan2tool``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        tuple (status, matrix16)
        """
        rsp = self.rpc("GetFlan2tool", types.side_req(side))
        tf = rsp.get("out_tf", {})
        return self.status_of(rsp), list(tf.get("value", []))

    def SetFlan2tool(self, side: types.SideLike, matrix16: Sequence[float]) -> int:
        """设置法兰→工具变换。

        服务端 RPC: ``arm_sdk.SetFlan2tool``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right
        ``matrix16``: 4×4 行主序

        返回:
        int —— SdkStatus，0 表示成功
        """
        req = {
            "side": types.side_value(side),
            "flan2tool": {"value": list(matrix16)},
        }
        return self.status_of(self.rpc("SetFlan2tool", req))

    def GetMdh(self, side: types.SideLike) -> List[List[float]]:
        """查询 MDH 连杆参数。

        服务端 RPC: ``arm_sdk.GetMdh``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        list[list[float]]
        """
        rsp = dict(self.rpc("GetMdh", types.side_req(side)))
        return list(rsp.get("mdh", []))

    def SetMdh(self, side: types.SideLike, mdh: Sequence[Sequence[float]]) -> int:
        """设置 MDH 参数。

        服务端 RPC: ``arm_sdk.SetMdh``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(
            self.rpc("SetMdh", {"side": types.side_value(side), "mdh": [list(r) for r in mdh]})
        )

    def GetMdhCompensation(self, side: types.SideLike) -> List[List[float]]:
        """查询 MDH 补偿参数。

        服务端 RPC: ``arm_sdk.GetMdhCompensation``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        list[list[float]]
        """
        rsp = dict(self.rpc("GetMdhCompensation", types.side_req(side)))
        return list(rsp.get("mdh", []))

    def SetMdhCompensation(
        self, side: types.SideLike, mdh: Sequence[Sequence[float]]
    ) -> int:
        """设置 MDH 补偿。

        服务端 RPC: ``arm_sdk.SetMdhCompensation``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(self.rpc("SetMdhCompensation", {"side": types.side_value(side), "mdh": [list(r) for r in mdh]}))

