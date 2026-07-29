"""
state 域 API
robot_system：关节状态、故障、full state。
"""

from __future__ import annotations

from core.client._base import ArmClientBase
from typing import Any, Dict, List, Tuple

from core import types


class ArmClientState(ArmClientBase):
    # ==================== 状态 / 故障 ====================

    def ReadJointState(self, side: types.SideLike) -> Dict[str, List[float]]:
        """读取关节位置、速度、力矩反馈。

        服务端 RPC: ``arm_sdk.ReadJointState``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        dict: ``position``/``velocity``/``torque``，单位 rad、rad/s、N·m
        """
        return dict(self.rpc("ReadJointState", types.side_req(side)))

    def GetFaultState(self, side: types.SideLike) -> Dict[str, Any]:
        """读取各轴故障标志与故障码。

        服务端 RPC: ``arm_sdk.GetFaultState``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        dict: ``joint_has_fault``, ``joint_fault_code``
        """
        return dict(self.rpc("GetFaultState", types.side_req(side)))

    def ResetFault(self, side: types.SideLike, axis_id: int = -1) -> int:
        """复位故障（FaultReset）。

        服务端 RPC: ``arm_sdk.ResetFault``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right
        ``axis_id``: int，-1 表示全部轴

        返回:
        int —— SdkStatus，0 表示成功
        """
        return self.status_of(
            self.rpc("ResetFault", {"side": types.side_value(side), "axis_id": axis_id})
        )

    def ReadFullState(self, side: types.SideLike) -> Dict[str, List[float]]:
        """读取单臂简化完整状态。

        服务端 RPC: ``arm_sdk.ReadFullState``

        参数:
        ``side``: :class:`types.ArmSide` 或 int —— 0=Bio(双臂), 1=Left, 2=Right

        返回:
        dict: ``joint_pos``/``joint_vel``/``joint_torque``
        """
        return dict(self.rpc("ReadFullState", types.side_req(side)))

    def ReadFullStateDual(self) -> Dict[str, Any]:
        """读取双臂完整状态。

        服务端 RPC: ``arm_sdk.ReadFullStateDual``

        返回:
        dict: ``left``、``right`` 子 dict
        """
        return dict(self.rpc("ReadFullStateDual"))

    def GetRobotState(self) -> Dict[str, Any]:
        """读取 MechUnit 整机状态（各机械单元 + 关节反馈）。

        服务端 RPC: ``arm_sdk.GetRobotState``

        返回:
        dict，主要字段:

        - ``status``: SdkStatus，0 表示成功
        - ``unit_count``: 机械单元数量（标准 4 槽）
        - ``command_authority``: 0=算法, 1=软件系统
        - ``timestamp_ns``: 快照时间戳（ns）
        - ``units``: list[dict]，每项含 ``unit_id`` / ``unit_type`` /
          ``mech_state`` / ``joint_count`` / ``joints``
        """
        return dict(self.rpc("GetRobotState"))

    def GetRobotPose(self, side: types.SideLike) -> Tuple[int, List[float]]:
        """读取当前臂末端 6D 位姿（MotionControl INO get_robot_pose）。

        服务端 RPC: ``arm_sdk.GetRobotPose``

        参数:
        ``side``: 0=Left, 1=Right（与 C++ SdkArmSide 一致）

        返回:
        tuple (status, pose) —— pose 为 [x, y, z, rx, ry, rz]（m + rad）
        """
        rsp = self.rpc("GetRobotPose", types.side_req(side))
        return self.status_of(rsp), list(rsp.get("value") or [])

    def GetRobotPoseDual(self) -> Tuple[int, Dict[str, List[float]]]:
        """读取双臂末端 6D 位姿（MotionControl INO get_robot_pose）。

        服务端 RPC: ``arm_sdk.GetRobotPoseDual``

        返回:
        tuple (status, poses) —— poses 为 ``{"left": [...], "right": [...]}``，各 6D（m + rad）
        """
        rsp = self.rpc("GetRobotPoseDual")
        status = self.status_of(rsp)
        return status, {
            "left": list(rsp.get("left") or []),
            "right": list(rsp.get("right") or []),
        }

    def GetJointState(self, side: types.SideLike) -> Tuple[int, List[float]]:
        """读取当前臂关节位置（MotionControl INO get_joint_state）。

        服务端 RPC: ``arm_sdk.GetJointState``

        参数:
        ``side``: 0=Left, 1=Right（与 C++ SdkArmSide 一致）

        返回:
        tuple (status, positions) —— positions 为关节角（rad）
        """
        rsp = self.rpc("GetJointState", types.side_req(side))
        return self.status_of(rsp), list(rsp.get("value") or [])

    def GetMotionRobotState(self, side: types.SideLike) -> Tuple[int, Dict[str, Any]]:
        """读取 MotionControl 单臂完整机器人状态（INO get_robot_state）。

        服务端 RPC: ``arm_sdk.GetMotionRobotState``

        与 ``GetRobotState``（MechUnit 整机状态）不同，本接口返回运动控制模块内的
        关节/笛卡尔/动力学快照（对应 C++ ``SdkMotionRobotStateRsp`` / ``RobotState``）。

        参数:
        ``side``: 0=Left, 1=Right

        返回:
        tuple (status, state_dict) —— state_dict 字段说明（均为 list[float]）:

        - ``q``: 反馈关节位置（rad，长度 7）
        - ``q_d``: 期望关节位置（rad）
        - ``dq``: 反馈关节速度（rad/s）
        - ``dq_d``: 期望关节速度（rad/s）
        - ``ddq_d``: 期望关节加速度（rad/s²）
        - ``current_J``: 反馈关节电流
        - ``tau_J``: 反馈关节力矩（N·m）
        - ``tau_J_d``: 期望关节力矩（N·m）
        - ``O_T_EE``: 反馈末端 6D 位姿 [x,y,z,rx,ry,rz]（m + rad）
        - ``O_T_EE_d``: 期望末端 6D 位姿
        - ``dO_T_EE``: 反馈末端 6D 速度
        - ``dO_T_EE_d``: 期望末端 6D 速度
        - ``ddO_T_EE_d``: 期望末端 6D 加速度
        - ``tau_e_J``: 关节外力/力矩
        - ``tau_e_EE``: 末端外力/力矩（6D）
        - ``M_mat``: 质量矩阵（7×7，列优先展平，长度 49）
        - ``C_mat``: 科里奥利矩阵（7×7，列优先展平）
        - ``G_mat``: 重力向量（长度 7）
        """
        rsp = self.rpc("GetMotionRobotState", types.side_req(side))
        status = self.status_of(rsp)
        state = {k: v for k, v in rsp.items() if k != "status"}
        return status, state

