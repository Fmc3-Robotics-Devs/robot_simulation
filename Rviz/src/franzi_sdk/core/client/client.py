# -*- coding: utf-8 -*-
"""
ArmClient 入口。
各域模块均继承 ``ArmClientBase``；本文件组合全部域类为完整客户端。
"""

from __future__ import annotations

from core.client._base import ArmClientBase
from core.client.cabinet import ArmClientCabinet
from core.client.cartesian import ArmClientCartesian
from core.client.core import ArmClientCore
from core.client.dynamics import ArmClientDynamics
from core.client.feedback import ArmClientFeedback
from core.client.force import ArmClientForce
from core.client.kinematics import ArmClientKinematics
from core.client.limits import ArmClientLimits
from core.client.motion_cmd import ArmClientMotionCmd
from core.client.motion_traj import ArmClientMotionTraj
from core.client.end_effector import ArmClientEndEffector
from core.client.rs485 import ArmClientRs485
from core.client.state import ArmClientState
from core.client.system import ArmClientSystem
from core.client.teleop import ArmClientTeleop
from core.client.user_loop import ArmClientUserLoop
# from arm_client_docs import apply_api_docstrings


class ArmClient(
    ArmClientCore,
    ArmClientSystem,
    ArmClientState,
    ArmClientMotionCmd,
    ArmClientKinematics,
    ArmClientMotionTraj,
    ArmClientForce,
    ArmClientLimits,
    ArmClientFeedback,
    ArmClientUserLoop,
    ArmClientTeleop,
    ArmClientCartesian,
    ArmClientDynamics,
    ArmClientCabinet,
    ArmClientRs485,
    ArmClientEndEffector,
    ArmClientBase,
):
    """机械臂 RPC 客户端（arm_sdk.*）。"""


# apply_api_docstrings(ArmClient)

__all__ = ["ArmClient"]
