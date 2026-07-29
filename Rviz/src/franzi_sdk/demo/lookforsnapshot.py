#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""查询 MechUnit 整机状态快照 — ``GetRobotState``（各机械单元 + 关节反馈）。"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from typing import Any, List

_SDK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SDK_ROOT not in sys.path:
    sys.path.insert(0, _SDK_ROOT)

from core import logger
from core.client import ArmClient
from core.types import (
    MechUnitStateType,
    SdkStatus,
    format_mechunit_error_code,
    mech_unit_state_name,
    mech_unit_type_name,
    sdk_status_message,
)

HOST = "192.168.23.30"
PORT = 8000

_COMMAND_AUTHORITY = {
    0: "算法",
    1: "软件系统",
}


def fmt_joints(vec: List[float], prec: int = 4) -> str:
    if not vec:
        return "[]"
    return "[" + ", ".join(f"{x:.{prec}f}" for x in vec) + "]"


def log_robot_snapshot(arm: ArmClient) -> None:
    """打印一次 MechUnit ``GetRobotState`` 快照。"""
    logger.info("======== MechUnit Snapshot (GetRobotState) ========")
    try:
        rsp = arm.GetRobotState()
    except Exception as ex:
        logger.info("GetRobotState: %s: %s", type(ex).__name__, ex)
        logger.info("======== MechUnit Snapshot end ========")
        return

    status = int(rsp.get("status", 0))
    if status != SdkStatus.OK:
        logger.info(
            "GetRobotState: status=%s %s rsp=%s",
            status,
            sdk_status_message(status),
            rsp,
        )
        logger.info("======== MechUnit Snapshot end ========")
        return

    authority = int(rsp.get("command_authority", 0))
    logger.info(
        "unit_count=%s command_authority=%s(%s) timestamp_ns=%s",
        rsp.get("unit_count"),
        authority,
        _COMMAND_AUTHORITY.get(authority, "未知"),
        rsp.get("timestamp_ns"),
    )

    units = rsp.get("units") or []
    if not units:
        logger.info("(no units)")
        logger.info("======== MechUnit Snapshot end ========")
        return

    for unit in units:
        unit_id = unit.get("unit_id") or "?"
        unit_type = int(unit.get("unit_type", 255))
        mech_state = int(unit.get("mech_state", 0))
        joint_count = int(unit.get("joint_count", 0))
        logger.info(
            "[%s] type=%s mech_state=%s joints=%s",
            unit_id,
            mech_unit_type_name(unit_type),
            mech_unit_state_name(mech_state),
            joint_count,
        )

        joints: List[Any] = unit.get("joints") or []
        if not joints:
            continue

        n = min(joint_count, len(joints)) if joint_count else len(joints)
        pos_deg = [
            math.degrees(float(j.get("position_rad", 0.0)))
            for j in joints[:n]
        ]
        vel = [float(j.get("velocity_rad_s", 0.0)) for j in joints[:n]]
        logger.info("  pos(deg)=%s", fmt_joints(pos_deg))
        logger.info("  vel(rad/s)=%s", fmt_joints(vel, 3))

        for i, j in enumerate(joints[:n]):
            err = int(j.get("error_code", 0))
            if err or int(j.get("has_fault", 0)) or mech_state == int(MechUnitStateType.FAULT):
                logger.info(
                    "  joint[%s] error_code=%s servo_state=%s follow_err=%s moving=%s",
                    i,
                    format_mechunit_error_code(err),
                    j.get("servo_state"),
                    j.get("follow_error_active"),
                    j.get("moving"),
                )

    logger.info("======== MechUnit Snapshot end ========")


def main() -> int:
    parser = argparse.ArgumentParser(description="查询 MechUnit GetRobotState 整机快照")
    parser.add_argument("--host", default=HOST, help=f"SDK 服务端 IP (默认 {HOST})")
    parser.add_argument("--port", type=int, default=PORT, help=f"SDK 服务端端口 (默认 {PORT})")
    parser.add_argument(
        "--watch",
        type=float,
        default=0.0,
        metavar="SEC",
        help="周期性查询间隔（秒）；0 表示只查一次",
    )
    args = parser.parse_args()

    arm = ArmClient(host=args.host, port=args.port, raise_on_error=False)
    try:
        if not arm.connect():
            logger.error(
                "连接 %s:%s 失败，请确认控制器已启动 SdkServerModule",
                args.host,
                args.port,
            )
            return 1

        if args.watch <= 0.0:
            log_robot_snapshot(arm)
        else:
            logger.info("周期查询间隔 %.1f s，Ctrl+C 退出", args.watch)
            while True:
                log_robot_snapshot(arm)
                time.sleep(args.watch)
    except KeyboardInterrupt:
        logger.info("Ctrl+C 中断")
    finally:
        arm.disconnect()
        logger.info("已断开连接")
    return 0


if __name__ == "__main__":
    sys.exit(main())
