#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""查询 SystemModule 聚合的急停状态 — ``GetEmergencyStopState`` / ``IsEmergencyStopActive``。"""

from __future__ import annotations

import argparse
import os
import sys
import time

_SDK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SDK_ROOT not in sys.path:
    sys.path.insert(0, _SDK_ROOT)

from core import logger
from core.client import ArmClient
from core.types import SdkStatus, sdk_status_message

HOST = "192.168.23.30"
PORT = 8000


def query_estop(arm: ArmClient) -> None:
    """打印一次急停快照。"""
    try:
        rsp = arm.GetEmergencyStopState()
    except Exception as ex:
        logger.info("GetEmergencyStopState: %s: %s", type(ex).__name__, ex)
        return

    status = int(rsp.get("status", 0))
    if status != SdkStatus.OK:
        logger.info(
            "GetEmergencyStopState: status=%s %s rsp=%s",
            status,
            sdk_status_message(status),
            rsp,
        )
        return

    estop_active = bool(rsp.get("estop_active", False))
    estop_reg = int(rsp.get("estop_reg", 0))
    timestamp_us = rsp.get("timestamp_us")
    is_active = arm.IsEmergencyStopActive()

    logger.info(
        "estop_active=%s reg=0x%04x timestamp_us=%s IsEmergencyStopActive=%s",
        estop_active,
        estop_reg,
        timestamp_us,
        is_active,
    )
    if estop_active:
        logger.info("急停有效 — 禁止上使能，需复位后再 Enable")
    else:
        logger.info("急停未激活 — 可尝试上使能（仍需满足其它安全条件）")


def main() -> int:
    parser = argparse.ArgumentParser(description="查询 arm_sdk 急停状态")
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
        if args.watch <= 0.0:
            query_estop(arm)
        else:
            logger.info("周期查询间隔 %.1f s，Ctrl+C 退出", args.watch)
            while True:
                query_estop(arm)
                time.sleep(args.watch)
    except KeyboardInterrupt:
        logger.info("Ctrl+C 中断")
    finally:
        arm.disconnect()
        logger.info("已断开连接")
    return 0


if __name__ == "__main__":
    sys.exit(main())
