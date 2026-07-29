#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""软急停 demo — 通过 arm_sdk.SoftStop(active=True/False) 切换按下/弹起。

服务端行为：
    1) 修改 SdkStopFacet::soft_stop_active_ 状态
    2) 后台 10ms 线程以最新状态向 emergency_stop/external_trigger
       持续发布 EmergencyStopReport
    3) IsReady() 在软急停按下时返回 false

订阅端（helloworld_module 等）按最新一条消息响应，无需记录上一次状态。
"""

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

# ------------------------------------------------------------------ 运行参数（按需修改）
HOST = "192.168.23.30"
PORT = 8000


def main() -> int:
    parser = argparse.ArgumentParser(description="arm_sdk.SoftStop 按下/弹起 demo")
    parser.add_argument("--host", default=HOST, help=f"SDK 服务端 IP (默认 {HOST})")
    parser.add_argument("--port", type=int, default=PORT, help=f"SDK 服务端端口 (默认 {PORT})")
    parser.add_argument(
        "--action",
        choices=("press", "release", "toggle", "status"),
        default="press",
        help="press=按下, release=弹起, toggle=翻转, status=仅查询 IsReady",
    )
    parser.add_argument(
        "--hold-sec",
        type=float,
        default=0.0,
        help="按下后保持多少秒再弹起（仅在 --action=press 时生效）",
    )
    args = parser.parse_args()

    arm = ArmClient(host=args.host, port=args.port, raise_on_error=False)

    try:
        # 先查一次 IsReady 作为基线
        ready = arm.IsReady()
        logger.info("[before] IsReady = %s", ready)

        if args.action == "press":
            ok = arm.SoftStop(active=True)
            logger.info("[press] SoftStop(active=True) ok=%s", ok)
            ready_after = arm.IsReady()
            logger.info("[press] IsReady = %s (按下后应为 False)", ready_after)

            if args.hold_sec > 0.0:
                logger.info("保持软急停 %s 秒...", args.hold_sec)
                time.sleep(args.hold_sec)
                ok = arm.SoftStop(active=False)
                logger.info("[release] SoftStop(active=False) ok=%s", ok)
                ready_after = arm.IsReady()
                logger.info("[release] IsReady = %s (弹起后应为 True)", ready_after)

        elif args.action == "release":
            ok = arm.SoftStop(active=False)
            logger.info("[release] SoftStop(active=False) ok=%s", ok)
            ready_after = arm.IsReady()
            logger.info("[release] IsReady = %s", ready_after)

        elif args.action == "toggle":
            cur = arm.IsReady()
            next_active = not cur
            ok = arm.SoftStop(active=next_active)
            logger.info("[toggle] IsReady=%s → SoftStop(active=%s) ok=%s", cur, next_active, ok)
            ready_after = arm.IsReady()
            logger.info("[toggle] IsReady = %s", ready_after)

        elif args.action == "status":
            logger.info("[status] IsReady = %s", arm.IsReady())

    except KeyboardInterrupt:
        logger.info("Ctrl+C 中断 — 尝试弹起软急停")
        try:
            arm.SoftStop(active=False)
        except Exception as e:
            logger.warning("弹起失败: %s", e)
    finally:
        arm.disconnect()
        logger.info("已断开连接")
    return 0


if __name__ == "__main__":
    sys.exit(main())


# 急停按下
# python soft_stop.py --action press
# 急停释放
# python soft_stop.py --action release