#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import math
import os
import sys
from time import sleep

_SDK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SDK_ROOT not in sys.path:
    sys.path.insert(0, _SDK_ROOT)

from core import logger
from core.client import ArmClient
from core.types import ArmSide, sdk_status_message


HOST = "192.168.23.30"
PORT = 8000

DEG2RAD = math.pi / 180.0


def main():
    arm = ArmClient(host=HOST, port=PORT, raise_on_error=False)

    # side=ArmSide.LEFT  -> 左臂
    # side=ArmSide.RIGHT -> 右臂
    side = ArmSide.LEFT

    repeat_times = 1      # 往返次数
    move_wait_s = 4.0     # 每次 MoveJ 后等待时间，按实际速度可调整

    # J4 设置为 40 deg，避免违反 J4 >= 20 deg 的限位
    q_start = [
        0.0,
        0.0,
        0.0,
        40.0 * DEG2RAD,
        0.0,
        0.0,
        0.0,
    ]

    # 右臂测试目标点
    q_target_right = [
        -0.3,
        -0.8,
        0.0,
        1.6,
        0.0,
        0.0,
        0.0,
    ]

    # 左臂测试目标点：注意左臂 J2 限位为 [-30, 180] deg，
    # 所以这里把 J2 改为正方向，避免 -0.8 rad 超限。
    q_target_left = [
        -0.3,
        0.8,
        0.0,
        1.6,
        0.0,
        0.0,
        0.0,
    ]

    q_target = q_target_left if side == ArmSide.LEFT else q_target_right

    try:
        # ------------------------------------------------------------------
        # 急停检测
        # ------------------------------------------------------------------
        estop = arm.GetEmergencyStopState()
        logger.info(
            "急停状态: status=%s estop_active=%s reg=0x%04x ts_us=%s",
            estop.get("status"),
            estop.get("estop_active"),
            int(estop.get("estop_reg", 0)),
            estop.get("timestamp_us"),
        )

        if arm.IsEmergencyStopActive():
            logger.error("检测到急停有效，跳过 MoveJ 测试")
            return

        logger.info("========== Pure MoveJ trajectory test without impedance ==========")

        # ------------------------------------------------------------------
        # 先运动到安全起点
        # ------------------------------------------------------------------
        logger.info("Move to q_start ...")
        ret_movej = arm.MoveJ(side=side, joint_path=[q_start])
        logger.info("MoveJ q_start ret=%s, msg=%s", ret_movej, sdk_status_message(ret_movej))

        if ret_movej != 0:
            logger.error("MoveJ q_start failed, stop test")
            return

        sleep(move_wait_s)

        # ------------------------------------------------------------------
        # 往返运动：q_start -> q_target -> q_start
        # ------------------------------------------------------------------
        for i in range(repeat_times):
            logger.info("\n---------- MoveJ cycle %d/%d: q_start -> q_target ----------",
                        i + 1,
                        repeat_times)

            ret_movej = arm.MoveJ(side=side, joint_path=[q_target])
            logger.info("MoveJ q_target ret=%s, msg=%s", ret_movej, sdk_status_message(ret_movej))

            if ret_movej != 0:
                logger.error("MoveJ q_target failed at cycle %d", i + 1)
                break

            sleep(move_wait_s)

            logger.info("\n---------- MoveJ cycle %d/%d: q_target -> q_start ----------",
                        i + 1,
                        repeat_times)

            ret_movej = arm.MoveJ(side=side, joint_path=[q_start])
            logger.info("MoveJ q_start ret=%s, msg=%s", ret_movej, sdk_status_message(ret_movej))

            if ret_movej != 0:
                logger.error("MoveJ q_start failed at cycle %d", i + 1)
                break

            sleep(move_wait_s)

        logger.info("Pure MoveJ trajectory test finished.")

    except KeyboardInterrupt:
        logger.info("\n\n===== 检测到 Ctrl+C 中断程序 =====")

    finally:
        arm.disconnect()
        logger.info("已断开连接，程序结束")


if __name__ == "__main__":
    main()