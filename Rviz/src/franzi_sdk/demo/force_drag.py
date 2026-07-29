#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ---------------------------------------------------------------------------
# Python SDK 力控测试用例 2：拖动示教进入与退出测试
# ---------------------------------------------------------------------------
# 测试目的：
# 1. 验证 StartDragMode 接口能否正常下发，使机器人进入拖动示教模式。
# 2. 验证 ExitDragMode 接口能否正常下发，使机器人退出拖动示教模式并保持当前位置。
# 3. 验证程序异常退出或 Ctrl+C 中断时，能够自动调用 ExitDragMode，避免机器人一直停留在拖动示教模式。
#
# 测试力控前请确认：
# 1. 机器人已在零位完成关节扭矩传感器清零；
# 2. 机器人动力学参数已完成辨识；
# 3. 机器人周围环境安全，拖动过程中操作者应保持急停按钮可用。
#
# 注意事项：
# 1. 运行本测试用例后，机器人将进入拖动示教
# 2. 本测试用例默认测试右臂（side=1），如需测试左臂请将 side 参数修改为 0。
# 3. 本测试用例默认拖动15秒后自动退出拖动示教模式，如需调整拖动时间请修改 sleep(15) 中的数值，也可自行加入键盘按键监听来控制退出时机。
# 4. 当前版本将 record_enable 设为 False，暂不支持轨迹记录。
# ------------------------------------------------------------------------------------------------------------------



from __future__ import annotations
import os
import sys
from time import sleep

_SDK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SDK_ROOT not in sys.path:
    sys.path.insert(0, _SDK_ROOT)

from core.client import ArmClient
from core.types import sdk_status_message
from core import logger

HOST = "192.168.23.30"
PORT = 8000

arm = ArmClient(host=HOST, port=PORT, raise_on_error=False)

# side=0 -> left arm
# side=1 -> right arm
side = 0

drag_enabled = False
exit_drag_called = False

try:
    logger.info("StartDragMode ...")
    ret = arm.StartDragMode(side=side, record_enable=True)
    logger.info("StartDragMode ret=%s, msg=%s", ret, sdk_status_message(ret))

    if ret != 0:
        logger.error("StartDragMode failed, skip ExitDragMode test")
    else:
        drag_enabled = True

        logger.info("DragMode enabled, keep for 10 seconds ...")
        sleep(15) 

        logger.info("ExitDragMode ...")
        ret_disable = arm.ExitDragMode(side=side)
        logger.info("ExitDragMode ret=%s, msg=%s", ret_disable, sdk_status_message(ret_disable))

        if ret_disable == 0:
            drag_enabled = False
            exit_drag_called = True

            logger.info("ExitDragMode succeeded.")
            logger.info("Keep program alive. Robot should remain in ExitDragMode hold state.")

            while True:
                sleep(1)

        else:
            logger.error("ExitDragMode failed")

except KeyboardInterrupt:
    logger.info("Ctrl+C 中断")

    if drag_enabled:
        logger.info("Ctrl+C detected while DragMode is active, try to ExitDragMode ...")
        ret_disable = arm.ExitDragMode(side=side)
        logger.info("ExitDragMode ret=%s, msg=%s", ret_disable, sdk_status_message(ret_disable))

        if ret_disable == 0:
            drag_enabled = False
            exit_drag_called = True

finally:
    if drag_enabled:
        logger.info("Program exiting while DragMode is still active, try to ExitDragMode ...")
        ret_disable = arm.ExitDragMode(side=side)
        logger.info("ExitDragMode ret=%s, msg=%s", ret_disable, sdk_status_message(ret_disable))
        drag_enabled = False

    arm.disconnect()
    logger.info("已断开连接")