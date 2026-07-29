#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""正逆解验证：cur_q -> Fk -> cur_pose -> IK -> tar_q -> Fk -> tar_pose。"""

from __future__ import annotations

import os
import sys

_SDK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SDK_ROOT not in sys.path:
    sys.path.insert(0, _SDK_ROOT)

from core.client import ArmClient
from core.types import sdk_status_message
from core import logger

# ------------------------------------------------------------------ 运行参数（按需修改）
HOST = "192.168.23.30"
PORT = 8000
SIDE = 2  # 0=左臂, 1=右臂

# 测试用关节角（rad）
cur_q = [-0.8, 1.6, -0.8, 0]

arm = ArmClient(host=HOST, port=PORT, raise_on_error=False)

try:
    logger.info("SDK version=%s IsReady=%s", arm.GetVersion(), arm.IsReady())
    logger.info("cur_q = %s", cur_q)

    rc, cur_pose = arm.Fk(SIDE, cur_q)
    logger.info("Fk(cur_q) rc=%s %s", rc, sdk_status_message(rc))
    logger.info("cur_pose = %s", cur_pose)

    rc, tar_q = arm.IK(SIDE, cur_pose)
    logger.info("IK(cur_pose) rc=%s %s", rc, sdk_status_message(rc))
    logger.info("tar_q = %s", tar_q)

    rc, tar_pose = arm.Fk(SIDE, tar_q)
    logger.info("Fk(tar_q) rc=%s %s", rc, sdk_status_message(rc))
    logger.info("tar_pose = %s", tar_pose)

except KeyboardInterrupt:
    logger.info("Ctrl+C 中断")
finally:
    arm.disconnect()
    logger.info("已断开连接")
