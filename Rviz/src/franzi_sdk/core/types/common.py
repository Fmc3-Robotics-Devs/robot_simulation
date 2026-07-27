# -*- coding: utf-8 -*-
"""
common — 连接默认值、返回码枚举。

业务错误码文案与异常见 ``errors`` 模块。
对应服务端：sdk_server_module/service/sdk_service_core.h（SdkStatus）。
"""

from __future__ import annotations

from enum import IntEnum

RPC_PREFIX = "arm_sdk."
"""RPC 方法前缀；完整名为 arm_sdk.<suffix>。"""


class SdkStatus(IntEnum):
    """业务返回码（响应字段 status，与 C++ SdkStatus 同值）。"""

    OK = 0
    SYNTAX_ERROR = -1
    PARAM_COUNT_MISMATCH = -2
    PARAM_INVALID = -3
    UNSUPPORTED = -4
    NOT_READY = -5


# 兼容旧 import 路径
from core.errors import ArmSdkError, sdk_status_message, sdk_status_name  # noqa: E402

__all__ = [
    "RPC_PREFIX",
    "SdkStatus",
    "ArmSdkError",
    "sdk_status_message",
    "sdk_status_name",
]
