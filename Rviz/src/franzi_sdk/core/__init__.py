# -*- coding: utf-8 -*-
"""
core — SDK 核心横切能力（日志等）。

日志命名空间为 ``core``；SDK 内部输出均经此模块，二开可承接并定制落盘。
"""

from core.logging import (
    LOGGER_NAME,
    SdkCoreFormatter,
    attach_rotating_file_handler,
    configure_default_logging,
    ensure_default_logging,
    get_logger,
    log_connect_failure,
    rpc_trace_sink,
    set_log_format,
)

__all__ = [
    "LOGGER_NAME",
    "SdkCoreFormatter",
    "configure_default_logging",
    "ensure_default_logging",
    "get_logger",
    "set_log_format",
    "attach_rotating_file_handler",
    "rpc_trace_sink",
    "log_connect_failure",
]

# 首次 import core 时保证控制台有默认输出
ensure_default_logging()
logger = get_logger("core")