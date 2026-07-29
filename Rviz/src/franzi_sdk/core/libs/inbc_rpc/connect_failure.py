# -*- coding: utf-8 -*-
"""TCP 建连失败原因分类（供 RpcClient.connect / 上位机展示）。"""

from __future__ import annotations

import errno
from typing import Any, Dict, Optional

# reason 取值（稳定契约，与 errors.ERRORS 键一致）
REASON_REFUSED = "refused"
REASON_TIMEOUT = "timeout"
REASON_WAIT_TIMEOUT = "connect_wait_timeout"
REASON_UNREACHABLE = "unreachable"
REASON_NETWORK = "network_error"
REASON_UNKNOWN = "unknown"


def classify_connect_oserror(ex: OSError) -> str:
    """将 socket.connect 的 OSError 映射为 reason 字符串。"""
    en = ex.errno
    if en is None:
        return REASON_NETWORK

    for code, reason in (
        (10060, REASON_TIMEOUT),
        (10061, REASON_REFUSED),
        (10065, REASON_UNREACHABLE),
        (10051, REASON_UNREACHABLE),
        (10049, REASON_UNREACHABLE),
        (10013, REASON_NETWORK),
    ):
        if en == code:
            return reason

    if en in (errno.ETIMEDOUT, getattr(errno, "WSAETIMEDOUT", -1)):
        return REASON_TIMEOUT
    if en in (errno.ECONNREFUSED, getattr(errno, "WSAECONNREFUSED", -1)):
        return REASON_REFUSED
    if en in (
        errno.EHOSTUNREACH,
        errno.ENETUNREACH,
        errno.EADDRNOTAVAIL,
        getattr(errno, "WSAEHOSTUNREACH", -1),
        getattr(errno, "WSAENETUNREACH", -1),
    ):
        return REASON_UNREACHABLE

    return REASON_NETWORK


def failure_dict(
    reason: str,
    message: str,
    *,
    host: str = "",
    port: int = 0,
    errno_code: Optional[int] = None,
    detail: str = "",
) -> Dict[str, Any]:
    """构造 connect 失败详情字典。字段：reason, message, host, port, errno, detail。"""
    return {
        "reason": reason,
        "message": message,
        "host": host,
        "port": port,
        "errno": errno_code,
        "detail": detail or message,
    }


def format_connect_failure_message(failure: Dict[str, Any]) -> str:
    """将 failure_dict 格式化为说明（含解决办法）。"""
    from core.errors import format_connect_failure

    return format_connect_failure(failure)
