# -*- coding: utf-8 -*-
"""
errors — SDK 统一错误码表（编号、码值、说明、解决办法）。

编号从 A1000 递增；抛出异常或建连失败时引用此处的码，
SDK 会自动写入 ``core`` 日志（编号 + 说明 + 解决办法）。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Union

ErrorCode = Union[int, str]


@dataclass(frozen=True)
class ErrorDef:
    """单条错误定义。"""

    no: str
    code: ErrorCode
    name: str
    message: str
    solution: str


def _e(no: str, code: ErrorCode, name: str, message: str, solution: str) -> ErrorDef:
    return ErrorDef(no, code, name, message, solution)


# ------------------------------------------------------------------ 业务返回码（arm_sdk 响应 status，与 C++ SdkStatus 同值）
ERRORS: Dict[ErrorCode, ErrorDef] = {
    0: _e("A1000", 0, "OK", "成功", "无需处理。"),
    -1: _e("A1001", -1, "SYNTAX_ERROR", "内部/语法错误", "检查请求参数格式；仍失败则联系 SDK 维护方。"),
    -2: _e(
        "A1002",
        -2,
        "PARAM_COUNT_MISMATCH",
        "参数个数不匹配",
        "核对 RPC 参数个数是否与 sdk_types.h 定义一致。",
    ),
    -3: _e("A1003", -3, "PARAM_INVALID", "参数非法", "检查 side、关节维数、枚举取值是否在合法范围。"),
    -4: _e("A1004", -4, "UNSUPPORTED", "接口不支持", "确认固件版本是否包含该 RPC；Mock 桩可能未实现。"),
    -5: _e(
        "A1005",
        -5,
        "NOT_READY",
        "执行层未就绪",
        "确认 SdkServerModule 与依赖模块已启动；业务 RPC 返回 -5 时查 INO/模块 enable 与初始化顺序。",
    ),
    -8: _e(
        "A1020",
        -8,
        "EMERGENCY_STOP_ACTIVE",
        "软急停有效，禁止上使能",
        "确认急停按钮/安全回路已复位；调用 GetEmergencyStopState 查看 estop_active；复位后再 SetMechUnitLifecycle(ENABLE)。",
    ),
    # ------------------------------------------------------------------ TCP 建连（connect_failure.reason）
    "refused": _e(
        "A1006",
        "refused",
        "CONN_REFUSED",
        "目标端口无监听或服务未启动",
        "确认控制器已启动 SdkServerModule，端口与 inbcrt.yaml 中 rpc_server.port 一致（默认 8000）。",
    ),
    "timeout": _e(
        "A1007",
        "timeout",
        "CONN_TIMEOUT",
        "TCP 握手超时",
        "检查 IP/网段、防火墙与网线；对端负载过高时可增大 connect 超时。",
    ),
    "connect_wait_timeout": _e(
        "A1008",
        "connect_wait_timeout",
        "CONN_WAIT_TIMEOUT",
        "限定时间内未完成建连",
        "确认 SdkServer 已就绪；必要时增大 connect(timeout_sec=...)。",
    ),
    "unreachable": _e(
        "A1009",
        "unreachable",
        "CONN_UNREACHABLE",
        "主机或路由不可达",
        "核对控制器 IP、子网掩码与路由；ping 测试网络连通性。",
    ),
    "network_error": _e(
        "A1010",
        "network_error",
        "CONN_NETWORK",
        "系统网络错误",
        "检查网卡、权限与地址有效性；Windows 可查看 WSA 错误码。",
    ),
    "unknown": _e(
        "A1011",
        "unknown",
        "CONN_UNKNOWN",
        "未知建连错误",
        "查看日志 detail/errno；仍无法定位则抓包或联系维护方。",
    ),
    # ------------------------------------------------------------------ 上位机校验（connect 后 RPC 探测）
    "rpc_timeout": _e(
        "A1012",
        "rpc_timeout",
        "RPC_VERIFY_TIMEOUT",
        "TCP 已连但 RPC 校验超时",
        "端口可能被非 SdkServer 进程占用；确认 8000 为机械臂 RPC 服务。",
    ),
    "rpc_verify": _e(
        "A1013",
        "rpc_verify",
        "RPC_VERIFY_FAIL",
        "端口可连但不是机械臂 RPC 服务",
        "确认加载 SdkServerModule 且注册了 arm_sdk.* 方法。",
    ),
    # ------------------------------------------------------------------ RPC 传输层
    "rpc.not_connected": _e(
        "A1014",
        "rpc.not_connected",
        "RPC_NOT_CONNECTED",
        "未连接或连接已断开",
        "先调用 connect()；长连接断开后需重连或开启 auto_reconnect。",
    ),
    "rpc.connect_failed": _e(
        "A1015",
        "rpc.connect_failed",
        "RPC_CONNECT_FAILED",
        "TCP 建连失败",
        "查看 get_last_connect_failure() 与日志中的 conn.* 错误码。",
    ),
    "rpc.timeout": _e(
        "A1016",
        "rpc.timeout",
        "RPC_TIMEOUT",
        "RPC 等待响应超时",
        "增大 timeout_ms；检查控制器是否阻塞或未注册该 RPC。",
    ),
    "rpc.connection_closed": _e(
        "A1017",
        "rpc.connection_closed",
        "RPC_CONNECTION_CLOSED",
        "连接被对端关闭",
        "检查 SdkServer 是否重启；可启用 auto_reconnect 自动重连。",
    ),
    "rpc.remote": _e(
        "A1018",
        "rpc.remote",
        "RPC_REMOTE",
        "服务端返回协议层错误",
        "确认 RPC 名与参数；查看服务端日志。",
    ),
    "rpc.generic": _e(
        "A1019",
        "rpc.generic",
        "RPC_GENERIC",
        "RPC 客户端内部错误",
        "查看日志 detail；重复出现请联系维护方。",
    ),
}

_UNKNOWN = _e("A1999", "?", "UNKNOWN", "未知错误", "查看日志 detail 或联系维护方。")

# 编号 → 码值 反向索引
ERRORS_BY_NO: Dict[str, ErrorDef] = {err.no: err for err in ERRORS.values()}


def _normalize_key(code: ErrorCode) -> ErrorCode:
    if isinstance(code, str) and code.lstrip("-").isdigit():
        return int(code)
    return code


def get_error(code: ErrorCode) -> ErrorDef:
    """按码查定义；未知码返回通用占位。"""
    key = _normalize_key(code)
    if key not in ERRORS:
        return _e("A1999", key, "UNKNOWN", f"未知错误码 {code!r}", _UNKNOWN.solution)
    return ERRORS[key]


def get_error_by_no(no: str) -> ErrorDef:
    """按编号查定义，如 ``A1005``。"""
    return ERRORS_BY_NO.get(no.upper(), _UNKNOWN)


def format_error(code: ErrorCode, *, detail: str = "") -> str:
    """格式化为「编号 + 码 + 说明 + 解决办法」单行文案。"""
    err = get_error(code)
    head = f"[{err.no}] {err.name}({err.code}): {err.message}"
    if detail:
        head = f"{head} — {detail}"
    return f"{head} | 解决办法: {err.solution}"


def log_error(code: ErrorCode, *, detail: str = "", logger: Any = None) -> str:
    """写入 core.arm 日志并返回完整说明。"""
    message = format_error(code, detail=detail)
    try:
        from core import get_logger

        log = logger if logger is not None else get_logger("arm")
        log.error(message)
    except ImportError:
        pass
    return message


def format_connect_failure(failure: Dict[str, Any]) -> str:
    """将 get_last_connect_failure() 字典格式化为说明（含编号与解决办法）。"""
    reason = str(failure.get("reason", "unknown"))
    host = failure.get("host", "")
    port = failure.get("port", "")
    target = f"{host}:{port}" if host or port else "?"
    detail_parts = [f"TCP 建连失败 {target}"]
    errno_code = failure.get("errno")
    if errno_code is not None:
        detail_parts.append(f"errno={errno_code}")
    extra = failure.get("detail") or failure.get("message", "")
    if extra and str(extra) not in detail_parts[-1]:
        detail_parts.append(str(extra))
    detail = "；".join(detail_parts)
    return format_error(reason, detail=detail)


# ------------------------------------------------------------------ 异常（抛出时自动打日志）
class SdkError(Exception):
    """SDK 统一异常基类。"""

    def __init__(
        self,
        code: ErrorCode,
        detail: str = "",
        *,
        response: Any = None,
        log: bool = True,
    ) -> None:
        self.code: ErrorCode = _normalize_key(code)
        self.error = get_error(self.code)
        self.no = self.error.no
        self.response = response
        if log:
            log_error(self.code, detail=detail)
        super().__init__(format_error(self.code, detail=detail))


class ArmSdkError(SdkError):
    """arm_sdk 业务 status != 0（raise_on_error=True 时抛出）。"""

    def __init__(self, status: int, message: str = "", *, response: Any = None, log: bool = True) -> None:
        self.status = int(status)
        super().__init__(self.status, detail=message, response=response, log=log)

    @property
    def status_name(self) -> str:
        return self.error.name


def sdk_status_name(status: int) -> str:
    """返回 status 枚举名。"""
    return get_error(int(status)).name


def sdk_status_message(status: int) -> str:
    """将 status 格式化为可读字符串（含编号与解决办法）。"""
    return format_error(int(status))
