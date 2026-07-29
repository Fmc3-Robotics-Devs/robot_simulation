"""SDK 异常定义。"""

from typing import Optional


def _rpc_msg(code: str, detail: str = "") -> str:
    from core.errors import format_error

    return format_error(code, detail=detail)


class InbcRpcError(Exception):
    """rest_rpc 协议或客户端逻辑错误基类。"""

    code: str = "rpc.generic"

    def __init__(self, message: str = "", *, code: Optional[str] = None, detail: str = "") -> None:
        self.code = code or self.code
        text = _rpc_msg(self.code, detail=detail or message)
        super().__init__(text)


class InbcRpcConnectionError(InbcRpcError):
    """TCP 连接失败或已断开。"""

    code = "rpc.not_connected"

    def __init__(
        self,
        message: str = "",
        *,
        code: Optional[str] = None,
        detail: str = "",
        reason: str = "unknown",
        errno: Optional[int] = None,
    ) -> None:
        self.reason = reason
        self.errno = errno
        super().__init__(message, code=code, detail=detail or message)


class InbcRpcTimeoutError(InbcRpcError):
    """同步/异步调用等待响应超时。"""

    code = "rpc.timeout"


class InbcRpcRemoteError(InbcRpcError):
    """服务端返回 result_code != OK。"""

    code = "rpc.remote"

    def __init__(self, message: str = "", code=None, *, detail: str = "") -> None:
        self.remote_code = code
        super().__init__(message, detail=detail or message)
