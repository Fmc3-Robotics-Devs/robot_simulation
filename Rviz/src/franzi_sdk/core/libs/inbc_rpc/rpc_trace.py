# -*- coding: utf-8 -*-
"""
rpc_trace — RpcClient 收发层的单行 RPC 日志。

格式（和终端约定对齐）::

    client-->>server:get_version, request:{}, response:"1.0.0", use_time(ms):1.53

说明：
  - 只在 RpcClient.call / async_call 路径里触发，心跳空帧不走这里
  - func 展示名会剥掉 arm_sdk. 前缀，日志短一点
  - 要自定义输出就传 sink；否则默认写入 ``core.rpc`` 日志（见 core.logging）
"""

from __future__ import annotations

import json
from typing import Any, Callable, Optional, Tuple

TraceSink = Callable[[str, Any, Any, float, Optional[str]], None]

# arm_sdk 注册名前缀；剥掉后日志里只留方法短名
_RPC_PREFIX_STRIP = "arm_sdk."


def request_view(args: Tuple[Any, ...]) -> Any:
    """
    把 call(func, *args) 里的 args 收成一块好序列化的对象。

    无参 → {}；单参 → 直接展开；多参 → list。
    """
    if not args:
        return {}
    if len(args) == 1:
        return args[0]
    return list(args)


def func_short_name(func: str) -> str:
    """日志里用的短函数名。"""
    if func.startswith(_RPC_PREFIX_STRIP):
        return func[len(_RPC_PREFIX_STRIP) :]
    if "." in func:
        return func.rsplit(".", 1)[-1]
    return func


def format_payload(obj: Any, *, compact: bool = True, max_bytes_preview: int = 64) -> str:
    """request/response 压成一行 JSON；bytes 只打长度。"""
    if obj is None:
        return "null"
    if isinstance(obj, (bytes, bytearray)):
        n = len(obj)
        return f"<bytes len={n}>"
    if isinstance(obj, str) and len(obj) > 256:
        return json.dumps(obj[:256] + "...", ensure_ascii=False)
    try:
        if compact:
            return json.dumps(
                obj, ensure_ascii=False, separators=(",", ":"), default=_json_default
            )
        return json.dumps(
            obj, ensure_ascii=False, indent=2, default=_json_default
        )
    except (TypeError, ValueError):
        return repr(obj)


def _json_default(o: Any) -> Any:
    if isinstance(o, (bytes, bytearray)):
        return f"<bytes len={len(o)}>"
    raise TypeError(f"{type(o).__name__} is not JSON serializable")


def log_exchange(
    func: str,
    request: Any,
    response: Any,
    use_time_ms: float,
    *,
    error: Optional[str] = None,
    sink: Optional[TraceSink] = None,
) -> None:
    """
    打一条 RPC 交换记录。

    func 是服务端完整注册名（例如 arm_sdk.IsReady）；
    use_time_ms 从发请求到收齐 body 算起。
    """
    if sink is not None:
        sink(func, request, response, use_time_ms, error)
        return

    name = func_short_name(func)
    req_s = format_payload(request)
    resp_s = format_payload(response) if response is not None else "null"
    parts = [f"client-->>server:{name}", f"request:{req_s}", f"response:{resp_s}"]
    if error:
        parts.append(f"error:{error!s}")
    parts.append(f"use_time(ms):{use_time_ms:.2f}")
    line = ", ".join(parts)
    try:
        from core.logging import ensure_default_logging, get_logger

        ensure_default_logging()
        get_logger("rpc").info("%s", line)
    except ImportError:
        import sys

        print(line, file=sys.stderr, flush=True)
