"""
msgpack 编解码层。

与 C++ `rest_rpc::rpc_service::msgpack_codec` 及 `client_util.hpp` 行为对齐：
  - 请求 body: msgpack 打包参数 tuple
  - 响应 body: msgpack tuple (result_code, ...) 其中 result_code==0 表示成功
  - 订阅推送: tuple (code, key, data)
"""

from __future__ import annotations

from typing import Any, Tuple

from .. import msgpack

from .exceptions import InbcRpcRemoteError

# 与 C++ result_code::OK 一致
RESULT_OK: int = 0


def pack_args(*args: Any) -> bytes:
    """
    打包 RPC 参数，等价于 C++ `codec.pack_args(args...)`。

    无参数时打包空 tuple ()。
    """
    return msgpack.packb(args, use_bin_type=True)


def pack_object(obj: Any) -> bytes:
    """打包单个对象（用于 publish 时先 pack 再作为 string 参数传递）。"""
    return msgpack.packb(obj, use_bin_type=True)


def unpack_raw(data: bytes) -> Any:
    """解包任意 msgpack 数据。"""
    return msgpack.unpackb(data, raw=False)


def _as_tuple(obj: Any) -> tuple:
    """
    C++ msgpack 序列化的 tuple 在 Python 侧常为 list，需统一为 tuple 再解析。
    """
    if isinstance(obj, tuple):
        return obj
    if isinstance(obj, list):
        return tuple(obj)
    raise InbcRpcRemoteError(
        f"invalid response format: expected tuple/list, got {type(obj).__name__}"
    )


def has_error(result: bytes) -> bool:
    """
    判断响应是否表示失败。

    对应 C++ `has_error(string_view)`：解包 (int,) 首元素非 0。
    """
    if not result:
        return True
    try:
        obj = _as_tuple(unpack_raw(result))
    except InbcRpcRemoteError:
        return True
    if len(obj) >= 1:
        return int(obj[0]) != RESULT_OK
    return True


def _unpack_tuple(data: bytes) -> tuple:
    return _as_tuple(unpack_raw(data))


def get_error_message(result: bytes) -> str:
    """从失败响应中取出错误字符串，对应 C++ `get_error_msg`。"""
    tp = _unpack_tuple(result)
    if len(tp) >= 2 and isinstance(tp[1], str):
        return tp[1]
    return str(tp)


def parse_result(result: bytes) -> Any:
    """
    解析成功响应的返回值。

    - void 调用：仅校验 code==0
    - 有返回值：tuple (0, value) 取 index 1
    """
    tp = _unpack_tuple(result)
    code = int(tp[0])
    if code != RESULT_OK:
        raise InbcRpcRemoteError(get_error_message(result), code=code)
    if len(tp) == 1:
        return None
    return tp[1]


def parse_sub_push(body: bytes) -> Tuple[int, str, bytes]:
    """
    解析订阅推送帧 body。

    对应 C++ `callback_sub`: unpack tuple (code, key, data)。
    data 在 C++ 中为 string，Python 侧统一为 bytes。
    """
    tp = _unpack_tuple(body)
    code = int(tp[0])
    key = tp[1] if isinstance(tp[1], str) else str(tp[1])
    payload = tp[2]
    if isinstance(payload, str):
        payload = payload.encode("utf-8")
    elif not isinstance(payload, (bytes, bytearray)):
        payload = msgpack.packb(payload, use_bin_type=True)
    return code, key, bytes(payload)
