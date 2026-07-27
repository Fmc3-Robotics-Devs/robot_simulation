"""
rest_rpc 线协议：帧头定义与打包/解包。

对应 C++:
  - tools/ref/rpc/include/rest_rpc/const_vars.h
  - rpc_client::write() / connection::read_head()
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum

# 魔数，用于校验帧合法性
MAGIC_NUM: int = 39

# 单帧 body 最大长度（与 C++ MAX_BUF_LEN 一致）
MAX_BODY_LEN: int = 1048576 * 10

# 帧头长度（#pragma pack(4) 下 sizeof(rpc_header) == 20）
# C++ 在 magic、req_type 之后有 2 字节填充，再跟 body_len。
HEAD_LEN: int = 20

# 与 C++ struct rpc_header 字段布局一致（含 2 字节对齐填充）
_HEADER_FMT = "<BB2xIQI"

# 默认 RPC 超时（毫秒），与 rest_rpc DEFAULT_TIMEOUT 一致
DEFAULT_TIMEOUT_MS: int = 5000

# 回调 req_id 最高位标记（C++ callback_id_ |= 1<<63）
CALLBACK_ID_FLAG: int = 1 << 63


class RequestType(IntEnum):
    """请求类型，写入帧头 req_type 字段。"""

    REQ_RES = 0   # 普通 RPC 请求/响应
    SUB_PUB = 1   # 订阅注册与服务端推送


@dataclass
class RpcHeader:
    """
    与 C++ `struct rpc_header` 内存布局一致（小端、4 字节对齐）。

    字段顺序: magic, req_type, body_len, req_id, func_id
    """

    magic: int
    req_type: RequestType
    body_len: int
    req_id: int
    func_id: int

    def pack(self) -> bytes:
        return struct.pack(
            _HEADER_FMT,
            self.magic & 0xFF,
            int(self.req_type) & 0xFF,
            self.body_len & 0xFFFFFFFF,
            self.req_id & 0xFFFFFFFFFFFFFFFF,
            self.func_id & 0xFFFFFFFF,
        )

    @classmethod
    def unpack(cls, data: bytes) -> RpcHeader:
        if len(data) < HEAD_LEN:
            raise ValueError(f"header too short: {len(data)}")
        magic, req_type, body_len, req_id, func_id = struct.unpack(
            _HEADER_FMT, data[:HEAD_LEN]
        )
        return cls(
            magic=magic,
            req_type=RequestType(req_type),
            body_len=body_len,
            req_id=req_id,
            func_id=func_id,
        )


def build_header(
    req_id: int,
    req_type: RequestType,
    body_len: int,
    func_id: int,
) -> bytes:
    """构造一帧完整的协议头字节流。"""
    return RpcHeader(MAGIC_NUM, req_type, body_len, req_id, func_id).pack()


def validate_header(header: RpcHeader) -> None:
    """校验魔数与 body 长度，非法时抛出 ValueError。"""
    if header.magic != MAGIC_NUM:
        raise ValueError(f"invalid magic: {header.magic}")
    if header.body_len > MAX_BODY_LEN:
        raise ValueError(f"body_len too large: {header.body_len}")
