"""
INBC Python RPC SDK — 与 rest_rpc (cinatra) 协议兼容的 TCP 客户端。

通讯层: `client.RpcClient`
编解码: `codec` / `protocol` / `md5_hash`
"""

from .client import RpcClient
from . import rpc_trace
from .exceptions import (
    InbcRpcConnectionError,
    InbcRpcError,
    InbcRpcRemoteError,
    InbcRpcTimeoutError,
)
from .md5_hash import md5_hash32
from .protocol import RequestType, DEFAULT_TIMEOUT_MS

__all__ = [
    "RpcClient",
    "InbcRpcError",
    "InbcRpcConnectionError",
    "InbcRpcTimeoutError",
    "InbcRpcRemoteError",
    "md5_hash32",
    "RequestType",
    "DEFAULT_TIMEOUT_MS",
]

__version__ = "0.1.0"
