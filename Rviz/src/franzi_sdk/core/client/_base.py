# -*- coding: utf-8 -*-
"""ArmClient 传输层：connect + rpc() 直达网络（最多 2 层调用）。"""

from __future__ import annotations

from typing import Any, Dict, Optional

import core.libs.inbc_rpc as inbc_rpc
from core.libs.inbc_rpc import exceptions as inbc_rpc_exceptions
from core.libs.inbc_rpc import rpc_trace

from core import types

TimeoutMs = Optional[int]


class ArmClientBase:
    """``arm_sdk.<PascalCase>`` RPC 客户端。"""

    def __init__(
        self,
        host: str = "192.168.23.30",
        port: int = 8000,
        *,
        timeout_ms: int = 3000,
        connect_timeout_sec: float = 5.0,
        auto_reconnect: bool = True,
        auto_heartbeat: bool = True,
        heartbeat_interval_sec: float = 5.0,
        close_on_context_exit: bool = False,
        raise_on_error: bool = True,
        rpc_trace: bool = False,
    ) -> None:
        self._host = host
        self._port = port
        self._timeout_ms = timeout_ms
        self._connect_timeout_sec = connect_timeout_sec
        self._close_on_context_exit = close_on_context_exit
        self._raise_on_error = raise_on_error
        self._want_auto_reconnect = auto_reconnect
        self._want_auto_heartbeat = auto_heartbeat
        self._heartbeat_interval_sec = heartbeat_interval_sec
        self._transport_options_applied = False
        self._rpc = inbc_rpc.RpcClient(host, port)
        self._rpc.set_connect_timeout(connect_timeout_sec)
        try:
            from core import ensure_default_logging

            ensure_default_logging()
        except ImportError:
            pass
        if rpc_trace:
            self._rpc.enable_rpc_trace(True)
        self.connect(self._host, self._port, self._connect_timeout_sec)

    def enable_rpc_trace(
        self,
        enable: bool = True,
        sink: Optional[rpc_trace.TraceSink] = None,
    ) -> None:
        self._rpc.enable_rpc_trace(enable, sink=sink)

    def _apply_transport_options(self) -> None:
        if self._transport_options_applied:
            return
        if self._want_auto_reconnect:
            self._rpc.enable_auto_reconnect(True)
        if self._want_auto_heartbeat:
            self._rpc._heartbeat_interval_sec = self._heartbeat_interval_sec
            self._rpc.enable_auto_heartbeat(True)
        self._transport_options_applied = True

    def connect(
        self,
        host: Optional[str] = None,
        port: Optional[int] = None,
        timeout_sec: Optional[float] = None,
    ) -> bool:
        ok = self._rpc.connect(
            host=host,
            port=port,
            timeout_sec=timeout_sec if timeout_sec is not None else self._connect_timeout_sec,
        )
        if ok:
            self._apply_transport_options()
        else:
            failure = self._rpc.get_last_connect_failure()
            if failure is not None:
                try:
                    from core.logging import log_connect_failure

                    log_connect_failure(failure)
                except ImportError:
                    pass
        return ok

    def get_last_connect_failure(self) -> Optional[Dict[str, Any]]:
        return self._rpc.get_last_connect_failure()

    def abort_connect(self) -> None:
        self._rpc.abort_connect()

    def disconnect(self) -> None:
        self._rpc.close()

    def shutdown(self) -> None:
        self._rpc.shutdown()

    def is_connected(self) -> bool:
        return self._rpc.has_connected()

    def set_keepalive(self, enable: bool = True, interval_sec: float = 5.0) -> None:
        self._want_auto_heartbeat = enable
        self._heartbeat_interval_sec = interval_sec
        if self.is_connected():
            self._rpc._heartbeat_interval_sec = interval_sec
            self._rpc.enable_auto_heartbeat(enable)

    def set_auto_reconnect(self, enable: bool = True) -> None:
        self._want_auto_reconnect = enable
        if self.is_connected():
            self._rpc.enable_auto_reconnect(enable)

    def set_rpc_timeout(self, timeout_ms: int) -> None:
        self._timeout_ms = timeout_ms

    @property
    def rpc_transport(self) -> inbc_rpc.RpcClient:
        return self._rpc

    def __enter__(self) -> ArmClientBase:
        if not self.connect():
            from core.errors import SdkError

            raise SdkError("rpc.connect_failed", detail=f"{self._host}:{self._port}", log=False)
        return self

    def __exit__(self, *args: Any) -> None:
        if self._close_on_context_exit:
            self.disconnect()

    def rpc(self, method: str, *args: Any, timeout_ms: TimeoutMs = None) -> Any:
        """``arm_sdk.<method>`` → TCP msgpack（公开 API 的唯一下一层）。"""
        if not self.is_connected():
            raise inbc_rpc_exceptions.InbcRpcConnectionError("ArmClient not connected")
        rsp = self._rpc.call(
            types.RPC_PREFIX + method,
            *args,
            timeout_ms=timeout_ms if timeout_ms is not None else self._timeout_ms,
        )
        if self._raise_on_error and isinstance(rsp, dict):
            st = rsp.get("status")
            if st is not None and int(st) != types.SdkStatus.OK:
                raise types.ArmSdkError(int(st), response=rsp)
        return rsp

    @staticmethod
    def status_of(rsp: Any) -> int:
        if isinstance(rsp, dict) and "status" in rsp:
            return int(rsp["status"])
        return 0
