"""
rest_rpc 客户端：TCP 通讯 + 同步/异步 RPC + 订阅/发布。

行为对齐 `tools/ref/rpc/include/rest_rpc/rpc_client.hpp` 与官方 client demo。
"""

from __future__ import annotations

import socket
import struct
import threading
import time
from concurrent.futures import Future
from typing import Any, Callable, Dict, List, Optional, Tuple

from . import codec, rpc_trace
from .connect_failure import (
    REASON_WAIT_TIMEOUT,
    classify_connect_oserror,
    failure_dict,
)
from .exceptions import (
    InbcRpcConnectionError,
    InbcRpcError,
    InbcRpcTimeoutError,
)
from .md5_hash import md5_hash32
from .protocol import (
    CALLBACK_ID_FLAG,
    DEFAULT_TIMEOUT_MS,
    HEAD_LEN,
    RequestType,
    RpcHeader,
    build_header,
    validate_header,
)

# 发布 RPC 在服务端注册的固定函数名
_RPC_PUBLISH = "publish"
_RPC_PUBLISH_BY_TOKEN = "publish_by_token"


class RpcClient:
    """
    INBC rest_rpc Python 客户端。

    典型用法::

        client = RpcClient("127.0.0.1", 8000)
        client.enable_auto_reconnect()
        client.enable_auto_heartbeat()
        if not client.connect():
            raise SystemExit("connect failed")
        print(client.call("add", 1, 2))
        client.close()
    """

    def __init__(self, host: str = "", port: int = 0):
        self._host = host
        self._port = port
        self._sock: Optional[socket.socket] = None
        self._connected = threading.Event()
        self._stop = threading.Event()

        # 写队列：与 C++ outbox_ 类似，保证单连接顺序写
        self._write_lock = threading.Lock()
        self._outbox: List[Tuple[bytes, bytes]] = []  # (header, body)

        # 请求 ID 分配
        self._req_lock = threading.Lock()
        self._next_req_id = 0
        self._next_callback_id = 0

        # 同步调用：req_id -> (Event, result_bytes)
        self._pending: Dict[int, Tuple[threading.Event, List[Optional[bytes]]]] = {}
        # future 异步：req_id -> Future
        self._futures: Dict[int, Future] = {}
        # 回调异步：req_id -> (callback, timer)
        self._callbacks: Dict[int, Tuple[Callable, Optional[threading.Timer]]] = {}

        # 订阅：composite_key -> callback；以及重连后需重发的 (key, token)
        self._sub_lock = threading.Lock()
        self._subscribers: Dict[str, Callable[[bytes], None]] = {}
        self._sub_keys: List[Tuple[str, str]] = []

        self._reader_thread: Optional[threading.Thread] = None
        self._heartbeat_thread: Optional[threading.Thread] = None
        self._reconnect_thread: Optional[threading.Thread] = None
        self._connect_lock = threading.Lock()
        self._last_connect_error: Optional[Exception] = None
        self._last_connect_failure: Optional[Dict[str, Any]] = None
        self._connecting_sock: Optional[socket.socket] = None
        self._connect_abort_requested = False

        self._connect_timeout_sec = 3
        self._enable_reconnect = False
        self._reconnect_unlimited = False
        self._heartbeat_interval_sec = 5
        self._error_callback: Optional[Callable[[Exception], None]] = None

        # RPC 轨迹：在 call / async_call 收发层打印 func / request / response / 耗时
        self._rpc_trace = False
        self._rpc_trace_sink: Optional[rpc_trace.TraceSink] = None
        self._rpc_trace_meta: Dict[int, Tuple[str, Tuple[Any, ...], float]] = {}

    # ------------------------------------------------------------------ 配置

    def set_connect_timeout(self, seconds: float) -> None:
        """连接等待超时（秒），对应 C++ set_connect_timeout（毫秒）的语义。"""
        self._connect_timeout_sec = seconds

    def enable_auto_reconnect(self, enable: bool = True) -> None:
        """启用断线后自动重连（对应 enable_auto_reconnect）。"""
        self._enable_reconnect = enable
        self._reconnect_unlimited = enable

    def enable_auto_heartbeat(self, enable: bool = True) -> None:
        """启用心跳；间隔见 _heartbeat_interval_sec（C++ 默认约 5s）。"""
        if enable and self._heartbeat_thread is None and not self._stop.is_set():
            self._heartbeat_thread = threading.Thread(
                target=self._heartbeat_loop, name="inbc-rpc-heartbeat", daemon=True
            )
            self._heartbeat_thread.start()
        elif not enable:
            self._heartbeat_interval_sec = 0

    def set_error_callback(self, callback: Optional[Callable[[Exception], None]]) -> None:
        """网络错误回调，对应 set_error_callback。"""
        self._error_callback = callback

    def enable_rpc_trace(
        self,
        enable: bool = True,
        sink: Optional[rpc_trace.TraceSink] = None,
    ) -> None:
        """
        开关 RPC 单行轨迹；在 call / async_call 收发点打点。

        格式约定在 libs.inbc_rpc.rpc_trace；要自定义输出就传 sink。
        """
        self._rpc_trace = enable
        if sink is not None:
            self._rpc_trace_sink = sink
        elif not enable:
            self._rpc_trace_sink = None

    def _finish_rpc_trace(
        self,
        func: str,
        args: Tuple[Any, ...],
        raw: Optional[bytes],
        use_time_ms: float,
        *,
        error: Optional[str] = None,
        parsed: Any = None,
    ) -> None:
        if not self._rpc_trace or not func:
            return
        req = rpc_trace.request_view(args)
        if error is not None:
            resp: Any = parsed
            if resp is None and raw:
                try:
                    resp = codec.unpack_raw(raw)
                except Exception:
                    resp = f"<raw bytes len={len(raw)}>"
            rpc_trace.log_exchange(
                func, req, resp, use_time_ms, error=error, sink=self._rpc_trace_sink
            )
            return
        if parsed is not None:
            rpc_trace.log_exchange(
                func, req, parsed, use_time_ms, sink=self._rpc_trace_sink
            )
            return
        if raw:
            try:
                rpc_trace.log_exchange(
                    func,
                    req,
                    codec.parse_result(raw),
                    use_time_ms,
                    sink=self._rpc_trace_sink,
                )
            except Exception as ex:
                try:
                    preview = codec.unpack_raw(raw)
                except Exception:
                    preview = f"<raw bytes len={len(raw)}>"
                rpc_trace.log_exchange(
                    func,
                    req,
                    preview,
                    use_time_ms,
                    error=str(ex),
                    sink=self._rpc_trace_sink,
                )
        else:
            rpc_trace.log_exchange(
                func, req, None, use_time_ms, sink=self._rpc_trace_sink
            )

    def _trace_rpc_begin(
        self, req_id: int, func: str, args: Tuple[Any, ...]
    ) -> None:
        if self._rpc_trace and func:
            self._rpc_trace_meta[req_id] = (func, args, time.perf_counter())

    def _trace_rpc_end(self, req_id: int, raw: Optional[bytes], *, error: Optional[str] = None) -> None:
        meta = self._rpc_trace_meta.pop(req_id, None)
        if meta is None:
            return
        func, args, t0 = meta
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        self._finish_rpc_trace(func, args, raw, elapsed_ms, error=error)

    def update_addr(self, host: str, port: int) -> None:
        self._host = host
        self._port = port

    # ------------------------------------------------------------------ 连接

    def connect(
        self,
        host: Optional[str] = None,
        port: Optional[int] = None,
        timeout_sec: Optional[float] = None,
    ) -> bool:
        """
        建立 TCP 连接并启动读线程。

        :return: 是否在超时内连接成功
        """
        if host is not None:
            self._host = host
        if port is not None:
            self._port = port
        if not self._host or not self._port:
            raise InbcRpcConnectionError("host/port not set")

        if self.has_connected():
            return True

        timeout = timeout_sec if timeout_sec is not None else self._connect_timeout_sec
        self._connect_abort_requested = False
        self._connected.clear()
        self._last_connect_error = None
        self._last_connect_failure = None
        self._open_socket(timeout)

        if self._reader_thread is None or not self._reader_thread.is_alive():
            self._reader_thread = threading.Thread(
                target=self._read_loop, name="inbc-rpc-reader", daemon=True
            )
            self._reader_thread.start()

        if self._connect_abort_requested:
            return False

        if self.has_connected():
            return True

        if self._last_connect_failure is not None:
            return False

        deadline = time.monotonic() + timeout
        while not self.has_connected():
            if self._connect_abort_requested:
                return False
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self._last_connect_failure = failure_dict(
                    REASON_WAIT_TIMEOUT,
                    f"在 {timeout:g}s 内未完成 TCP 连接",
                    host=self._host,
                    port=int(self._port),
                )
                return False
            self._connected.wait(min(0.05, remaining))

        if self._connect_abort_requested:
            return False

        return self.has_connected()

    def abort_connect(self) -> None:
        """中断进行中的 connect（关闭半成品 socket，唤醒等待）。"""
        self._connect_abort_requested = True
        with self._connect_lock:
            pending = self._connecting_sock
            if pending is not None:
                try:
                    pending.close()
                except OSError:
                    pass
                self._connecting_sock = None
        self._connected.set()
        self.close()

    def get_last_connect_failure(self) -> Optional[Dict[str, Any]]:
        """
        最近一次 connect() 失败详情。

        字段：reason, message, host, port, errno, detail。
        reason 见 connect_failure 模块常量。
        """
        if self._last_connect_failure is not None:
            return dict(self._last_connect_failure)
        if self._last_connect_error is None:
            return None
        ex = self._last_connect_error
        if isinstance(ex, InbcRpcConnectionError):
            return failure_dict(
                ex.reason,
                str(ex),
                host=self._host,
                port=int(self._port),
                errno_code=ex.errno,
                detail=str(ex),
            )
        return failure_dict(
            "unknown",
            str(ex),
            host=self._host,
            port=int(self._port),
            detail=str(ex),
        )

    def async_connect(self, host: str, port: int) -> None:
        """非阻塞发起连接（在后台线程执行 connect）。"""
        threading.Thread(
            target=lambda: self.connect(host, port),
            name="inbc-rpc-async-connect",
            daemon=True,
        ).start()

    def has_connected(self) -> bool:
        return self._connected.is_set() and self._sock is not None

    def close(self) -> None:
        """关闭连接；不停止心跳/读线程守护（进程退出时自动结束）。"""
        self._connected.clear()
        if self._sock:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        self._clear_pending(InbcRpcConnectionError("connection closed"))

    def shutdown(self) -> None:
        """完全停止客户端（含心跳与读循环）。"""
        self._stop.set()
        self.close()

    # ------------------------------------------------------------------ 同步 RPC

    def call(self, func: str, *args: Any, timeout_ms: int = DEFAULT_TIMEOUT_MS) -> Any:
        """
        同步 RPC，对应 C++ `call<R>(rpc_name, args...)`。

        :param func: 服务端注册的函数名
        :param timeout_ms: 等待响应超时（毫秒）
        """
        if not self.has_connected():
            raise InbcRpcConnectionError("not connected")

        req_id = self._alloc_req_id()
        body = codec.pack_args(*args)
        func_id = md5_hash32(func)
        event = threading.Event()
        t0 = time.perf_counter()
        with self._req_lock:
            self._pending[req_id] = (event, [None])

        self._enqueue_write(req_id, RequestType.REQ_RES, body, func_id)

        if not event.wait(timeout_ms / 1000.0):
            elapsed_ms = (time.perf_counter() - t0) * 1000.0
            with self._req_lock:
                self._pending.pop(req_id, None)
            self._finish_rpc_trace(
                func,
                args,
                None,
                elapsed_ms,
                error=f"timeout after {timeout_ms}ms",
            )
            raise InbcRpcTimeoutError(f"call {func!r} timeout after {timeout_ms}ms")

        with self._req_lock:
            slot = self._pending.pop(req_id, None)
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        if slot is None or slot[1][0] is None:
            self._finish_rpc_trace(
                func, args, None, elapsed_ms, error="connection lost during call"
            )
            raise InbcRpcConnectionError("connection lost during call")
        raw = slot[1][0]
        try:
            result = codec.parse_result(raw)
        except Exception as ex:
            self._finish_rpc_trace(func, args, raw, elapsed_ms, error=str(ex))
            raise
        self._finish_rpc_trace(func, args, raw, elapsed_ms, parsed=result)
        return result

    def call_void(self, func: str, *args: Any, timeout_ms: int = DEFAULT_TIMEOUT_MS) -> None:
        """无返回值的同步 RPC，对应 C++ `call<void>`。"""
        self.call(func, *args, timeout_ms=timeout_ms)

    # ------------------------------------------------------------------ 异步 RPC

    def async_call_future(self, func: str, *args: Any) -> Future:
        """
        返回 concurrent.futures.Future，对应 C++ `async_call<FUTURE>`。
        结果字节在 Future 中，可用 `.result()` 后 `codec.parse_result`。
        """
        if not self.has_connected():
            raise InbcRpcConnectionError("not connected")

        req_id = self._alloc_req_id()
        fut: Future = Future()
        with self._req_lock:
            self._futures[req_id] = fut
        self._trace_rpc_begin(req_id, func, args)

        body = codec.pack_args(*args)
        self._enqueue_write(req_id, RequestType.REQ_RES, body, md5_hash32(func))
        return _RpcFuture(fut, req_id, self)

    def async_call(
        self,
        func: str,
        callback: Callable[[Optional[Exception], Any], None],
        *args: Any,
        timeout_ms: int = DEFAULT_TIMEOUT_MS,
    ) -> None:
        """
        回调式异步 RPC，对应 C++ `async_call<R, TIMEOUT>(name, cb, args...)`。

        callback(ec_or_none, result)：成功时 ec 为 None。
        """
        if not self.has_connected():
            callback(InbcRpcConnectionError("not connected"), None)
            return

        req_id = self._alloc_callback_id()
        body = codec.pack_args(*args)

        def on_timeout() -> None:
            with self._req_lock:
                entry = self._callbacks.pop(req_id, None)
            if entry:
                self._trace_rpc_end(
                    req_id, None, error=f"timeout after {timeout_ms}ms"
                )
                callback(InbcRpcTimeoutError(f"async_call {func!r} timeout"), None)

        timer: Optional[threading.Timer] = None
        if timeout_ms > 0:
            timer = threading.Timer(timeout_ms / 1000.0, on_timeout)
            timer.daemon = True
            timer.start()

        def deliver(data: bytes, err: Optional[Exception]) -> None:
            if timer:
                timer.cancel()
            if err:
                self._trace_rpc_end(req_id, None, error=str(err))
                callback(err, None)
                return
            try:
                result = codec.parse_result(data)
                self._trace_rpc_end(req_id, data)
                callback(None, result)
            except Exception as ex:
                self._trace_rpc_end(req_id, data, error=str(ex))
                callback(ex, None)

        with self._req_lock:
            self._callbacks[req_id] = (deliver, timer)
        self._trace_rpc_begin(req_id, func, args)

        self._enqueue_write(req_id, RequestType.REQ_RES, body, md5_hash32(func))

    # ------------------------------------------------------------------ 订阅 / 发布

    def subscribe(
        self,
        key: str,
        callback: Callable[[bytes], None],
        token: str = "",
    ) -> None:
        """
        订阅 topic，对应 C++ `subscribe(key, f)` 或 `subscribe(key, token, f)`。

        服务端推送到达后 callback 收到 msgpack payload 原始 bytes。
        """
        composite = key + token
        with self._sub_lock:
            if composite in self._subscribers:
                raise InbcRpcError(f"duplicated subscribe: {composite!r}")
            self._subscribers[composite] = callback
            self._sub_keys.append((key, token))

        if self.has_connected():
            self._send_subscribe(key, token)

    def publish(self, key: str, data: Any, timeout_ms: int = 3000) -> None:
        """
        向 topic 发布，对应 C++ `publish(key, t)`（经 RPC publish 转发）。
        """
        # 第三参数为二进制 blob，与 C++ std::string(buf.data(), buf.size()) 一致
        packed = codec.pack_object(data)
        self.call(_RPC_PUBLISH, key, "", packed, timeout_ms=timeout_ms)

    def publish_by_token(
        self, key: str, token: str, data: Any, timeout_ms: int = 3000
    ) -> None:
        """对应 C++ `publish_by_token`。"""
        packed = codec.pack_object(data)
        self.call(
            _RPC_PUBLISH_BY_TOKEN,
            key,
            token,
            packed,
            timeout_ms=timeout_ms,
        )

    # ------------------------------------------------------------------ 内部：通讯

    def _alloc_req_id(self) -> int:
        with self._req_lock:
            self._next_req_id += 1
            return self._next_req_id

    def _alloc_callback_id(self) -> int:
        with self._req_lock:
            self._next_callback_id += 1
            return self._next_callback_id | CALLBACK_ID_FLAG

    def _start_reconnect_thread(self) -> None:
        if not self._enable_reconnect or self._stop.is_set():
            return
        if self._reconnect_thread is not None and self._reconnect_thread.is_alive():
            return
        self._reconnect_thread = threading.Thread(
            target=self._reconnect_loop, name="inbc-rpc-reconnect", daemon=True
        )
        self._reconnect_thread.start()

    def _open_socket(self, timeout: float) -> None:
        with self._connect_lock:
            if self.has_connected():
                return
            old_sock = self._sock
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._connecting_sock = sock
            sock.settimeout(timeout)
            try:
                sock.connect((self._host, self._port))
            except OSError as ex:
                sock.close()
                self._connecting_sock = None
                if self._connect_abort_requested:
                    return
                reason = classify_connect_oserror(ex)
                err = InbcRpcConnectionError(
                    str(ex), reason=reason, errno=ex.errno
                )
                self._last_connect_error = err
                self._last_connect_failure = failure_dict(
                    reason,
                    str(ex),
                    host=self._host,
                    port=int(self._port),
                    errno_code=ex.errno,
                    detail=str(ex),
                )
                self._emit_error(err)
                self._start_reconnect_thread()
                return
            finally:
                if self._connecting_sock is sock:
                    self._connecting_sock = None
            if self._connect_abort_requested:
                try:
                    sock.close()
                except OSError:
                    pass
                return
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(None)
            if old_sock is not None:
                try:
                    old_sock.close()
                except OSError:
                    pass
            self._sock = sock
            self._last_connect_error = None
            self._connected.set()
            self._resend_subscribes()

    def _enqueue_write(
        self,
        req_id: int,
        req_type: RequestType,
        body: bytes,
        func_id: int,
    ) -> None:
        header = build_header(req_id, req_type, len(body), func_id)
        with self._write_lock:
            self._outbox.append((header, body))
            # 仅在队列为单条时刷新；必须在同一把锁内发送，避免重入死锁
            if len(self._outbox) == 1:
                self._flush_outbox_locked()

    def _flush_outbox_locked(self) -> None:
        """在已持有 _write_lock 时发送 outbox（不可再次 acquire 同一把 Lock）。"""
        if not self._sock or not self.has_connected():
            return
        while self._outbox:
            header, body = self._outbox[0]
            try:
                self._sock.sendall(header)
                if body:
                    self._sock.sendall(body)
            except OSError as ex:
                self._on_disconnect(InbcRpcConnectionError(str(ex)))
                return
            self._outbox.pop(0)

    def _send_subscribe(self, key: str, token: str) -> None:
        body = codec.pack_args(key, token)
        self._enqueue_write(0, RequestType.SUB_PUB, body, md5_hash32(key))

    def _resend_subscribes(self) -> None:
        with self._sub_lock:
            keys = list(self._sub_keys)
        for key, token in keys:
            self._send_subscribe(key, token)

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            if not self._sock:
                time.sleep(0.05)
                continue
            try:
                header_bytes = self._recv_exact(HEAD_LEN)
                header = RpcHeader.unpack(header_bytes)
                validate_header(header)
                body = b""
                if header.body_len > 0:
                    body = self._recv_exact(header.body_len)
                elif header.body_len == 0:
                    # 心跳应答帧，继续读下一帧
                    continue
                self._dispatch(header, body)
            except (OSError, ValueError, struct.error, InbcRpcConnectionError) as ex:
                if not self._stop.is_set():
                    self._on_disconnect(ex)
                break

    def _recv_exact(self, n: int) -> bytes:
        assert self._sock is not None
        buf = bytearray()
        while len(buf) < n:
            chunk = self._sock.recv(n - len(buf))
            if not chunk:
                raise InbcRpcConnectionError("connection closed by peer")
            buf.extend(chunk)
        return bytes(buf)

    def _dispatch(self, header: RpcHeader, body: bytes) -> None:
        req_id = header.req_id
        if header.req_type == RequestType.REQ_RES:
            self._dispatch_rpc_response(req_id, body)
        elif header.req_type == RequestType.SUB_PUB:
            self._dispatch_sub_push(body)

    def _dispatch_rpc_response(self, req_id: int, body: bytes) -> None:
        if req_id & CALLBACK_ID_FLAG:
            with self._req_lock:
                entry = self._callbacks.pop(req_id, None)
            if entry:
                deliver, timer = entry
                if timer:
                    timer.cancel()
                deliver(body, None)
            return

        with self._req_lock:
            pending = self._pending.get(req_id)
            if pending:
                pending[1][0] = body
                pending[0].set()
                return
            fut = self._futures.pop(req_id, None)
        if fut and not fut.done():
            fut.set_result(body)
            self._trace_rpc_end(req_id, body)

    def _dispatch_sub_push(self, body: bytes) -> None:
        try:
            _code, key, data = codec.parse_sub_push(body)
            with self._sub_lock:
                cb = self._subscribers.get(key)
            if cb:
                cb(data)
        except Exception as ex:
            self._emit_error(ex)

    def _on_disconnect(self, err: Exception) -> None:
        self.close()
        self._emit_error(err)
        if self._enable_reconnect and not self._stop.is_set():
            self._start_reconnect_thread()

    def _reconnect_loop(self) -> None:
        while not self._stop.is_set() and not self.has_connected():
            self._open_socket(self._connect_timeout_sec)
            if self.has_connected():
                break
            time.sleep(1.0)

    def _heartbeat_loop(self) -> None:
        while not self._stop.is_set():
            interval = self._heartbeat_interval_sec
            if interval <= 0:
                break
            time.sleep(interval)
            if self.has_connected():
                # 空 body 心跳帧，与 C++ write(0, req_res, buffer(0), 0) 一致
                self._enqueue_write(0, RequestType.REQ_RES, b"", 0)

    def _clear_pending(self, err: Exception) -> None:
        with self._req_lock:
            for rid, (_ev, slot) in list(self._pending.items()):
                slot[0] = None
                _ev.set()
                self._trace_rpc_end(rid, None, error=str(err))
            self._pending.clear()
            for rid, fut in list(self._futures.items()):
                if not fut.done():
                    fut.set_exception(err)
                self._trace_rpc_end(rid, None, error=str(err))
            self._futures.clear()
            for rid, (_cb, timer) in list(self._callbacks.items()):
                if timer:
                    timer.cancel()
                self._trace_rpc_end(rid, None, error=str(err))
            self._callbacks.clear()
            self._rpc_trace_meta.clear()

    def _emit_error(self, err: Exception) -> None:
        if self._error_callback:
            try:
                self._error_callback(err)
            except Exception:
                pass


class _RpcFuture:
    """包装 Future，提供 `.as()` 解析结果（对齐 C++ req_result::as）。"""

    def __init__(self, fut: Future, req_id: int, client: RpcClient):
        self._fut = fut
        self._req_id = req_id
        self._client = client

    def result(self, timeout: Optional[float] = None) -> Any:
        raw = self._fut.result(timeout)
        return codec.parse_result(raw)

    def as_(self, typ: type = None) -> Any:  # noqa: A003 — 对齐 C++ API 命名
        return self.result()

    def wait(self, timeout: Optional[float] = None) -> bool:
        try:
            self._fut.result(timeout)
            return True
        except Exception:
            return False

    def wait_for(self, timeout: float) -> bool:
        """对齐 C++ future_result::wait_for。"""
        try:
            self._fut.result(timeout)
            return True
        except Exception:
            return False
