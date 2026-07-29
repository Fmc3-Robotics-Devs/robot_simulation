# -*- coding: utf-8 -*-
"""
core.logging — SDK 统一日志（命名空间 ``core``）

默认格式（与产品约定一致）::

    INFO 20260528-19:23:230 具体文本内容

时间戳为 ``YYYYMMDD-HH:MM:SS`` 后紧跟 3 位毫秒（秒与毫秒无分隔，如 23 秒 0 毫秒 → ``19:23:230``）。

二开可 ``logging.getLogger("core")`` 挂载 Handler，定制格式与落盘策略；
子模块日志名：``core.rpc``、``core.arm`` 等。
"""

from __future__ import annotations

import logging
import sys
from datetime import datetime
from logging.handlers import RotatingFileHandler
from typing import Optional, TextIO, Union

# 根命名空间；子模块使用 core.xxx
LOGGER_NAME = "core"

_DEFAULT_FORMAT = "%(levelname)s %(asctime)s %(message)s"
_configured_default = False


class SdkCoreFormatter(logging.Formatter):
    """
    SDK 默认日志格式器。

    输出示例::

        INFO 20260528-19:23:230 连接成功
    """

    def format(self, record: logging.LogRecord) -> str:
        record.asctime = self.format_timestamp(record)
        return f"{record.levelname} {record.asctime} {record.getMessage()}"

    @staticmethod
    def format_timestamp(record: logging.LogRecord) -> str:
        """
        生成 ``YYYYMMDD-HH:MM:SSmmm`` 时间戳（秒与毫秒直接拼接）。

        :param record: LogRecord
        :return: 如 ``20260528-19:23:230``（19:23:23.000）
        """
        dt = datetime.fromtimestamp(record.created)
        ms = int(record.msecs)
        return (
            dt.strftime("%Y%m%d")
            + f"-{dt.hour:02d}:{dt.minute:02d}:{dt.second}{ms:03d}"
        )


def get_logger(name: Optional[str] = None) -> logging.Logger:
    """
    获取 ``core`` 命名空间下的 Logger。

    :param name: 子模块名；None 表示根 ``core``
    :return: ``logging.Logger`` 实例
    """
    if name:
        return logging.getLogger(f"{LOGGER_NAME}.{name}")
    return logging.getLogger(LOGGER_NAME)


def configure_default_logging(
    level: Union[int, str] = logging.INFO,
    stream: Optional[TextIO] = None,
    *,
    force: bool = False,
) -> logging.Logger:
    """
    为 ``core`` 配置默认控制台 Handler（仅当尚无 Handler 或 ``force=True``）。

    :param level: 日志级别，默认 INFO
    :param stream: 输出流，默认 ``sys.stderr``
    :param force: True 时先清除已有 Handler 再配置
    :return: 根 logger ``core``
    """
    global _configured_default
    logger = get_logger()
    if force:
        for h in list(logger.handlers):
            logger.removeHandler(h)
            h.close()
        _configured_default = False

    if not logger.handlers:
        handler = logging.StreamHandler(stream if stream is not None else sys.stderr)
        handler.setFormatter(SdkCoreFormatter())
        logger.addHandler(handler)
        _configured_default = True

    logger.setLevel(level)
    logger.propagate = False
    return logger


def set_log_format(
    fmt: Union[str, logging.Formatter],
    *,
    logger: Optional[logging.Logger] = None,
) -> None:
    """
    修改 ``core`` 已有 Handler 的格式（二开定制格式，步骤 3）。

    :param fmt: 格式字符串或 Formatter 实例
    :param logger: 目标 logger，默认根 ``core``
    """
    log = logger if logger is not None else get_logger()
    formatter: logging.Formatter
    if isinstance(fmt, str):
        formatter = logging.Formatter(fmt)
    else:
        formatter = fmt
    for h in log.handlers:
        h.setFormatter(formatter)


def attach_rotating_file_handler(
    filepath: str,
    *,
    max_bytes: int = 10 * 1024 * 1024,
    backup_count: int = 5,
    level: Union[int, str] = logging.DEBUG,
    encoding: str = "utf-8",
    formatter: Optional[logging.Formatter] = None,
    logger: Optional[logging.Logger] = None,
) -> RotatingFileHandler:
    """
    为 ``core`` 增加按大小滚动的文件 Handler（二开落盘策略，步骤 4）。

    :param filepath: 日志文件路径
    :param max_bytes: 单文件最大字节，默认 10MB
    :param backup_count: 保留历史文件个数
    :param level: 该 Handler 级别
    :param encoding: 文件编码
    :param formatter: 默认使用 :class:`SdkCoreFormatter`
    :param logger: 目标 logger，默认根 ``core``
    :return: 已挂载的 RotatingFileHandler
    """
    log = logger if logger is not None else get_logger()
    fh = RotatingFileHandler(
        filepath,
        maxBytes=max_bytes,
        backupCount=backup_count,
        encoding=encoding,
    )
    fh.setLevel(level)
    fh.setFormatter(formatter if formatter is not None else SdkCoreFormatter())
    log.addHandler(fh)
    return fh


def ensure_default_logging() -> logging.Logger:
    """若尚未配置 Handler，则调用 ``configure_default_logging()``。"""
    logger = get_logger()
    if not logger.handlers:
        return configure_default_logging()
    return logger


def log_connect_failure(
    failure: dict,
    *,
    logger: Optional[logging.Logger] = None,
) -> str:
    """将 RPC 建连失败详情写入 ``core.arm``（默认 ERROR）。"""
    from core.errors import format_connect_failure

    message = format_connect_failure(failure)
    log = logger if logger is not None else get_logger("arm")
    log.error(message)
    return message


def rpc_trace_sink(
    func: str,
    request: object,
    response: object,
    use_time_ms: float,
    error: Optional[str] = None,
) -> None:
    """
    供 ``libs.inbc_rpc`` RPC 轨迹使用的默认 sink，写入 ``core.rpc``。

    :param func: 完整 RPC 名
    :param request: 请求体视图
    :param response: 响应体
    :param use_time_ms: 耗时毫秒
    :param error: 可选错误说明
    """
    from core.libs.inbc_rpc import rpc_trace

    name = rpc_trace.func_short_name(func)
    req_s = rpc_trace.format_payload(request)
    resp_s = rpc_trace.format_payload(response) if response is not None else "null"
    parts = [f"client-->>server:{name}", f"request:{req_s}", f"response:{resp_s}"]
    if error:
        parts.append(f"error:{error!s}")
    parts.append(f"use_time(ms):{use_time_ms:.2f}")
    get_logger("rpc").info(", ".join(parts))
