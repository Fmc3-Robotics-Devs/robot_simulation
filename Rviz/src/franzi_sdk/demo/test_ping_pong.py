#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Ping/Pong RPC 往返时延基准测试（剥离业务，仅测网络 + Python SDK）。"""

from __future__ import annotations

import math
import os
import statistics
import sys
import time
from typing import List, Sequence

_SDK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SDK_ROOT not in sys.path:
    sys.path.insert(0, _SDK_ROOT)

from core.client import ArmClient
from core import logger

# ------------------------------------------------------------------ 运行参数（按需修改）
# 目标 SDK 服务端地址；需与 arm_client_demo.py 等 demo 保持一致，且服务端已部署 arm_sdk.Ping。
HOST = "192.168.23.30"
# RPC 监听端口（默认 8000）；若 inbcrt / sdk_server 改了端口，此处同步修改。
PORT = 8000
# 预热次数：不计入统计。用于建立 TCP、完成 msgpack 编解码路径的 JIT/缓存，避免冷启动拉高均值。
WARMUP = 100
# 正式采样次数：越大统计越稳，但受网络抖动影响时耗越长；建议对比测试时固定同一 COUNT。
COUNT = 2000
# 直方图分桶数：越大曲线越细，样本少时可能显得稀疏；仅用于日志 ASCII 可视化，不影响数值指标。
HISTOGRAM_BINS = 20


def _percentile(sorted_values: Sequence[float], p: float) -> float:
    """线性插值分位数（0~100）。输入须已升序排列。"""
    if not sorted_values:
        return 0.0
    if len(sorted_values) == 1:
        return sorted_values[0]
    k = (len(sorted_values) - 1) * p / 100.0
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return sorted_values[int(k)]
    return sorted_values[f] * (c - k) + sorted_values[c] * (k - f)


def _ascii_histogram(values: Sequence[float], bins: int = 20) -> List[str]:
    """将时延样本渲染为等宽 ASCII 直方图行，便于在终端快速观察分布形态。"""
    if not values:
        return ["(empty)"]
    lo = min(values)
    hi = max(values)
    if math.isclose(lo, hi):
        return [f"all values = {lo:.4f} ms"]
    width = (hi - lo) / bins
    counts = [0] * bins
    for v in values:
        idx = min(bins - 1, int((v - lo) / width))
        counts[idx] += 1
    peak = max(counts) or 1
    lines: List[str] = []
    for i, cnt in enumerate(counts):
        left = lo + i * width
        right = left + width
        bar_len = int(40 * cnt / peak)
        lines.append(f"[{left:8.3f}, {right:8.3f}) | {'#' * bar_len} {cnt}")
    return lines


def run_sync_benchmark(arm: ArmClient, count: int, warmup: int) -> List[float]:
    """
    执行同步 Ping 采样。

    每次调用 arm.Ping(sync=True)，计时区间为「发起 RPC」到「收到 pong 字符串」，
    包含 Python SDK、TCP、服务端 Ping handler 的完整往返，不含业务模块逻辑。
    """
    latencies_ms: List[float] = []
    for _ in range(count):
        t0 = time.perf_counter()
        resp= arm.GetRobotState()  # 该接口获取数据,及时返回。 无需再做额外做一个 Ping -Pong 链路
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        # if pong != "pong":
        #     raise RuntimeError(f"Ping: unexpected response {pong!r}")
        latencies_ms.append(elapsed_ms)
    return latencies_ms


def print_report(latencies_ms: List[float], total_sec: float) -> None:
    """
    打印统计报告。

    - 最大调用频率：COUNT / total_sec，反映连续同步调用下的实际吞吐。
    - 理论峰值频率：1000 / min_ms，单次最快往返对应的极限 Hz（理想下界，通常高于吞吐）。
    - mean / variance / stdev：样本均值与离散程度；假设近似正态时可读作 N(μ, σ²)。
    - p50/p90/p99：分位数，比极值更能反映典型时延与长尾。
    """
    n = len(latencies_ms)
    mean_ms = statistics.mean(latencies_ms)
    min_ms = min(latencies_ms)
    max_ms = max(latencies_ms)
    var_ms2 = statistics.variance(latencies_ms) if n > 1 else 0.0
    stdev_ms = statistics.stdev(latencies_ms) if n > 1 else 0.0
    max_freq_hz = n / total_sec if total_sec > 0 else 0.0
    peak_freq_hz = 1000.0 / min_ms if min_ms > 0 else 0.0

    sorted_ms = sorted(latencies_ms)
    p50 = _percentile(sorted_ms, 50)
    p90 = _percentile(sorted_ms, 90)
    p99 = _percentile(sorted_ms, 99)

    logger.info("========== Ping/Pong 同步基准 (%d 次) ==========", n)
    logger.info("总耗时: %.3f s", total_sec)
    logger.info("最大调用频率 (吞吐): %.2f Hz", max_freq_hz)
    logger.info("理论峰值频率 (1/min): %.2f Hz", peak_freq_hz)
    logger.info("往返时延 min: %.4f ms", min_ms)
    logger.info("往返时延 max: %.4f ms", max_ms)
    logger.info("往返时延 mean: %.4f ms", mean_ms)
    logger.info("往返时延 variance: %.6f ms²", var_ms2)
    logger.info("往返时延 stdev (σ): %.4f ms", stdev_ms)
    logger.info("分位数 p50/p90/p99: %.4f / %.4f / %.4f ms", p50, p90, p99)
    logger.info(
        "正态分布 N(μ=%.4f, σ²=%.6f): 约 68%% 落在 [%.4f, %.4f] ms",
        mean_ms,
        var_ms2,
        mean_ms - stdev_ms,
        mean_ms + stdev_ms,
    )
    logger.info(
        "正态分布 N(μ=%.4f, σ²=%.6f): 约 95%% 落在 [%.4f, %.4f] ms",
        mean_ms,
        var_ms2,
        mean_ms - 2 * stdev_ms,
        mean_ms + 2 * stdev_ms,
    )
    logger.info("---------- 时延直方图 (近似正态分布形状) ----------")
    for line in _ascii_histogram(latencies_ms, bins=HISTOGRAM_BINS):
        logger.info("%s", line)


def main() -> None:
    # 关闭 heartbeat / auto_reconnect，避免后台线程额外 RPC 干扰时延测量。
    # raise_on_error=True：Ping 失败或非 0 status 时立即抛错，便于发现未注册 Ping 的旧服务端。
    arm = ArmClient(
        host=HOST,
        port=PORT,
        auto_heartbeat=False,
        auto_reconnect=False,
        raise_on_error=True,
    )
    try:
        t0 = time.perf_counter()
        latencies_ms = run_sync_benchmark(arm, count=COUNT, warmup=WARMUP)
        total_sec = time.perf_counter() - t0
        print_report(latencies_ms, total_sec)
    finally:
        arm.disconnect()
        logger.info("已断开连接")


if __name__ == "__main__":
    main()
