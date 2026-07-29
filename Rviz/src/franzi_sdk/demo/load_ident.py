#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
load_ident.py — 负载辨识调试脚本

直接修改本文件顶部的 ``_MODE`` 切换模式：

    _MODE = "full"     # 轨迹采集 + 计算
    _MODE = "collect"  # 仅采集轨迹
    _MODE = "compute"  # 仅用已有文件计算
    _MODE = "invalid"  # 非法选项组合（返回 -3）

返回码语义::

    0     - 成功
    -3    - 参数非法（文件缺失 / 选项组合非法）
    -5    - 下游 INO 调用失败（超时 / 函数未注册）
"""

from __future__ import annotations

import os
import sys
from time import perf_counter

_SDK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _SDK_ROOT not in sys.path:
    sys.path.insert(0, _SDK_ROOT)

from core.client import ArmClient
from core.types import SdkStatus, sdk_status_message
from core import logger

# ------------------------------------------------------------------ 运行参数（按需修改）
HOST = "192.168.23.30"
PORT = 8000

# ------------------------------------------------------------------ 负载辨识参数
# 填空则使用服务端默认路径；或指定具体目录（末尾自动补 /）
UNLOAD_PATH = "/inodata"
LOAD_PATH   = "/inodata"


# 切换模式：full / collect / compute / invalid(用不到，防呆)
_MODE = "collect"


def fmt_params(params: list) -> str:
    if not params:
        return "[]"
    return "[" + ", ".join(f"{x:.6g}" for x in params) + "]"


def fmt_result(status: int, result: dict) -> None:
    name = sdk_status_message(status)
    logger.info(f"  status={status} ({name})")
    if status == SdkStatus.OK:
        logger.info(f"  参数: {fmt_params(result.get('parameter', []))}")
        logger.info(f"  采样数: {result.get('written_samples', 0)}")
        logger.info(f"  丢帧数: {result.get('dropped_samples', 0)}")
        logger.info(f"  超时: {result.get('timed_out', False)}")
    else:
        logger.warning(f"  失败 result={result}")


RIGHT_SIDE = 0  # 对应 C++ SdkArmSide::Right，直接写数值避免 Python 枚举错误


def _run_load_ident(arm: ArmClient, side: int, unload_path: str, load_path: str,
                    options: dict) -> tuple:
    """直接发 RPC，timeout_ms 由 options 指定，不走 ArmClientTeleop.RunLoadIdent。"""
    timeout_ms = options.get("timeout_ms", 180000)
    req = {
        "side": side,
        "unload_path": unload_path,
        "load_path": load_path,
        "options": options,
    }
    rsp = arm.rpc("RunLoadIdent", req, timeout_ms=timeout_ms)
    return arm.status_of(rsp), dict(rsp.get("result", {}))


def do_full(arm: ArmClient) -> int:
    """模式 3：轨迹采集 + 计算一气呵成"""
    logger.info("[模式 3] collect + identify 同时执行（默认超时 180s）")
    options = {
        "collect_load_data": True,
        "do_identification": True,
        "timeout_ms": 180000,
    }
    t0 = perf_counter()
    status, result = _run_load_ident(arm, RIGHT_SIDE, UNLOAD_PATH, LOAD_PATH, options)
    elapsed = perf_counter() - t0
    logger.info(f"  耗时: {elapsed:.1f}s")
    fmt_result(status, result)
    return status


def do_collect_only(arm: ArmClient) -> int:
    """模式 1：只跑轨迹采集"""
    logger.info("[模式 1] 仅采集轨迹（采集完成后请手动重命名文件，再以 compute 模式计算）")
    options = {
        "collect_load_data": True,
        "do_identification": False,
        "timeout_ms": 180000,
    }
    t0 = perf_counter()
    status, result = _run_load_ident(arm, RIGHT_SIDE, UNLOAD_PATH, LOAD_PATH, options)
    elapsed = perf_counter() - t0
    logger.info(f"  耗时: {elapsed:.1f}s")
    fmt_result(status, result)
    if status == SdkStatus.OK:
        logger.info("  采集完成！请将空载文件重命名为 'unload.txt'，带载文件重命名为 'load.txt'，")
        logger.info("  然后以 --mode compute 重新运行进行计算。")
    return status


def do_compute_only(arm: ArmClient) -> int:
    """模式 2：直接用已有文件计算"""
    logger.info("[模式 2] 仅计算（需提前准备好 unload.txt 和 load.txt）")
    options = {
        "collect_load_data": False,
        "do_identification": True,
        "timeout_ms": 180000,
    }
    t0 = perf_counter()
    status, result = _run_load_ident(arm, RIGHT_SIDE, UNLOAD_PATH, LOAD_PATH, options)
    elapsed = perf_counter() - t0
    logger.info(f"  耗时: {elapsed:.1f}s")
    fmt_result(status, result)
    return status


def do_invalid(arm: ArmClient) -> int:
    """模式 4：非法选项组合"""
    logger.info("[模式 4] collect=false 且 identify=false → 参数非法，返回 -3")
    options = {
        "collect_load_data": False,
        "do_identification": False,
        "timeout_ms": 180000,
    }
    status, result = _run_load_ident(arm, RIGHT_SIDE, UNLOAD_PATH, LOAD_PATH, options)
    fmt_result(status, result)
    return status


def main() -> None:
    arm = ArmClient(host=HOST, port=PORT, raise_on_error=False)

    try:
        logger.info(f"连接 {HOST}:{PORT} ...")
        if not arm.is_connected():
            logger.error("连接失败")
            sys.exit(1)

        logger.info(f"SDK Version: {arm.GetVersion()}")
        ready = arm.IsReady()
        logger.info(f"IsReady: {ready}")
        if not ready:
            logger.warning("IsReady=False，继续执行...")

        arm.set_rpc_timeout(200000)

        # 右臂上使能 (robot_type=10, lifecycle_cmd=1)
        ret_right = arm.SetEnableState(10, 1)
        logger.info(f"右臂使能: {ret_right}")

        mode_map = {
            "full": do_full,
            "collect": do_collect_only,
            "compute": do_compute_only,
            "invalid": do_invalid,
        }
        status = mode_map[_MODE](arm)
        logger.info(f"最终返回: status={status} ({sdk_status_message(status)})")

    except KeyboardInterrupt:
        logger.info("Ctrl+C 中断")
    finally:
        arm.disconnect()
        logger.info("已断开连接")


if __name__ == "__main__":
    main()
