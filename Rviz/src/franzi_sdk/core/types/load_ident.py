# -*- coding: utf-8 -*-
"""
load_ident — 负载辨识选项类型。

对应服务端：SdkLoadIdentOptions（sdk_api_load_ident.h）。
"""

from __future__ import annotations

from typing import TypedDict


class LoadIdentOptions(TypedDict, total=False):
    """负载辨识可选参数（run_load_ident 的 options 字段）。"""

    capture_capacity: int
    timeout_ms: int
    collect_load_data: bool
    do_identification: bool


def default_load_ident_options() -> LoadIdentOptions:
    """
    负载辨识默认选项（与 C++ 默认一致）。

    :return: LoadIdentOptions dict
    """
    return {
        "capture_capacity": 200000,
        "timeout_ms": 180000,
        "collect_load_data": True,
        "do_identification": True,
    }
