# -*- coding: utf-8 -*-
"""
组装 SDK 发布包：core/ 除 libs/inbc_rpc 外保留 .py 源码，inbc_rpc 编译为 .pyc。

在 sdk_py 目录执行::

    python package_release.py
"""

from __future__ import annotations

import compileall
import platform
import re
import shutil
import sys
import tarfile
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent
CORE_SRC = ROOT / "core"
BUILD_DIR = ROOT / "build" / "release_core"
DIST_ROOT = ROOT.parent / "dist"
INBC_RPC_REL = Path("libs") / "inbc_rpc"

EXCLUDE_DIR_NAMES = frozenset({"arm_client", "arm_types", "__pycache__"})
# Cython/本地编译残留，发布包中不应出现
SKIP_SUFFIXES = frozenset({".c", ".pyd", ".so", ".pyx"})


def _read_version() -> str:
    text = (ROOT / "pyproject.toml").read_text(encoding="utf-8")
    m = re.search(r'^version\s*=\s*["\']([^"\']+)["\']', text, re.M)
    return m.group(1) if m else "0.0.0"


def _platform_tag() -> str:
    sys_name = platform.system().lower()
    machine = platform.machine().lower()
    if sys_name == "windows":
        return "win_amd64"
    if sys_name == "linux":
        if machine in ("x86_64", "amd64"):
            return "linux_x86_64"
        if machine.startswith(("aarch64", "arm64")):
            return "linux_aarch64"
        return f"linux_{machine}"
    return f"{sys_name}_{machine}"


def _python_tag() -> str:
    return f"cp{sys.version_info.major}{sys.version_info.minor}"


def _should_keep_msgpack_dir(name: str, plat: str) -> bool:
    if name == "msgpack_fallback":
        return True
    if plat == "win_amd64":
        return name == "msgpack_310_win_amd64"
    if plat == "linux_x86_64":
        return name == "msgpack_310_linux_x86_64"
    if plat == "linux_aarch64":
        return name == "msgpack_310_linux_aarch64"
    return name == "msgpack_fallback"


def _is_skipped_path(rel_parts: tuple[str, ...], plat: str) -> bool:
    if any(p in EXCLUDE_DIR_NAMES for p in rel_parts):
        return True
    for part in rel_parts:
        if part.startswith("msgpack_310_") or part == "msgpack_fallback":
            return not _should_keep_msgpack_dir(part, plat)
    return False


def _purge_build_artifacts(root: Path) -> int:
    """删除 core 下 Cython/扩展编译残留（.c .pyd .so），返回删除数量。"""
    removed = 0
    for path in sorted(root.rglob("*")):
        if path.is_file() and path.suffix.lower() in SKIP_SUFFIXES:
            path.unlink()
            removed += 1
    return removed


def _copy_core_tree(dst_core: Path, plat: str) -> None:
    """拷贝 core 源码与资源（过滤历史副本、无关平台 msgpack、编译残留）。"""
    if dst_core.exists():
        shutil.rmtree(dst_core)
    dst_core.mkdir(parents=True)
    for src in sorted(CORE_SRC.rglob("*")):
        if src.is_dir():
            continue
        if src.suffix.lower() in SKIP_SUFFIXES:
            continue
        rel_parts = src.relative_to(CORE_SRC).parts
        if _is_skipped_path(rel_parts, plat):
            continue
        out = dst_core / src.relative_to(CORE_SRC)
        out.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, out)


def _compile_inbc_rpc_to_pyc(core_dir: Path) -> None:
    """仅将 libs/inbc_rpc 编译为 .pyc 并删除对应 .py。"""
    rpc_dir = core_dir / INBC_RPC_REL
    if not rpc_dir.is_dir():
        raise RuntimeError(f"未找到 inbc_rpc 目录: {rpc_dir}")

    ok = compileall.compile_dir(
        str(rpc_dir),
        force=True,
        legacy=True,
        quiet=1,
    )
    if not ok:
        raise RuntimeError(f"compileall 失败: {rpc_dir}")

    missing: list[str] = []
    for py in sorted(rpc_dir.rglob("*.py")):
        pyc = py.with_suffix(".pyc")
        if not pyc.is_file():
            missing.append(str(py.relative_to(core_dir)))
        else:
            py.unlink()
    if missing:
        raise RuntimeError("inbc_rpc 以下文件未生成 .pyc:\n  " + "\n  ".join(missing))


def _write_install_txt(out_dir: Path, plat: str, version: str) -> None:
    py_tag = _python_tag()
    text = f"""INBC Arm Python SDK {version} ({plat}, {py_tag})

目录说明
  core/                  SDK 主体（.py 源码）
  core/libs/inbc_rpc/    RPC 传输层（.pyc 字节码，无 .py）
  demo/                  示例脚本（源码）

环境要求
  Python {sys.version_info.major}.{sys.version_info.minor}.x（inbc_rpc 字节码须同主版本）

使用方式
  1. 将本目录加入 PYTHONPATH：
     import sys
     sys.path.insert(0, r"<本目录绝对路径>")

  2. 示例：
     from core.client import ArmClient
     from core import types

  3. 运行 demo：
     python demo/arm_client_demo.py
"""
    (out_dir / "INSTALL.txt").write_text(text, encoding="utf-8")


def assemble() -> Path:
    version = _read_version()
    plat = _platform_tag()
    py_tag = _python_tag()
    name = f"inbc-arm-sdk-{version}-{plat}-{py_tag}"
    out_dir = DIST_ROOT / name

    print(f"==> copy core sources + compile libs/inbc_rpc -> .pyc ({py_tag})")
    n = _purge_build_artifacts(CORE_SRC)
    if n:
        print(f"    cleaned {n} stale .c/.pyd/.so under core/")
    _copy_core_tree(BUILD_DIR, plat)
    _compile_inbc_rpc_to_pyc(BUILD_DIR)

    if out_dir.exists():
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True)
    shutil.copytree(BUILD_DIR, out_dir / "core")

    if (ROOT / "demo").is_dir():
        shutil.copytree(ROOT / "demo", out_dir / "demo")
    for fname in ("README.md", "pyproject.toml"):
        src = ROOT / fname
        if src.is_file():
            shutil.copy2(src, out_dir / fname)

    _write_install_txt(out_dir, plat, version)

    zip_path = DIST_ROOT / f"{name}.zip"
    tgz_path = DIST_ROOT / f"{name}.tar.gz"
    for p in (zip_path, tgz_path):
        if p.exists():
            p.unlink()

    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
        for f in sorted(out_dir.rglob("*")):
            if f.is_file():
                zf.write(f, f.relative_to(DIST_ROOT.parent).as_posix().replace("\\", "/"))

    with tarfile.open(tgz_path, "w:gz") as tf:
        tf.add(out_dir, arcname=name)

    print(f"OK release dir : {out_dir}")
    print(f"OK zip         : {zip_path}")
    print(f"OK tar.gz      : {tgz_path}")
    return out_dir


if __name__ == "__main__":
    assemble()
