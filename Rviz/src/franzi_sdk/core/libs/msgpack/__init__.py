# ruff: noqa: F401
import os, platform

# 查询得知:链接库方式,比源码快5-50倍
if platform.system() == "windows":
    # Windows 64位 (x86_64/amd64)
    from .msgpack_310_win_amd64.exceptions import *  # noqa: F403
    from .msgpack_310_win_amd64.ext import ExtType, Timestamp

elif platform.system() == "linux":
    machine = platform.machine().lower()
    if machine in ("x86_64", "amd64"):
        # Linux x86_64
        from .msgpack_310_linux_x86_64.exceptions import *
        from .msgpack_310_linux_x86_64.ext import ExtType, Timestamp
    elif machine.startswith(("aarch64", "arm64")):
        # Linux ARM64 (aarch64)
        from .msgpack_310_linux_aarch64.exceptions import *
        from .msgpack_310_linux_aarch64.ext import ExtType, Timestamp
    else:
        # 其他 Linux 架构（如 armv7l, ppc64le 等）回退到纯 Python
        from .msgpack_fallback import *

else:
    # macOS, FreeBSD 等其他系统
    from .msgpack_fallback import *

version = (1, 1, 2)
__version__ = "1.1.2"


def pack(o, stream, **kwargs):
    """
    Pack object `o` and write it to `stream`

    See :class:`Packer` for options.
    """
    packer = Packer(**kwargs)
    stream.write(packer.pack(o))


def packb(o, **kwargs):
    """
    Pack object `o` and return packed bytes

    See :class:`Packer` for options.
    """
    return Packer(**kwargs).pack(o)


def unpack(stream, **kwargs):
    """
    Unpack an object from `stream`.

    Raises `ExtraData` when `stream` contains extra bytes.
    See :class:`Unpacker` for options.
    """
    data = stream.read()
    return unpackb(data, **kwargs)


# alias for compatibility to simplejson/marshal/pickle.
load = unpack
loads = unpackb

dump = pack
dumps = packb
