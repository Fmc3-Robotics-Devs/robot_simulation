"""
rest_rpc 路由用的 MD5Hash32。

与 C++ `tools/ref/rpc/include/rest_rpc/md5.hpp` 中 MD5CE::Hash32 完全一致，
用于将 RPC 函数名映射为帧头中的 func_id。
"""

from __future__ import annotations

# MD5 轮常量（与 C++ kConstants 相同）
_K = (
    0xD76AA478, 0xE8C7B756, 0x242070DB, 0xC1BDCEEE, 0xF57C0FAF, 0x4787C62A,
    0xA8304613, 0xFD469501, 0x698098D8, 0x8B44F7AF, 0xFFFF5BB1, 0x895CD7BE,
    0x6B901122, 0xFD987193, 0xA679438E, 0x49B40821, 0xF61E2562, 0xC040B340,
    0x265E5A51, 0xE9B6C7AA, 0xD62F105D, 0x02441453, 0xD8A1E681, 0xE7D3FBC8,
    0x21E1CDE6, 0xC33707D6, 0xF4D50D87, 0x455A14ED, 0xA9E3E905, 0xFCEFA3F8,
    0x676F02D9, 0x8D2A4C8A, 0xFFFA3942, 0x8771F681, 0x6D9D6122, 0xFDE5380C,
    0xA4BEEA44, 0x4BDECFA9, 0xF6BB4B60, 0xBEBFBC70, 0x289B7EC6, 0xEAA127FA,
    0xD4EF3085, 0x04881D05, 0xD9D4D039, 0xE6DB99E5, 0x1FA27CF8, 0xC4AC5665,
    0xF4292244, 0x432AFF97, 0xAB9423A7, 0xFC93A039, 0x655B59C3, 0x8F0CCC92,
    0xFFEFF47D, 0x85845DD1, 0x6FA87E4F, 0xFE2CE6E0, 0xA3014314, 0x4E0811A1,
    0xF7537E82, 0xBD3AF235, 0x2AD7D2BB, 0xEB86D391,
)

_SHIFTS = (7, 12, 17, 22, 5, 9, 14, 20, 4, 11, 16, 23, 6, 10, 15, 21)

_INIT_A, _INIT_B, _INIT_C, _INIT_D = 0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476


def _swap_endian(v: int) -> int:
    v &= 0xFFFFFFFF
    return (
        ((v & 0xFF) << 24)
        | (((v >> 8) & 0xFF) << 16)
        | (((v >> 16) & 0xFF) << 8)
        | ((v >> 24) & 0xFF)
    )


def _left_rotate(value: int, bits: int) -> int:
    value &= 0xFFFFFFFF
    return ((value << bits) | (value >> (32 - bits))) & 0xFFFFFFFF


def _calc_f(i: int, b: int, c: int, d: int) -> int:
    if i < 16:
        return d ^ (b & (c ^ d))
    if i < 32:
        return c ^ (d & (b ^ c))
    if i < 48:
        return b ^ c ^ d
    return c ^ (b | (~d & 0xFFFFFFFF))


def _calc_g(i: int) -> int:
    if i < 16:
        return i
    if i < 32:
        return (5 * i + 1) % 16
    if i < 48:
        return (3 * i + 5) % 16
    return (7 * i) % 16


def _padded_length(n: int) -> int:
    return (((n + 1 + 8) + 63) // 64) * 64


def _padded_byte(data: bytes, n: int, m: int, i: int) -> int:
    if i < n:
        return data[i]
    if i == n:
        return 0x80
    if i >= m - 8:
        return (n * 8 >> ((i - (m - 8)) * 8)) & 0xFF
    return 0


def _padded_word(data: bytes, n: int, m: int, i: int) -> int:
    return (
        _padded_byte(data, n, m, i)
        | (_padded_byte(data, n, m, i + 1) << 8)
        | (_padded_byte(data, n, m, i + 2) << 16)
        | (_padded_byte(data, n, m, i + 3) << 24)
    )


def _round_data(data: bytes, n: int, m: int, offset: int) -> list[int]:
    return [_padded_word(data, n, m, offset + 4 * k) for k in range(16)]


def _process_message(data: bytes) -> tuple[int, int, int, int]:
    n = len(data)
    m = _padded_length(n)
    a, b, c, d = _INIT_A, _INIT_B, _INIT_C, _INIT_D

    for offset in range(0, m, 64):
        words = _round_data(data, n, m, offset)
        aa, bb, cc, dd = a, b, c, d
        for i in range(64):
            g = _calc_g(i)
            f = (_calc_f(i, bb, cc, dd) + aa + _K[i] + words[g]) & 0xFFFFFFFF
            s = _SHIFTS[(i // 16) * 4 + (i % 4)]
            aa, bb, cc, dd = dd, (bb + _left_rotate(f, s)) & 0xFFFFFFFF, bb, cc
        a = (a + aa) & 0xFFFFFFFF
        b = (b + bb) & 0xFFFFFFFF
        c = (c + cc) & 0xFFFFFFFF
        d = (d + dd) & 0xFFFFFFFF
    return a, b, c, d


def md5_hash32(name: str) -> int:
    """
    计算与 C++ `MD5::MD5Hash32(name)` 相同的 32 位路由键。

    :param name: RPC 函数名或 subscribe 的 topic key
    """
    raw = name.encode("utf-8") if isinstance(name, str) else name
    a, _, _, _ = _process_message(raw)
    return _swap_endian(a)


def md5_hash32_bytes(name: bytes) -> int:
    """对 bytes 计算 Hash32（与 C++ 带长度版本一致）。"""
    a, _, _, _ = _process_message(name)
    return _swap_endian(a)
