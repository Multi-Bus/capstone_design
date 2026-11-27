"""SD 로더의 PC 시험: sd_app.py 가 만든 N6APP.BIN 을 보드와 같은 C 검사 함수(loader/app_header.c)로 읽는다.

    python -m pytest firmware/uwb/n6_sd_loader/tools/test_sd_app.py

gcc 가 있어야 C 쪽을 시험한다(없으면 그 시험은 건너뜀).
"""
import ctypes
import shutil
import struct
import subprocess
import sys
import tempfile
import zlib
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
LOADER = HERE.parent / "loader"
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(LOADER))
import sd_app  # noqa: E402
import prepare_loader  # noqa: E402

BASE = 0x34000400


class Header(ctypes.Structure):
    _fields_ = [("version", ctypes.c_uint16), ("tag_id", ctypes.c_uint8), ("load", ctypes.c_uint32),
                ("length", ctypes.c_uint32), ("crc", ctypes.c_uint32), ("name", ctypes.c_char * 13)]


def image(length=4000, sp=0x34200000, entry=BASE + 0x201):
    body = bytes((i * 7) & 0xFF for i in range(length))
    return struct.pack("<II", sp, entry) + body[8:]


@pytest.fixture(scope="module")
def lib():
    if shutil.which("gcc") is None:
        pytest.skip("gcc 없음")
    out = Path(tempfile.mkdtemp()) / "app_header.dll"
    subprocess.run(["gcc", "-O2", "-std=c11", "-Wall", "-Wextra", "-Werror", "-shared", "-o", str(out),
                    str(LOADER / "app_header.c")], check=True)
    so = ctypes.CDLL(str(out))
    so.app_check_header.argtypes = [ctypes.c_char_p, ctypes.c_uint8, ctypes.c_uint32, ctypes.POINTER(Header)]
    so.app_check_vectors.argtypes = [ctypes.c_char_p, ctypes.c_uint32, ctypes.c_uint32]
    so.app_crc32.argtypes = [ctypes.c_uint32, ctypes.c_char_p, ctypes.c_uint32]
    so.app_crc32.restype = ctypes.c_uint32
    so.app_check_name.restype = ctypes.c_char_p
    return so


def check(so, data, my_id=3, file_bytes=None):
    """로더와 같은 순서: 헤더, CRC, 벡터. 이유 이름을 돌려준다."""
    header = Header()
    size = len(data) if file_bytes is None else file_bytes
    result = so.app_check_header(data[:32], my_id, size, ctypes.byref(header))
    if result:
        return so.app_check_name(result).decode()
    body = data[32:32 + header.length]
    if so.app_crc32(0, body, header.length) != header.crc:
        return "bad_crc"
    return so.app_check_name(so.app_check_vectors(body, header.length, BASE)).decode()


def test_header_layout():
    data = sd_app.build(image(), 3, "tagf-v2-lfs")
    magic, version, tag, _, load, length, crc, name = struct.unpack_from("<4sHBBIII12s", data)
    assert (magic, version, tag, load, length) == (b"N6AP", 1, 3, BASE, 4000)
    assert crc == zlib.crc32(data[32:]) and name.rstrip(b"\0") == b"tagf-v2-lfs" and len(data) == 4032


def test_build_rejects_non_app_link_image():
    with pytest.raises(ValueError):
        sd_app.build(image(entry=0x34180400 + 0x201), 3)
    with pytest.raises(ValueError):
        prepare_loader.check_builtin(image(entry=0x34180400 + 0x201))
    prepare_loader.check_builtin(image())


def test_c_accepts_good_file(lib):
    data = sd_app.build(image(), 3, "abc")
    header = Header()
    assert lib.app_check_header(data[:32], 3, len(data), ctypes.byref(header)) == 0
    assert (header.length, header.crc, header.name) == (4000, zlib.crc32(data[32:]), b"abc")
    assert check(lib, data) == "ok"
    assert check(lib, data, file_bytes=(len(data) + 511) // 512 * 512) == "ok"   # 섹터 단위로 올린 크기


def test_c_crc_matches_zlib(lib):
    for blob in (b"", b"a", bytes(range(256)) * 9):
        assert lib.app_crc32(0, blob, len(blob)) == zlib.crc32(blob)
    half = lib.app_crc32(0, b"hello ", 6)
    assert lib.app_crc32(half, b"world", 5) == zlib.crc32(b"hello world")


@pytest.mark.parametrize("change, reason", [
    (lambda d: b"X6AP" + d[4:], "bad_magic"),
    (lambda d: d[:4] + b"\x02\x00" + d[6:], "bad_version"),
    (lambda d: d[:6] + b"\x04" + d[7:], "wrong_id"),
    (lambda d: d[:8] + struct.pack("<I", 0x34180400) + d[12:], "bad_length"),
    (lambda d: d[:12] + struct.pack("<I", 0) + d[16:], "bad_length"),
    (lambda d: d[:12] + struct.pack("<I", 513 * 1024) + d[16:], "bad_length"),
    (lambda d: d[:-1], "bad_length"),
    (lambda d: d[:-1] + bytes([d[-1] ^ 1]), "bad_crc"),
])
def test_c_reasons(lib, change, reason):
    assert check(lib, change(sd_app.build(image(), 3))) == reason


@pytest.mark.parametrize("sp, entry", [(0x341C0000, BASE + 1), (0x34200008, BASE + 1), (0x341FFFFC, BASE + 1),
                                       (0x34200000, BASE - 2), (0x34200000, BASE + 4000)])
def test_c_bad_vectors(lib, sp, entry):
    body = image(sp=sp, entry=entry)
    assert lib.app_check_vectors(body, len(body), BASE) == 6     # APP_BAD_VECTORS


def test_app_link_patch(tmp_path):
    sys.path.insert(0, str(HERE.parents[1] / "n6_flight_tag" / "tag"))
    import prepare_tag
    fsbl = tmp_path / "FSBL"
    (fsbl / "Core" / "Src").mkdir(parents=True)
    (fsbl / "STM32N657X0HXQ_AXISRAM2_fsbl.ld").write_bytes(
        b"  ROM    (xrw)    : ORIGIN = 0x34180400,   LENGTH = 255K\r\n")
    (fsbl / "Core" / "Src" / "stm32n6xx_it.c").write_bytes(
        b"  /* USER CODE END HardFault_IRQn 0 */\r\n  while (1)\r\n")
    prepare_tag.set_app_link(fsbl)
    assert b"ORIGIN = 0x34000400,   LENGTH = 511K\r\n" in (fsbl / "STM32N657X0HXQ_AXISRAM2_fsbl.ld").read_bytes()
    assert b"*/ NVIC_SystemReset();\r\n  while" in (fsbl / "Core" / "Src" / "stm32n6xx_it.c").read_bytes()
    with pytest.raises(ValueError):
        prepare_tag.set_app_link(fsbl)        # 두 번 바꾸지 않는다
