"""펌웨어 .bin 에 헤더를 붙여 SD 로더가 읽는 N6APP.BIN 을 만든다 (DESIGN_KO.md 3장, 표준 라이브러리만 씀).

    python tools/sd_app.py <app_id3.bin> --tag-id 3 --drive E:\\        # 카드에 바로 쓰고 다시 읽어 대조
    python tools/sd_app.py <app_id3.bin> --tag-id 3 --out N6APP.BIN     # 파일로만 만듦

<app_id3.bin> 은 prepare_tag.py ... --app-link 로 빌드한 펌웨어를 objcopy -O binary 한 것이다.
태그 번호는 기체 번호 - 1 이고, 로더의 번호와 다르면 로더가 쓰지 않는다(reason=wrong_id).
카드에서 파일이 여러 조각이면 로더가 쓰지 않는다(reason=fragmented). UWBLOG.BIN 을 만든 카드에 이어서 쓰면 보통 한 조각이다.
"""
import argparse
import os
import struct
import sys
import zlib
from pathlib import Path

NAME = "N6APP.BIN"
VERSION = 1
LOAD_ADDR = 0x34000400
MAX_LEN = 512 * 1024
RAM = (0x341C0000, 0x34200000)


def build(image: bytes, tag_id: int, fw_name: str = "tagf-v2-lfs") -> bytes:
    if not 0 <= tag_id <= 255:
        raise ValueError("tag id must be 0..255")
    if not 8 <= len(image) <= MAX_LEN:
        raise ValueError(f"image is {len(image)} bytes, expected 8 .. {MAX_LEN}")
    sp, entry = struct.unpack_from("<II", image)
    if not (RAM[0] < sp <= RAM[1] and sp % 8 == 0 and LOAD_ADDR <= entry & ~1 < LOAD_ADDR + len(image)):
        raise ValueError(f"vectors sp=0x{sp:08x} entry=0x{entry:08x}: build the firmware with --app-link")
    label = fw_name.encode("ascii")
    if len(label) > 12:
        raise ValueError("firmware name must be at most 12 ASCII characters")
    header = struct.pack("<4sHBBIII12s", b"N6AP", VERSION, tag_id, 0, LOAD_ADDR, len(image),
                         zlib.crc32(image), label)
    return header + image


def write_card(folder: str, data: bytes) -> Path:
    path = Path(folder) / NAME
    with open(path, "wb") as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())
    if path.read_bytes() != data:
        raise OSError(f"{path}: read-back differs")
    return path


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("image", type=Path, help="--app-link 펌웨어의 .bin")
    p.add_argument("--tag-id", type=int, required=True, help="기체 번호 - 1 (0~5)")
    p.add_argument("--name", default="tagf-v2-lfs", help="로더 줄에 남길 펌웨어 이름 (12글자까지)")
    where = p.add_mutually_exclusive_group(required=True)
    where.add_argument("--drive", help="카드의 맨 위 폴더 (예: E:\\)")
    where.add_argument("--out", type=Path, help="파일로만 만들 경로")
    opt = p.parse_args()
    try:
        data = build(opt.image.read_bytes(), opt.tag_id, opt.name)
        if opt.out:
            opt.out.write_bytes(data)
            path = opt.out
        else:
            path = write_card(opt.drive, data)
    except (OSError, ValueError) as error:
        sys.exit(str(error))
    print(f"만들었습니다: {path} (id {opt.tag_id}, 펌웨어 {len(data) - 32} 바이트, CRC 0x{zlib.crc32(data[32:]):08x}). "
          "카드를 '안전하게 제거'한 뒤 보드에 꽂으세요.")


if __name__ == "__main__":
    main()
