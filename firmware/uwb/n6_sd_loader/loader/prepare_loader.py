"""핸드오프 FSBL 프로젝트 복사본에 SD 로더를 설치한다 (DESIGN_KO.md).

    python loader/prepare_loader.py --fsbl-copy <copy>/FSBL --tag-id 3 --builtin <app_id3.bin> --st-package <pkg>

<app_id3.bin> 은 같은 태그 번호로 `prepare_tag.py ... --flight --app-link` 해서 빌드한 펌웨어를 objcopy -O binary 한 것.
SD 읽기(sdlog.c, sdfat.c)와 HAL SD 드라이버는 prepare_tag.py 의 --sd 설치를 그대로 쓰고, 응답기 자리에 loader.c 를 넣는다.
그다음 STM32CubeIDE 로 Debug 구성을 빌드하고, 지금 브리지와 같은 방법으로 서명한다(-t fsbl).
"""
from __future__ import annotations

import argparse
import shutil
import struct
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "n6_flight_tag" / "tag"))
import prepare_tag  # noqa: E402

APP_LOAD_ADDR = 0x34000400
APP_RAM = (0x341C0000, 0x34200000)
BUILTIN_MAX = 200 * 1024   # 로더(약 40 KB 예상) + 기본 펌웨어가 FSBL 자리 255 KB 안에 들어가야 한다


def check_builtin(data: bytes) -> None:
    """--app-link 로 빌드한 펌웨어인지 벡터 표로 확인한다 (app_header.c 의 app_check_vectors 와 같은 규칙)."""
    if not 8 <= len(data) <= BUILTIN_MAX:
        raise ValueError(f"builtin is {len(data)} bytes, expected 8 .. {BUILTIN_MAX}")
    sp, entry = struct.unpack_from("<II", data)
    entry &= ~1
    if not (APP_RAM[0] < sp <= APP_RAM[1] and sp % 8 == 0 and APP_LOAD_ADDR <= entry < APP_LOAD_ADDR + len(data)):
        raise ValueError(f"builtin vectors sp=0x{sp:08x} entry=0x{entry:08x}: not an --app-link build")


def prepare(fsbl: Path, tag_id: int, builtin: Path, name: str, st_package: Path) -> None:
    data = builtin.read_bytes()
    check_builtin(data)
    if not name.isascii() or not 1 <= len(name) <= 12 or '"' in name or "\\" in name:
        raise ValueError("--builtin-name must be 1..12 ASCII characters")
    prepare_tag.prepare(fsbl, 16385, packets="f", opt="O2", tag_id=tag_id, tags=6, sd=True, st_package=st_package)
    example = fsbl / "Core" / "UWB" / "Src" / "examples" / "ex_05b_ds_twr_resp"
    shutil.copyfile(HERE / "loader.c", example / "ds_twr_responder.c")
    for filename in ("app_header.c", "app_header.h", "loader_builtin.c"):
        shutil.copyfile(HERE / filename, example / filename)
    target = example / "loader_builtin.bin"
    target.write_bytes(data)
    (example / "loader_builtin.h").write_text(
        f'#define LOADER_BUILTIN_PATH "{target.resolve().as_posix()}"\n#define LOADER_BUILTIN_NAME "{name}"\n',
        encoding="ascii")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--fsbl-copy", required=True, type=Path, help="FSBL directory in a disposable handoff copy")
    parser.add_argument("--tag-id", required=True, type=int, help="기체 번호 - 1 (0~5). 기본 펌웨어와 같아야 함")
    parser.add_argument("--builtin", required=True, type=Path, help="--app-link 펌웨어의 .bin (기본 펌웨어)")
    parser.add_argument("--builtin-name", default="tagf-v2-lfs", help="로더 줄에 남길 기본 펌웨어 이름")
    parser.add_argument("--st-package", required=True, type=Path, help="HAL SD 드라이버가 있는 STM32Cube N6 패키지")
    args = parser.parse_args()
    try:
        prepare(args.fsbl_copy, args.tag_id, args.builtin, args.builtin_name, args.st_package)
    except (OSError, ValueError) as error:
        parser.exit(2, f"{error}\n")
