"""SD 카드 확인 펌웨어(sd_check.c)를 팀 FSBL 프로젝트 사본에 넣는다.

    python sdcheck/prepare_sdcheck.py --fsbl-copy <copy>/FSBL --st-package <STM32Cube_FW_N6_V1.x 폴더> [--no-write]

그 프로젝트에는 HAL 의 SD 드라이버가 들어 있지 않아서, ST 패키지(STM32Cube_FW_N6)에서 SD 드라이버 파일들을 사본으로 복사하고
stm32n6xx_hal_conf.h 에서 SD 모듈을 켠다. 그다음 STM32CubeIDE 로 Debug 구성을 빌드해 RAM 에 올린다.
"""
import argparse
import re
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "tag"))
import prepare_tag  # noqa: E402

HAL_FILES = [("Inc", "stm32n6xx_hal_sd.h"), ("Inc", "stm32n6xx_hal_sd_ex.h"), ("Inc", "stm32n6xx_ll_sdmmc.h"),
             ("Inc", "stm32n6xx_ll_dlyb.h"),
             ("Src", "stm32n6xx_hal_sd.c"), ("Src", "stm32n6xx_hal_sd_ex.c"), ("Src", "stm32n6xx_ll_sdmmc.c"),
             ("Src", "stm32n6xx_ll_dlyb.c")]


def add_hal_sd(fsbl: Path, package: Path) -> None:
    """ST 패키지의 SD 드라이버를 프로젝트 사본에 넣고 HAL 설정에서 켠다."""
    hal = package / "Drivers" / "STM32N6xx_HAL_Driver"
    if not (hal / "Src" / "stm32n6xx_hal_sd.c").is_file():
        raise ValueError(f"{hal} 에 SD 드라이버가 없습니다. --st-package 를 확인하세요")
    for folder, name in HAL_FILES:
        path = hal / folder / name
        if path.is_file():                                 # _ex 파일은 없는 판도 있다
            shutil.copyfile(path, fsbl / "Core" / folder / name)
    conf = fsbl / "Core" / "Inc" / "stm32n6xx_hal_conf.h"
    text = conf.read_bytes().decode("utf-8")
    text, count = re.subn(r"/\*\s*#define HAL_SD_MODULE_ENABLED\s*\*/", "#define HAL_SD_MODULE_ENABLED", text)
    if count != 1 and "\n#define HAL_SD_MODULE_ENABLED" not in text:
        raise ValueError("stm32n6xx_hal_conf.h 에서 HAL_SD_MODULE_ENABLED 를 찾지 못했습니다")
    conf.write_bytes(text.encode("utf-8"))


def prepare(fsbl: Path, package: Path, write_test: bool, opt: str, test: str = "card") -> None:
    prepare_tag.prepare(fsbl, 16385, "9", opt)           # 예제 선택과 최적화 설정은 태그와 같은 방법으로
    example = fsbl / "Core" / "UWB" / "Src" / "examples" / "ex_05b_ds_twr_resp"
    if test == "log":                                    # 기록 모듈 시험: tag/sdlog.c, tag/sdfat.c 와 함께
        tag = HERE.parent / "tag"
        for name in ("sdlog.h", "sdfat.h", "sdfat.c"):
            shutil.copyfile(tag / name, example / name)
        (example / "sdlog.c").write_text("#define SDLOG_TEST_HOOKS\n" + (tag / "sdlog.c").read_text(encoding="utf-8"),
                                         encoding="utf-8")
        source = (HERE / "sdlog_test.c").read_text(encoding="utf-8")
    else:
        source = (HERE / "sd_check.c").read_text(encoding="utf-8")
        if not write_test:
            source = "#define SDCHECK_WRITE 0\n" + source
    (example / "ds_twr_responder.c").write_text(source, encoding="utf-8")
    add_hal_sd(fsbl, package)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--fsbl-copy", required=True, type=Path)
    parser.add_argument("--st-package", required=True, type=Path, help="STM32Cube_FW_N6_V1.x 폴더")
    parser.add_argument("--no-write", action="store_true", help="쓰기 지연 시험을 뺀다(읽기만)")
    parser.add_argument("--test", default="card", choices=["card", "log"],
                        help="card = 카드 확인(sd_check.c), log = 기록 모듈 시험(sdlog_test.c)")
    parser.add_argument("--opt", default="", choices=["", *prepare_tag.OPT_VALUES])
    args = parser.parse_args()
    try:
        prepare(args.fsbl_copy, args.st_package, not args.no_write, args.opt, args.test)
    except (OSError, ValueError) as error:
        parser.exit(2, f"{error}\n")
