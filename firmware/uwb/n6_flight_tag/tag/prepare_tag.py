"""Install the tag responder in a dedicated copy of the team's FSBL project.

    python prepare_tag.py --fsbl-copy <copy>/FSBL                       # 9패킷 (tag.c v2), 프로젝트 설정 그대로
    python prepare_tag.py --fsbl-copy <copy>/FSBL --packets 7 --opt O2  # 7패킷 (tag7.c), Debug 구성을 -O2 로
    python prepare_tag.py --fsbl-copy <copy>/FSBL --packets 7 --opt O2 --sweep   # S, D1, D2 를 바꿔 가며 재는 판
    python prepare_tag.py --fsbl-copy <copy>/FSBL --packets 7 --opt O2 --slot-us 400 --resp-delay-us 450 --final-delay-us 450
    python prepare_tag.py --fsbl-copy <copy>/FSBL --packets f --opt O2 --tag-id 3 --tags 6   # 프레임 방식 (tag_f.c), 태그 3번
    python prepare_tag.py --fsbl-copy <copy>/FSBL --packets f --opt O2 --tag-id 3 --sd --st-package <STM32Cube_FW_N6_V1.x>
                                                                    # 같은 것 + SD 카드 기록 (캐리어 보드의 microSD)
    python prepare_tag.py --fsbl-copy <copy>/FSBL --packets f --opt O2 --tag-id 3 --sd --flight --st-package <...>
                                                                    # 비행용: + BLE 중계, 비행 제어기 연결, 대본

그다음 STM32CubeIDE 로 그 FSBL 프로젝트의 Debug 구성을 빌드한다.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import shutil

OPT_OPTION = "com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.compiler.option.optimization.level"
OPT_VALUES = {"O0": "o0", "O1": "o1", "O2": "o2", "O3": "o3", "Os": "os", "Og": "og"}
INTERVAL_MAX = 2900  # frames.h 의 UWB7_INTERVAL_MAX
TIME_MIN = 200       # frames.h 의 UWB7_TIME_MIN


def set_optimization(fsbl: Path, level: str) -> None:
    """Debug 구성의 C 컴파일러 최적화 수준을 바꾼다 (.cproject 의 첫 번째 구성이 Debug)."""
    cproject = fsbl / ".cproject"
    text = cproject.read_bytes().decode("utf-8")
    pattern = re.compile(r'<option id="(' + re.escape(OPT_OPTION) + r'\.\d+)"[^>]*superClass="' +
                         re.escape(OPT_OPTION) + r'"[^>]*/>')
    match = pattern.search(text)
    debug = text.find('name="Debug"')
    release = text.find('name="Release"')
    if match is None or debug < 0 or not debug < match.start() or (0 <= release < match.start()):
        raise ValueError("could not find the Debug optimization option in .cproject")
    option = (f'<option id="{match[1]}" name="Optimization level" superClass="{OPT_OPTION}" '
              f'value="{OPT_OPTION}.value.{OPT_VALUES[level]}" valueType="enumerated"/>')
    cproject.write_bytes((text[:match.start()] + option + text[match.end():]).encode("utf-8"))


def check_timing(slot: int, response: int, final: int) -> None:
    if min(slot, response, final) < TIME_MIN:
        raise ValueError(f"S, D1, D2 must be at least {TIME_MIN} us")
    if 2 * slot + response > INTERVAL_MAX or 2 * slot + final > INTERVAL_MAX:
        raise ValueError(f"2S + D1 and 2S + D2 must not exceed {INTERVAL_MAX} us (distance formula limit 3000 us)")


def prepare(fsbl: Path, antenna_delay: int, packets: str = "9", opt: str = "", slot: int = 600,
            response: int = 900, final: int = 700, sweep: bool = False, sweep_bundles: int = 300,
            tag_id: int = 0, tags: int = 6, sd: bool = False, st_package: Path | None = None,
            frame_us: int = 20000, flight: bool = False) -> None:
    packets = str(packets)
    if not 0 <= antenna_delay <= 65535:
        raise ValueError("antenna delay must be a uint16 in DW time units")
    if packets not in ("7", "9", "f"):
        raise ValueError("packets must be 9, 7 or f")
    if not 1 <= tags <= 16 or not 0 <= tag_id < tags:
        raise ValueError("tags must be 1..16 and tag id 0..tags-1")
    check_timing(slot, response, final)
    if not 10 <= sweep_bundles <= 100000:
        raise ValueError("sweep bundles must be 10..100000")
    uwb = fsbl / "Core" / "UWB" / "Src"
    example = uwb / "examples" / "ex_05b_ds_twr_resp"
    selection = uwb / "example_selection.h"
    responder = example / "ds_twr_responder.c"
    if not responder.is_file() or not selection.is_file():
        raise ValueError("expected the handoff project's Qorvo responder and example_selection.h")
    contents = selection.read_bytes().decode("utf-8")
    selector = re.compile(r"^([ \t]*)(?://[ \t]*)?(#define[ \t]+TEST_DS_TWR_RESPONDER)\b", re.M)
    if len(selector.findall(contents)) != 1:
        raise ValueError("the handoff must have exactly one TEST_DS_TWR_RESPONDER selector")
    contents = re.sub(r"^([ \t]*)(#define[ \t]+TEST_[A-Z0-9_]+)\b", r"\1//\2", contents, flags=re.M)
    contents = selector.sub(r"\1\2", contents)
    if opt:
        set_optimization(fsbl, opt)
    source = Path(__file__).resolve().parent
    for filename in ("frames.h", "radio.h"):
        shutil.copyfile(source / filename, example / filename)
    (example / "tag_antenna_delay.h").write_text(f"#define UWB3_ANTENNA_DELAY {antenna_delay}U\n", encoding="ascii")
    config = example / "tag7_config.h"
    frame_config = example / "tagf_config.h"
    if sd and (packets != "f" or st_package is None):
        raise ValueError("--sd needs --packets f and --st-package")
    if flight and not sd:
        raise ValueError("--flight needs --sd (기록이 안 되면 대본을 시작하지 않는다)")
    if packets == "f":
        frame_config.write_text(f"#define TAGF_ID {tag_id}\n#define UWBF_TAGS {tags}\n"
                                f"#define TAGF_BUILD_OPT \"{opt or 'project'}\"\n"
                                f"#define TAGF_SD {int(sd)}\n#define TAGF_FRAME_US {frame_us}\n"
                                f"#define TAGF_FLIGHT {int(flight)}\n", encoding="ascii")
        for name in ("fcl.c", "fcl.h", "flight.c", "flight.h", "link_failsafe.h"):
            if flight:
                shutil.copyfile(source / name, example / name)
            elif (example / name).exists():
                (example / name).unlink()
        for name in ("sdlog.c", "sdlog.h", "sdfat.c", "sdfat.h"):
            if sd:
                shutil.copyfile(source / name, example / name)
            elif (example / name).exists():
                (example / name).unlink()
        if sd:
            # 팀 프로젝트에는 HAL 의 SD 드라이버가 없다. ST 패키지에서 사본으로 가져오고 HAL 설정에서 켠다
            import sys
            sys.path.insert(0, str(source.parent / "sdcheck"))
            import prepare_sdcheck
            prepare_sdcheck.add_hal_sd(fsbl, Path(st_package))
    elif frame_config.exists():
        frame_config.unlink()
    if packets == "7":
        config.write_text(
            f"#define UWB7_SLOT_US {slot}\n#define UWB7_RESPONSE_DELAY_US {response}\n"
            f"#define UWB7_FINAL_DELAY_US {final}\n#define TAG7_SWEEP {int(sweep)}\n"
            f"#define TAG7_SWEEP_BUNDLES {sweep_bundles}\n#define TAG7_BUILD_OPT \"{opt or 'project'}\"\n",
            encoding="ascii")
    elif config.exists():
        config.unlink()
    shutil.copyfile(source / {"9": "tag.c", "7": "tag7.c", "f": "tag_f.c"}[packets], responder)
    selection.write_bytes(contents.encode("utf-8"))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--fsbl-copy", required=True, type=Path, help="FSBL directory in a disposable handoff copy")
    parser.add_argument("--antenna-delay", type=int, default=16385)
    parser.add_argument("--packets", default="9", choices=["9", "7", "f"],
                        help="9 = tag.c (v2), 7 = tag7.c, f = tag_f.c (프레임 방식)")
    parser.add_argument("--tag-id", type=int, default=0, help="프레임 방식: 이 태그의 번호 (0 ~ tags-1)")
    parser.add_argument("--tags", type=int, default=6, help="프레임 방식: 태그 수. 앵커의 --tags 와 같아야 함")
    parser.add_argument("--sd", action="store_true", help="프레임 방식: SD 카드(UWBLOG.BIN)에도 기록")
    parser.add_argument("--st-package", type=Path, help="--sd 일 때: STM32Cube_FW_N6_V1.x 폴더 (HAL 의 SD 드라이버를 가져옴)")
    parser.add_argument("--frame-us", type=int, default=20000, help="프레임 방식: 프레임 길이 (SD 쓰기 구간을 정하는 데 씀)")
    parser.add_argument("--flight", action="store_true",
                        help="프레임 방식: 비행용 (BLE 중계 + 비행 제어기 연결 + 대본). --sd 가 있어야 함")
    parser.add_argument("--opt", default="", choices=["", *OPT_VALUES],
                        help="set the Debug configuration's optimization level (default: leave the project as is)")
    parser.add_argument("--slot-us", type=int, default=600, help="7패킷: S")
    parser.add_argument("--resp-delay-us", type=int, default=900, help="7패킷: D1")
    parser.add_argument("--final-delay-us", type=int, default=700, help="7패킷: D2")
    parser.add_argument("--sweep", action="store_true", help="7패킷: tag7.c 의 표대로 S, D1, D2 를 바꿔 가며 잰다")
    parser.add_argument("--sweep-bundles", type=int, default=300, help="7패킷 --sweep: 표의 한 줄마다 묶음 수")
    args = parser.parse_args()
    try:
        prepare(args.fsbl_copy, args.antenna_delay, args.packets, args.opt, args.slot_us, args.resp_delay_us,
                args.final_delay_us, args.sweep, args.sweep_bundles, args.tag_id, args.tags, args.sd, args.st_package,
                args.frame_us, args.flight)
    except (OSError, ValueError) as error:
        parser.exit(2, f"{error}\n")
