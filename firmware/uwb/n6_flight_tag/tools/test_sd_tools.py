"""sd_prepare.py 와 sd_extract.py 시험 (카드 없이, 임시 폴더에서).

    python tools/test_sd_tools.py

펌웨어가 쓰는 방식 그대로 파일 안에 글자를 채워 넣고(섹터 단위, 덜 찬 섹터는 줄바꿈으로 채움, 두 번 켬) 다시 꺼내 견준다.
"""
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import frame_measure as fm  # noqa: E402
import sd_extract  # noqa: E402
import sd_prepare  # noqa: E402

FAILS = []


def check(name, got, want):
    ok = got == want
    print(f"{'PASS' if ok else 'FAIL'}  {name}: {got!r}" + ("" if ok else f" (기대 {want!r})"))
    if not ok:
        FAILS.append(name)


def firmware_write(path, start_block, lines, flush_every=40):
    """sdlog.c 처럼: 줄을 이어 붙여 512바이트 섹터로 쓰고, flush_every 줄마다 덜 찬 섹터를 줄바꿈으로 채운다."""
    blocks, current = [], b""
    for i, line in enumerate(lines, 1):
        current += (line + "\r\n").encode("ascii")
        while len(current) >= 512:
            blocks.append(current[:512])
            current = current[512:]
        if i % flush_every == 0 and current:
            blocks.append(current + b"\n" * (512 - len(current)))
            current = b""
    if current:
        blocks.append(current + b"\n" * (512 - len(current)))
    with open(path, "r+b") as f:
        f.seek(start_block * 512)
        f.write(b"".join(blocks))
    return start_block + len(blocks)


def run_lines(tag, first_frame, frames):
    lines = ["TF_READY,channel=5", f"TF_INFO,fw=tagf-v1,id={tag},tags=6,addr={256 + tag},ant_dly=16385,records=256,sd=1,opt=O2",
             "TF_SD,ready=1,card=1,card_error=0x00000000,file=ok,first_block=9472,file_blocks=524288,next_block=0"]
    for f in range(first_frame, first_frame + frames):
        mine = f % 6 == tag
        if mine:
            for a in (1, 2, 3):
                lines.append(f"R,{f},{a},{2000 + 700 * a},1,F,-20500,-21500,{0x10000000 + f * 4992189 + 149760 * (a - 1):08X},2800546")
        lines.append(f"F,{f},{0x10000000 + f * 4992189:08X},{f % 6},7F,{'R' if mine else '-'},{480 if mine else 0},210,0")
        if f % 250 == 0:
            lines.append(f"S,{f},0,0,0,0")
            lines.append(f"D,{f},1,{f // 3},{f // 3},0,0,1300,290,0,0")
    return lines


def main():
    with tempfile.TemporaryDirectory() as d:
        tmp = Path(d)
        print("--- 1) 기록 파일 만들기")
        path = sd_prepare.create(tmp, 4, progress=False)
        check("이름", path.name, "UWBLOG.BIN")
        check("크기", path.stat().st_size, 4 * 1024 * 1024)
        check("0 으로 차 있음", path.read_bytes().count(0), 4 * 1024 * 1024)
        try:
            sd_prepare.create(tmp, 4, progress=False)
            check("이미 있으면 거절", False, True)
        except ValueError:
            check("이미 있으면 거절", True, True)

        print("--- 2) 펌웨어가 쓰는 방식으로 두 번 켜서 쓰고, 꺼내기")
        first = run_lines(3, 100, 600)
        end = firmware_write(path, 0, first)
        second = run_lines(3, 5000, 300)
        end2 = firmware_write(path, end, second)
        info = sd_extract.extract(path, tmp / "out")
        check("쓴 섹터 수", info["blocks_used"], end2)
        check("켠 횟수", len(info["runs"]), 2)
        check("첫 기록의 파일 이름", info["runs"][0]["file"], "tag3_run1.raw.log")
        check("첫 기록의 줄 수", info["runs"][0]["lines"], len(first))
        check("첫 기록의 프레임 범위", info["runs"][0]["frames"], (100, 699))
        check("둘째 기록의 프레임 범위", info["runs"][1]["frames"], (5000, 5299))
        got = (tmp / "out" / "tag3_run1.raw.log").read_text(encoding="utf-8").splitlines()
        check("꺼낸 줄이 쓴 줄과 같음", got == first, True)

        print("--- 3) 꺼낸 기록을 frame_measure 로 읽기")
        log = fm.parse_tag_log(tmp / "out" / "tag3_run1.raw.log")
        check("태그 번호", log.tag, 3)
        check("프레임 줄 수", len(log.frames), 600)
        check("거리 줄이 있는 프레임 수", len(log.ranges), 100)
        check("SD 줄(D)을 읽음", log.sd[-1][:3] if log.sd else None, [500, 1, 166])
        summary = fm.analyze([log], [])
        check("세 거리가 다 나온 비율", summary["tag"][3]["position"]["success_rate"], 1.0)

        print("--- 4) 다시 만들기(--overwrite)는 앞 기록을 지움")
        sd_prepare.create(tmp, 4, overwrite=True, progress=False)
        info = sd_extract.extract(path, tmp / "out2")
        check("쓴 섹터 0", info["blocks_used"], 0)
    print(f"\n{'모두 통과' if not FAILS else '실패: ' + ', '.join(FAILS)}")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
