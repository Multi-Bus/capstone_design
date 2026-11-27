"""SD 카드의 기록 파일(UWBLOG.BIN)에서 글자 기록을 꺼냅니다 (Windows / Linux 공용, 표준 라이브러리만 씀).

    python tools/sd_extract.py --file E:\\UWBLOG.BIN --out results\\flight1

- 파일의 앞에서부터, 첫 글자가 0 인 섹터가 나올 때까지가 기록입니다.
- 태그를 켤 때마다 TF_READY 줄이 들어가므로 거기서 나눠 `tag<번호>_run<순서>.raw.log` 로 씁니다.
  태그 번호는 TF_INFO 줄의 id 입니다. 이 파일들을 그대로 `tools/frame_measure.py --tag-logs` 에 주면 됩니다.
- 전원이 갑자기 꺼지면 마지막 0.5 s 쯤은 카드에 없을 수 있습니다.
"""
import argparse
import sys
from pathlib import Path

BLOCK = 512


def read_text(path):
    """(글자, 쓴 섹터 수, 파일 섹터 수)"""
    data = bytearray()
    used = 0
    with open(path, "rb") as f:
        total = f.seek(0, 2) // BLOCK
        f.seek(0)
        while True:
            block = f.read(BLOCK)
            if len(block) < BLOCK or block[0] == 0:
                break
            data += block
            used += 1
    return data.replace(b"\x00", b"").decode("ascii", errors="replace"), used, total


def split_runs(text):
    """TF_READY 줄에서 나눈다. [(태그 번호 또는 None, [줄...]), ...]"""
    runs, current = [], None
    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.startswith("TF_READY"):
            current = [None, []]
            runs.append(current)
        elif current is None:               # 앞이 잘린 기록
            current = [None, []]
            runs.append(current)
        if line.startswith("TF_INFO,"):
            for kv in line.split(",")[1:]:
                if kv.startswith("id="):
                    try:
                        current[0] = int(kv[3:])
                    except ValueError:
                        pass
        current[1].append(line)
    return [(tag, lines) for tag, lines in runs]


def extract(path, out):
    text, used, total = read_text(path)
    runs = split_runs(text)
    out.mkdir(parents=True, exist_ok=True)
    written = []
    for i, (tag, lines) in enumerate(runs, 1):
        name = f"tag{tag if tag is not None else 'X'}_run{i}.raw.log"
        (out / name).write_text("\n".join(lines) + "\n", encoding="utf-8")
        frames = [int(x.split(",")[1]) for x in lines if x.startswith("F,") and x.split(",")[1].isdigit()]
        written.append({"file": name, "tag": tag, "lines": len(lines),
                        "frames": (min(frames), max(frames)) if frames else None,
                        "ranges": sum(1 for x in lines if x.startswith("R,"))})
    return {"blocks_used": used, "blocks_total": total, "runs": written}


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--file", required=True, type=Path, help="카드의 UWBLOG.BIN")
    p.add_argument("--out", required=True, type=Path, help="결과 폴더")
    opt = p.parse_args()
    if not opt.file.is_file():
        sys.exit(f"{opt.file} 가 없습니다")
    info = extract(opt.file, opt.out)
    print(f"쓴 부분 {info['blocks_used']} / {info['blocks_total']} 섹터 "
          f"({info['blocks_used'] * BLOCK / 1024:.0f} kB), 켠 횟수 {len(info['runs'])}")
    for r in info["runs"]:
        frames = f"프레임 {r['frames'][0]}~{r['frames'][1]}" if r["frames"] else "프레임 줄 없음"
        print(f"  {r['file']}: 태그 {r['tag']}, {r['lines']}줄, {frames}, 거리 줄 {r['ranges']}개")
    print(f"결과 폴더: {opt.out}")


if __name__ == "__main__":
    main()
