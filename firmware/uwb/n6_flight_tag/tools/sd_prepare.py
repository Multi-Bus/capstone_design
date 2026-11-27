"""SD 카드에 기록 파일(UWBLOG.BIN)을 만듭니다 (Windows / Linux 공용, 표준 라이브러리만 씀).

    python tools/sd_prepare.py --drive E:\\            # 256 MiB 파일을 만듦
    python tools/sd_prepare.py --drive E:\\ --size-mb 1024
    python tools/sd_prepare.py --drive E:\\ --overwrite   # 이미 있는 파일을 0 으로 다시 채움 (앞 기록을 지움)

태그 펌웨어(TAGF_SD)는 파일 시스템에 쓰지 않고, 이 파일이 차지한 자리에 차례로 씁니다. 그래서
- 카드는 FAT32 여야 합니다(32 GB 이하 카드의 보통 포맷. exFAT 은 안 됨).
- 파일이 한 덩어리로 이어져 있어야 합니다. **카드를 포맷한 직후에 이 파일을 가장 먼저 만드세요.**
  조각나 있으면 태그가 시작할 때 `TF_SD,...,file=fragmented` 라고 알려 줍니다.
- 파일은 0 으로 채웁니다. 태그는 0 이 아닌 첫 글자가 있는 곳까지를 "이미 쓴 부분"으로 보고 그 뒤에 이어 씁니다.

크기 어림: 지금 기록은 초당 약 3 kB 입니다(프레임 줄과 거리 줄). 256 MiB 면 20시간이 넘습니다.
"""
import argparse
import os
import shutil
import sys
from pathlib import Path

NAME = "UWBLOG.BIN"
CHUNK = 1024 * 1024


def create(folder, size_mb, overwrite=False, progress=True):
    path = Path(folder) / NAME
    if not Path(folder).is_dir():
        raise ValueError(f"{folder} 폴더가 없습니다")
    if path.exists() and not overwrite:
        raise ValueError(f"{path} 가 이미 있습니다. 앞 기록을 지우고 다시 만들려면 --overwrite")
    if not 1 <= size_mb <= 3072:
        raise ValueError("크기는 1 ~ 3072 MiB (FAT32 의 파일 하나는 4 GiB 까지)")
    free = shutil.disk_usage(folder).free + (path.stat().st_size if path.exists() else 0)
    if free < size_mb * CHUNK:
        raise ValueError(f"빈 자리가 모자랍니다 ({free // CHUNK} MiB 남음)")
    zeros = bytes(CHUNK)
    with open(path, "wb") as f:
        for i in range(size_mb):
            f.write(zeros)
            if progress and (i + 1) % 32 == 0:
                print(f"  {i + 1} / {size_mb} MiB")
        f.flush()
        os.fsync(f.fileno())
    return path


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--drive", required=True, help="카드의 맨 위 폴더 (예: E:\\ 또는 /media/user/CARD)")
    p.add_argument("--size-mb", type=int, default=256)
    p.add_argument("--overwrite", action="store_true", help="이미 있으면 0 으로 다시 채움")
    opt = p.parse_args()
    try:
        path = create(opt.drive, opt.size_mb, opt.overwrite)
    except (OSError, ValueError) as error:
        sys.exit(str(error))
    print(f"만들었습니다: {path} ({opt.size_mb} MiB). 카드를 '안전하게 제거'한 뒤 보드에 꽂으세요.")


if __name__ == "__main__":
    main()
