"""DWM3001CDK 에 앵커 펌웨어 올리기 (SEGGER J-Link, Windows / Linux 공용).

    python flash.py --id 2 --serial 000760226502
    python flash.py --list                       # 연결된 J-Link 일련번호 보기

- J-Link 일련번호(--serial)는 꼭 지정한다. 여러 앵커를 꽂아 두었을 때 엉뚱한 보드에 쓰지 않도록.
- 보드의 J9(J-Link) USB 를 PC 에 연결한다.
- 기존 플래시 중 이미지가 들어갈 자리만 지우고 쓴 뒤, 같은지 확인(verifybin)하고 재시작한다. UICR 은 건드리지 않는다.
"""
import argparse
import glob
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def find_jlink(given):
    if given:
        return Path(given)
    name = "JLink.exe" if os.name == "nt" else "JLinkExe"
    on_path = shutil.which(name)
    if on_path:
        return Path(on_path)
    patterns = [str(Path.home() / "opt/SEGGER/JLink*/JLinkExe"), "/opt/SEGGER/JLink*/JLinkExe",
                r"C:\Program Files\SEGGER\JLink*\JLink.exe", r"C:\Program Files (x86)\SEGGER\JLink*\JLink.exe"]
    for pattern in patterns:
        hits = sorted(glob.glob(pattern))
        if hits:
            return Path(hits[-1])
    sys.exit("J-Link Commander 를 못 찾았습니다. --jlink <JLink.exe 경로> 로 지정하세요.")


def jlink_run(jlink, args, script):
    with tempfile.NamedTemporaryFile("w", suffix=".jlink", delete=False, encoding="ascii") as f:
        f.write(script)
        path = f.name
    try:
        result = subprocess.run([str(jlink), "-NoGui", "1", "-ExitOnError", "1"] + args + ["-CommandFile", path],
                                capture_output=True, text=True, errors="replace")
    finally:
        os.unlink(path)
    return result.returncode, result.stdout + result.stderr


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--id", type=int, choices=[1, 2, 3], help="앵커 번호 (build/A<번호>/anchor_A<번호>.hex)")
    p.add_argument("--serial", help="J-Link 일련번호 (보드 J9)")
    p.add_argument("--jlink", help="JLinkExe / JLink.exe 경로")
    p.add_argument("--list", action="store_true", help="연결된 J-Link 목록만 보기")
    opt = p.parse_args()
    jlink = find_jlink(opt.jlink)

    if opt.list:
        code, out = jlink_run(jlink, [], "ShowEmuList\nexit\n")
        print("\n".join(line for line in out.splitlines() if "Serial number" in line or "J-Link[" in line) or out)
        return
    if opt.id is None or not opt.serial:
        sys.exit("--id 와 --serial 이 필요합니다 (--list 로 일련번호 확인)")
    hexfile = ROOT / "build" / f"A{opt.id}" / f"anchor_A{opt.id}.hex"
    binfile = hexfile.with_suffix(".bin")
    if not hexfile.is_file() or not binfile.is_file():
        sys.exit(f"{hexfile} 가 없습니다. 먼저 python build.py --id {opt.id}")

    script = "\n".join([
        "r", "h",
        f'loadfile "{hexfile}"',
        f'verifybin "{binfile}" 0x0',
        "r", "g", "exit", ""])
    args = ["-USB", opt.serial, "-device", "NRF52833_XXAA", "-if", "SWD", "-speed", "4000", "-autoconnect", "1"]
    print(f"A{opt.id} -> J-Link {opt.serial}: {hexfile.name}")
    code, out = jlink_run(jlink, args, script)
    keep = ("Downloading", "O.K.", "Verify", "ERROR", "Error", "failed", "Flash download", "Program", "Comparing",
            "Cannot connect", "differs")
    print("\n".join(line for line in out.splitlines() if any(k in line for k in keep)))
    failed = code != 0 or "ERROR" in out or "differs" in out or "Verify failed" in out or "Cannot connect" in out
    if failed:
        print(out[-2000:])
        sys.exit("플래시 실패")
    print("완료: 쓰기와 확인 끝, 재시작함")


if __name__ == "__main__":
    main()
