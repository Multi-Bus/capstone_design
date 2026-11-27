"""태그(NUCLEO-N657) 펌웨어 ELF 를 RAM 에 올리고 실행합니다 (Windows / Linux 공용, 표준 라이브러리만 씀).

    python tools/load_tag.py --list                            # 연결된 ST-LINK 보기
    python tools/load_tag.py tag/build/<파일>.elf               # ST-LINK 가 하나일 때
    python tools/load_tag.py tag/build/<파일>.elf --sn 0023...  # 여러 개일 때
    python tools/load_tag.py tag/build/<파일>.elf --dry-run     # 명령만 보여 줌

ELF 마다 시작 주소(PC)가 다릅니다. 이 스크립트는 그 값을 ELF 에서 읽어 STM32_Programmer_CLI 명령을 만듭니다.
RAM 에만 올립니다. 외부 플래시는 건드리지 않습니다. 보드는 개발 모드(BOOT1 2-3)여야 합니다.
올리기 전에 보드를 리셋합니다. 앞서 돌던 프로그램이 오류 처리 상태(HardFault)로 멈춰 있었다면, 리셋 없이 올린
새 프로그램은 인터럽트가 막힌 채로 돌아 시간 지연에서 멈춥니다(2026-11-24 에 실제로 겪음). --no-reset 으로 끌 수 있습니다.
"""
import argparse
import glob
import os
import re
import shutil
import struct
import subprocess
import sys
import time
from pathlib import Path

RAM_LO, RAM_HI = 0x34000000, 0x34400000      # 태그 이미지는 AXISRAM (0x3418xxxx) 에서 돈다


def elf_start(path):
    """(처음 스택 값 MSP, 시작 주소 PC). 벡터 표(.isr_vector)의 첫 두 칸이다."""
    data = Path(path).read_bytes()
    if data[:4] != b"\x7fELF" or data[4] != 1 or data[5] != 1:
        sys.exit(f"{path}: 32비트 리틀엔디언 ELF 가 아닙니다")
    entry, shoff = struct.unpack_from("<I", data, 0x18)[0], struct.unpack_from("<I", data, 0x20)[0]
    shentsize, shnum, shstrndx = struct.unpack_from("<HHH", data, 0x2E)
    sections = [struct.unpack_from("<IIIIIIIIII", data, shoff + i * shentsize) for i in range(shnum)]
    names_off = sections[shstrndx][4]
    for name, _type, _flags, addr, offset, size, *_ in sections:
        end = data.index(b"\0", names_off + name)
        if data[names_off + name:end] == b".isr_vector" and size >= 8:
            msp, reset = struct.unpack_from("<II", data, offset)
            if reset & ~1 != entry & ~1:
                sys.exit(f"{path}: 벡터 표의 시작 주소 0x{reset:08X} 와 ELF 의 0x{entry:08X} 가 다릅니다")
            if not (RAM_LO <= addr < RAM_HI and RAM_LO < msp <= RAM_HI and RAM_LO <= reset < RAM_HI):
                sys.exit(f"{path}: RAM 에서 도는 이미지가 아닙니다 (벡터 0x{addr:08X}, MSP 0x{msp:08X}, PC 0x{reset:08X})")
            return msp, reset & ~1
    sys.exit(f"{path}: .isr_vector 를 찾지 못했습니다")


def find_cli(given):
    if given:
        return Path(given)
    name = "STM32_Programmer_CLI.exe" if os.name == "nt" else "STM32_Programmer_CLI"
    on_path = shutil.which(name)
    if on_path:
        return Path(on_path)
    patterns = [
        r"C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",
        r"C:\ST\STM32CubeIDE_*\STM32CubeIDE\plugins\*cubeprogrammer*\tools\bin\STM32_Programmer_CLI.exe",
        r"C:\Program Files\STMicroelectronics\STM32CubeIDE*\STM32CubeIDE\plugins\*cubeprogrammer*\tools\bin\STM32_Programmer_CLI.exe",
        "/opt/st/stm32cubeide_*/plugins/*cubeprogrammer*/tools/bin/STM32_Programmer_CLI",
        str(Path.home() / "STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI"),
    ]
    for pattern in patterns:
        hits = sorted(glob.glob(pattern))
        if hits:
            return Path(hits[-1])
    sys.exit("STM32_Programmer_CLI 를 못 찾았습니다. --cli <경로> 로 지정하세요.")


def list_probes(cli):
    result = subprocess.run([str(cli), "-l", "stlink"], capture_output=True, text=True, errors="replace")
    serials = re.findall(r"ST-LINK SN\s*:\s*([0-9A-Za-z]+)", result.stdout)
    return serials, re.sub(r"\x1b\[[0-9;]*m", "", result.stdout)      # 색 표시 문자는 뺀다


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("elf", nargs="?", type=Path, help="올릴 ELF")
    p.add_argument("--sn", help="ST-LINK 일련번호 (하나만 연결돼 있으면 생략)")
    p.add_argument("--cli", help="STM32_Programmer_CLI 경로")
    p.add_argument("--list", action="store_true", help="연결된 ST-LINK 만 보여 줌")
    p.add_argument("--dry-run", action="store_true", help="명령만 보여 주고 실행하지 않음")
    p.add_argument("--no-reset", action="store_true", help="올리기 전에 보드를 리셋하지 않음")
    opt = p.parse_args()

    if opt.list:
        serials, out = list_probes(find_cli(opt.cli))
        print("\n".join(serials) if serials else out)
        return
    if opt.elf is None:
        p.error("ELF 파일을 주세요 (또는 --list)")
    if not opt.elf.is_file():
        sys.exit(f"{opt.elf} 가 없습니다")
    msp, pc = elf_start(opt.elf)
    print(f"{opt.elf.name}: MSP 0x{msp:08X}, PC 0x{pc:08X}")
    sn = opt.sn
    cli = None
    if not opt.dry_run or opt.cli:
        cli = find_cli(opt.cli)
    if sn is None and not opt.dry_run:
        serials, out = list_probes(cli)
        if len(serials) != 1:
            print(out)
            sys.exit(f"ST-LINK 가 {len(serials)}개 보입니다. --sn 으로 하나를 고르세요.")
        sn = serials[0]
    cmd = [str(cli or "STM32_Programmer_CLI"), "-c", "port=SWD", f"sn={sn or '<ST-LINK SN>'}", "mode=HOTPLUG",
           "freq=1000", "-halt", "-d", str(opt.elf), "-v", "-halt", "-coreReg", f"MSP=0x{msp:08X}",
           f"PC=0x{pc:08X}", "XPSR=0x01000000", "-run"]
    reset = [cmd[0], "-c", "port=SWD", f"sn={sn or '<ST-LINK SN>'}", "mode=HOTPLUG", "freq=1000", "-hardRst"]
    if not opt.no_reset:
        print("리셋: " + " ".join(f'"{c}"' if " " in c else c for c in reset))
    print("명령: " + " ".join(f'"{c}"' if " " in c else c for c in cmd))
    if opt.dry_run:
        return
    if not opt.no_reset:
        done = subprocess.run(reset, capture_output=True, text=True, errors="replace")
        if done.returncode != 0:
            print((done.stdout + done.stderr)[-1500:])
            sys.exit("리셋 실패. 위 메시지를 그대로 알려 주세요.")
        time.sleep(1.0)
    result = subprocess.run(cmd, capture_output=True, text=True, errors="replace")
    out = result.stdout + result.stderr
    keep = ("Error", "error", "Download", "verified", "RUNNING", "Core run", "halted", "ST-LINK SN", "Device name")
    print("\n".join(line for line in out.splitlines() if any(k in line for k in keep)))
    if result.returncode != 0 or "Download verified successfully" not in out:
        print(out[-3000:])
        sys.exit("올리기 실패. 위 메시지를 그대로 알려 주세요.")
    print("완료: RAM 에 올리고 실행했습니다. 기록 창에 그 펌웨어의 시작 줄(TWR3_READY, TF_READY, SD,START 등)이 나와야 합니다.")


if __name__ == "__main__":
    main()
