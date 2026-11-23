"""DWM3001CDK 앵커 펌웨어 빌드 (Windows / Linux 공용).

    python build.py --id 2                 # 앵커 2번 -> build/A2/anchor_A2.hex
    python build.py --id 1 --id 2 --id 3   # 세 대 모두

컴파일러: --gcc-bin 으로 지정하거나, PATH 의 arm-none-eabi-gcc, 없으면 STM32CubeIDE 안의 GNU Tools 를 찾는다.
세 앵커는 같은 --slot-us 로 빌드해야 한다.
"""
import argparse
import glob
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SRC = ROOT / "src"
TP = ROOT / "third_party"
DRV = TP / "qorvo_dwt_uwb_driver"
MDK = TP / "nordic_mdk"
CMSIS = TP / "cmsis"
EXE = ".exe" if os.name == "nt" else ""

OWN_SOURCES = [SRC / "main.c", SRC / "port.c", SRC / "radio_config.c", SRC / "syscalls.c"]
THIRD_PARTY_SOURCES = [
    DRV / "deca_interface.c", DRV / "deca_compat.c", DRV / "deca_rsl.c",
    DRV / "lib" / "qmath" / "src" / "qmath.c", DRV / "dw3000" / "dw3000_device.c",
    MDK / "system_nrf52833.c", MDK / "gcc_startup_nrf52833.S",
]
INCLUDES = [SRC, DRV, DRV / "lib" / "qmath" / "include", MDK, CMSIS]
CPU = ["-mcpu=cortex-m4", "-mthumb", "-mfloat-abi=soft"]


def find_gcc_bin(given):
    if given:
        return Path(given)
    on_path = shutil.which("arm-none-eabi-gcc")
    if on_path:
        return Path(on_path).parent
    patterns = [
        "/opt/st/stm32cubeide_*/plugins/*gnu-tools-for-stm32*/tools/bin",
        r"C:\ST\STM32CubeIDE_*\STM32CubeIDE\plugins\*gnu-tools-for-stm32*\tools\bin",
        r"C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\*\bin",
        r"C:\Program Files\Arm GNU Toolchain arm-none-eabi\*\bin",
    ]
    for pattern in patterns:
        hits = sorted(glob.glob(pattern))
        if hits:
            return Path(hits[-1])
    sys.exit("arm-none-eabi-gcc 를 못 찾았습니다. --gcc-bin <툴체인 bin 폴더> 로 지정하세요.")


def run(cmd):
    result = subprocess.run([str(c) for c in cmd], capture_output=True, text=True)
    if result.stdout:
        print(result.stdout, end="")
    if result.stderr:
        print(result.stderr, end="", file=sys.stderr)
    if result.returncode != 0:
        sys.exit(f"실패: {Path(cmd[0]).name} (종료 코드 {result.returncode})")
    return result


def build(anchor_id, opt, gcc_bin):
    gcc = gcc_bin / f"arm-none-eabi-gcc{EXE}"
    objcopy = gcc_bin / f"arm-none-eabi-objcopy{EXE}"
    size = gcc_bin / f"arm-none-eabi-size{EXE}"
    out = ROOT / "build" / f"A{anchor_id}"
    obj_dir = out / "obj"
    obj_dir.mkdir(parents=True, exist_ok=True)
    defines = [
        "-DNRF52833_XXAA", "-DUSE_DRV_DW3000", "-D__STARTUP_CLEAR_BSS", "-D__START=main",
        f"-DUWB3_ANCHOR_ID={anchor_id}", f"-DUWB3_ANTENNA_DELAY={opt.antenna_delay}U",
        f"-DCDK_POWER_INDEX={opt.power_index}", f"-DANCHOR_SLOT_US={opt.slot_us}U",
    ]
    common = CPU + ["-std=c11", "-Os", "-g", "-ffunction-sections", "-fdata-sections"] + defines + \
        [f"-I{p}" for p in INCLUDES]
    objects = []
    for source in OWN_SOURCES + THIRD_PARTY_SOURCES:
        obj = obj_dir / (source.stem + ".o")
        warn = ["-Wall", "-Wextra", "-Werror"] if source in OWN_SOURCES else ["-w"]
        lang = ["-x", "assembler-with-cpp"] if source.suffix == ".S" else []
        run([gcc] + common + warn + lang + ["-c", source, "-o", obj])
        objects.append(obj)
    elf = out / f"anchor_A{anchor_id}.elf"
    run([gcc] + CPU + objects + [
        "-nostartfiles", "--specs=nano.specs",
        f"-L{MDK}", f"-T{MDK / 'nrf52833_xxaa.ld'}", "-Wl,--gc-sections",
        f"-Wl,-Map={out / f'anchor_A{anchor_id}.map'}", "-lm", "-o", elf])
    hexfile = elf.with_suffix(".hex")
    run([objcopy, "-O", "ihex", elf, hexfile])
    run([objcopy, "-O", "binary", elf, elf.with_suffix(".bin")])
    run([size, elf])
    digest = hashlib.sha256(hexfile.read_bytes()).hexdigest()
    manifest = {
        "anchor_id": anchor_id, "firmware": "anchor3to1-v1", "slot_us": opt.slot_us,
        "antenna_delay_dtu": opt.antenna_delay, "tx_power_index": opt.power_index,
        "channel": 5, "hex": hexfile.name, "hex_sha256": digest,
        "built": time.strftime("%Y-%m-%d %H:%M:%S"), "gcc": str(gcc),
    }
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"A{anchor_id}: {hexfile.relative_to(ROOT)}  sha256 {digest}")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--id", type=int, action="append", required=True, choices=[1, 2, 3], help="앵커 번호")
    p.add_argument("--slot-us", type=int, default=20000, help="앵커 한 칸 (us). 세 대 같게. 기본 20000")
    p.add_argument("--antenna-delay", type=int, default=16385, help="안테나 지연 (DTU). 기본 16385 (보정 전)")
    p.add_argument("--power-index", type=int, default=27, help="채널 5 송신 전력 인덱스. 기본 27")
    p.add_argument("--gcc-bin", help="arm-none-eabi-gcc 가 있는 bin 폴더")
    opt = p.parse_args()
    if not 5000 <= opt.slot_us <= 1000000:
        sys.exit("--slot-us 는 5000~1000000")
    if not 0 <= opt.antenna_delay <= 65535 or not 0 <= opt.power_index <= 255:
        sys.exit("--antenna-delay 0~65535, --power-index 0~255")
    gcc_bin = find_gcc_bin(opt.gcc_bin)
    print(f"컴파일러: {gcc_bin}")
    for anchor_id in sorted(set(opt.id)):
        build(anchor_id, opt, gcc_bin)


if __name__ == "__main__":
    main()
