#!/usr/bin/env bash
# 처음 한 번만: 파이썬 가상환경(.venv) 만들고 pymavlink 설치 + 자주 막히는 설정 확인
set -e
cd "$(dirname "$0")"

if ! command -v python3 >/dev/null; then
    echo "python3 가 없습니다: sudo apt install python3 python3-venv"; exit 1
fi
if [ ! -x .venv/bin/python ]; then
    python3 -m venv .venv || { echo "가상환경을 못 만들었습니다: sudo apt install python3-venv 후 다시 실행"; exit 1; }
fi
.venv/bin/pip install -q --upgrade pip
.venv/bin/pip install -q -r requirements.txt
.venv/bin/python -c "import pymavlink, serial; print('pymavlink', pymavlink.__version__, '/ pyserial', serial.VERSION, '설치됨')"

ok=1
if ! id -nG | grep -qw dialout; then        # 지금 로그인 세션 기준 (그룹 추가 후 재로그인 전이면 여기서 걸림)
    echo "주의: '$USER' 가 dialout 그룹이 아니라서 FC 포트를 못 열 수 있습니다."
    echo "      sudo usermod -aG dialout \$USER  실행 후 로그아웃/로그인 (또는 재부팅)"
    ok=0
fi
if systemctl is-active --quiet ModemManager 2>/dev/null; then
    echo "주의: ModemManager 가 켜져 있습니다. FC 를 꽂을 때 포트를 건드려서 연결이 끊길 수 있습니다."
    echo "      sudo apt remove modemmanager  권장"
    ok=0
fi
ls /dev/serial/by-id/usb-ArduPilot_* 2>/dev/null && echo "FC 가 보입니다." || echo "(지금은 FC 가 안 꽂혀 있음)"
[ $ok = 1 ] && echo "준비 끝."
exit 0
