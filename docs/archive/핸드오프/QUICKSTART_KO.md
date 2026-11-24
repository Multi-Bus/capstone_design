# 따라하기 — STM32(NUCLEO) 보드 + BLE로 드론 연결하기

이 문서는 **순서대로 따라 하는 용도**입니다. 명령의 자세한 설명과 문제 해결 표는 `README_KO.md`에 있습니다.
어디까지 진행됐는지는 `HANDOFF_KO.md`를 보세요.

> ⚠ **보드에 플래시하는 명령과 모터가 도는 명령(시동·이륙)은 실행 전에 팀에 먼저 알리세요.**
> 처음에는 **반드시 프롭을 떼고** 하세요.

---

## 0. 알아 둘 것

```
노트북 ──BLE──> XIAO (N6-DRONE-1) ──115200──> NUCLEO (브리지) ──921600──> FC (ArduPilot)
```

- **NUCLEO 외부 플래시에는 브리지 펌웨어가 이미 들어 있습니다**(2026-11-19). 펌웨어를 고칠 게 아니면 **1장은 건너뛰고 2장부터** 하면 됩니다.
- NUCLEO는 바이트를 그대로 넘기기만 합니다. 비행 동작을 바꾸려면 `tools/n6_fly.py`(Python)를 고칩니다.
- 준비물
  - 드론(배터리 포함)
  - BLE가 되는 노트북. 내장 블루투스가 없으면 USB BLE 동글을 씁니다.
  - USB-C 케이블: 펌웨어를 올릴 때만 필요합니다.
- 실제로 확인한 환경은 **Linux(Ubuntu, BlueZ) + Python 3.13 + bleak 3.1.0**입니다. Windows는 코드상 지원하지만 시험하지 않았습니다.
- **Windows에서 하는 분은 이 문서의 명령 대신 8장을 보세요.** 아래 명령은 Linux 기준입니다.

### 보드 스위치·점퍼 (NUCLEO-N657X0-Q)

| 모드 | BOOT0 | BOOT1 | CN9 | 전원 | 쓰는 때 |
|---|---|---|---|---|---|
| **비행 (플래시 부팅)** | **1-2** | **1-2** | **3-4** | 캐리어 J12 → NUCLEO **CN8** | 평소. 지금 이 상태 |
| **개발** (RAM 실행, 플래시 쓰기) | 상관없음 | **2-3** | **1-2** (STLK) | PC → ST-LINK USB **CN10** | 펌웨어를 고칠 때만 |

**스위치·점퍼는 항상 전원을 끈 상태에서** 바꾸세요.

### LED 읽는 법

| LED | 상태 | 뜻 |
|---|---|---|
| NUCLEO **LED1** | 1초마다 **짧게(0.05 s)** 깜빡 | 펌웨어는 돌고 있음. BLE 데이터가 아직 없음 |
| NUCLEO **LED1** | **0.5 s 켜짐 / 0.5 s 꺼짐** | XIAO가 BLE 연결됨 + 최근 1초 안에 노트북 데이터를 받음 (스크립트 실행 중 정상) |
| NUCLEO **LED1** | 꺼져 있음 | 펌웨어가 안 돎. 스위치·전원 확인 |
| NUCLEO **LED2** | 켜짐 | 오류(`Error_Handler`): UART 초기화 실패 또는 보드레이트 오차 |
| XIAO 파랑 | 켜짐 | 무언가 BLE로 연결돼 있음 |
| XIAO 초록 | 켜짐 | 최근 1초 안에 노트북 데이터를 받음 |
| XIAO 둘 다 꺼짐 | | 광고 중(연결 대기) |

---

## 1. (필요할 때만) NUCLEO 펌웨어 빌드·올리기

### 1-1. 설치
- **STM32CubeIDE**: Linux는 st.com의 *Debian Linux Installer*로 설치합니다. 개발 PC에는 `/opt/st/stm32cubeide_2.2.0`이 있습니다.
- **CubeProgrammer CLI**는 CubeIDE 안에 들어 있습니다. 경로를 찾아서 변수로 잡아 두세요.
  ```bash
  export CUBEPROG=$(dirname "$(find /opt/st -name STM32_Programmer_CLI -path '*cubeprogrammer*' | head -1)")
  $CUBEPROG/STM32_Programmer_CLI --version      # 2.21 이상이면 OK (개발 PC는 2.23.0)
  ```

### 1-2. 빌드
1. CubeIDE를 엽니다. **File → Import → General → Existing Projects into Workspace**를 고릅니다.
2. 폴더로 `nucleo/n6_bridge/STM32CubeIDE`를 고르고 **Search for nested projects**를 체크합니다. `n6_bridge_FSBL`을 선택합니다.
   - **Copy projects into workspace는 끄세요.** 상대 경로가 깨집니다.
3. 빌드 구성을 `Release`로 하고 망치 아이콘을 누릅니다.
4. 결과물: `nucleo/n6_bridge/STM32CubeIDE/FSBL/Release/n6_bridge_FSBL.bin`

명령줄로 빌드하는 방법은 `README_KO.md` 3-2를 보세요.

> **CubeMX로 코드를 다시 생성하지 마세요.** 브리지 코드를 덮어씁니다.

### 1-3. 개발 모드에서 먼저 시험 (RAM 실행, 선택)
1. 전원을 끈 상태에서 **BOOT1 = 2-3, CN9 = 1-2**로 바꾸고, PC를 **CN10**에 연결합니다.
2. CubeIDE에서 `n6_bridge_FSBL`을 우클릭하고 **Debug As → STM32 C/C++ Application**을 고릅니다.
3. LED1이 깜빡이면 정상입니다.
   - 디버거로 볼 변수는 `nucleo/n6_bridge/README.md`의 표에 있습니다.
   - 개발 모드는 RAM에서 돌기 때문에 전원을 끄면 사라집니다.

### 1-4. 서명하고 외부 플래시에 쓰기 (비행용)
> **쓰기 직전에 CN10 USB를 뽑았다 다시 꽂으세요.**
> 브리지가 디버거 없이 돌고 있으면 ST-LINK가 코어에 붙지 못합니다(`Unable to get core ID`). 이 상태는 전원을 껐다 켜야만 풀립니다.

```bash
cd nucleo/n6_bridge/STM32CubeIDE/FSBL/Release

# (1) FSBL 헤더 붙이기 (파일만 만듦, 보드 안 건드림)
$CUBEPROG/STM32_SigningTool_CLI -bin n6_bridge_FSBL.bin -nk -of 0x80000000 -t fsbl \
  -o n6_bridge_FSBL-trusted.bin -hv 2.3 -align -dump n6_bridge_FSBL-trusted.bin

# (2) 외부 플래시 0x70000000 에 쓰기 (스위치는 개발 모드 그대로: BOOT1 = 2-3, CN9 = 1-2)
$CUBEPROG/STM32_Programmer_CLI -c port=SWD mode=HOTPLUG \
  -el $CUBEPROG/ExternalLoader/MX25UM51245G_STM32N6570-NUCLEO.stldr \
  -hardRst -w n6_bridge_FSBL-trusted.bin 0x70000000
```

(3) **전원을 끈 상태에서**(CN10을 뽑고) **BOOT0 = 1-2, BOOT1 = 1-2, CN9 = 3-4**로 바꿉니다. 그다음 2장으로 갑니다.

---

## 2. 비행 구성으로 전원 넣기

1. 스위치가 **비행 모드**(BOOT0 1-2, BOOT1 1-2, CN9 3-4)이고, 캐리어 **J12 → NUCLEO CN8** 케이블이 꽂혀 있는지 확인합니다.
2. **프롭을 뗍니다**(처음 시험이라면).
3. **드론 배터리를 연결합니다.** FC, 캐리어, NUCLEO, XIAO에 모두 전원이 들어갑니다.
   - XIAO는 배터리가 연결돼야 BLE가 안정적입니다.
4. 기대 상태: NUCLEO LED1이 **1초마다 짧게 깜빡**이고, LED2는 꺼져 있고, XIAO LED는 둘 다 꺼져 있습니다(광고 중).
5. 기체는 **평평한 바닥에** 두세요. FC는 켜질 때 높이 기준을 잡습니다. 책상에서 켠 뒤 바닥으로 옮기면 높이가 틀어지니, 옮겼으면 배터리를 뺐다 다시 꽂습니다.
   - FC가 "위치 찾아 달라"는 소리를 내면 배터리를 뺐다 다시 꽂아도 됩니다.

---

## 3. 노트북 준비 (처음 한 번)

### 3-1. Python
```bash
cd tools
python3 -m venv .venv            # 이 폴더를 받았다면 기존 .venv 는 지우고 새로 만드세요
source .venv/bin/activate
pip install -r requirements.txt  # bleak, pymavlink
```
Python 3.10 이상이 필요합니다. Windows는 `py -m venv .venv` → `.venv\Scripts\Activate.ps1`입니다.

### 3-2. 블루투스
- **운영체제 블루투스 설정에서 XIAO와 페어링하거나 연결하지 마세요.** 다른 기기가 연결돼 있으면 XIAO가 광고를 멈춰서 스크립트가 찾지 못합니다.
- Linux에서 어댑터 확인:
  ```bash
  ls /sys/class/bluetooth          # hci0 가 있어야 함 (비어 있으면 어댑터 없음)
  bluetoothctl show | grep Powered # Powered: yes
  ```
- 드론 배터리를 연결한 상태에서 스캔해 봅니다(연결은 하지 않음).
  ```bash
  python dev/ble_scan.py           # "N6-DRONE-1  FB:55:...  RSSI -50 dBm" 처럼 나오면 OK
  ```

---

## 4. 연결 시험 (`--no-arm`: 모터 안 돎)

```bash
cd tools && source .venv/bin/activate
python n6_fly.py --no-arm
```

정상이면 몇 초 안에 이런 줄이 나옵니다.
```
BLE 연결됨 (FB:55:E2:D6:29:FB, 조각 244 B, write-without-response)
FC HEARTBEAT 수신: 모드 STABILIZE, 시동 OFF
노트북→FC 방향 확인됨 (COMMAND_ACK 수신)
[FC INFO] EKF3 IMU0 origin set
페일세이프 파라미터 OK (지상국 sysid 250, FS_GCS_ENABLE 5, FS_GCS_TIMEOUT 3 s)
[상태] BLE OK | 링크 0.1s | STABILIZE | 시동 OFF | EKF높이 +0.03m | 거리센서 0.02m | 15.93V
```

- 이때 LED는 이렇게 바뀝니다: XIAO는 파랑 + 초록이 켜지고, NUCLEO LED1은 0.5초 켜짐 / 0.5초 꺼짐이 됩니다.
- **상태줄 읽는 법**

  | 항목 | 뜻 |
  |---|---|
  | `링크` | FC에서 마지막으로 받은 뒤 지난 시간 (1초 안쪽이면 정상) |
  | 모드 | FC 비행 모드 (STABILIZE, GUIDED, LAND) |
  | `시동` | ON / OFF |
  | `EKF높이` | FC가 계산한 높이 |
  | `거리센서` | 실제 바닥까지 거리 (바닥에 두면 0.02 m) |
  | 마지막 숫자 | 배터리 전압 |

  - **바닥에서 `EKF높이`가 거리센서와 0.1 m 넘게 차이 나면** 비행하지 말고 배터리를 뺐다 꽂아 재부팅하세요.
- 끝낼 때는 `q`를 누릅니다.
- 화면 내용은 모두 `tools/logs/n6_fly_*.log`에도 저장됩니다.

## 5. BLE 문제 해결

| 증상 | 해결 |
|---|---|
| `SCAN ERROR` / `org.bluez ... timed out` | PC에 블루투스 어댑터가 없거나 꺼져 있음. USB 동글을 꽂고 `sudo systemctl start bluetooth` |
| 스캔 결과 0개, `journalctl -k`에 `hci0: command 0x2005 tx timeout` | USB 동글 자동 절전 문제 (개발 PC의 Realtek RTL8761BU에서 실제로 발생) |
| `'N6-DRONE-1' 찾는 중...`이 계속 나옴 | ① 드론 배터리 연결 ② **XIAO 파란 LED가 켜져 있으면** 다른 기기(휴대폰, 다른 스크립트, OS 설정)가 이미 연결 중 → 끊기 ③ `--name`이 XIAO 이름과 같은지 |
| BLE가 자꾸 끊김 | 드론 배터리를 연결했는지 (USB 5V만으로는 XIAO가 불안정) |
| BLE는 연결되는데 `FC 응답 없음` | NUCLEO LED1/LED2, 스위치가 비행 모드인지, FC 전원 |
| HEARTBEAT는 오는데 `노트북→FC 방향 확인됨`이 안 나옴 | 노트북 → FC 방향만 끊김. 새 기체라면 **FC의 "Dji" 점퍼 패드 납땜**을 확인 (`HANDOFF_KO.md` 5장) |
| 상태줄의 모드·시동 표시가 몇 초 늦음 | 알려진 문제 (XIAO가 FC → 노트북 데이터 일부를 버림). 스크립트는 ACK로 확인하므로 동작에는 지장 없음. 고치려면 6장 |

USB 동글 자동 절전 끄기 (Linux):
```bash
# 지금 바로 (1-2 는 동글이 꽂힌 USB 포트 번호. lsusb -t 나 dmesg 로 확인)
echo on | sudo tee /sys/bus/usb/devices/1-2/power/control
bluetoothctl power off; bluetoothctl power on
# 재부팅 뒤에도 유지
echo "options btusb enable_autosuspend=n" | sudo tee /etc/modprobe.d/btusb.conf
```

## 6. (선택) XIAO 펌웨어 v2 올리기

- **지금 XIAO에 들어 있는 것**: 원래 문서 "XIAO BLE 중계 펌웨어 올리기"의 v1입니다. 동작은 하지만 FC → 노트북 데이터 일부를 버립니다.
- **v2**는 `tools/xiao/n6_ble_relay/n6_ble_relay.ino`입니다. 바이트를 모아서 보내도록 고쳤고, 나머지(이름, 핀, LED 규칙)는 v1과 같습니다.
  - 파일 맨 위 주석에 무엇을 고쳤는지 적어 두었습니다.
  - **아직 한 번도 컴파일하거나 올려 보지 않았습니다.** 올린 뒤 4장 시험을 다시 하세요.

올리는 법 (Arduino IDE 2):
1. **File → Preferences → Additional boards manager URLs**에 다음 주소를 넣습니다.
   `https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json`
2. 보드 매니저에서 **"Seeed nRF52 Boards"**를 설치합니다. **mbed가 붙은 쪽이 아닙니다.** Bluefruit 라이브러리가 같이 들어 있습니다.
3. 보드로 **Seeed XIAO nRF52840**을 고릅니다.
4. 파일 맨 위의 `DRONE_ID`를 확인합니다. 1이면 이름이 `N6-DRONE-1`이 됩니다.
5. XIAO를 **캐리어에서 빼고** USB-C로 PC에 연결한 뒤 업로드합니다.
   - 업로드가 안 되면 리셋 버튼을 빠르게 두 번 누르세요(부트로더 모드).
6. 시리얼 모니터(115200)를 엽니다. 노트북 스크립트가 연결되면 `BLE connected`가 나옵니다. 데이터를 버리면 `txDropped=`가 나옵니다.
7. XIAO를 캐리어에 다시 꽂고 4장 시험을 합니다.

---

## 7. 이륙 시험 (모터가 돕니다)

`HANDOFF_KO.md` 6장을 먼저 읽으세요. **프롭 단 비행은 아직 성공하지 못했습니다.**

- 순서는 `README_KO.md` 8장 체크리스트를 따릅니다. **프롭 없이** 킬, 블루투스 끄기, 전체 순서를 먼저 확인합니다.
- 실행: `python n6_fly.py --alt 0.5 --hover 3` → `t` → `y`
- 키 (**영문 입력 상태로**)

  | 키 | 동작 |
  |---|---|
  | **스페이스 / 엔터 / `k`** | **킬**: 그 자리 LAND, 착지 후 FC가 시동을 끔 |
  | `x`를 1초 안에 두 번 | 비상 모터 정지 (**공중이면 추락**). 바닥에서 시동이 안 꺼질 때 사용 |
  | `q` | 종료 (시동 상태면 LAND 먼저) |
- 링크가 끊기면 FC가 3초 뒤 스스로 LAND합니다(GCS 페일세이프).

---

## 8. Windows에서 할 때

**아직 Windows에서 실제로 돌려 본 적은 없습니다.** 코드는 Windows도 고려해서 썼습니다(키 입력 `msvcrt`, BLE는 bleak의 WinRT 백엔드). 처음 해 보는 분이 결과를 이 문서에 적어 주세요.

### 8-1. Python과 스크립트 (PowerShell)
```powershell
cd <압축 푼 곳>\stm32_projects\tools
py -3 -m venv .venv                       # Python 3.10 이상 (python.org 설치판 권장)
.venv\Scripts\python -m pip install -r requirements.txt
.venv\Scripts\python dev\ble_scan.py      # N6-DRONE-1 이 보이면 OK
.venv\Scripts\python n6_fly.py --no-arm   # 연결 시험 (4장과 같음)
```
- `Activate.ps1`이 실행 정책 때문에 막히면, 위처럼 `.venv\Scripts\python`을 직접 부르면 됩니다.
- **Windows Terminal**에서 실행하세요. 옛 콘솔에서도 동작은 하지만 한글이나 상태줄이 깨져 보일 수 있습니다.
- 키는 같습니다(스페이스 = 킬). **영문 입력 상태**로 두세요.
- 로그 파일은 `tools\logs\`에 UTF-8로 저장됩니다. 메모장이나 VS Code로 열면 됩니다.

### 8-2. 블루투스
- Windows 10(1809 이상)이나 11이 필요하고, BLE가 되는 어댑터가 있어야 합니다. 내장 블루투스나 USB 동글 모두 됩니다.
- **Windows 설정 → 블루투스에서 XIAO를 "장치 추가"로 페어링하지 마세요.** 스크립트가 직접 연결합니다.
  - 이미 추가해 두었다면 **장치 제거** 후 다시 시도하세요. Windows가 연결을 잡고 있으면 XIAO가 광고를 멈춥니다.
- XIAO 펌웨어를 바꾼 뒤 연결이 이상하면, 설정에서 장치를 제거해 Windows에 남은 연결 정보를 지우세요.
- 5장의 USB 자동 절전(`btusb`) 해결법은 Linux 전용입니다.

### 8-3. NUCLEO 펌웨어 (1장)
- STM32CubeIDE Windows 설치판을 쓰면 ST-LINK 드라이버도 같이 설치됩니다. 빌드는 GUI로 하면 됩니다(1-2와 같음).
- CubeProgrammer CLI 위치는 둘 중 하나입니다.
  - CubeIDE 안: `C:\ST\STM32CubeIDE_<버전>\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_<버전>\tools\bin\`
  - 독립 설치판: `C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\`
- 서명과 플래시 (PowerShell, 각각 한 줄로):
```powershell
$CUBEPROG = Split-Path (Get-ChildItem C:\ST, "C:\Program Files\STMicroelectronics" -Recurse -Filter STM32_Programmer_CLI.exe -ErrorAction SilentlyContinue | Select-Object -First 1).FullName
cd <압축 푼 곳>\stm32_projects\nucleo\n6_bridge\STM32CubeIDE\FSBL\Release
& "$CUBEPROG\STM32_SigningTool_CLI.exe" -bin n6_bridge_FSBL.bin -nk -of 0x80000000 -t fsbl -o n6_bridge_FSBL-trusted.bin -hv 2.3 -align -dump n6_bridge_FSBL-trusted.bin
& "$CUBEPROG\STM32_Programmer_CLI.exe" -c port=SWD mode=HOTPLUG -el "$CUBEPROG\ExternalLoader\MX25UM51245G_STM32N6570-NUCLEO.stldr" -hardRst -w n6_bridge_FSBL-trusted.bin 0x70000000
```
- 스위치 순서와 "쓰기 직전에 CN10을 뽑았다 꽂기"는 1-4와 같습니다. 플래시에 쓰는 명령이니 **실행 전에 팀에 알리세요.**
- 압축 경로가 너무 길면(260자 제한) 빌드가 실패할 수 있습니다. `C:\n6\` 같은 짧은 곳에 압축을 푸세요.

### 8-4. FC 설정 (QGC와 FC 설정 스크립트)
- QGC Windows판은 그대로 쓰면 됩니다. FC USB는 `COM<숫자>`로 잡힙니다.
- **`../fc_control/n6_fc_setup/`의 스크립트(`apply_params.py` 등)는 Linux 전용**입니다.
  - FC 포트를 `/dev/serial/by-id/...`로 찾고, 포트를 쓰는 프로그램을 `/proc`에서 찾습니다. Windows에서는 그대로 돌아가지 않습니다.
  - 값을 몇 개만 바꿀 때는 **QGC → Vehicle Setup → Parameters**를 쓰세요.
  - 스크립트가 꼭 필요하면 `--port COM5`처럼 COM 포트를 바로 받게 고쳐서 씁니다(`CLAUDE.md` 참고).
- 노트북 쪽 BLE 도구(`tools\dev\param_read.py` 등)는 OS와 상관없이 동작하도록 썼습니다.

### 8-5. XIAO 펌웨어 (6장)
- Arduino IDE 2 Windows판에서 순서는 같습니다.
- 업로드 포트가 안 보이면 XIAO 리셋 버튼을 두 번 누르세요. 부트로더 드라이브가 나타납니다.
