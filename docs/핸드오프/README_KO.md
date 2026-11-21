# N6 드론 — NUCLEO 브리지 펌웨어 + 노트북 BLE 비행 스크립트

노트북에서 BLE 로 명령을 보내 드론을 **0.8 m 이륙 → 5 s 호버 → 착륙** 시킵니다.

```
노트북 (tools/n6_fly.py : bleak + pymavlink)
  --BLE (Nordic UART Service)-->  XIAO nRF52840  (기존 중계 펌웨어, 이름 N6-DRONE-x)
  --USART2 115200-->              NUCLEO-N657X0-Q (nucleo/n6_bridge : 투명 브리지)
  --UART7 921600-->               FC (DAKEFPV H743, ArduPilot 4.7.x, SERIAL4 = MAVLink2)
```

- NUCLEO 는 바이트를 양방향으로 그대로 넘기기만 합니다. 비행 로직은 전부 노트북 파이썬에 있습니다.
- 노트북이 0.2 s 마다 지상국 HEARTBEAT(sysid 250)를 보냅니다. 노트북·BLE·XIAO·NUCLEO 어디서 끊기든
  FC 의 GCS 페일세이프(`FS_GCS_TIMEOUT` 3 s)가 **스스로 LAND** 합니다.
- 예전 설계(NUCLEO 가 PD11 을 보고 하트비트를 대신 보냄)는 이 구조로 대체했습니다. PD11 은 이제 LED 표시에만 씁니다.

| 폴더/파일 | 내용 |
|---|---|
| `nucleo/n6_bridge/` | 브리지 펌웨어 (CubeIDE 프로젝트 `n6_bridge_FSBL`). 자세한 변경 내용은 그 안의 `README.md` |
| `tools/n6_fly.py` | 노트북 비행 스크립트 |
| `tools/requirements.txt` | 파이썬 의존성 |
| `tools/dev/` | 보조 도구: `sim_test.py`(오프라인 시뮬레이터), `param_read.py`(파라미터 읽기 전용), `height_check.py`(EKF 높이 vs 거리센서), `flow_check.py`(광류 센서 방향), `ble_scan.py` |
| `tools/xiao/n6_ble_relay/` | XIAO 중계 펌웨어 v2 (아직 업로드 안 함) |
| `tools/logs/` | 스크립트 실행 로그 (실행할 때 생김) |

> 진행 상황은 **`HANDOFF_KO.md`**, 따라하기는 **`QUICKSTART_KO.md`** 에 있습니다.

> ⚠ **보드에 플래시하는 명령, 모터가 도는 명령(시동·이륙)은 아래 순서를 지키고, 처음에는 반드시 프롭을 떼고 하세요.**

---

## 1. 설치

### 1-1. STM32CubeIDE (Linux)
- 이 PC 에는 이미 `/opt/st/stm32cubeide_2.2.0` 이 설치되어 있습니다.
- 새 PC 라면 st.com 에서 *STM32CubeIDE Debian Linux Installer* 를 받아 `sudo sh ./st-stm32cubeide_*.deb_bundle.sh`.
  ST-LINK udev 규칙도 같이 설치됩니다 (설치 후 USB 를 다시 꽂기).

### 1-2. STM32CubeProgrammer
- 별도 설치 없이 **CubeIDE 에 들어 있는 CLI(v2.23.0)** 를 씁니다. 편하게 쓰려고 변수로 잡아 둡니다:
  ```bash
  export CUBEPROG=/opt/st/stm32cubeide_2.2.0/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.linux64_2.2.500.202603051304/tools/bin
  $CUBEPROG/STM32_Programmer_CLI --version      # 2.23.0 이 나오면 OK (보드 접속 안 함)
  ```
- 독립 설치판(STM32CubeProgrammer)을 써도 됩니다. 그 경우 `bin/` 폴더를 `CUBEPROG` 로 잡으세요.

### 1-3. Python (노트북)
Python **3.10 이상** (Ubuntu 22.04 기본 3.10 OK).

Ubuntu 22.04:
```bash
cd ~/stm32_projects/tools
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```
BlueZ 5.64 기준으로 확인했습니다. root 권한은 필요 없습니다(D-Bus 사용).

Windows (PowerShell):
```powershell
cd stm32_projects\tools
py -m venv .venv
.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

---

## 2. 배선 (확인된 값)

| 연결 | MCU 핀 | AF | 설정 |
|---|---|---|---|
| USART2_TX → XIAO D7 | PA2 | `GPIO_AF7_USART2` | 115200 8N1 |
| USART2_RX ← XIAO D6 | PF6 | `GPIO_AF7_USART2` | |
| UART7_TX → FC RX4 (J2 2번) | PH4 | `GPIO_AF8_UART7` | 921600 8N1 |
| UART7_RX ← FC TX4 (J2 3번) | PE7 | `GPIO_AF8_UART7` | |
| XIAO_INT ← XIAO D0 (링크 살아 있음) | PD11 | GPIO 입력, 내부 풀다운 | LED 표시에만 사용 |

- NUCLEO 전원 (모드별, 5 장 표 참고)
  - 개발·플래시 쓰기: PC → **ST-LINK USB (CN10)**, **CN9 = 1-2 (STLK)**
  - 비행: 캐리어 **J12 → NUCLEO CN8(USB-C)**, **CN9 = 3-4**
- 펌웨어는 위 핀 외에는 건드리지 않습니다(SD PC0~PC5/PE4, I2C1 PH9/PC1, MUX_RSTn PE1, LED PE13/PE14, PD7 UWB WAKEUP 등).
  NUCLEO 보드 자체 LED1(PG8)·LED2(PG10)만 BSP 로 씁니다.
- 핀/AF 는 CubeMX DB(`GPIO-STM32N6_gpio_v1_0_Modes.xml`)와 STM32CubeN6 V1.4.1 HAL 헤더로 다시 확인했습니다.

---

## 3. 펌웨어 빌드

### 3-1. CubeIDE (GUI)
1. `File > Import... > General > Existing Projects into Workspace`
2. *Select root directory* 에 `~/stm32_projects/nucleo/n6_bridge/STM32CubeIDE` → **Search for nested projects** 체크
   → `n6_bridge_FSBL` 선택 → Finish. (*Copy projects into workspace* 는 **끄세요** — 상대 경로가 깨집니다.)
3. 프로젝트 선택 → 망치 아이콘(Build). 비행용은 `Release` 를 권장합니다(`Project > Build Configurations > Set Active`).
4. 결과물: `nucleo/n6_bridge/STM32CubeIDE/FSBL/Release/n6_bridge_FSBL.bin` (Debug 면 `Debug/`).

### 3-2. 명령줄 (같은 프로젝트를 headless 로)
```bash
/opt/st/stm32cubeide_2.2.0/headless-build.sh -data /tmp/ws_n6 \
  -import ~/stm32_projects/nucleo/n6_bridge/STM32CubeIDE/FSBL \
  -build n6_bridge_FSBL/Release
```
이 방법으로 Debug·Release 둘 다 **오류 0, 경고 0** 으로 빌드되는 것을 확인했습니다 (Release: text 24.5 KB, bss 11 KB).

### 3-3. 보드레이트 오차 (주석과 실행 중 검사)
`SystemClock_Config` 는 예제 그대로이고 **PCLK1 = 100 MHz** 입니다
(HSI 64 MHz → PLL1 1200 MHz → IC2 /3 = 400 MHz → AHB /4 = 100 MHz → APB1 /1).

| UART | 목표 | BRR | 실제 | 오차 |
|---|---|---|---|---|
| USART2 | 115 200 | 868 | 115 207 | +0.006 % |
| UART7 | 921 600 | 109 | 917 431 | −0.45 % |

둘 다 2 % 이하. 펌웨어가 시작할 때 `HAL_RCC_GetPCLK1Freq()` 와 BRR 로 같은 계산을 다시 해서
2 % 를 넘으면 `Error_Handler`(LED2 켜짐)로 멈춥니다.

---

## 4. 개발 모드 실행 (RAM, 시험용)

1. 보드 스위치 **BOOT1 = 2-3** (BOOT0 는 상관없음), **CN9 = 1-2 (STLK)**, PC 를 **ST-LINK USB (CN10)** 에 연결. 보드 리셋.
2. CubeIDE 에서 `n6_bridge_FSBL` 우클릭 → `Debug As > STM32 C/C++ Application` (또는 Run As).
3. 정상이면:
   - **LED1** 이 1 s 주기로 깜빡임 = 펌웨어 살아 있음
     - 0.5 s 켜짐 / 0.5 s 꺼짐 → PD11 high = XIAO 가 **BLE 연결 + 최근 1 s 안에 데이터 받음** (스크립트 실행 중이면 이 상태)
     - 0.05 s 짧게 깜빡 → PD11 low = BLE 미연결, 또는 연결됐지만 1 s 넘게 데이터 없음, 또는 D0 배선 없음
   - BLE 연결 자체는 **XIAO LED** 로 보는 게 정확합니다: 파랑 = 연결됨, 초록 = 최근 1 s 데이터 수신, 둘 다 꺼짐 = 광고 중(검색 대기)
   - **LED2** 꺼짐. 켜져 있으면 `Error_Handler` (UART 초기화 실패 또는 보드레이트 오차 > 2 %)
4. Live Expressions 로 볼 변수는 `nucleo/n6_bridge/README.md` 의 표 참고 (`g_stat_fc.rx_bytes` 가 늘면 FC 에서 받는 중).
   - D-Cache 가 켜져 있어서 디버거는 RAM 만 보므로, 펌웨어가 0.5 s 마다 이 변수들을 캐시에서 RAM 으로 내보냅니다
     → 값은 0.5 s 단위로 갱신됩니다.
   - **Debug 세션은 리셋부터 계속 붙여 둔 채로 보세요.** 디버거 없이 돌고 있는 보드에 나중에 attach 하는 것은
     이 보드에서 실패하는 것을 봤습니다 (`Target unknown error 32`, AP1 접근 불가).
   - UART 데이터 레지스터 `RDR`(오프셋 0x24)을 디버거 메모리 창으로 읽지 마세요. 읽는 순간 RX FIFO 에서 바이트가 빠져
     브리지 데이터가 사라집니다. (Expressions/SFR 창에 USART2/UART7 을 펼쳐 두는 것도 같은 문제)

개발 모드는 RAM 에서 돌기 때문에 전원을 끄면 사라집니다. 비행용은 5 장처럼 플래시에 올립니다.

---

## 5. 서명·플래시 부팅 (비행용) — 실행 전에 한 번 더 확인!

ST 예제 README 절차 그대로이고, CubeProgrammer **v2.21 이상이라 `-align` 을 추가**했습니다
(내장 v2.23.0 의 `--help` 에서 철자 `-align` 확인).

> **플래시 직전에 ST-LINK USB(CN10)를 뽑았다 다시 꽂으세요** (개발 모드 그대로).
> 브리지가 디버거 없이 RAM 에서 돌고 있으면 ST-LINK 가 CM55 코어(AP1)에 못 붙고
> (`Unable to get core ID`), 이 상태는 `-hardRst` 로도 안 풀리고 전원을 껐다 켜야 풀리는 것을 봤습니다.
> 전원을 다시 넣은 직후(부트 ROM 대기, LED1 꺼짐)에 바로 (2)를 실행하면 됩니다.

```bash
cd ~/stm32_projects/nucleo/n6_bridge/STM32CubeIDE/FSBL/Release

# (1) FSBL 헤더 붙이기 (키 없음). 파일만 만들고 보드는 건드리지 않음
$CUBEPROG/STM32_SigningTool_CLI -bin n6_bridge_FSBL.bin -nk -of 0x80000000 -t fsbl \
  -o n6_bridge_FSBL-trusted.bin -hv 2.3 -align -dump n6_bridge_FSBL-trusted.bin

# (2) 외부 플래시 0x70000000 에 쓰기 — 스위치는 개발 모드(BOOT1 = 2-3) 그대로인 상태에서
$CUBEPROG/STM32_Programmer_CLI -c port=SWD mode=HOTPLUG \
  -el $CUBEPROG/ExternalLoader/MX25UM51245G_STM32N6570-NUCLEO.stldr \
  -hardRst -w n6_bridge_FSBL-trusted.bin 0x70000000
```

(3) **전원을 끈 상태에서** (ST-LINK USB CN10 을 뽑고) 스위치·점퍼를 **BOOT0 = 1-2, BOOT1 = 1-2, CN9 = 3-4** 로 바꾼 뒤,
캐리어 J12 → CN8 로 전원을 넣음 → 플래시에서 부팅. LED1 이 깜빡이면 성공.

### 스위치·점퍼 요약

| 상태 | BOOT0 | BOOT1 | CN9 | 전원 |
|---|---|---|---|---|
| 개발 모드 (CubeIDE 로 RAM 실행, 플래시 쓰기) | 상관없음 | **2-3** | **1-2** (STLK) | PC → ST-LINK USB **CN10** |
| 플래시 부팅 (비행) | **1-2** | **1-2** | **3-4** | 캐리어 J12 → **CN8** |

스위치·점퍼는 **전원을 끈 상태에서** 바꾸세요.

---

## 6. 노트북 스크립트 `tools/n6_fly.py`

```bash
cd ~/stm32_projects/tools && source .venv/bin/activate
python n6_fly.py --no-arm            # 처음엔 연결·텔레메트리만 (t 키가 막힘)
python n6_fly.py                     # N6-DRONE-1 에 연결, 0.8 m / 5 s
python n6_fly.py --name N6-DRONE-2 --alt 0.8 --hover 5
```

| 인자 | 기본 | 설명 |
|---|---|---|
| `--name` | `N6-DRONE-1` | XIAO BLE 광고 이름 |
| `--alt` | `0.8` | 이륙 높이 m (0.3 ~ 2.0 만 허용 — 오타로 8 m 가 되는 것 방지) |
| `--hover` | `5` | 호버 시간 s (0 ~ 30) |
| `--no-arm` | | 연결·텔레메트리만. `t` 막힘 (킬·비상정지는 동작) |
| `--log-dir` | `tools/logs` | 로그 폴더 |

### 키

| 키 | 동작 | 보내는 것 |
|---|---|---|
| `t` → `y` | 이륙 시험 (`t` 다음 `y` 로 한 번 더 확인) | 원점 → `DO_SET_MODE(1,4)` → GUIDED 확인 → `ARM(1)` → 시동 확인 → `NAV_TAKEOFF(p7=alt)` → **거리센서 기준**(시동 때 값에서) (alt−0.1) m 도달 또는 8 s → 호버 → LAND → 시동 꺼짐 대기(≤30 s) |
| **스페이스 / 엔터 / `k`** | **킬 = 즉시 LAND** (언제나 최우선) | `DO_SET_MODE(1,9)` 를 0.1 s 간격 3번, 진행 중 시험 취소 |
| `x` 를 1 s 안에 두 번 | 비상 모터 정지 — **공중이면 추락** | `COMPONENT_ARM_DISARM(0, 21196)` (3번) |
| `q` (또는 Ctrl-C) | 종료. 시동 상태면 LAND 먼저 | |
| `h` | 도움말 | |

- 키는 한 글자 즉시 반응합니다 (Linux termios / Windows msvcrt). 안 되는 환경이면 줄 입력(글자+엔터, **빈 엔터 = 킬**).
- 한/영 키가 한글이어도 `ㅅ`(t) `ㅌ`(x) `ㅂ`(q) `ㅏ`(k) `ㅛ`(y) 를 받아 주지만, IME 조합 때문에 늦게 들어갈 수 있으니 **영문으로 두세요**.
  스페이스·엔터 킬은 한/영과 상관없이 동작합니다.

### 화면
- 맨 아래 상태줄(1 Hz): `BLE OK | 링크 0.1s | GUIDED | 시동 ON | EKF높이 +0.48m | 거리센서 0.50m | 15.40V | 시험:호버`
  - 링크 = FC 에서 마지막으로 받은 뒤 지난 시간. EKF높이 = FC 추정(−LOCAL_POSITION_NED.z), 거리센서 = DISTANCE_SENSOR.
  - 도달 판단은 거리센서로 합니다 (EKF 높이는 프롭 바람·기압 드리프트로 틀어질 수 있음 — 2026-11-19/20 실측, `HANDOFF_KO.md` 6장).
- **STATUSTEXT 와 COMMAND_ACK 는 전부** 시간과 함께 출력합니다 (PreArm 실패 이유 확인용).
- 화면에 나오는 모든 내용은 `tools/logs/n6_fly_YYYYmmdd_HHMMSS.log` 에도 날짜·시간과 함께 저장됩니다.

### 연결하면 자동으로 하는 일
1. FC HEARTBEAT 를 한 번 들은 뒤,
2. `REQUEST_DATA_STREAM(ALL, 0, stop)` 으로 기본 스트림 끄기 — **반드시 먼저**. ArduPilot 은 스트림 속도를 바꾸면
   그 스트림에 속한 메시지 간격을 덮어쓰기 때문에, 순서가 바뀌면 아래 설정이 지워집니다.
3. `SET_MESSAGE_INTERVAL`: LOCAL_POSITION_NED 5 Hz, DISTANCE_SENSOR 5 Hz, SYS_STATUS 1 Hz (ACK 확인, 최대 3번)
4. `SET_GPS_GLOBAL_ORIGIN` (37.5665, 126.9780, 0 m) — GPS 없이 GUIDED 이륙에 필요. 이미 있으면 FC 가 무시.
5. **페일세이프 파라미터 읽기**: `MAV_GCS_SYSID`(또는 `SYSID_MYGCS`)가 250 이 아니거나 `FS_GCS_ENABLE` 이 0 이면
   경고하고 `t` 를 막습니다 (그 상태에선 링크가 끊겨도 FC 가 착륙하지 않기 때문).

### 안전 동작 정리
| 상황 | 스크립트 동작 |
|---|---|
| 시험 중 모드가 GUIDED 가 아니게 바뀜 (페일세이프 LAND, 조종기 등) | 즉시 시험 중단, **아무 명령도 안 보냄. 절대 GUIDED 로 되돌리지 않음** |
| 시험 중 시동이 꺼짐 | 즉시 시험 중단 |
| BLE 끊김 | "BLE 끊김 — FC가 3초 뒤 스스로 착륙" 표시, 시험 취소, 자동 재연결. 재연결돼도 시험은 다시 시작 안 함 |
| BLE 가 3 s 안에 다시 붙었는데 기체가 아직 GUIDED+시동 (페일세이프가 안 걸린 경우) | 그대로 두면 명령 없이 떠 있으므로 **LAND 를 보냄** |
| BLE 는 붙어 있는데 FC 가 3 s 이상 조용함 (NUCLEO 리셋 등) | 시험 중단. FC 쪽은 하트비트를 못 받으니 스스로 LAND |
| 우리 쪽 실패 (ACK 거부·시간 초과) 후 시동+GUIDED 상태 | LAND 를 보냄 |
| `t` 를 누를 수 없는 조건 | `--no-arm`, BLE 미연결, FC HEARTBEAT 없음, 노트북→FC 방향 미확인(ACK 못 받음), 이미 시동, 페일세이프 파라미터 문제 |

참고: GUIDED·시동·LAND 는 **FC 의 COMMAND_ACK(ACCEPTED)** 로 확인합니다 (ArduPilot 은 실제로 바뀐 뒤에만 ACCEPTED 를 보냄). ACK 를 못 받으면 HEARTBEAT 로 최대 5 s 기다립니다. FC→노트북 방향은 XIAO 구간에서 HEARTBEAT 가 많이 빠지므로(알려진 문제, 9 장) 상태줄의 모드·시동 표시는 몇 초 늦을 수 있습니다.

---

## 7. ArduPilot 파라미터 (확인·설정용)

| 구분 | 값 |
|---|---|
| NUCLEO 포트 | `SERIAL4_PROTOCOL` 2, `SERIAL4_BAUD` 921, `OSD_TYPE2` 0 |
| 지상국 번호 | `MAV_GCS_SYSID` 250 (4.6 이하는 `SYSID_MYGCS`) |
| 링크 끊김 착륙 | `FS_GCS_ENABLE` 5, `FS_GCS_TIMEOUT` 3, `FS_OPTIONS` 0 |
| RC 없이 | `FS_THR_ENABLE` 0, `ARMING_SKIPCHK` 비트 6(RC) 켜기 (구버전은 `ARMING_CHECK` 에서 RC 해제) |
| 기체 | `FRAME_CLASS` 1, `FRAME_TYPE` 12, `MOT_PWM_TYPE` 5, `SERVO_BLH_RVMASK` 기체별 |
| MTF-01P | `SERIAL1_PROTOCOL` 1, `SERIAL1_BAUD` 115, `MAV2_OPTIONS` 2, `FLOW_TYPE` 5, `RNGFND1_TYPE` 10, `RNGFND1_ORIENT` 25, `RNGFND1_MIN` 0.01, `RNGFND1_MAX` 8 |
| EKF(광류) | `EK3_SRC1_POSXY` 0, `EK3_SRC1_VELXY` 5, `EK3_SRC1_POSZ` 1, `EK3_SRC1_VELZ` 0, `EK3_SRC_OPTIONS` 0 |
| 높이(거리센서) | `EK3_RNG_USE_HGT` 70, `EK3_RNG_USE_SPD` 2 (기본), `RNGFND1_GNDCLR` 0.05 m (4.7 이름; 옛 `RNGFND1_GNDCLEAR` cm). **검토 중: `EK3_SRC1_POSZ` 2** (`HANDOFF_KO.md` 6장) |
| 나침반 | `COMPASS_ENABLE` 0 (보드에 없음, EKF3 는 광류 사용 시 방위 정렬 불필요) |

메모
- `MAV2_OPTIONS` 2 = "MAVLink 전달(forward) 안 함". MTF-01P 의 고속 OPTICAL_FLOW/DISTANCE_SENSOR 가 SERIAL4(→BLE 115200)로
  전달돼 링크가 넘치는 것을 막습니다. `MAVn` 번호는 MAVLink 포트 순서(보통 SERIAL0=USB 가 MAV1, SERIAL1 이 MAV2)라서
  SERIAL 구성이 바뀌면 번호도 바뀝니다.
- QGC(USB, sysid 255)가 같이 붙어 있어도 `MAV_GCS_SYSID` 가 250 이라 QGC 하트비트는 페일세이프를 막지 못합니다 → 이 스크립트의 링크만 감시됨.

---

## 8. 시험 체크리스트 — **모두 프롭 없이**

시동은 스크립트 `t` → `y`(프롭 없이 이륙 시퀀스가 돌아감) 또는 QGC 의 Arm 으로 겁니다.
ArduPilot 은 바닥에서 시동만 걸어 두면 `DISARM_DELAY`(기본 10 s) 뒤 스스로 끄니 그 안에 시험하세요.

- [ ] **0. 브리지 단독**: 플래시 부팅 후 NUCLEO LED1 1 Hz 짧은 깜빡임, LED2 꺼짐. 스크립트가 연결돼 하트비트를 보내기 시작하면 LED1 이 0.5 s 켜짐/꺼짐으로 바뀜
- [ ] **1. 브리지 확인**: `python n6_fly.py --no-arm`
  - [ ] "FC HEARTBEAT 수신", "노트북→FC 방향 확인됨", "페일세이프 파라미터 OK" 가 보임
  - [ ] 상태줄의 링크 나이가 1 s 안쪽, 모드·배터리 전압·높이가 표시됨
- [ ] **2. 안전장치** (시동 후 각각 따로, 약 3 s 뒤 QGC 에 "GCS Failsafe" → 바닥이라 바로 시동 꺼짐)
  - [ ] ① 노트북 블루투스 끄기
  - [ ] ② XIAO 커넥터 빼기
  - [ ] ③ NUCLEO 리셋 버튼
  - [ ] ④ 스크립트 종료 (`q`/Ctrl-C — 시동 중이면 LAND 를 먼저 보내므로 하트비트 정지가 아니라 LAND 로 꺼질 수 있음. 순수 하트비트 정지 시험은 터미널 창을 강제로 닫아서)
- [ ] **3. 킬 키**: 시동 상태에서 스페이스 → 화면에 "킬! LAND 3회" → 모드 LAND → 시동 꺼짐
- [ ] **4. 그다음에야** 프롭 달고 **안전줄 묶고** 첫 호버(`python n6_fly.py`, `t` → `y`, 0.8 m)

---

## 9. 문제 해결

| 증상 | 확인할 것 |
|---|---|
| `BLE 스캔 오류 ... org.bluez ... timed out` | PC 에 블루투스 어댑터가 없음 (`ls /sys/class/bluetooth` 가 비어 있으면 bluetoothd 가 아예 안 뜸). USB BLE 동글을 꽂거나 BIOS 에서 내장 BT 를 켠 뒤 `sudo systemctl start bluetooth` |
| 스캔에 장치가 0개, 커널 로그(`journalctl -k`)에 `hci0: command 0x2005/0x2041 tx timeout` | USB 동글(Realtek RTL8761B 등)의 USB 자동 절전 문제. `echo on \| sudo tee /sys/bus/usb/devices/<포트>/power/control` 후 `bluetoothctl power off; bluetoothctl power on`. 영구 적용: `/etc/modprobe.d/btusb.conf` 에 `options btusb enable_autosuspend=n`. 메인보드 뒷면 포트에 직접 꽂기 |
| `'N6-DRONE-1' 찾는 중...` 이 계속 | XIAO 전원·광고 이름, 노트북 블루투스, 다른 프로그램(휴대폰 등)이 이미 연결 중인지 (연결돼 있으면 XIAO 가 광고를 멈춤. **XIAO 파란 LED 가 켜져 있으면** 무언가 연결된 것) |
| BLE 는 연결됐는데 "FC 응답 대기 중..." | NUCLEO LED1/LED2, UART7 TX/RX 교차(PH4→FC RX4, PE7←FC TX4), `SERIAL4_PROTOCOL 2`/`BAUD 921`, FC 전원 |
| ACK 없음 + FC 의 `@SYS/uarts.txt` 에서 SERIAL4(UART4) **RX=0, FE/OE/NE=0** (2026-11-19 실제 사례) | FC RX4 핀에 신호가 아예 안 들어감 = 배선/패드 문제. NUCLEO 는 PH4 로 정상 송신 중이었음(디버거로 3,682 B 확인). **DAKEFPV H743 은 HD VTX 커넥터의 RX4 핀이 점퍼 패드 설정에 따라 아날로그 VTX 영상 신호로 바뀜**(hwdef README: "The HD VTX connector can have RX4 replaced by the analog VTX signal") → 패드가 RX4 쪽인지 확인, 또는 PH4 를 FC 의 R4 패드에 직접 연결. **해결: 윗면 "Dji" 점퍼 패드(가운데 ↔ Dji)가 열려 있었음 → 납땜 후 정상** |
| BLE 가 자꾸 끊김 / 연결이 불안정 | 드론 배터리를 연결했는지. 캐리어 USB 5V 만으로는 XIAO 가 불안정했음 |
| 바닥인데 EKF높이가 거리센서와 0.1 m 넘게 다름 (특히 모터를 돌린 뒤) | 기압 드리프트가 EKF 에 남은 것. **바닥에서 배터리를 뺐다 꽂아 재부팅** (파라미터 변경만으로는 안 풀림) |
| 킬 후 바닥에 내려앉았는데 LAND 상태로 시동이 안 꺼짐 (2026-11-20 11:04 실제 사례, 32 s) | FC 가 아직 공중이라고 판단 (EKF 높이 오류). 바닥이 확실하면 `x x` 로 강제 정지 |
| FC HEARTBEAT 는 오는데 ACK 가 안 옴 | 노트북→FC 방향만 끊긴 것. 경로 **XIAO D6 → PF6(USART2_RX) → PH4(UART7_TX) → FC RX4(J2 2번)** 중 어디인지. 디버거로 `g_stat_ble.rx_bytes`(XIAO 에서 받음), `g_stat_fc.tx_bytes`(FC 로 보냄)가 느는지 보면 구간이 나뉨 |
| `t` 후 "GUIDED 거부" | 위치 추정 없음 → 광류/거리센서/EKF 원점 확인 (FC STATUSTEXT 에 이유가 나옴) |
| `t` 후 "시동 거부" | 바로 위의 `PreArm:` / `Arm:` 메시지가 이유 |
| 상태줄 "높이 -" | LOCAL_POSITION_NED 는 EKF 원점이 있어야 나옴. DISTANCE_SENSOR 는 `RNGFND1_ORIENT 25` 인지 |
| FC→노트북 프레임이 깨짐/빠짐 (2026-11-19 측정: 처음엔 약 5 %, 스트림 설정 후에는 1 s 마다 몰려 오는 HEARTBEAT/SYS_STATUS 의 약 70 %) | **XIAO 쪽 알려진 문제.** XIAO 펌웨어가 UART 로 받은 바이트를 1~4 B 짜리 BLE 알림으로 바로바로 보내다가, 알림 큐(3개)가 차면 그 바이트를 버림 (`bleuart.write` 반환값 미확인). 깨진 지점이 항상 "1 B 알림 3개 뒤 몇 바이트 빠짐". 노트북→FC 방향(하트비트·킬)과 FC 페일세이프에는 영향 없음. 스크립트는 모드·시동을 COMMAND_ACK 로 확인해서 우회. 고친 펌웨어: `tools/xiao/n6_ble_relay/n6_ble_relay.ino` (v2, 모아서 보내기, **아직 업로드 안 함** — `QUICKSTART_KO.md` 6장) |
| 디버거에서 `rb_fc2ble.dropped` 가 계속 증가 | FC 가 SERIAL4 로 너무 많이 보냄 → 스트림 끄기가 먹었는지, `MAV2_OPTIONS` 전달 금지 확인 |
