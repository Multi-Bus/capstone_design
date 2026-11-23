# DWM3001CDK 앵커 펌웨어 (3대1 DS-TWR, 보조배터리로 혼자 동작)

2026-11-23. 드론 태그(NUCLEO-N657 + DWM3000EVB) 1대와 DWM3001CDK 앵커 3대로 거리 3개를 재는 펌웨어입니다.
**앵커는 PC 없이 보조배터리만으로 돕니다.** 플래시에 올라가 있어서 전원만 넣으면 시작합니다.

## 1. 태그와의 관계

| 항목 | 값 |
|---|---|
| 짝이 되는 태그 | `3000태그_펌웨어_3to1_20261123.zip` (= drone-hw-bench `task6-qorvo-3to1-20261123` 브랜치 `qorvo_3to1/tag.c`) |
| 순서 | 앵커가 **Poll** → 태그가 **Response** → 앵커가 **Final** → **태그가 거리 계산·출력** (`TWR3,...` 줄) |
| 주소 | 앵커 1·2·3, 태그 `0x0100`, PAN `0xDECA` |
| 무선 | 채널 5, 프리앰블 128, PAC 8, 코드 9, SFD type 1, 6.8 Mb/s, STS 끔 (핸드오프 `config_options.c` 옵션 35 = 태그와 같음) |
| 타이밍 | 태그 응답 900 µs, 앵커 Final 700 µs, 대기 700/300 µs (Qorvo 예제 기본값, 태그와 같음) |
| 패킷 코드 | `src/frames.h`, `src/radio.h` = 태그 압축본과 내용이 같음(줄바꿈만 다름) |

### 학생 코드(drone-hw-bench)와 다른 점
- 측정 한 번(`range()`), 패킷, DWM3001 SPI 포트, 송신 전력 변환은 **학생 코드를 그대로** 옮겼습니다(`task6-qorvo-3to1-20261123`, 커밋 b934e69).
- 학생 버전은 **PC가 J-Link(SWD)로 A1 → A2 → A3를 한 번씩 호출**하는 방식이라 세 앵커가 모두 PC에 USB로 연결돼 있어야 하고, RAM에서 돕니다.
- 이 버전은 **앵커끼리 서로의 Poll을 듣고 차례를 맞추고**, **플래시**에서 돕니다. 그래서 학생 `capture.py`(SWD 메일박스 사용)는 이 앵커와 같이 쓸 수 없습니다. 거리는 태그 VCP의 `TWR3` 줄로 봅니다.

## 2. 차례 맞추기
- 칸 하나 20 ms, 한 바퀴 60 ms. A1 → A2 → A3가 20 ms 간격으로 돌아갑니다. **앵커당 초당 약 16.7회**.
- 앵커 j의 Poll이 들리면, 내 다음 Poll = 그 Poll 시각 + ((내 번호 − j) mod 3) × 20 ms.
- 다른 앵커가 안 들리면 혼자 60 ms마다 측정하고, 시작 시각에 0~10 ms 난수를 더해 다른 앵커와 계속 겹치지 않게 합니다.
- 20 ms는 태그가 측정마다 거리 한 줄(약 90자, 115200 bps에서 약 8 ms)을 출력하는 시간을 감안한 값입니다.
- **세 앵커는 같은 `--slot-us`로 빌드해야 합니다.** 번호가 겹치면 `A?_STAT`의 `dup_id`가 올라갑니다.
- 시뮬레이터 `python tools/sched_sim.py`: 시계 오차 ±20 ppm, 켜는 시각 무작위(동시 포함) 800가지 경우에서 안정된 뒤 충돌 0회.

## 3. 보드 표시 (꼭 붙여 두세요)

| 앵커 | J-Link 일련번호 (J9) | nRF FICR | 상태 |
|---|---|---|---|
| **A1** | **000760226397** | 9958F88F617E722B | **2026-11-23 올림, 단독 동작 확인** |
| **A2** | **000760226502** | 11877F611E189726 | **2026-11-23 올림, 단독 동작 확인** |
| A3 | | | 아직 안 올림 |

### 올린 이미지 (2026-11-23)

이 압축의 `build/A?/`에 있는 파일이 보드에 들어간 그대로입니다. 설정은 모두 기본값입니다(칸 20000 µs, 안테나 지연 16385, 전력 인덱스 27).

| 앵커 | 보드 (J-Link) | 파일 | SHA-256 (.hex) | 확인 |
|---|---|---|---|---|
| A1 | 000760226397 | `build/A1/anchor_A1.hex` | `c02cd104b2fce2e60ee8397d50b562d9011558d2f0dd592830ff0854f5dfbbb9` | 쓰기 후 J-Link 비교 성공 |
| A2 | 000760226502 | `build/A2/anchor_A2.hex` | `8a918aaeb3b1cf2377a8c7f547ac67f66659ddb8cd217738654b471044430047` | 쓰기 후 J-Link 비교 성공, 나중에 보드 플래시를 다시 읽어 `.bin`과 바이트 단위로 같음 |
| A3 | (아직) | `build/A3/anchor_A3.hex` | `ab377ade59c1d0a45128bfc00ba498e1b1cbe331301125757a1ae61046e05bdf` | 빌드만 함, 안 올림 |

빌드 환경: Linux, STM32CubeIDE 2.2.0 안의 GNU Tools for STM32 14.3.1, `python build.py --id 1 --id 2 --id 3`. 자작 소스는 `-Wall -Wextra -Werror`로 경고 0.

- 이미지 안에 Qorvo 드라이버의 파일 경로 문자열이 하나 들어 있습니다(`dw3000_device.c`의 assert). 개발 PC 경로는 `/home/tom/stm32_projects/uwb_anchor_dwm3001/...`입니다.
- 그래서 **다른 폴더나 다른 PC에서 다시 빌드하면 SHA-256이 위 값과 달라집니다.** 코드가 달라서가 아니라 이 문자열 때문입니다.
- 보드에 들어간 것과 똑같은 파일이 필요하면 `build/A?/`의 파일을 그대로 쓰세요.

## 4. 빌드와 올리기

준비물: Python 3, ARM GCC(STM32CubeIDE에 들어 있는 것을 자동으로 찾음), SEGGER J-Link 소프트웨어.

```bash
python build.py --id 1 --id 2 --id 3          # build/A1~A3/anchor_A?.hex, manifest.json(SHA-256)
python flash.py --list                         # 꽂힌 J-Link 일련번호
python flash.py --id 2 --serial 000760226502   # 앵커 2번에 올리기 (쓰고, 비교 확인, 재시작)
```

- Windows: `python` 대신 `py`를 써도 됩니다. J-Link는 `C:\Program Files\SEGGER\JLink*\JLink.exe`를 자동으로 찾습니다.
- 옵션: `--antenna-delay`(기본 16385, 보정 전), `--power-index`(기본 27 = 학생 Task 1 LISTCAL 값), `--slot-us`(기본 20000).
- 개발 PC(Linux) J-Link: `~/opt/SEGGER/JLink_Linux_V996a_x86_64/`.

## 5. 확인하는 법

**LED** (보드 인쇄 D9~D12)

| LED | 뜻 |
|---|---|
| D9 | 켜져 있으면 UWB 초기화 실패 (2 s 뒤 저절로 재시작해서 다시 시도) |
| D10 | Poll 보낼 때마다 깜빡 (켜고 끄기 반복) |
| D11 | 최근 0.5 s 안에 태그 Response를 받음 = 태그와 통신 중 |
| D12 | 다른 앵커의 Poll이 들림 = 차례 맞춤 중 |

**시리얼** (J9 USB를 PC에 연결 → J-Link VCOM, 115200 8N1)

```
A2_BOOT,fw=anchor3to1-v1,ficr=...,reset=0x00004,dev_id=0xDECA0302,session=0x...,slot_us=20000,ant_dly=16385
A2_READY,channel=5,tag=256,power_index=27,tx_power=0x6D6D6D6D,pll_bias=7
A2,<회차>,<단계>,<Response RSSI>,<첫경로>,<동기 1/0>,<Response 상태>
A2_STAT,polls=..,resp=..,finals=..,heard1=..,heard2=..,heard3=..,dup_id=..,resyncs=..,synced=..,log_drop=..
```
- 단계: 1 = Poll 보냄, 2 = Response 받음, 4 = Final 보냄. **7이면 앵커 쪽 성공.** 1만 나오면 태그가 응답하지 않는 것입니다.
- `dev_id`가 `0xDECA0302`가 아니면 DW3110 SPI 문제입니다. `reset`의 `0x00002`는 감시 타이머 재시작입니다.
- RSSI 값이 `32767`이면 읽지 못한 것입니다(태그와 같은 규칙).

**태그 쪽**: 태그 README대로 RAM 실행 후 ST-LINK VCP(115200)에서 `TWR3,<세션>,<회차>,<앵커>,...,<거리mm>,...` 줄을 봅니다. 앵커 번호가 1·2·3으로 번갈아 나와야 합니다.

## 6. 보조배터리
- 앵커는 계속 수신 상태라 대략 수십 mA를 씁니다. 전류가 적으면 몇십 초 뒤 저절로 꺼지는 보조배터리가 있으니, 켜 두고 D10이 계속 깜빡이는지 몇 분 보세요.
- J9(J-Link)과 J20(nRF USB) 중 어느 쪽으로 전원을 넣어도 됩니다. 시리얼 로그는 J9로만 나옵니다.

## 7. 확인한 것과 안 한 것 (2026-11-23)
- ✅ A1·A2 보드 각각: 플래시 쓰기·비교, 플래시 부팅, DW3110 인식, 무선 초기화, 송신 전력 값(학생 기록과 같음), Poll 초당 16회 송신, 응답 대기 시간 초과 처리. (케이블이 하나라 한 번에 한 대씩 PC에 연결해서 확인)
- ✅ 오프라인: 학생 `test_frames.c` 통과, 차례 맞추기 시뮬레이터 충돌 0.
- ❌ **태그와 실제 거리 측정은 아직 안 함** (태그 보드가 없었음). 태그 README에도 3대1 실기 시험은 아직이라고 되어 있습니다.
- ❌ **앵커 여러 대가 실제로 차례를 맞추는지** (두 대를 동시에 켜 보지 못했음). 한 대는 보조배터리, 한 대는 PC에 꽂고 PC 쪽 `A?_STAT`의 `heard*`가 올라가고 `synced=1`이 되는지, LED D12가 켜지는지 확인하세요.
- ❌ 보조배터리 장시간 동작.

## 8. 원래 펌웨어로 되돌리기
A1·A2 보드에 있던 학생 펌웨어(둘 다 `T6 Lv1 B: libdeca TWR on DWM3001CDK, responder (anchor)`)는 `backups/anchor1_*`, `backups/anchor2_*`에 512 KB 그대로 있습니다. 아래는 A2 예시이고, A1은 일련번호 000760226397과 `anchor1_flash_orig_20261123.bin`을 쓰면 됩니다.

```
JLinkExe -USB 000760226502 -device NRF52833_XXAA -if SWD -speed 4000 -autoconnect 1
J-Link> loadbin backups/anchor2_flash_orig_20261123.bin 0x0
J-Link> r
J-Link> g
```

## 9. 폴더
| 경로 | 내용 |
|---|---|
| `src/main.c` | 앵커 본체: 초기화, `range()`(학생 코드), 차례 맞추기, 로그·LED |
| `src/port.c` | DWM3001CDK 포트: SPI·리셋(학생 코드), HFXO, UART, LED, 난수, 감시 타이머 |
| `src/radio_config.c` | 무선 설정(옵션 35)과 송신 전력 변환 |
| `src/frames.h`, `src/radio.h` | 패킷 형식 (태그와 공통) |
| `third_party/` | Qorvo DW3000 드라이버 08.19.02(태그와 같음), Nordic nRF52833 MDK, CMSIS |
| `tools/sched_sim.py` | 차례 맞추기 시뮬레이터 |
| `backups/` | A2 원래 플래시·UICR |
