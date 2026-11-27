# 3000 태그 펌웨어 v2 (3대1 DS-TWR, 측정 기록 추가)

2026-11-23. NUCLEO-N657X0-Q + DWM3000EVB 태그용입니다. 학생 태그(`3000태그_펌웨어_3to1_20261123.zip`, `drone-hw-bench` `task6-qorvo-3to1-20261123` 커밋 b934e69의 `tag.c`)에 다음 두 측정에 필요한 기록을 더했습니다.

| 측정 | 방법 | 비교 대상 |
|---|---|---|
| 앵커별 성공률과 연속 실패 | 앵커는 Poll을 보낼 때마다 회차를 1씩 올립니다. 그래서 회차 번호로 앵커가 실제로 보낸 Poll 수를 알 수 있고, 이것을 태그의 거리유효=1 수와 맞춰 봅니다. 몇 번 연속 실패했는지도 셉니다. | 시뮬레이터의 앵커 손실 가정(지금 1%, 0·1·5·30%를 훑음) |
| 묶음에 걸린 시간 | A1 Poll 시작부터 A3 교환 끝(Final 수신 또는 시간 초과)까지를 **태그 DW3000 시계 하나**로 잽니다. USB 로그는 거리와 성공 여부에만 씁니다. | 20 ms 프레임 안의 위치 측정 예산(약 2~2.5 ms)에 9패킷이 들어가는지 |

분석은 PC의 `../tools/twr3_measure.py`가 합니다(아래 4장).

> **2026-11-24: 7패킷 태그 `tag7.c` 를 더했습니다.** 이 문서는 9패킷 태그(v2, `tag.c`) 설명이고 v2 는 바꾸지 않았습니다.
> 7패킷 태그의 빌드, 올리기, 로그 형식, 측정 순서는 **`../README_7PACKET_KO.md`** 에 있습니다.
> - `frames.h` 는 뒤쪽에 7패킷 부분을 더했습니다(앞쪽 9패킷 부분은 그대로). `prepare_tag.py` 는 `--packets 7 --opt O2` 등을 더했고, 옵션 없이 쓰면 전과 같습니다.
> - `../tools/load_tag.py <ELF>` 로 올리면 시작 주소를 ELF 에서 읽어 줍니다. **v2 는 PC `0x34181B60`, 7패킷 ELF 는 `0x341816F4` 로 서로 다릅니다.**
> - 11-23 밤 실제 측정에서 v2 는 20 ms 칸에서 `B` 없이 돌았고 준비 시간은 평균 234 µs, 최대 256 µs 였습니다.

## 1. v1과 다른 점

- **같은 것**
  - 무선 설정(채널 5, 프리앰블 128, 코드 9, 6.8 Mb/s, STS 끔)
  - 패킷(`frames.h`, `radio.h`는 바꾸지 않음)
  - 응답 지연 900 µs, Final 대기 500/220 µs
  - `TWR3` 줄 14칸. 학생 `capture.py`도 그대로 읽습니다.
- **더한 것 1: `TWR3T` 줄.** 각 `TWR3` 줄 바로 뒤에 나옵니다.
  ```
  TWR3T,<n>,<앵커>,<회차>,<Poll 시각>,<준비 µs>,<끝 ns>,<끝 종류>,<수신오류 누계>,<기타프레임 누계>,<늦은Poll 누계>
  ```
  | 칸 | 뜻 |
  |---|---|
  | n | 태그가 받은 Poll 일련번호(1부터). 건너뛰면 USB 로그 줄이 빠진 것 |
  | Poll 시각 | Poll 타임스탬프(RMARKER)의 DW 시각 상위 32비트, hex. 1 = 4.006 ns, 17.2 s마다 한 바퀴 |
  | 준비 µs | Poll 타임스탬프부터 펌웨어가 Poll을 알아챌 때까지. 응답 지연 900 µs 중 얼마를 쓰는지 봅니다 |
  | 끝 ns | Poll 타임스탬프부터 교환 끝까지. F는 Final 타임스탬프, 나머지는 시간 초과나 오류를 알아챈 DW 시스템 시각 |
  | 끝 종류 | `F` Final 받음, `T` Final 시간 초과, `E` 수신 오류, `X` 다른 프레임, `N` 10 ms 동안 사건 없음, `L` Response 송신 실패(늦음), `B` 출력 중이라 늦게 알아챈 Poll(응답 안 함) |
- **더한 것 2: 출력을 모았다가 조용할 때 내보냄.**
  - UART가 115200 bps이고 한 글자씩 기다리며 보냅니다. 두 줄(약 137자)을 보내는 데 약 12 ms가 걸립니다.
  - 그래서 교환 직후 바로 출력하지 않습니다. 무선이 2~3 ms 동안 조용해지면 모아 둔 줄을 한꺼번에 보냅니다.
- **더한 것 3: 출력하는 동안 들어온 Poll은 `B`로 남김.**
  - 출력하는 동안에도 수신은 켜 둡니다. 이때 들어온 Poll은 응답할 시각(Poll + 900 µs)이 이미 지났을 수 있습니다.
  - 그런 Poll에는 응답하지 않고 `B`로 기록합니다. 측정 장치 때문에 생긴 손실을 무선 손실과 나눠 세기 위해서입니다.
- 시작할 때 `TWR3_READY` 다음에 `TWR3_INFO,fw=tag3to1-v2,...` 줄이 하나 더 나옵니다.

**칸 간격 주의.**
- 한 교환은 약 1.7 ms 걸립니다. 그 뒤 2~3 ms 기다리고, 출력에 약 12 ms가 걸립니다. 그래서 앵커 칸 간격이 **17 ms 이상**이어야 다음 Poll과 겹치지 않습니다.
- 지금 앵커 기본값 20 ms는 괜찮습니다.
- 앵커를 `--slot-us` 5000~17000으로 빌드하면 `B`와 Poll 손실이 늘어납니다. 이것은 무선 탓이 아니라 측정 장치 탓입니다.

## 2. 빌드

1. 팀 `n6_drone_handoff_20261120.zip`을 **새 폴더**에 풉니다.
2. 이 폴더의 설치 스크립트를 실행합니다. 학생 원본과 같은 스크립트이고, 이 폴더의 `tag.c`, `frames.h`, `radio.h`를 넣습니다.
   ```
   python prepare_tag.py --fsbl-copy <푼 폴더>\stm32_projects\FSBL
   ```
3. STM32CubeIDE에서 `stm32_projects_FSBL` 프로젝트를 열고 **Debug** 구성으로 빌드합니다.

개발 PC(Linux)의 결과(2026-11-23, CubeIDE 2.2.0, GNU Tools 14.3.1):

| 항목 | 값 |
|---|---|
| 빌드 | 오류 0, 경고 0 |
| ELF | `build/stm32_projects_FSBL_tag3to1_v2.elf` (3,278,064 B) |
| SHA-256 | `8d6e09d84c2130bf3adc96d69cc82433f49bd664d93178dbe9915595312362d2` |
| 시작 값 | **MSP `0x34200000`, PC `0x34181B60`** (v1과 같음) |
| 빌드 절차 확인 | 같은 절차로 v1(`tag_v1_original.c`)을 빌드하면, 올라가는 이미지가 학생 ELF와 **바이트 단위로 같았습니다**(ELF 파일 해시는 경로 문자열 때문에 다름) |

## 3. RAM 실행 (외부 플래시는 건드리지 않음)

보드 설정은 v1과 같습니다.
- NUCLEO: BOOT0 1–2, **BOOT1 2–3(개발 모드)**, CN9 1–2, PC에는 **CN10만** 연결
- EVB: J1 1–2

**RAM에 있으므로 USB를 뽑거나 리셋하면 지워집니다. 그때마다 다시 올립니다.** 시작 줄을 놓치지 않으려면 4장의 기록을 먼저 켜 두고 올립니다. 올리기는 SWD로 하므로 기록 중인 COM 포트와 충돌하지 않습니다.

```powershell
# STM32CubeProgrammer 또는 STM32CubeIDE 안의 CLI
#   C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe
#   C:\ST\STM32CubeIDE_*\STM32CubeIDE\plugins\*cubeprogrammer.win32*\tools\bin\STM32_Programmer_CLI.exe
$cli = '<STM32_Programmer_CLI.exe 경로>'
$elf = '<이 폴더>\build\stm32_projects_FSBL_tag3to1_v2.elf'
& $cli -l stlink        # ST-LINK SN 확인
if ((Get-FileHash -LiteralPath $elf -Algorithm SHA256).Hash -ne '8D6E09D84C2130BF3ADC96D69CC82433F49BD664D93178DBE9915595312362D2') { throw 'ELF가 다릅니다' }
& $cli -c port=SWD "sn=<태그 ST-LINK SN>" mode=HOTPLUG freq=1000 -halt -d $elf -v -halt -coreReg MSP=0x34200000 PC=0x34181B60 XPSR=0x01000000 -run
```

- 끝에 `Download verified successfully`와 `Core run`이 나오면 됩니다. 기록 창에는 `TWR3_READY`와 `TWR3_INFO,fw=tag3to1-v2,...`가 나옵니다.
- 태그 보드 ST-LINK SN:
  - 2026-11-23에 v2를 확인한 보드: `002300233335511035383531`
  - 11/22 1대1 시험 보드: `002A00483335511035383531`
- 개발 PC(Linux)의 CLI는 `/opt/st/stm32cubeide_2.2.0/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.linux64_*/tools/bin/STM32_Programmer_CLI`입니다.

## 4. 측정

앵커 세 대는 보조배터리로 켜고, 태그만 PC에 연결합니다.

```
python ..\tools\twr3_measure.py --tag-port COM5 --seconds 120 --out ..\results\P1 --point "P1 삼각형 안"
```

- COM 번호는 장치 관리자의 "STMicroelectronics STLink Virtual COM Port (COMx)"나 `python -m serial.tools.list_ports`로 확인합니다.
- 5초마다 앵커별 `성공/받은 Poll` 수가 찍힙니다. 끝나면 결과가 화면과 `--out` 폴더에 나옵니다. `--out`은 매번 새 폴더를 줍니다.
- 앵커 한 대를 J9 USB로 PC에 같이 꽂고 `--anchor-port COM7`을 더할 수 있습니다. 그러면 그 앵커의 **Response 손실과 Final 손실을 따로** 셉니다. 태그 혼자서는 둘을 구분할 수 없습니다.
- pyserial이 필요합니다: `python -m pip install pyserial`
- 저장한 로그는 다시 분석할 수 있습니다: `--tag-log ..\results\P1\tag.raw.log`
- 결과 폴더에 생기는 파일:
  | 파일 | 내용 |
  |---|---|
  | `summary.txt`, `summary.json` | 요약 |
  | `exchanges.csv` | 앵커 회차마다 한 줄. 빠진 회차도 들어 있음 |
  | `packets.csv` | 팀 문서 형식: 시각, 앵커ID, 패킷 종류, 성공 여부, 신호 세기, 첫 경로 세기, 거리값 |
  | `bundles.csv` | 묶음마다 한 줄 |

**결과 읽는 법**

1. **앵커별 성공률과 연속 실패**
   - 표에 나오는 값: 앵커마다 보낸 Poll 수, 태그가 받은 수, 성공(거리유효=1) 수, Poll 손실, Response/Final 손실, `B` 수, 최대 연속 실패, 연속 실패 길이 분포
   - "독립이면 기대 최대": 실패가 서로 독립이라고 가정할 때 같은 손실률에서 기대되는 최대 연속 실패와, 관측한 만큼 길게 이어질 확률입니다. 확률이 아주 작으면 손실이 몰려서 난 것입니다.
   - 위치 성공률: 세 거리가 모두 유효한 묶음의 비율입니다.
   - 메시지당 손실 환산 q = 1 − 성공률^(1/9). 시뮬레이터 가정(1%)과 비교합니다.
   - 마지막에 팀 문서 형식의 한 줄 요약이 나옵니다.
2. **묶음에 걸린 시간**
   - 묶음 시간: A1 Poll 시작부터 A3 끝까지
   - 교환 시간: Poll 타임스탬프부터 Final 타임스탬프까지
   - Poll 간격(A1→A2, A2→A3), 태그 준비 시간
   - 판정: 예산(`--budget-ms`, 기본 2.5 ms)에 들어가는지
   - 프리앰블·SFD 138.4 µs와 Final 뒷부분 60.5 µs는 데이터시트 식으로 **계산한 값**입니다. 나머지는 모두 태그 무선 시계로 잰 값입니다.

**미리 알아 둘 것**
- 지금 앵커는 칸 20 ms로 돕니다. 그래서 묶음 시간은 약 2 × 20 ms + 교환 1.8 ms ≈ 42 ms가 나옵니다. 이것은 앵커 설정값입니다.
- 9패킷이 예산에 들어가는지는 "세 교환을 간격 0으로 붙인 하한(교환 시간의 합)"으로 판단합니다. 응답 지연 900/700 µs면 교환 한 번이 약 1.8 ms이므로 하한은 약 5.4 ms로 예상합니다.
- 교환 사이 실제 간격까지 포함한 값을 재려면, 앵커가 짧은 간격으로 연달아 묻게 바꿔야 합니다. 이때 바퀴 길이는 따로 두어야 합니다. 앵커 펌웨어를 고치고 다시 올려야 합니다.

## 5. 확인한 것과 안 한 것 (2026-11-23)
- ✅ v2 빌드 오류 0·경고 0. 시작 값이 v1과 같음. v1 빌드는 학생 이미지와 바이트 단위로 같음
- ✅ 분석 스크립트 오프라인 시험 `python ../tools/test_twr3_measure.py` 40/40 통과
  - 가짜 로그에 넣은 것: Poll 손실, 시간 초과, `B`, 연속 실패, DW 시계 한 바퀴 넘김, 앵커 재시작, 40 s 공백, USB 줄 빠짐, v1 로그, 앵커 로그
  - 가상 시리얼로 실시간 기록도 확인
- ✅ 보드(ST-LINK SN `002300233335511035383531`)에서 RAM 실행
  - 올리기와 비교 확인 성공
  - `TWR3_READY`(DWM3000 인식·설정 완료)와 `TWR3_INFO,fw=tag3to1-v2` 출력 확인
- ❌ **앵커와 함께 측정은 아직 안 함.** 실제 `준비 µs` 값과, 출력 타이밍이 20 ms 칸에서 `B` 없이 도는지는 앵커를 켜고 확인해야 합니다.
- ❌ `twr3_measure.py`를 Windows에서 실행해 본 적은 없습니다(표준 라이브러리 + pyserial만 씀).

## 6. 파일
| 파일 | 내용 |
|---|---|
| `tag.c` | v2 응답기(9패킷). 설치하면 `ds_twr_responder.c` 자리에 들어감 |
| `tag7.c` | 7패킷 응답기. `prepare_tag.py --packets 7 --opt O2` 로 설치 |
| `tag_v1_original.c` | 학생 원본(b934e69), 비교용 |
| `radio.h` | 학생 원본 그대로 |
| `frames.h` | 학생 원본 + 뒤쪽에 7패킷 부분 (앵커의 `src/frames.h` 와 같은 파일) |
| `prepare_tag.py` | 학생 원본에 `--packets`, `--opt`, `--slot-us`, `--resp-delay-us`, `--final-delay-us`, `--sweep` 를 더함 |
| `build/` | v2 ELF와 map, 7패킷 ELF 3개(-O2) |
