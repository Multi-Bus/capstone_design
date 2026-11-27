# 작업: SD 로더 + 기록 늘린 펌웨어(`tagf-v2-log`)를 시험하고 실기체에 적용

- 브랜치: `feat/sd-loader` (커밋 `f9d8a78` 로더, `8ab30f3` 기록 늘림)
- 설계: [DESIGN_KO.md](DESIGN_KO.md), 사용법: [README_KO.md](README_KO.md), 기록 줄 표: [../n6_flight_tag/README_KO.md](../n6_flight_tag/README_KO.md)
- 상태 (2026-11-28): PC 시험 통과, 6대 빌드 끝. **보드 시험 전.**

## 0. 지킬 것

- 프롭은 1~6단계 동안 뺀다. 7단계(호버)에서만 단다.
- 시동, 이륙, 모터 명령은 7단계의 `n6_fly.py` 말고는 보내지 않는다.
- 플래시 쓰기 전에 반드시 지금 플래시를 백업한다(1단계). 백업 `.bin`은 커밋하지 않는다.
- FC는 한 번에 한 대만 연결한다. 6호기 배터리는 점검 기록을 확인한 뒤 연결한다.
- 호버 목표 높이는 1.3 m.
- 한 기체에서 1~8단계를 모두 통과한 뒤에 다른 기체로 넘어간다. 실패하면 9장(되돌리기)으로 간다.

## 기체별 값

| 기체 | 태그 id | ST-LINK SN | 로더 | SD 펌웨어 |
| --- | --- | --- | --- | --- |
| 1호기 | 0 | `002300413335511135383531` | `build/loader_id0-trusted.bin` | `build/app_id0.bin` |
| 2호기 | 1 | `002A0048…` (기록 확인) | `build/loader_id1-trusted.bin` | `build/app_id1.bin` |
| 3호기 | 2 | 미확인 | `build/loader_id2-trusted.bin` | `build/app_id2.bin` |
| 4호기 | 3 | `0036004D3335510735383531` | `build/loader_id3-trusted.bin` | `build/app_id3.bin` |
| 5호기 | 4 | `001900333335511135383531` | `build/loader_id4-trusted.bin` | `build/app_id4.bin` |
| 6호기 | 5 | `004700233335511035383531` | `build/loader_id5-trusted.bin` | `build/app_id5.bin` |

- 보드가 기체 사이에서 바뀐 기록이 있다. 시작 전에 `STM32_Programmer_CLI -l st-link`로 SN을 보고 이 표와 맞는 기체인지 확인한다.
- RAM 시험용 ELF(`build/app_id<N>.elf`, `build/loader_id<N>.elf`)는 커밋하지 않는다. 없으면 README_KO.md "다시 빌드하기"로 만든다.
- 아래 명령의 `<N>`은 태그 id, `<SN>`은 ST-LINK SN, `<COM>`은 ST-LINK 가상 COM(921600)이다.

## 1. 준비와 백업

점퍼: 개발 구성 (BOOT0 1-2, BOOT1 2-3, CN9 1-2), PC를 CN10, EVB J1 1-2. 배터리 연결(SD 카드 전원), 프롭 없음.

- [ ] ST-LINK SN이 표의 기체와 같다
- [ ] 외부 플래시 1 MB 백업: `STM32_Programmer_CLI -c port=SWD sn=<SN> mode=HOTPLUG -el <MX25UM51245G_STM32N6570-NUCLEO.stldr> -r 0x70000000 0x100000 firmware\nucleo\backup_<SN 앞 8자>_<날짜>.bin`
- [ ] 백업 안의 펌웨어 이름을 적는다 (`strings`로 `TF_INFO,fw=` 또는 브리지 판 확인)
- [ ] SD 카드: FAT32, 맨 위에 `UWBLOG.BIN`이 있다 (없으면 PC에서 `python firmware\uwb\n6_flight_tag\tools\sd_prepare.py --drive E:\`)

## 2. 펌웨어를 AXISRAM1에서 직접 실행 (로더 없이)

목적: `--app-link` 펌웨어가 AXISRAM1(`0x34000400`)에서 도는지, 새 기록 줄이 나오는지.

```
python firmware\uwb\n6_flight_tag\tools\load_tag.py firmware\uwb\n6_sd_loader\build\app_id<N>.elf --sn <SN>
```

VCP(`<COM>`, 921600)를 30초 받는다. 통과 기준:

- [ ] `TF_READY`
- [ ] `TF_INFO,fw=tagf-v2-log,id=<N>,...,flight=1,...,boot=direct,boot_reason=0`
- [ ] `TF_SD,ready=1,card=1,...,file=ok`
- [ ] `E,...,flight_uart,1`, `E,...,fc_seen,...`
- [ ] `U` 줄에서 FC 수신 바이트가 늘고 오류 0
- [ ] `C` 줄 칸이 22개
- [ ] `A`(거리센서), `O`(광류), `B`(배터리) 줄이 나온다 (FC에 해당 센서가 켜져 있으면)
- [ ] 앵커가 켜져 있으면 `R` 줄 거리

실패하면: `TF_INIT_FAILED`는 EVB를 다시 꽂고 재시도. RAM 쓰기 실패나 무반응이면 AXISRAM1 접근 문제이므로 멈추고 보고.

## 3. 로더 RAM 실행: SD 파일 없음

목적: 로더가 기본 펌웨어로 넘어가는지.

- [ ] SD 카드에 `N6APP.BIN`이 없다 (있으면 PC에서 지운다)
- [ ] `python firmware\uwb\n6_flight_tag\tools\load_tag.py firmware\uwb\n6_sd_loader\build\loader_id<N>.elf --sn <SN>`
- [ ] `LDR,ver=1,id=<N>,src=builtin,reason=no_file,fw=tagf-v2-log,...,builtin=ok` 줄
- [ ] 이어서 2단계와 같은 `TF_READY`, `TF_INFO`(`boot=builtin,boot_reason=3`), `TF_SD,ready=1`
- [ ] `LDR` 줄의 `ms=` 값 기록: ______ ms (목표 1000 ms 미만)

## 4. 로더 RAM 실행: 정상 SD 파일

- [ ] PC에서: `python firmware\uwb\n6_sd_loader\tools\sd_app.py firmware\uwb\n6_sd_loader\build\app_id<N>.bin --tag-id <N> --drive E:\` 후 '안전하게 제거', 카드를 보드에 꽂는다
- [ ] 로더 RAM 실행 (3단계 명령)
- [ ] `LDR,...,src=sd,fw=tagf-v2-log,len=...,crc=0x...,ms=...` 줄. `crc`가 `sd_app.py` 출력과 같다
- [ ] `TF_INFO,...,boot=sd,boot_reason=0`, `TF_SD,ready=1`, FC 수신
- [ ] `ms=` 값 기록: ______ ms (목표 1000 ms 미만)

## 5. 로더 RAM 실행: 나쁜 파일은 쓰지 않음

각 경우마다 `--out`으로 파일을 만들어 카드에 `N6APP.BIN` 이름으로 복사한 뒤 로더 RAM 실행.

| 경우 | 만드는 방법 | 기대 |
| --- | --- | --- |
| 다른 기체 id | `sd_app.py ... --tag-id <N 이 아닌 값> --out N6APP.BIN` | `src=builtin,reason=wrong_id`, `boot_reason=8` |
| CRC 틀림 | 정상 파일의 마지막 바이트 하나를 바꿈 (PC에서) | `src=builtin,reason=bad_crc`, `boot_reason=10` |

- [ ] 두 경우 모두 기본 펌웨어로 정상 동작 (`TF_READY`, FC 수신)
- [ ] 끝나면 카드에 정상 `N6APP.BIN`(4단계)을 다시 넣는다

## 6. 외부 플래시에 로더 쓰기

- [ ] 1단계 백업이 있다
- [ ] `STM32_Programmer_CLI -c port=SWD sn=<SN> mode=HOTPLUG -el <stldr> -hardRst -w firmware\uwb\n6_sd_loader\build\loader_id<N>-trusted.bin 0x70000000 -v` 에서 `Download verified successfully`
- [ ] 다시 읽어 대조: 1 MB를 읽어 앞부분이 이미지와 바이트 단위로 같다 (읽기 크기는 `0x100000`처럼 정렬된 값)
- [ ] `build/SHA256.txt`의 해시와 쓴 파일 해시가 같다

## 7. 비행 구성 부팅과 되돌리기 확인 (프롭 없음)

점퍼: 비행 구성 (BOOT0 1-2, BOOT1 1-2, CN9 3-4). 배터리 연결.

- [ ] 전원 ON → VCP에 `LDR,...,src=sd`, `TF_INFO,...,boot=sd`
- [ ] `python gcs\tools\n6_fly.py --name N6-DRONE-<기체 번호> --no-arm`: BLE 연결, 노트북→FC(COMMAND_ACK), FC→노트북(HEARTBEAT, 상태줄) 확인
- [ ] NUCLEO RESET 버튼(B2) → `src=builtin,reason=after_reset`, `boot_reason=12`
- [ ] RESET 한 번 더 → `src=sd`
- [ ] 배터리를 뺐다 꽂음 → `src=sd`
- [ ] SD 카드를 빼고 켬 → `src=builtin,reason=no_card` 또는 `no_fat`, 그래도 `--no-arm` 양방향 정상
- [ ] 카드를 다시 꽂는다

VCP를 볼 수 없는 비행 구성이면, 위 확인은 8단계에서 SD 기록의 `TF_INFO` 줄(`boot=`, `boot_reason=`)로 대신한다.

## 8. 호버와 기록 확인

- [ ] 앵커 켜기 (프레임 방식 `build/f/` 앵커)
- [ ] 프롭 달고 주변 확인 후: `python gcs\tools\n6_fly.py --name N6-DRONE-<기체 번호> --alt 1.3 --hover 10`
- [ ] 이륙, 1.3 m 호버, 착륙, 시동 꺼짐이 정상
- [ ] 배터리 분리 후 SD 카드를 PC에: `python firmware\uwb\n6_flight_tag\tools\sd_extract.py --file E:\UWBLOG.BIN --out results\<날짜>_drone<기체 번호>`
- [ ] 꺼낸 기록에 `TF_INFO,fw=tagf-v2-log,...,boot=sd` 와 아래 줄이 있다

| 줄 | 기대 빈도 (호버 중) | 실제 |
| --- | --- | --- |
| `R` | 앵커마다 50 Hz 근처 | |
| `C` (22칸) | 50 Hz 근처 (노트북이 덮어쓰면 10초 안에 회복) | |
| `A` | 20 Hz | |
| `O` | 20 Hz (광류 센서) | |
| `B` | 2 Hz | |
| `G`, `M` | `n6_fly.py`가 보낸 만큼 (이륙·LAND 명령 등) | |

- [ ] `python firmware\uwb\n6_flight_tag\tools\frame_measure.py --tag-logs results\<...>\tag<N>_run*.raw.log`로 읽혀서 깨진 줄 수가 0 또는 거의 0
- [ ] `D` 줄의 SD 버린 글자 수(`dropped`)가 0

## 9. 되돌리기 (실패했을 때)

- 로더 쓰기 전 실패(2~5단계): 플래시를 건드리지 않았으므로 할 것 없음. 점퍼만 비행 구성으로 되돌린다.
- 로더 쓴 뒤 문제: SD 카드를 빼면 기본 펌웨어(`tagf-v2-log`)로 난다. 그래도 문제면 1단계 백업을 같은 쓰기 명령으로 `0x70000000`에 쓰고 다시 읽어 대조한다.
- 이전 판을 바로 쓰려면: `firmware\uwb\n6_flight_tag\build\n6_flight_tagf_v2lfs_id<N>-trusted.bin` (로더 없는 `tagf-v2-lfs`).

## 10. 기록 남기기 (기체마다)

- [ ] `VERSIONS.md` 기체 요약 줄과 NUCLEO 표: 로더 파일, 해시, SN, 날짜, 카드의 `N6APP.BIN` 펌웨어와 CRC
- [ ] 이 파일 아래 표에 결과 한 줄

| 기체 | 날짜 | 2 | 3 (ms) | 4 (ms) | 5 | 6 | 7 | 8 | 메모 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| | | | | | | | | | |

## 11. 다 끝나면

- [ ] 한 기체 1~8단계 통과 → 나머지 기체 차례로 (FC 한 대씩)
- [ ] `feat/sd-loader` 푸시와 PR (보드 결과를 PR 본문에)
- [ ] 동시 호버(`n6_fly_multi.py`)로 6대 기록을 프레임 번호로 맞춰 보기
