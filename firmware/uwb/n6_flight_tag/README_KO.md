# n6_flight_tag: BLE 중계 + UWB 태그 + SD 기록 + 링크 끊김 강제 DISARM

NUCLEO-N657X0-Q(+ DWM3000EVB)에 올리는 비행용 펌웨어다. `n6_bridge`를 대신한다. 한 펌웨어로 `n6_fly.py` 호버(BLE 중계)와 UWB 측정, SD 카드 기록을 같이 한다.

## 출처와 바꾼 것

- 바탕: 팀 Dropbox `STM32_DWM3000_태그_펌웨어_실측실험_20261124.zip`의 `tag/`(tagf v2, 비행용). 설명은 `README_FLIGHT_KO.md`, `README_FRAME_KO.md`(원본 그대로).
- 바꾼 것: `n6_bridge`의 링크 끊김 강제 DISARM(`firmware/nucleo/n6_bridge/FSBL/Inc/link_failsafe.h`)을 `tag/fcl.c`에 붙였다. 조건은 브리지와 같다. 노트북 HEARTBEAT(sysid 250)가 5초 넘게 없고 FC가 LAND + 시동 ON을 9초 이어 가면 강제 DISARM(`COMMAND_LONG` 400, param2 21196)을 1초마다 보낸다. 보내면 SD에 `E,<프레임>,linkfs_cut,<횟수>,<모드>` 줄이 남는다. GUIDED 등 다른 모드에서는 아무것도 안 한다.
- 시작 줄의 펌웨어 이름은 `fw=tagf-v2-lfs`다.
- 팀원이 2026-11-26 실측에 쓴 `tagf-v3.1`의 소스는 없어서 v2를 바탕으로 했다. v2와 v3.1의 차이는 모른다.

## 확인한 것과 안 한 것

- PC: `python tag/test_fcl.py` 전부 통과(원본 시험 + 링크 끊김 시험 6개).
- 빌드: STM32CubeIDE 2.3.0 headless, Debug 구성 `-O2`, 6개 모두 오류 0, 경고 0. 코드 약 111 kB.
- 서명: `STM32_SigningTool_CLI -nk -of 0x80000000 -t fsbl -hv 2.3 -align`. 페이로드가 0x400에 있는 것을 확인했다.
- 보드 (4호기, id3, 2026-11-27): RAM 실행에서 `TF_INFO,fw=tagf-v2-lfs`, 앵커 3대 거리(`R` 줄), SD 카드 응답(`card=1`, 캐리어 전원을 넣어야 함), FC 하트비트를 확인했다. 외부 플래시에 쓴 뒤 비행 부팅, `n6_fly.py --no-arm` 양방향, 1.3 m 호버를 확인했다.
- 아직 확인하지 않은 것: SD 기록 파일(`UWBLOG.BIN`)을 만든 뒤의 비행 기록, 링크 끊김 강제 DISARM의 실제 동작(시동이 필요).
- 이 펌웨어는 LED1을 1초마다 깜빡이지 않는다. 파란 LED는 앵커 1의 프레임을 받을 때만 5초마다 0.1초 켜진다.

## 파일

`build/n6_flight_tagf_v2lfs_id<N>-trusted.bin`(외부 플래시용), `build/n6_flight_tagf_v2lfs_id<N>.elf`(RAM 시험용, 커밋하지 않음). N은 태그 번호 0~5이고 기체마다 달라야 한다. SHA-256은 `build/SHA256.txt`.

## 다시 빌드하기

1. 팀 `n6_drone_handoff_20261120.zip`의 `stm32_projects`를 태그 번호마다 새 폴더에 푼다.
2. ST HAL v1.4.0의 SD 드라이버 8개(`stm32n6xx_hal_sd*.{c,h}`, `stm32n6xx_ll_sdmmc.{c,h}`, `stm32n6xx_ll_dlyb.{c,h}`)를 `<pkg>/Drivers/STM32N6xx_HAL_Driver/{Inc,Src}`에 둔다(GitHub `STMicroelectronics/stm32n6xx-hal-driver` 태그 `v1.4.0`). 핸드오프 프로젝트의 HAL이 1.4.0이다.
3. `python tag/prepare_tag.py --fsbl-copy <폴더>\stm32_projects\FSBL --packets f --opt O2 --tag-id N --sd --flight --st-package <pkg>`
4. `headless-build.bat -data <작업공간> -import <폴더>\stm32_projects\FSBL -build stm32_projects_FSBL/Debug`
5. `arm-none-eabi-objcopy -O binary`, 그다음 위 서명 명령.

## 올리기 전에

- 앵커: 이 태그는 프레임 방식(`--packets f`) 앵커가 있어야 측정한다. Dropbox `DWM3001_앵커_펌웨어_실측실험_20261124.zip`의 `build/f/`(앵커 1은 대본 없는 기본 판). 앵커가 없어도 BLE 중계는 된다.
- SD 카드: FAT32 포맷 뒤 `python tools/sd_prepare.py --drive E:\`. 카드가 없어도 중계는 되지만 기록은 남지 않는다.
- 올리는 순서와 확인할 줄은 `README_FLIGHT_KO.md` 4장, 6장. 먼저 RAM(`python tools/load_tag.py build/..._id<N>.elf`)으로 `TF_INFO,fw=tagf-v2-lfs,...,flight=1`, `TF_SD,ready=1`, `E,0,flight_uart,1`을 보고 외부 플래시에 쓴다. 쓰기 전에 지금 플래시를 백업한다.
- 되돌리기: `firmware/nucleo/release/n6_bridge_FSBL-trusted.bin`을 같은 명령으로 쓴다.
