# n6_sd_loader: SD 카드로 브리지 펌웨어 바꾸기

외부 플래시에 로더를 한 번 쓰면, 그다음부터는 SD 카드의 `N6APP.BIN`만 바꿔서 브리지 펌웨어를 바꾼다. 점퍼와 ST-LINK가 필요 없다.
설계와 이유는 [DESIGN_KO.md](DESIGN_KO.md).

## 상태 (2026-11-27)

- PC 시험: `python -m pytest firmware/uwb/n6_sd_loader/tools/test_sd_app.py` 18개 통과 (헤더 형식, CRC, 거부 이유 각각, `--app-link` 패치).
- 빌드: 6개 기체(id0~id5) 모두 펌웨어(`--app-link`)와 로더 오류 0, 경고 0. 로더 이미지 약 165 KB(FSBL 자리 255 KB 안).
- **보드에서는 아직 시험하지 않았다.** DESIGN_KO.md 8장의 2~6단계를 한 기체에서 끝낸 뒤 플래시에 쓴다.

## 파일

| 파일 | 무엇 |
| --- | --- |
| `build/loader_id<N>-trusted.bin` | 외부 플래시용 로더 (기본 펌웨어 `tagf-v2-lfs` id N 포함). 지금 브리지와 같은 명령으로 `0x70000000`에 쓴다 |
| `build/app_id<N>.bin` | SD용 펌웨어 원본 (`--app-link`). `tools/sd_app.py`로 헤더를 붙여 카드에 쓴다 |
| `build/SHA256.txt` | 위 파일들의 SHA-256 |
| `loader/` | 로더 소스와 설치 스크립트 |
| `tools/sd_app.py` | `N6APP.BIN` 만들기와 카드에 쓰기 |

ELF(RAM 시험용)는 커밋하지 않는다.

## SD 카드에 펌웨어 넣기

```
python firmware\uwb\n6_sd_loader\tools\sd_app.py firmware\uwb\n6_sd_loader\build\app_id3.bin --tag-id 3 --drive E:\
```

- 태그 번호는 기체 번호 - 1. 로더와 다르면 로더가 쓰지 않는다(`reason=wrong_id`).
- 카드는 FAT32. 기록 파일 `UWBLOG.BIN`(`n6_flight_tag/tools/sd_prepare.py`)과 같은 카드를 쓴다.
- 부팅하면 ST-LINK 가상 COM(921600)에 `LDR,...,src=sd` 또는 `LDR,...,src=builtin,reason=...` 줄이 나온다.

## SD 펌웨어가 멈추거나 오류가 났을 때

- 오류(HardFault): 펌웨어가 스스로 리셋하고, 로더가 기본 펌웨어로 켠다(`reason=after_reset`).
- 오류 없이 멈춤: NUCLEO RESET 버튼(B2)을 누르면 기본 펌웨어로 켜진다.
- 배터리를 뺐다 꽂으면 다시 SD 펌웨어를 시도한다. 카드를 빼면 항상 기본 펌웨어다.
- 새 `N6APP.BIN`은 바닥에서 `n6_fly.py --no-arm`으로 확인한 뒤 호버한다.

## 다시 빌드하기

기체마다(N = 0~5):

1. 핸드오프 `stm32_projects`를 두 폴더에 푼다 (펌웨어용, 로더용).
2. 펌웨어: `python firmware/uwb/n6_flight_tag/tag/prepare_tag.py --fsbl-copy <폴더1>/stm32_projects/FSBL --packets f --opt O2 --tag-id N --sd --flight --st-package <pkg> --app-link`,
   빌드, `arm-none-eabi-objcopy -O binary <elf> app_idN.bin`.
3. 로더: `python firmware/uwb/n6_sd_loader/loader/prepare_loader.py --fsbl-copy <폴더2>/stm32_projects/FSBL --tag-id N --builtin app_idN.bin --st-package <pkg>`,
   빌드, objcopy, `STM32_SigningTool_CLI -bin loader_idN.bin -nk -of 0x80000000 -t fsbl -hv 2.3 -align -o loader_idN-trusted.bin -s`.

빌드는 `headless-build.bat -data <작업공간> -import <폴더>/stm32_projects/FSBL -build stm32_projects_FSBL/Debug`(CubeIDE 2.3.0). `<pkg>`는 `n6_flight_tag/README_KO.md`의 HAL SD 드라이버 8개가 있는 폴더.
