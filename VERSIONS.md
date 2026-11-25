# 기체별 올라간 버전

기체에 코드를 올리거나 바꾸면 이 문서를 같이 고친다. 해시는 SHA-256이다.
"미확인"은 아직 기체에서 직접 확인하지 않았다는 뜻이다.

## 한눈에 보기

| 기체 | NUCLEO 브리지 | FC 펌웨어 / 파라미터 | XIAO BLE 릴레이 |
| --- | --- | --- | --- |
| 1호기 | n6_bridge | 4.7.3, base + `drone1.param` | 미확인 |
| 2호기 | 미확인 | 미확인 (`drone2.param` 있음) | 미확인 |
| 3호기 | 미확인 | 미확인 | 미확인 |
| 4호기 | n6_bridge | 4.7.3, base + `drone4.param` (높이 소스 거리센서) | v1 |
| 5호기 | 미확인 | 미확인 | 미확인 |
| 6호기 | n6_bridge + 링크 끊김 강제 DISARM (시험 중) | 4.7.3, base (`drone6_asis_20261120.param` 있음) | 미확인 |

## 부품별 상세

### NUCLEO 브리지 (`firmware/nucleo/`)

| 기체 | 올라간 파일 | 해시 | 확인일 |
| --- | --- | --- | --- |
| 1호기 | `firmware/nucleo/release/n6_bridge_FSBL-trusted.bin` | `444f1e07f68cc54ae4c6c60269f2c02537c1355d1c2c8959f3ef0921f602e014` | 2026-11-21 |
| 4호기 | `firmware/nucleo/release/n6_bridge_FSBL-trusted.bin` | `444f1e07f68cc54ae4c6c60269f2c02537c1355d1c2c8959f3ef0921f602e014` | 2026-11-21 |
| 6호기 | `firmware/nucleo/release/n6_bridge_linkfs_FSBL-trusted.bin` | `943b43dc4577aa7e9ea97c5379093908525d96906b397647d8a0135d8c3f36d3` | 2026-11-25 |

`n6_bridge_linkfs_FSBL-trusted.bin`은 기존 브리지에 링크 끊김 강제 DISARM(`FSBL/Inc/link_failsafe.h`)을 더한 것이다. 노트북 하트비트가 12초 없고 FC가 LAND + 시동 ON이면 FC에 강제 DISARM을 보낸다. 6호기에서 시험한 뒤 다른 기체에 올린다. 6호기의 이전 플래시(기존 브리지)는 `firmware/nucleo/backup_drone6_20261125.bin`에 백업했다(커밋하지 않음).

### FC 파라미터 (`fc/`)

| 기체 | 펌웨어 | 기준 파일 | 기체별 변경 | 확인일 |
| --- | --- | --- | --- | --- |
| 1호기 | ArduCopter 4.7.3 | `firmware/fc/base/n6_base.param` | `firmware/fc/drones/drone1.param`, `FLOW_ORIENT_YAW 18000` | 2026-11-21 |
| 3호기 | ArduCopter 4.7.3 | `firmware/fc/base/n6_base.param` | `firmware/fc/drones/drone3.param`: `FLOW_ORIENT_YAW 18000`, `SERVO_BLH_RVMASK 15` | 2026-11-23 |
| 4호기 | ArduCopter 4.7.3 | `firmware/fc/base/n6_base.param` | `firmware/fc/drones/drone4.param`: `FLOW_ORIENT_YAW 18000`, `SERVO_BLH_RVMASK 15` | 2026-11-23 |
| 5호기 | ArduCopter 4.7.3 | `firmware/fc/base/n6_base.param` | `firmware/fc/drones/drone5.param`: `FLOW_ORIENT_YAW 18000`, `SERVO_BLH_RVMASK 15` | 2026-11-23 |
| 6호기 | ArduCopter 4.7.3 | `firmware/fc/base/n6_base.param` (47개 중 46개 일치, 다른 값은 학습된 `MOT_THST_HOVER`) | 파일 없음. FC 값 `FLOW_ORIENT_YAW 18000`, `SERVO_BLH_RVMASK 15`는 아직 시험으로 확인하지 않음 | 2026-11-25 |

`EK3_SRC1_POSZ 2`(거리센서 높이)는 2026-11-23부터 `n6_base.param` 공통값이다. 4호기 호버 시험에서 확인했다.

### XIAO BLE 릴레이 (`firmware/xiao/`)

| 기체 | 버전 | 근거 | 확인일 |
| --- | --- | --- | --- |
| 4호기 | v1 (`firmware/xiao/n6_ble_relay_v1/`) | BLE 알림 44개 중 80%가 4바이트 이하, 최대 22바이트 (`gcs/tools/diag/xiao_version.py`) | 2026-11-21 |

`firmware/xiao/n6_ble_relay_v2/`는 어느 기체에도 올라가지 않았다. FC에서 오는 데이터가 쪼개져 일부 유실되는 문제를 고친 버전이고, 올리기 전에 따로 시험해야 한다.

## 코드 출처

- `firmware/nucleo/`, `firmware/xiao/n6_ble_relay_v2/`, `firmware/fc/base`, `firmware/fc/drones/drone1.param`, `firmware/fc/setup/`, `gcs/tools/n6_fly.py`, `gcs/tools/dev/`, `docs/archive/핸드오프/`: 팀 Dropbox 핸드오프 `n6_drone_handoff_20261120.zip`에서 가져왔다.
- `firmware/xiao/n6_ble_relay_v1/`, `firmware/fc/drones/drone2.param`, `firmware/fc/drones/drone6_asis_20261120.param`: 이 저장소에 원래 있던 파일이다.
- `firmware/uwb/anchor_dwm3001_3to1/`: 팀 Dropbox `3001앵커_펌웨어_3to1_20261123.zip`에서 가져왔다. `backups/`(A1·A2 원래 플래시)와 `.elf`, `.map`은 빼고, 보드에 올린 `.hex`, `.bin`, `manifest.json`은 넣었다. `backups/`는 Dropbox zip에 있다.
- `gcs/tools/diag/`, `gcs/tools/analysis/`: 이번 호버링 시험 중에 작성한 점검·로그 분석 스크립트다(옛 `tools/hover_test/`). Dropbox 자료가 아니다.

시험 절차와 결과는 `docs/호버링_시험_절차.md`에 있다.
