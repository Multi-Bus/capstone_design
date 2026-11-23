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
| 6호기 | 미확인 | 미확인 (`drone6_asis_20261120.param` 있음) | 미확인 |

## 부품별 상세

### NUCLEO 브리지 (`nucleo/`)

| 기체 | 올라간 파일 | 해시 | 확인일 |
| --- | --- | --- | --- |
| 1호기 | `nucleo/release/n6_bridge_FSBL-trusted.bin` | `444f1e07f68cc54ae4c6c60269f2c02537c1355d1c2c8959f3ef0921f602e014` | 2026-11-21 |
| 4호기 | `nucleo/release/n6_bridge_FSBL-trusted.bin` | `444f1e07f68cc54ae4c6c60269f2c02537c1355d1c2c8959f3ef0921f602e014` | 2026-11-21 |

### FC 파라미터 (`fc/`)

| 기체 | 펌웨어 | 기준 파일 | 기체별 변경 | 확인일 |
| --- | --- | --- | --- | --- |
| 1호기 | ArduCopter 4.7.3 | `fc/base/n6_base.param` | `fc/drones/drone1.param`, `FLOW_ORIENT_YAW 18000` | 2026-11-21 |
| 3호기 | ArduCopter 4.7.3 | `fc/base/n6_base.param` | `fc/drones/drone3.param`: `FLOW_ORIENT_YAW 18000`, `SERVO_BLH_RVMASK 15` | 2026-11-23 |
| 4호기 | ArduCopter 4.7.3 | `fc/base/n6_base.param` | `fc/drones/drone4.param`: `FLOW_ORIENT_YAW 18000`, `SERVO_BLH_RVMASK 15` | 2026-11-23 |
| 5호기 | ArduCopter 4.7.3 | `fc/base/n6_base.param` | `fc/drones/drone5.param`: `FLOW_ORIENT_YAW 18000`, `SERVO_BLH_RVMASK 15` | 2026-11-23 |

`EK3_SRC1_POSZ 2`(거리센서 높이)는 2026-11-23부터 `n6_base.param` 공통값이다. 4호기 호버 시험에서 확인했다.

### XIAO BLE 릴레이 (`xiao/`)

| 기체 | 버전 | 근거 | 확인일 |
| --- | --- | --- | --- |
| 4호기 | v1 (`xiao/n6_ble_relay_v1/`) | BLE 알림 44개 중 80%가 4바이트 이하, 최대 22바이트 (`tools/hover_test/xiao_version.py`) | 2026-11-21 |

`xiao/n6_ble_relay_v2/`는 어느 기체에도 올라가지 않았다. FC에서 오는 데이터가 쪼개져 일부 유실되는 문제를 고친 버전이고, 올리기 전에 따로 시험해야 한다.

## 코드 출처

- `nucleo/`, `xiao/n6_ble_relay_v2/`, `fc/base`, `fc/drones/drone1.param`, `fc/setup/`, `tools/n6_fly.py`, `tools/dev/`, `docs/핸드오프/`: 팀 Dropbox 핸드오프 `n6_drone_handoff_20261120.zip`에서 가져왔다.
- `xiao/n6_ble_relay_v1/`, `fc/drones/drone2.param`, `fc/drones/drone6_asis_20261120.param`: 이 저장소에 원래 있던 파일이다.
- `tools/hover_test/`: 이번 호버링 시험 중에 작성한 점검 스크립트다. Dropbox 자료가 아니다.

시험 절차와 결과는 `docs/호버링_시험_절차.md`에 있다.
