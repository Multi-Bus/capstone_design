# 기체별 올라간 버전

기체에 코드를 올리거나 바꾸면 이 문서를 같이 고친다. 해시는 SHA-256이다.
"미확인"은 아직 기체에서 직접 확인하지 않았다는 뜻이다.

## 한눈에 보기

| 기체 | NUCLEO 브리지 | FC 펌웨어 / 파라미터 | XIAO BLE 릴레이 |
| --- | --- | --- | --- |
| 1호기 | n6_bridge | 4.7.3, base + `drone1.param` | v3 |
| 2호기 | n6_bridge + 링크 끊김 강제 DISARM (linkfs `b5674a42`) | 4.7.3, base + `drone2.param` | v3 (2026-11-27 여분 XIAO로 교체) |
| 3호기 | 미확인 | 미확인 | v3 |
| 4호기 | n6_bridge | 4.7.3, base + `drone4.param` (높이 소스 거리센서) | v3 |
| 5호기 | 미확인 | 미확인 | v3 |
| 6호기 | UWB 태그 펌웨어 (2026-11-26 확인, 브리지 아님) | 4.7.3, base (`drone6_asis_20261120.param` 있음) | v3 |

## 부품별 상세

### NUCLEO 브리지 (`firmware/nucleo/`)

| 기체 | 올라간 파일 | 해시 | 확인일 |
| --- | --- | --- | --- |
| 1호기 | `firmware/nucleo/release/n6_bridge_FSBL-trusted.bin` | `444f1e07f68cc54ae4c6c60269f2c02537c1355d1c2c8959f3ef0921f602e014` | 2026-11-21 |
| 2호기 | `firmware/nucleo/release/n6_bridge_linkfs_FSBL-trusted.bin` | `b5674a42126de7cd28d08bcd44a2ce82ad8a7dd8fc0ede882cc02dc43c785654` | 2026-11-27 |
| 4호기 | `firmware/nucleo/release/n6_bridge_FSBL-trusted.bin` | `444f1e07f68cc54ae4c6c60269f2c02537c1355d1c2c8959f3ef0921f602e014` | 2026-11-26 |
| 6호기 | `firmware/nucleo/release/n6_bridge_linkfs_FSBL-trusted.bin` | `b5674a42126de7cd28d08bcd44a2ce82ad8a7dd8fc0ede882cc02dc43c785654` | 2026-11-25 |

`n6_bridge_linkfs_FSBL-trusted.bin`은 기존 브리지에 링크 끊김 강제 DISARM(`FSBL/Inc/link_failsafe.h`)을 더한 것이다. 노트북 하트비트가 12초 없고 FC가 LAND + 시동 ON이면 FC에 강제 DISARM을 보낸다. 해시 `b5674a42126de7cd28d08bcd44a2ce82ad8a7dd8fc0ede882cc02dc43c785654`. 노트북 하트비트가 5초 넘게 없고 FC가 LAND + 시동 ON을 9초 이어 가면 강제 DISARM한다. LAND 시간은 FC가 LAND로 바뀐 때부터 센다.

이전 판(`44b2a0c6…`, 링크가 끊긴 때부터 12초)은 6호기에서 부팅과 1.3 m 호버를 확인했다. 시간을 5초로 줄인 시험판으로 0.2 m에서 BLE를 끊었더니, FC가 LAND로 바뀐 지 1.8초 만에(아직 0.23 m 높이) 브리지가 시동을 껐다. LAND 하강이 그 사이 거의 시작되지 않았다. 그래서 LAND 시간을 따로 세도록 바꿨다. 6호기에서 시험한 뒤 다른 기체에 올린다.

2026-11-25 6호기에 처음 올린 판(`943b43dc…`)은 부팅하지 않았다. 서명할 때 `-align`을 빠뜨려 페이로드가 0x400이 아닌 0x240에 들어갔기 때문이다. 기존 브리지로 되돌린 뒤, `-align`을 넣어 다시 서명했다. 같은 명령으로 기존 소스를 서명하면 기존 릴리스 이미지와 헤더·페이로드가 같고, 무작위 패딩(바이트 169~576)만 다르다. 6호기의 이전 플래시(기존 브리지)는 `firmware/nucleo/backup_drone6_20261125.bin`에 백업했다(커밋하지 않음).

2026-11-26 6호기 NUCLEO를 읽어 보니 위 표의 linkfs 브리지가 아니라 UWB 태그 거리 측정 펌웨어(DW3000 드라이버, `T2 LIBDECA DS-TWR role=initiator address=2`, 이미지 65,408바이트, 시작 주소 `0x3418cd65`)가 올라가 있었다. `firmware/nucleo/backup_drone6_20261126.bin`에 백업했고(커밋하지 않음) 브리지는 다시 쓰지 않았다. 같은 날 6호기는 배터리를 연결할 때 연기가 났다. FC는 USB 전원으로 IMU 두 개, 기압계가 정상이었다.

2026-11-27 2호기 NUCLEO(ST-LINK `002A0048…`)를 읽어 보니 기존 브리지(`n6_bridge_FSBL-trusted.bin`과 페이로드가 같음)가 올라가 있었다. 시험 PC에 백업하고(`nucleo_backup_drone2_20261127_133811.bin`, 커밋하지 않음) linkfs 브리지를 써서 검증(`-v`)했다. 비행 구성으로 부팅한 뒤 BLE 양방향(파라미터 5개 응답, FC SERIAL4 RX 1,369 B)을 두 번 확인했다.

2026-11-26 4호기 NUCLEO에는 UWB 통신용 펌웨어(112,608바이트, 시작 주소 `0x34182c89`)가 올라가 있었다. 그 플래시를 `firmware/nucleo/backup_drone4_20261126.bin`에 백업하고(커밋하지 않음) 위 브리지를 다시 써서 검증(`-v`)했다.

### FC 파라미터 (`fc/`)

| 기체 | 펌웨어 | 기준 파일 | 기체별 변경 | 확인일 |
| --- | --- | --- | --- | --- |
| 1호기 | ArduCopter 4.7.3 | `firmware/fc/base/n6_base.param` | `firmware/fc/drones/drone1.param`, `FLOW_ORIENT_YAW 18000` | 2026-11-21 |
| 2호기 | ArduCopter 4.7.3 | `firmware/fc/base/n6_base.param` | `firmware/fc/drones/drone2.param`: `SERVO_BLH_RVMASK 15`, `FLOW_ORIENT_YAW 18000`. 2026-11-27 base + drone2 54개 중 53개 일치. 다른 1개는 `SERVO_BLH_RVMASK`(FC 15, 파일 0)로, 파일을 FC 값에 맞춤 | 2026-11-27 |
| 3호기 | ArduCopter 4.7.3 | `firmware/fc/base/n6_base.param` | `firmware/fc/drones/drone3.param`: `FLOW_ORIENT_YAW 18000`, `SERVO_BLH_RVMASK 15`. 2026-11-26 `ATC_RAT_YAW_I` 0.018 → 0.03 직접 씀(이륙 때 요가 20~26° 밀려서), 읽어서 확인 | 2026-11-26 |
| 4호기 | ArduCopter 4.7.3 | `firmware/fc/base/n6_base.param` | `firmware/fc/drones/drone4.param`: `FLOW_ORIENT_YAW 18000`, `SERVO_BLH_RVMASK 15` | 2026-11-23 |
| 5호기 | ArduCopter 4.7.3 | `firmware/fc/base/n6_base.param` | `firmware/fc/drones/drone5.param`: `FLOW_ORIENT_YAW 18000`, `SERVO_BLH_RVMASK 15`. 2026-11-25 고도 펜스(`FENCE_*`) 씀, 읽어서 확인 | 2026-11-25 |
| 6호기 | ArduCopter 4.7.3 | `firmware/fc/base/n6_base.param` (47개 중 46개 일치). `MOT_THST_HOVER` 0.404 → 0.455로 직접 씀(2026-11-25 호버 로그 `ThO` 평균, 자동 학습이 안 됐음) | 파일 없음. FC 값 `FLOW_ORIENT_YAW 18000`, `SERVO_BLH_RVMASK 15`는 아직 시험으로 확인하지 않음 | 2026-11-25 |

2026-11-27 2호기 호버 시험(`n6_fly.py --alt 1.0 --hover 10`, 로그 `n6_fly_20261127_160420`): 이륙, 호버, 착륙, 시동 꺼짐까지 자동으로 끝났다. 0.91 m까지 7.3초. 호버 10초 동안 EKF 높이 평균 0.99 m(0.96~1.00), 거리센서 0.93~1.04 m. 롤이 평균 -1.7°(최대 2.6°) 기울어 있었다. 요는 호버 중 14°, 착륙 때 24° 돌아 착지 때 처음보다 34° 돌아가 있었다. 전압 15.7 V → 호버 끝 14.54 V. BLE 링크 지연 0.0~0.1초.

`EK3_SRC1_POSZ 2`(거리센서 높이)는 2026-11-23부터 `n6_base.param` 공통값이다. 4호기 호버 시험에서 확인했다.

`ATC_RAT_YAW_I 0.03`은 2026-11-26부터 `n6_base.param` 공통값이다. 6대 모두 FC USB로 직접 쓰고 읽어서 확인했다: 3호기 2026-11-26, 1·2·4·5·6호기 2026-11-26~27 (`gcs/tools/diag/fc_setparam.py`, 쓰기 전 값은 모두 0.018).

### XIAO BLE 릴레이 (`firmware/xiao/`)

| 기체 | 버전 | 근거 | 확인일 |
| --- | --- | --- | --- |
| 4호기 | v3 (`dfu/n6_ble_relay_v3_id4_dfu.zip`). 이전에는 v1 | 2026-11-25 DFU. USB 시리얼 `stat ... mtu=247`, BLE 연결 확인. 1분 시험(노트북→FC 초당 5개, 자세 10 Hz)에서 FC→노트북 손실 0.40%, 왕복 0.67%. 첫 시험은 노트북→FC가 0%였고, XIAO↔NUCLEO 커넥터를 다시 꽂자 정상 | 2026-11-25 |
| 6호기 | v3 (`firmware/xiao/n6_ble_relay_v3/`, `dfu/n6_ble_relay_v3_id6_dfu.zip`) | BLE 알림 최대 73바이트. 평소 부하(노트북→FC 초당 5개, 자세 10 Hz) 1분 시험에서 FC→노트북 손실 0.81%, v1일 때는 75% | 2026-11-25 |
| 3호기 | v3 (`dfu/n6_ble_relay_v3_id3_dfu.zip`) | 2026-11-25 DFU. USB 시리얼 `stat ... mtu=247`, BLE 연결 확인. 1분 시험(노트북→FC 초당 5개, 자세 10 Hz)에서 FC→노트북 992개 손실 0%, 왕복 300/300 | 2026-11-25 |
| 1호기 | v3 (`dfu/n6_ble_relay_v3_id1_dfu.zip`) | 2026-11-25 DFU. USB 시리얼 `stat ... mtu=247`, BLE 연결 확인. 1분 시험(노트북→FC 초당 5개, 자세 10 Hz)에서 FC→노트북 992개 손실 0%, 왕복 300/300 | 2026-11-25 |
| 5호기 | v3 (`dfu/n6_ble_relay_v3_id5_dfu.zip`) | 2026-11-25 DFU. USB 시리얼 `stat ... mtu=247`, BLE 연결 확인. 1분 시험(노트북→FC 초당 5개, 자세 10 Hz)에서 FC→노트북 992개 손실 0%, 왕복 300/300. 커넥터를 다시 꽂기 전에는 FC→노트북 3.7~4.8%, 왕복 2~19% 손실, 깨진 프레임이 있었고 XIAO `stat`은 `lost=0`이었다 | 2026-11-25 |
| 2호기 | v3 (`dfu/n6_ble_relay_v3_id2_dfu.zip`), 여분 XIAO(BLE `D6:01:DA:1E:1C:48`) | 2026-11-27 교체. 이전 XIAO(BLE `F5:1B:E0:A8:4B:21`)는 노트북→FC가 막혀 FC SERIAL4 RX가 0 B였다. 펌웨어를 다시 올려도 같았고, 1호기 XIAO를 꽂자 정상이어서 이전 XIAO의 D6(TX) 출력 고장으로 판단했다. 교체 후 파라미터 5개 응답, FC SERIAL4 RX 1,390 B, 두 번 확인 | 2026-11-27 |

`firmware/xiao/n6_ble_relay_v2/`는 어느 기체에도 올라가지 않았다. FC에서 오는 데이터가 쪼개져 일부 유실되는 문제를 고친 버전이고, 올리기 전에 따로 시험해야 한다.

## 코드 출처

- `firmware/nucleo/`, `firmware/xiao/n6_ble_relay_v2/`, `firmware/fc/base`, `firmware/fc/drones/drone1.param`, `firmware/fc/setup/`, `gcs/tools/n6_fly.py`, `gcs/tools/dev/`, `docs/archive/핸드오프/`: 팀 Dropbox 핸드오프 `n6_drone_handoff_20261120.zip`에서 가져왔다.
- `firmware/xiao/n6_ble_relay_v1/`, `firmware/fc/drones/drone2.param`, `firmware/fc/drones/drone6_asis_20261120.param`: 이 저장소에 원래 있던 파일이다.
- `firmware/xiao/n6_ble_relay_v3/`: 팀 Dropbox `n6_ble_relay_v3_20261125.zip`에서 가져왔다. DFU zip은 `dfu/SHA256.txt`로 해시를 확인했다.
- v3는 노트북이 초당 50개 이상 쓰면 FC→노트북 방향이 거의 멈춘다 (2026-11-25 시험). `n6_fly`처럼 초당 5개면 문제없다.
- `firmware/uwb/anchor_dwm3001_3to1/`: 팀 Dropbox `3001앵커_펌웨어_3to1_20261123.zip`에서 가져왔다. `backups/`(A1·A2 원래 플래시)와 `.elf`, `.map`은 빼고, 보드에 올린 `.hex`, `.bin`, `manifest.json`은 넣었다. `backups/`는 Dropbox zip에 있다.
- `gcs/tools/diag/`, `gcs/tools/analysis/`: 이번 호버링 시험 중에 작성한 점검·로그 분석 스크립트다(옛 `tools/hover_test/`). Dropbox 자료가 아니다.

시험 절차와 결과는 `docs/호버링_시험_절차.md`에 있다.
