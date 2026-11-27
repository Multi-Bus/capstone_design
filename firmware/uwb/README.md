# uwb — 트랙 B (UWB 통신 기반, CBF-Link 통신 방식 구현 가능성)

논문의 통신 방식(공유 UWB 채널: 측위, heartbeat, 조정자가 배정한 피어 DS-TWR 교환)을 DWM3000 + NUCLEO-N657에 올릴 수 있는지 확인한다.

| 경로 | 내용 | 상태 |
| --- | --- | --- |
| `scripts/airtime_budget.py` | 20 ms 프레임 airtime 예산 (F2). DS-TWR 실측값을 `--twr-us`로 넣는다 | 근사식 |
| `protocol/include/uwb_protocol.h`, `protocol/src/uwb_protocol.c` | 패킷 직렬화, TDMA 슬롯 배치, 조정자 배정, 거리 일관성 검사, 40비트 시계 오프셋. 순수 C99 | PC 테스트 통과, Cortex-M55 컴파일 확인 |
| `protocol/test/test_uwb_protocol.c` | 위 모듈 점검 | |
| `anchor_dwm3001_3to1/` | DWM3001CDK 앵커 펌웨어 (3대1 DS-TWR, 플래시 단독 동작). 설명은 그 폴더 `README_KO.md` | 3대 올림, 1호기 태그와 실측 (2026-11-23) |
| `results/2026-11-23_3to1_trilateration/` | 앵커 3대 + 1호기 태그 삼변측량 결과, 측정 CSV, 재계산 스크립트 | 바닥 높이 수평 오차 4.5~7.9 cm (보정 전) |
| `results/2026-11-24_3to1_laser_check/` | 다른 배치에서 태그–앵커 거리를 레이저 거리와 비교 (앵커 이름 = 펌웨어 ID) | 반복성 0.5 cm, A3 +2 cm, A1 −7 cm, A2 −13~−21 cm |
| `results/2026-11-24_3to1_A2_calibration/` | A2 거리 보정 전후 비교 (레이저 기준, 두 지점) | A2 −12~−14 cm → 0~+0.2 cm |
| `results/2026-11-24_3to1_A1_calibration/` | A1 보정 후 지점 2·4 확인, 지점 3에서 태그 높이 12 cm 차이 비교 | 보정한 A1·A2는 지점 2·4에서 2 cm 이내, 지점 3 −6.5~+2.4 cm |
| `n6_flight_tag/` | 비행용 NUCLEO 펌웨어: BLE 중계 + UWB 태그(프레임 방식) + SD 기록 + 링크 끊김 강제 DISARM. 팀 Dropbox 태그 v2에 `n6_bridge`의 `link_failsafe.h`를 붙임. 설명은 그 폴더 `README_KO.md` | 4호기에 올림, 1.3 m 호버 (2026-11-27) |
| (예정) N6 UWB 프로젝트 | N6 CubeIDE 프로젝트: Qorvo DW3000 드라이버 포팅, SPI + IRQ, DS-TWR | U0. 드라이버는 Qorvo에서 받아야 한다 |

```powershell
python firmware/uwb/scripts/airtime_budget.py --twr-us 1500
```

```powershell
cd firmware/uwb/protocol; gcc -std=c99 -Wall -Wextra -Iinclude -o test/test_uwb_protocol test/test_uwb_protocol.c src/uwb_protocol.c -lm; test/test_uwb_protocol
```

## 확인 항목과 단계

| 단계 | 할 일 | 판정하는 항목 |
| --- | --- | --- |
| U0 | Qorvo API를 N6에 포팅, EVB 2개로 DS-TWR | F1 드라이버 |
| U1 | DS-TWR 1회 시간을 payload 0 / 18 / 51 / 116 B로 측정 | F2 airtime (`airtime.py`), F5 지연 송신 전 payload 준비 |
| U2 | 조정자 비콘 + heartbeat 슬롯, 보드 2~3개 | F3 시계 오차 ε_c, 손실률, F6 앵커 스케줄 |
| U3 | DS-TWR 프레임에 상태 레코드(18 B)를 실어 교환, 거리 일관성 검사 | 논문 시뮬레이터와 같은 범위(상태만 교환) |
| U4 | repair QP 쌍대 변수로 점수 → 조정자 배정 | 트랙 A와 합류 |

현재 airtime 계산 (6대, 앵커 3, 측위 10 Hz, K = 3): DS-TWR 1회가 1.5 ms면 11.1 ms로 들어가고, 3 ms면 21 ms로 넘는다.
