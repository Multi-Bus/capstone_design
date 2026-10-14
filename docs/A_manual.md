# A 통합 실행 매뉴얼

2026-10-14 후속 업데이트: 원본 수신 기록, 10Hz 주기 개선, 불확실성 요약 및 반복 실행은 [묶음 검증 보고서](A_diagnostics_2026-10-14.md)를 함께 참고한다. `accuracy_summary`는 전체 공분산과 별개다. `raw_state.jsonl`, `raw_bridge_*.jsonl`과 대응 `.summary.json`이 실행 폴더에 추가된다.

작성: 2026-10-13. 작업 브랜치: `a/integration`.

## 1. 지금 만든 것

이 저장소는 B의 방·카메라·거리 센서에 A의 비행제어기와 Python API를 연결한다.

`Gazebo의 센서 → sensor_bridge → ArduPilot의 EKF → fleet_api → 속도 명령 → 가상 드론`

Gazebo는 가상 공간의 실제 움직임을 계산한다. ArduPilot은 센서로 위치를 추정하고 모터를 제어한다. `fleet_api`는 Python 프로그램이 추정 위치를 읽고 동·북·위 방향의 속도를 보내는 공통 창구다. CSV는 실제 위치와 추정 위치를 함께 기록한다.

실제 기체 사양은 아직 주어지지 않았으므로 공식 Iris를 기준으로 한다. B의 센서 부착 코드를 사용하며 실제 제작 기체의 질량·관성·모터 특성을 적용했다고 간주하지 않는다.

## 2. 폴더와 환경

- 실행 저장소: `/home/jmhwang391/capstone_design`
- 기존 정상 비행 환경: `/home/jmhwang391/drone_project` — 별도 보존.
- 비행제어기: `/home/jmhwang391/ardupilot`
- Gazebo 플러그인: `/home/jmhwang391/gz_ws/src/ardupilot_gazebo`
- A는 Ubuntu 22.04.5, Python 3.10.12, Gazebo 8.15.0을 유지한다.
- B의 Ubuntu 24.04용 venv를 복사하지 않는다. A에서는 Gazebo 바인딩과 OpenCV를 함께 읽을 수 있는 `/usr/bin/python3`를 사용한다.
- `env.a.sh`가 경로를 설정한다. `AP_ROOT`, `AP_GZ_ROOT`, `A_PYTHON`으로 변경 가능하다.
- Intel Arc/WSLg 화면 문제 때문에 기본값은 소프트웨어 렌더링이다. 시험은 창 없이 실행해도 카메라·거리 센서를 실제로 렌더링한다.
- B의 기존 실행 스크립트 대신 아래 A 실행기를 사용한다.

## 3. 다시 실행하기

먼저 기존 Gazebo/SITL 시험을 종료한다. 같은 포트로 두 시험을 동시에 실행하지 않는다.

Windows PowerShell에서 Ubuntu에 들어간다.

```powershell
wsl -d Ubuntu-22.04
```

이후 **Ubuntu 터미널**에서 실행한다. 별도 Gazebo나 MAVProxy를 먼저 켤 필요가 없다. 실행기가 필요한 프로세스를 시작하고 종료한다.

```bash
cd ~/capstone_design
bash scripts/run_a.sh --stage sensors --timeout 180
```

GPS OFF 1대 이륙 → 2m 호버 → 명령 만료 확인 → 착륙:

```bash
cd ~/capstone_design
bash scripts/run_a.sh --stage hover --timeout 600
```

GPS OFF 1대 2m 정사각형:

```bash
cd ~/capstone_design
bash scripts/run_a.sh --stage square --timeout 900
```

GPS OFF 2대 동시 이륙 → 0번 이동 → 1번 이동 → 착륙:

```bash
cd ~/capstone_design
bash scripts/run_a.sh --stage two --timeout 900
```

`RESULT True`와 `runner_result.json`의 `status: passed`가 성공 기준이다. 시험은 가상 기체에만 연결하도록 localhost 주소를 사용한다. 도중 `Ctrl+C`를 누르면 이 실행기가 시작한 프로세스 그룹만 종료한다. 수동 비행의 정상 착륙 절차와 달리, 이 경우에는 시뮬레이션 자체가 중단된다.

결과는 `runs/a_날짜_시각/`에 남는다. 실패 로그도 보존된다. 실행기는 폴더 삭제, EEPROM 초기화 옵션, 전역 프로세스 종료를 사용하지 않는다.

## 4. 결과 파일

| 파일 | 의미 |
|---|---|
| metadata.json | 버전·커밋·소스 해시·렌더링 설정 |
| check_result.json | 성공 여부, 센서 수신량, 파라미터 확인값, 원점, 착륙 여부, MAVLink 응답 |
| integrated.csv | B 공통 열 + 원시 추정 좌표 + 시험 단계 + truth 시각 |
| check.log | 단계별 진행 결과와 실패 원인 |
| gazebo.log / sitl_*.log / bridge_*.log / mavproxy_*.log | 구성요소별 진단 로그 |
| drone_*/ | 시험 전용 파라미터 및 비행 로그 |

그래프와 지표를 만들 때는 실행 후 표시된 실제 폴더를 넣는다.

```bash
cd ~/capstone_design
/usr/bin/python3 scripts/analyze_a.py runs/a_20261013_214053
```

위 폴더는 실제 정사각형 시험 기록이다. 다른 시험을 분석할 때는 그 폴더로 바꾼다. `trajectory.png`, `metrics.json`이 생긴다.

RMSE는 시작 전 정지 상태에서 한 번 정한 위치 offset을 적용한 뒤 최신 truth/estimate 쌍으로 계산한다. 엄밀한 동일 시각 보간 평가가 아니다. 비행 중 offset을 재조정하지 않으므로 드리프트가 숨겨지지 않는다. B의 별도 구간별 offset 평가와 직접 같은 수치로 비교하면 안 된다.

## 5. API 계약

`scripts/fleet_api.py`의 `Fleet`을 한 번 만들고 공유한다. 같은 UDP 포트에 다른 수신기를 동시에 만들지 않는다.

```python
from fleet_api import Fleet

fleet = Fleet([0, 1])
try:
    state = fleet.get_estimate(0)
    print(state)
    # 이 명령은 유효한 위치, GUIDED 모드, armed 상태에서만 허용된다.
    # 실제 제어 루프에서는 계속 갱신해야 한다.
    # fleet.send_velocity(0, 0.2, 0.0, 0.0)
finally:
    fleet.close()
```

- 좌표: ENU, x=동쪽, y=북쪽, z=위. 위치 m, 속도 m/s.
- 내부 MAVLink NED 변환은 API에서 수행한다.
- `drone_id=i`, sysid=`i+1`, component=1만 상태 수신 대상으로 삼는다.
- `get_estimate`: `position_enu_m`, `velocity_enu_mps`, `time_boot_ms`, `time_sim_s`, `valid`, `stale`, `age_host_s`, `mode`, `armed`, `aligned`, `position_cov_enu_m2`, `uncertainty_valid`, `uncertainty_source` 등.
- 처음 위치는 EKF 로컬 좌표다. `set_alignment(world_minus_local_enu_m, sim_minus_boot_s)` 후에만 Gazebo 월드 좌표와 연결된다. `aligned`를 확인한다. 원시 값은 `raw_position_enu_m`에 남는다.
- 지리 원점과 월드 offset은 별개다. 시험은 `set_origin(37,127,0)` 후 `GPS_GLOBAL_ORIGIN` 응답을 확인한다.
- 속도 한도: 수평 0.5m/s, 수직 0.3m/s. 시험 목표 수평속도는 0.3m/s.
- 명령 송신 10Hz, 갱신이 0.5초 끊기면 0 속도를 보낸다.
- 제어 중 추정값이 무효가 되면 LAND를 요청한다. 연결 자체가 끊겼다면 전달·착륙은 보장할 수 없다.
- API 기본 위치 수신 유효기간은 0.5초. 소프트웨어 렌더링 통합 시험은 명시적으로 1.0초를 사용한다. heartbeat/EKF 상태도 별도로 검사한다.
- 공분산 메시지가 없으면 `None`, `uncertainty_valid=False`다. EKF innovation 지표를 위치 분산으로 바꾸지 않는다.

## 6. 시험 범위와 남은 일

- 자동 시험은 센서 수신, GPS OFF 파라미터 읽기, EKF 상태, 원점, 이륙, 목표 도달, 명령 만료, 착륙·disarm을 검사한다.
- 정사각형 도달 판정은 **추정 위치** 기준이다. 실제 위치 정확도는 CSV로 별도 평가한다.
- 두 기체 시험은 비대상 기체의 실제 수평 이동이 해당 구간에서 0.35m를 넘으면 실패한다.
- 시야 판정은 B의 수평 FOV 함수이며 벽 가림이나 실제 영상 인식이 아니다.
- 위치 공분산 확보는 미완료다. 현재 A 펌웨어는 메시지 64, 230 요청을 거부한다.
- 실제 기체 사양 반영, B PC에서의 재실행, 장시간 안정성·센서 잡음·가림 시험은 남아 있다.
- 정확도 기준을 확정하고 원점/자기장/광류 축·시간/펌웨어 차이를 분리해 평가해야 한다.
- GitHub에 아직 올리지 않았다. 로컬 변경과 테스트 결과를 검토한 뒤 팀 공유하면 된다.

## 7. 회귀 테스트

```bash
cd ~/capstone_design
/usr/bin/python3 scripts/test_fleet_api.py
/usr/bin/python3 scripts/test_visibility.py
```

API 테스트는 ENU/NED, 공분산 회전·결측, 다른 기체 ID 무시, stale·재부팅, 잘못된 속도 차단, 명령 만료·LAND 동작을 검사한다.

초기 `Arm: Accels inconsistent`는 센서 초기화 직후 요청이 거부된 사례였다. 안전 검사를 끄지 않고 준비 재시도로 정상 승인·비행을 확인했다. 두 기체 첫 시험의 상태 무효 중단은 재시험에서 재현되지 않았으므로 원인 확정 없이 실패 기록을 남겼다.
