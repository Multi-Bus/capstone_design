# A 인수인계: B 산출물과 A 요청 사항

- 작성: B. 기준일 2026-10-12. 대상: A (ArduPilot, 본체, EKF 담당)
- 저장소: WSL `~/drone_project` (`\\wsl$\Ubuntu-24.04\home\subniin\drone_project\`)
- 수치는 B 작업 기록(2026-10-11) 값이다. 모두 바람과 센서 잡음이 없는 시뮬레이션 결과다.
- 표기: [미확인]은 아직 확인하지 못한 항목이다. 빈칸으로 두고 임의로 채우지 않았다.

## 1. 요약

- B가 A 없이 할 수 있는 작업은 모두 끝냈다. 12m × 12m × 5m 방 월드, 센서 3종, 실제 위치 `get_truth`, 광류·거리 MAVLink 브리지, 다중 드론 스폰, CSV 로거를 제공한다.
- GPS OFF 검증: 1대 정사각형(2m) 추정 RMSE xy 0.011~0.022m. 2대 GPS OFF 비행은 2회 통과했다.
- 2대 스침 시험(GPS ON)은 3회 실행했다. 3회 모두 `visible_ids`가 `[상대]` → `[]`로 바뀌었다.
- A에게 필요한 것: `get_estimate` 연결, 본체 SDF 센서 장착, EKF 원점(37.0/127.0) 적용, 위치 분산 확인, 실제−추정 offset 담당 정하기, OS·커밋 비교.
- 마감 2026-10-10이 지났다. 새 마감은 [미확인]이다.

## 2. B가 제공하는 것 (A가 쓰는 인터페이스)

### 2.1 실제 위치 `get_truth`

| 항목 | 내용 |
|---|---|
| 파일 | `scripts/get_truth.py` |
| 실행 | 시스템 `python3`를 쓰고 venv는 끈다. `python3 scripts/get_truth.py iris_with_ardupilot`는 0.5초마다 출력한다. |
| 호출 | `Truth(world="room", model_fmt="drone_{}")`로 만들고 `tr.get(drone_id)`를 부른다. 모델 이름을 직접 줄 때는 `tr.get(0, model_name="...")`를 쓴다. |
| 반환 키 | `time_sim_s`, `position_enu_m`, `yaw_enu_rad`, `camera_position_enu_m`, `camera_yaw_enu_rad`, `valid` |

- 카메라 위치는 기체 위치에 본체 기준 (0.1, 0, 0.05) m을 더한 값을 yaw로 회전한 것이다. roll과 pitch는 무시한다.
- `valid`는 마지막 수신 후 0.5초 이내(호스트 시계)이면 True다. 모델을 아직 못 받았으면 `{"valid": False}`만 돌아온다.
- z 기준: 착륙 상태 z는 약 0.19m다. z가 모델 원점(`base_link`) 높이이기 때문이다. 로그에는 truth z를 그대로 쓰고, 지면 대비 높이는 분석할 때 0.19를 뺀다. 이 기준은 A와 공유한다.
- 예 (시험값, 스폰 (-2, 0), `takeoff 2` 후): `position_enu_m = [-1.99, 0.003, 2.19]`, `yaw_enu_rad = 0`.

### 2.2 센서 모듈과 장착

| 센서 (`sensors/`) | 종류, 해상도, 범위 | 주기 | 토픽 (단일 / 드론별) | 본체 기준 장착 pose (x y z r p y) | 수평 FOV |
|---|---|---|---|---|---|
| `front_camera` | camera, 640×480, clip 0.1~20m | 15Hz (설정값, 측정 안 함) | `/front_camera/image` / `/drone_{i}/front_camera/image` | 0.1 0 0.05 0 0 0 | 1.5708 rad (90°) |
| `down_camera` | camera, 320×240, clip 0.1~10m. 센서 안 pitch 1.5708로 아래를 본다. | 30Hz 설정 (헤드리스 측정 약 18Hz) | `/down_camera/image` / `/drone_{i}/down_camera/image` | 0 0 -0.05 0 0 0 | 1.0472 rad (60°) |
| `down_range` | gpu_lidar 레이 1개, 0.1~5.0m. 센서 안 pitch 1.5708로 아래를 본다. | 30Hz 설정 | `/down_range/scan` / `/drone_{i}/down_range/scan` | 0 0 -0.05 0 0 0 | 해당 없음 |

- 장착 주체
  - A: 본체 SDF의 장착 링크에 fixed joint로 붙인다. 소유 규칙에 따른다. 카메라 권장 위치는 `docs/front_camera_spec.md`에 있다. 하향 두 센서의 위치는 B 시험값이라 A가 사양에 맞춰 확인해야 한다.
  - B: `spawn_drones.py`가 드론마다 `iris_with_standoffs::base_link`에 `{name}_joint` fixed joint로 붙인다. B 시험용이다. A 본체에 같은 부모 링크 이름이 있는지는 [미확인]이다.
  - `sensors/iris_flow_test`는 기본 iris에 하향 센서 두 개를 붙인 B 시험용 래퍼다. A 본체 SDF가 아니다.
  - 센서 모델을 include만 하면 고정되지 않는다. joint가 있어야 한다.
- 정면 방향: 링크 +x가 정면이다. yaw 0에서 (+2, 0) 상자는 영상 중심 열 319.5에 찍혔다. (+2, +1)은 왼쪽, (-2, 0)은 보이지 않았다.
- 예 (토픽 확인): `gz topic -l | grep -E "camera|range"`. 드론 i의 토픽은 `/drone_{i}/...` 형식이다.
- 벽과 바닥에 재질이 없으면 카메라 영상이 검게 나온다. 현재 월드는 수정했다(7장).

### 2.3 센서 브리지 `scripts/sensor_bridge.py`

- 실행: `~/venv-ardupilot/bin/python scripts/sensor_bridge.py --mav <주소> [--cam /down_camera/image] [--range /down_range/scan] [--fov 1.0472] [--min-resp 0.3] [--min-m 0.1] [--max-m 5.0]`
- 주소: 직접 TCP 시험은 `tcp:127.0.0.1:5762`. MAVProxy 경로는 드론 0 `udpin:127.0.0.1:15550`, 드론 1 `udpin:127.0.0.1:15560`.
- 입력: `/down_camera/image` 연속 프레임과 `/down_range/scan`의 첫 값.
- 광류: `cv2.phaseCorrelate`(Hanning 창)로 픽셀 이동 (dx, dy)를 구한다. f = 160 / tan(FOV/2) = 277.1px. dt는 이미지 헤더 시간차이며 0 < dt < 0.2초인 쌍만 쓴다.

| 메시지 | 필드 | 값과 단위 |
|---|---|---|
| OPTICAL_FLOW | `flow_rate_x`, `flow_rate_y` | rad/s. x = dx/f/dt, y = dy/f/dt. dx는 영상 오른쪽이 양수, dy는 영상 아래가 양수다. |
| OPTICAL_FLOW | `quality` | 0~255. 위상 상관 응답 × 255이며 응답이 0.3 미만이면 0이다. |
| OPTICAL_FLOW | `ground_distance` | m. 거리 값을 받기 전에는 -1.0이다. |
| OPTICAL_FLOW | `sensor_id`, `flow_x`, `flow_y`, `flow_comp_m_x/y` | 0, 0, 0, 0.0. 확장 필드(`flow_rate`)만 쓴다. |
| OPTICAL_FLOW | `time_usec` | 호스트 monotonic 시각(µs). 시뮬 시각이 아니다. |
| DISTANCE_SENSOR | `current_distance` | cm. 범위 밖이거나 값이 없으면 501이다. |
| DISTANCE_SENSOR | `min_distance`, `max_distance` | 10cm, 500cm |
| DISTANCE_SENSOR | `orientation`, `type`, `id`, `covariance` | 25 (아래), 0, 0, 255 |
| DISTANCE_SENSOR | `time_boot_ms` | 호스트 monotonic ms. 시뮬 시각이 아니다. |

- 하트비트: 1Hz. source system 200, component 191, `MAV_TYPE_ONBOARD_CONTROLLER`.
- MAVLink 2가 필요하다. 코드 맨 위에서 `MAVLINK20=1`을 설정한다. 빠지면 `optical_flow_send()`가 인자 오류로 실패한다.
- MAVProxy(udpin) 경로에서는 브리지가 주기적으로 `recv_match`를 해야 송신이 전달된다. 이 수신 루프를 빼면 안 된다.
- A가 EKF 쪽에서 확인할 것
  - `FLOW_TYPE 5`, `RNGFND1_TYPE 10`이 수신되는지 본다 (`param show FLOW*`, `RNGFND1*`).
  - EKF 메시지 `EKF3 IMU0 fusing optical flow`와 `started relative aiding`이 나오는지 본다. B는 2회 모두 확인했고 EKF 플래그 0x37f를 기록했다.
  - 축과 부호: 북과 동으로 이동할 때 추정과 truth가 일치했다 (2.11, 7.4절 기록).
  - 타임스탬프가 호스트 시계 기준이라는 점이 EKF에 문제가 되는지는 [미확인]이다.
- 예: `sensor_bridge.py --mav udpin:127.0.0.1:15550`을 실행하면 `bridge 시작: ...`이 나온 뒤 `flow n=... range n=... 최근 fx=... q=...` 줄이 주기적으로 나온다.

### 2.4 다중 드론 스폰 `scripts/spawn_drones.py`

- 실행: `source ~/drone_project/env.sh` 후 `python3 scripts/spawn_drones.py <id...> [--world room] [--pos ID=x,y[,z[,yaw]]] [--gen-only]`. gz 바인딩(`gz.transport13`, `gz.msgs10`)이 있는 python을 쓴다. `--gen-only`는 SDF만 만든다.
- 입력: 드론 ID와 위치 (ENU m, yaw rad).
- 출력: `models/generated/drone_{i}/model.sdf` (git 제외). 스폰 결과는 `drone_{i} at (x,y,z) yaw r: OK|FAIL fdm_port_in=...` 줄로 나온다.

| 항목 | 값 |
|---|---|
| 모델 이름 | `drone_{i}` |
| 플러그인 `fdm_port_in` | 9002 + 10i (드론 0은 9002, 드론 1은 9012) |
| SITL | `-I{i} --sysid {i+1}`, TCP 5760 + 10i |
| MAVProxy 제어 / 브리지 UDP | 14550 + 10i / 15550 + 10i (A 가이드 §3과 같음) |
| 기본 위치 (z 0.2, yaw 0) | 0 (-2, 0), 1 (2, 0), 2 (-2, 3), 3 (2, 3), 4 (-2, -3), 5 (2, -3). 6 이상은 `--pos`가 필요하다. |
| 위치 제한 | \|x\|, \|y\| ≤ 5.5m. 벽 ±6m에서 0.5m 안쪽이다. 벗어나면 종료한다. |

- 센서는 `sensors/*/model.sdf`의 링크를 읽어 토픽만 `/drone_{i}/...`로 바꿔 붙인다.
- 한계: 3대 이상 동시 비행은 시험하지 않았다. 드론 1에 0.5m/s로 6초 명령했을 때 이동은 1.72m였다. 명령값 3m보다 짧은 원인은 분석하지 않았다 [미확인].

### 2.5 CSV 로거와 시야 판정

`scripts/csv_logger.py`

- 실행 (`scripts` 폴더에서): `python3 csv_logger.py --ids 0 1 --world room --out ../logs/out.csv --duration 20 [--run-id X] [--no-data-timeout 30]`
  - `--duration`은 시뮬 시간(초)이다. `--no-data-timeout`은 호스트 초이며 기본 10이다. 이 시간 동안 새 시뮬 시각이 없으면 종료한다.
- 주기: 10Hz 시뮬 시간. 한 행은 한 시각과 한 드론이다.
- 컬럼 (A 가이드 §4와 같다):

```text
run_id,time_sim_s,drone_id,estimate_time_boot_ms,estimate_age_s,
truth_x,truth_y,truth_z,estimate_x,estimate_y,estimate_z,
estimate_vx,estimate_vy,estimate_vz,
position_var_x,position_var_y,position_var_z,
uncertainty_valid,uncertainty_source,estimate_valid,visible_ids
```

- 규칙: 값이 없으면 빈칸이다. 불확실성은 0으로 채우지 않는다. 분산 칸은 `position_cov_enu_m2`의 대각 값이 유효할 때만 채운다. `visible_ids`는 JSON 배열 문자열이다 (예: `[1]`).
- 추정 소스 교체 지점: `main(argv=None, estimate_source=stub_estimate)`. `estimate_source(drone_id)`는 `get_estimate` 형식의 dict 또는 None을 돌려준다. 로거가 읽는 키는 `valid`, `stale`, `time_boot_ms`, `time_sim_s`, `position_enu_m`, `velocity_enu_mps`, `uncertainty_valid`, `uncertainty_source`, `position_cov_enu_m2`다.
- 현재 상태: 기본 `stub_estimate`는 항상 None을 돌려준다. 그래서 추정 칸은 모두 빈칸이고 `estimate_valid=false`, `uncertainty_valid=false`, `uncertainty_source=unavailable`이다.
- 예 (A의 `get_estimate`를 넘기는 래퍼. `__main__`과 달리 `os._exit(0)`로 끝내야 종료 시 segfault를 피한다):

```python
import os, sys
sys.path.insert(0, os.path.expanduser("~/drone_project/scripts"))
import csv_logger
def my_estimate(drone_id):
    return get_estimate(drone_id)   # A의 함수. 반환 키는 위 목록과 같아야 한다.
csv_logger.main(["--ids", "0", "1", "--out", "out.csv", "--duration", "20"], estimate_source=my_estimate)
os._exit(0)
```

`scripts/visibility.py`

- 함수: `visible_drones(observer_id, camera_position_enu, positions_enu, camera_yaw_enu, horizontal_fov_rad=pi/2, max_range_m=10.0)`. 정렬된 ID 리스트를 돌려준다.
- 판정: 3차원 거리 10m 이하, 수평각 ±FOV/2 이내(경계 포함), 자기 자신 제외, 수평 위치가 같으면 제외. 수직 FOV와 가림은 판정하지 않는다. A 가이드 §13의 참조 구현이다.
- 로거는 `from visibility import visible_drones`로 가져온다. A 최종본이 오면 이 파일을 교체한다.
- 테스트: `python3 test_visibility.py` → `test_visibility: all passed`.
- 샘플 `logs/sample_logger_check.csv`: 140개 시각, 13.9초. drone_0의 `visible_ids`가 `[]` → `[1]` → `[]`로 바뀌었고 전환 위치는 y ≈ ±3.9m다.

### 2.6 시간 정렬 규칙 (Gazebo 시각과 SITL 시각)

- 공식: `sim_s = time_boot_ms/1000 + offset`. offset은 실행을 시작할 때 다시 재고, SITL을 다시 띄우면 또 잰다.
- 시작 시 측정 (드론 5, 지상, 2026-10-11): offset 평균 0.0006s, 범위 ±0.010s, 기울기 0.9997.
- Gazebo 15초 일시정지: SITL `time_boot_ms`가 멈추고 HEARTBEAT도 끊긴다. 재개 후 offset 변화는 +0.0018s다.
- SITL만 재시작: `time_boot_ms`가 0으로 돌아가지 않고 Gazebo 시각에서 이어진다 (43300, sim 43.297). offset 변화는 -0.009s로 잡음 수준이다.
- 시뮬/실시간 배율은 시간 정렬 시험에서 0.4~0.55였다. 정사각형 재검증에서는 약 0.81이었다. 두 측정의 차이 원인은 [미확인]이다.
- 규칙: `time_boot_ms`를 호스트 시간으로 쓰지 않는다. CSV에는 `time_sim_s`와 `estimate_time_boot_ms`를 둘 다 남긴다.

### 2.7 점검 스크립트 세 개

| 스크립트 | 확인하는 것 | 연결 | 기록상 통과 기준 |
|---|---|---|---|
| `run_flow_check_mavproxy.sh` | 1대 GPS OFF, 정지와 정사각형 정확도 | SITL 출력 제어 14550, 브리지 15550 | EKF 메시지 2종, `flow n` 증가, 꼭짓점 4개 도달, failsafe 없음 |
| `run_two_flow_check.sh` | 2대 GPS OFF, 드론별 명령 분리 | 드론 0 제어 14550 / 브리지 15550, 드론 1 제어 14560 / 브리지 15560 | 종료 코드 `rc=0`, `leftover: none` |
| `run_pass_check.sh` | 2대 스침, GPS ON, `visible_ids` 전환 | SITL `--no-mavproxy`, TCP 5760+10·id | 아래 3개 조건 |

(a) `scripts/run_flow_check_mavproxy.sh` (1대 GPS OFF)

- 실행: `PART=aA RUN=aA INST=0 scripts/run_flow_check_mavproxy.sh`. 기본 시험은 `flow_accuracy_check.py`(정지 60초 후 정사각형)다. 인자를 주면 그 명령을 대신 실행한다.
- 환경 변수: `PART` (gz 파티션, 기본 `agentA`), `RUN` (`runs/` 실행 폴더, 기본 `agentA`), `INST` (SITL 번호, 기본 0), `LOGD` (기본 `/tmp/$RUN`).
- 순서: 헤드리스 Gazebo → `iris_flow_test` 스폰 (-2, 0, 0.2) → SITL (`-w`, `flow_test.parm`) → 브리지 → 시험 (`timeout 900`) → 자기 프로세스 그룹만 종료.
- 측정 (MAVProxy 경로, GPS OFF):

| 회차 | 정사각형 RMSE xy | 정사각형 최대 xy | 정지 최대 xy |
|---|---|---|---|
| 1 | 0.011m | 0.021m | 0.017m (60초) |
| 2 | 0.022m | 0.040m | 0.020m (60초) |
| 재검증 | 0.012m | 0.025m | 0.018m (20초) |

- 오차 = 추정 − 실제 − 정지 구간 평균 offset. 제어는 추정 위치만 쓰고, 실제 위치는 평가에만 쓴다.

(b) `scripts/run_two_flow_check.sh` (2대 GPS OFF)

- 실행: `PART=aM RUN=aM bash scripts/run_two_flow_check.sh` (bash로 실행한다).
- 환경 변수: `PART` (기본 `agentM`), `RUN` (기본 `agentM`), `LOGD` (기본 `/tmp/agentM/$RUN`).
- 시험: 둘 다 2m 이륙 → 20초 정지 → 드론 0만 동쪽 1.5m → 드론 1만 동쪽 1.5m → 착륙. 시험 본체는 `two_flow_check.py`이며 제한 시간은 900초다.
- 기록상 통과 기준: 출력의 `rc=0`과 `leftover: none`. 세부 판정 값은 `two_flow_check.py`에 있고 이 문서에서는 확인하지 않았다 [미확인].
- 측정 (2회 모두 통과):

| 회차 | 드론 0 이동 / 드론 1 최대 | 드론 1 이동 / 드론 0 최대 | 움직인 드론의 추정−실제 차 (드론 0 / 드론 1) |
|---|---|---|---|
| 1 | 1.501 / 0.007m | 1.497 / 0.022m | 0.007 / 0.011m |
| 2 | 1.489 / 0.016m | 1.491 / 0.029m | 0.020 / 0.004m |

(c) `scripts/run_pass_check.sh` (2대 스침, GPS ON)

- 실행: `scripts/run_pass_check.sh` (기본 드론 0, 1). 다른 ID는 `IDS="2 3" PART=agentD RUN=agentD scripts/run_pass_check.sh`처럼 준다.
- 환경 변수: `IDS` (기본 "0 1"), `PART` (기본 `pass_check`), `RUN` (기본 `pass_check`), `LOGD` (기본 `/tmp/$RUN`), `OUT` (CSV, 기본 `$LOGD/pass_check.csv`), `DUR` (로그 시뮬 시간, 기본 70초).
- 시나리오: 드론 0은 (-4, -0.75)에서 yaw 0으로 동쪽, 드론 1은 (4, 0.75)에서 yaw π로 서쪽. 고도 2m, 0.5m/s, 각 8m 이동. 회피 없음. GPS ON (`frame.parm`만 사용).
- 성공 기준 (B 매뉴얼 10.3절):
  1. `pass_check.log` 끝에 `DONE`이 있고 종료 코드가 0이며, 착륙 후 armed가 둘 다 False다.
  2. SITL 로그에 `no JSON`, `failsafe`, `PreArm`이 없다.
  3. CSV에서 각 드론의 `visible_ids`가 `[상대]` → `[]`로 한 번 바뀐다.
- 측정:

| 실행 | 이동 | 전환 x | 최소 거리 | visible 재계산 불일치 |
|---|---|---|---|---|
| 드론 2, 3 | 8.30 / 8.34m | ∓0.77 | 1.493m | 0 / 1402행 |
| 드론 0, 1 (1차) | 8.32 / 8.33m | ∓0.80 | 1.497m | 0 / 1400행 |
| 드론 0, 1 (2차, 수정 후) | 8.31 / 8.37m | ∓0.79 | 1.494m | 0 / 1400행 |

- 최대 고도 2.23m, 착륙 후 z는 0.19였다. 재계산은 같은 `visibility.py`를 다시 호출한 것이라 로거가 함수를 올바르게 쓰는지만 확인한다.

## 3. 파라미터와 A가 검토할 것

### 3.1 `config/flow_test.parm` (전체 12줄)

```text
FRAME_CLASS 1
GPS1_TYPE 0
FLOW_TYPE 5
RNGFND1_TYPE 10
RNGFND1_ORIENT 25
RNGFND1_MIN 0.1
RNGFND1_MAX 5.0
EK3_SRC1_POSXY 0
EK3_SRC1_VELXY 5
EK3_SRC1_POSZ 2
EK3_SRC1_VELZ 0
EK3_SRC1_YAW 1
```

### 3.2 `config/frame.parm`

`FRAME_CLASS 1` 한 줄이다. `--add-param-file`로 넘긴다. 새 실행 폴더에서 `FRAME_CLASS`가 0으로 시작해 시동이 거절되는 문제를 막는다. 확인된 항목이다.

### 3.3 A 가이드 §10.2와 비교

| 항목 | A 가이드 §10.2 | B `flow_test.parm` | 판정 |
|---|---|---|---|
| `EK3_ENABLE` | 1 | 없음 | 차이. B 로그에 EKF3 메시지가 나와 EKF3는 동작했다. 파일 값은 [미확인]. |
| `AHRS_EKF_TYPE` | 3 | 없음 | 차이. 위와 같은 이유로 [미확인]. |
| `EK3_SRC_OPTIONS` | 0 | 없음 | 차이. 파일 값은 [미확인]. |
| `GPS1_TYPE` | 0 | 0 | 같음 |
| `GPS2_TYPE` | 0 (존재하는 버전만) | 없음 | 이 빌드에 존재하는지 [미확인]. B는 `GPS1_TYPE`만 쓴다. |
| `FLOW_TYPE` | 5 | 5 | 같음 |
| `RNGFND1_TYPE` / `RNGFND1_ORIENT` | 10 / 25 | 10 / 25 | 같음 |
| `RNGFND1_MIN` / `RNGFND1_MAX` | 0.1~5m 목표, 단위 확인 | 0.1 / 5.0 | 값은 일치. A 가이드는 0.1~5m 목표만 적고 파라미터 이름과 값은 적지 않았다. |
| `EK3_SRC1_POSXY` / `VELXY` / `POSZ` / `VELZ` / `YAW` | 0 / 5 / 2 / 0 / 1 | 0 / 5 / 2 / 0 / 1 | 같음 |
| `FRAME_CLASS` | 없음 | 1 | B만 추가. 새 실행 폴더에서 필요하다. |
| 파일 이름 | `nogps.parm` (§12) | `flow_test.parm` | 이름만 다르다. |

- 실제로 값이 다른 항목은 없다. A 목록에 있고 B 파일에 없는 것은 `EK3_ENABLE`, `AHRS_EKF_TYPE`, `EK3_SRC_OPTIONS`, `GPS2_TYPE`이다. B 파일에만 있는 것은 `FRAME_CLASS`, `RNGFND1_MIN`, `RNGFND1_MAX`다.
- A는 `param show`로 실제 값을 확인해야 한다. B 파일이 누락된 항목을 기본값에 맡긴 상태이기 때문이다.

## 4. A에게 필요한 것

| 번호 | 요청 | 이유 | B 쪽 연결 위치 |
|---|---|---|---|
| 1 | `get_estimate(drone_id)` 구현과 로거 연결 | 추정 칸이 지금 전부 빈칸이다. A 계약 §4를 완성해야 한다. | `scripts/csv_logger.py`의 `main(estimate_source=...)`, 2.5절 예시 |
| 2 | `send_velocity(drone_id, vx, vy, vz)` 구현 | A 계약 §4 항목이다. B 점검 스크립트 4개는 각자 `SET_POSITION_TARGET_LOCAL_NED`를 직접 보낸다. A 함수로 옮길지 합의가 필요하다. | `two_flow_check.py:77`, `pass_check.py:53`, `flow_accuracy_check.py:62`, `flow_flight_check.py:68` |
| 3 | 최종 시야 판정 함수 | 현재는 A 가이드의 참조 구현이다. 최종본이 오면 교체한다. | `scripts/visibility.py` (로거 import) |
| 4 | 위치 분산 확보 여부 (§11) | 분산 칸이 비어 있다 (`unavailable`). `LOCAL_POSITION_NED_COV` 송신 여부와 배열 배치를 확인해야 한다. | `csv_logger.py`의 `make_row`, `position_cov_enu_m2` |
| 5 | 실제−추정 offset 기록 담당 (§3) | 로거에는 offset 칸이 없다. B의 정사각형 평가는 정지 구간 평균 offset을 빼서 계산한다. 담당을 정해야 한다. | `flow_accuracy_check.py` |
| 6 | EKF 원점 37.0 / 127.0 적용 (§10.2) | B 기록상 기본 원점으로 시험했다. 원점 설정 후 EKF 응답을 확인해야 한다. | `setorigin 37.0 127.0 0` (MAVProxy). B 기록상 미적용. |
| 7 | 본체 SDF에 센서 장착 | 장착 명세와 실제 본체 링크 이름이 필요하다. 카메라 권장 (0.1, 0, 0.05), 하향 시험값 (0, 0, -0.05). | `docs/front_camera_spec.md`, `spawn_drones.py`의 `SENSORS` (15~19행) |
| 8 | 브리지 타임스탬프 확인 | `time_usec`와 `time_boot_ms`가 호스트 monotonic 시각이다. EKF가 이 값을 어떻게 쓰는지 확인해야 한다. | `sensor_bridge.py` 51행, 62행 |
| 9 | `FRAME_CLASS 1`을 A 파라미터 파일에 반영 | 새 실행 폴더의 기본값에 없다. | `config/frame.parm` |
| 10 | 환경 비교 (OS, 커밋) | 아래 표. A의 `versions.txt`를 받아 비교해야 한다. | `docs/versions.txt` |
| 11 | 마감 합의 | 2026-10-10이 지났다. | — |

환경 비교 (B 값은 `docs/versions.txt`, 2026-10-11 기록):

| 항목 | B 값 | A 값 |
|---|---|---|
| OS | Ubuntu 24.04.5 LTS (`Ubuntu-24.04`) | A 가이드는 22.04 기준. 실제 A 값 [미확인] |
| 커널 | 6.18.40.1-microsoft-standard-WSL2 | [미확인] |
| gz sim | 8.15.0 (Gazebo Harmonic) | [미확인] |
| ArduPilot 커밋 | `d67add0b9b65aaed713357d91c95b0d2e8568982` (ArduCopter 4.6.0-beta1, 작업 기록) | [미확인] |
| ardupilot_gazebo 커밋 | `082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5` | [미확인] |
| Python | 3.12.3 | [미확인] |
| 플러그인 경로 | `~/ardupilot_gazebo` | A 가이드 §6.5는 `~/gz_ws/src/ardupilot_gazebo`. 경로만 다르다. |

## 5. 합의가 필요한 결정

| 항목 | B 현재 상태 | 선택지 | 권장 |
|---|---|---|---|
| OS 버전 | Ubuntu 24.04.5 실제 사용. A 가이드는 22.04. | (a) B를 22.04로 다시 맞춤 (b) 24.04 유지, 차이 기록 (c) A도 24.04 사용 | A의 실제 OS를 확인한 뒤 같은 쪽으로 맞춘다. 확인 전에는 차이를 기록으로 남긴다. |
| 실제−추정 offset 담당 | 로거에 칸이 없다. 정사각형 평가 스크립트에서만 계산한다. | (a) A가 분석 단계에서 계산 (b) 로거에 offset 칸 추가 (B 작업) | (a). 로거는 원자료만 남기고, offset 정의는 A 가이드 §3을 따른다. |
| EKF 원점 37.0 / 127.0 | B 기록상 미적용. 기본 원점으로 시험했다. | (a) 적용 (b) 기본 원점 유지 | (a). 적용한 뒤 EKF 응답과 로그를 함께 기록한다. |
| 위치 분산 (§11) | 분산 칸이 비어 있다 (`unavailable`). | (a) 기존 메시지(`LOCAL_POSITION_NED_COV` 등)로 확보 (b) EKF 내부 출력 추가 개발 (c) 미확보 유지 | (a) 가능 여부부터 확인한다. 안 되면 (c)로 두고 미완료로 표시한다. |
| 시야 판정 최종본 | 참조 구현(`visibility.py`)을 쓴다. | (a) A 최종본으로 교체 (b) 참조 구현 유지 | (a). 교체 후 `test_visibility.py`로 같은 케이스를 확인한다. |
| 센서 장착 값 | front (0.1, 0, 0.05)는 권장값, 하향 (0, 0, -0.05)은 시험값. 실제 기체 사양은 미확정. | (a) 사양에 맞춰 세 곳 갱신 (`front_camera_spec.md`, `get_truth` 오프셋, `spawn_drones.py` `SENSORS`) (b) 시험값 유지 | (a). 사양을 확보하면 세 곳을 같은 값으로 맞춘다. |
| 직접 제작 드론 사양 | 기본 iris 사용. 사양 미확보. | (a) 기본 iris로 진행 (b) 사양 확보 후 반영 | 사양 확보 전까지 결과에 `baseline_iris`를 표기한다 (A 가이드 §2). |
| 저장소 공유 | 원격 저장소가 없다. GitHub 로그인이 필요하다. | (a) B가 원격을 만들고 공유 (b) 폴더를 직접 복사 | B가 결정할 사항이다 [미확인]. |
| 마감 | 2026-10-10이 지났다. | 연장 / 유지 | A와 합의해서 새 날짜를 적는다. |

## 6. 재현 방법 (A의 PC에서 점검 스크립트 실행)

- 저장소와 `env.sh`를 A의 PC에 준비한다. 저장소 복사 방법은 [미확인]이다. `env.sh`는 git에 올라가지 않으니 매뉴얼 2.5절 내용으로 직접 만든다. 플러그인 경로는 A의 경로로 맞춘다.
- 스크립트는 `~/venv-ardupilot/bin/python`과 `~/ardupilot/Tools/autotest/sim_vehicle.py`를 고정 경로로 쓴다. A PC에 이 경로가 없으면 venv(OpenCV, numpy, pymavlink, gz 바인딩)를 준비하거나 스크립트 안의 `PY` 줄을 고친다 [미확인].
- `pkill`은 쓰지 않는다. 스크립트는 자기가 띄운 프로세스 그룹만 끈다. 끝나면 `leftover` 줄을 확인한다.
- `PART`는 A의 Gazebo GUI가 쓰는 파티션과 다른 값으로 준다.

```bash
# 터미널마다 한 번
source ~/drone_project/env.sh
cd ~/drone_project
chmod +x scripts/*.sh          # \\wsl$에서 편집하면 실행 권한이 빠질 수 있다

# 1. 1대 GPS OFF (정지 + 정사각형)
PART=aA RUN=aA INST=0 scripts/run_flow_check_mavproxy.sh

# 2. 2대 GPS OFF
PART=aM RUN=aM bash scripts/run_two_flow_check.sh

# 3. 2대 스침 (GPS ON)
PART=aP RUN=aP IDS="0 1" scripts/run_pass_check.sh
```

- 확인할 출력: 1번은 EKF 메시지 2종과 `flow n` 증가, 정사각형 RMSE. 2번은 `rc=0`과 `leftover: none`. 3번은 `pass_check.log`의 `DONE`과 CSV의 `visible_ids` 전환.
- 각 시험은 `timeout 900`(15분) 안에 끝나야 한다.
- 로그 위치: `/tmp/$RUN` (또는 `LOGD`).

## 7. 알려진 문제와 해결

| 증상 (원문) | 원인 | 해결 |
|---|---|---|
| `Arm: Motors: Check frame class and type` | 새 실행 폴더에서 `FRAME_CLASS`가 0 | `--add-param-file=config/frame.parm`을 준다. A 파라미터 파일에도 넣는다. |
| `no JSON sensor message received`, MAVProxy `FTP: request timed out` | Imu 플러그인(`gz-sim-imu-system`) 누락, 또는 SITL을 스폰보다 먼저 켬 | 월드에 Imu가 있는지 확인한다. 순서는 Gazebo → 스폰 → SITL이다. |
| `Unable to find uri[model://front_camera]` | `env.sh` 적용 전에 Gazebo를 켬 | `source env.sh` 후 Gazebo를 다시 켠다. |
| 카메라 영상이 검게 나옴 | 벽·바닥 visual에 ambient/diffuse가 없음 | 벽 0.7, 바닥 1로 `<material>`/`<ambient>`/`<diffuse>`를 넣는다. B는 이미 수정했다. |
| `Arm: Accels inconsistent`가 약 1분 반복 | EKF 준비 전에 시동 | 기다렸다가 시동을 재시도한다. |
| `takeoff`가 `result 4`로 거절됨 | STABILIZE 모드에서 시동 | GUIDED로 바꾼 뒤 EKF가 준비되면 시동한다. |
| `optical_flow_send() takes from 9 to 10 positional arguments but 11 were given` | MAVLink 1로 동작 | `MAVLINK20=1`을 import 전에 설정한다. B 브리지에는 이미 들어 있다. |
| 브리지 송신이 SITL/MAVProxy에 도달하지 않음 | `udpin`은 수신한 패킷에서 상대 주소를 배운다 | 브리지의 `recv_match` 수신 루프를 유지한다. |
| `csv_logger` 종료 시 `Segmentation fault` | gz.transport 구독 정리 문제 | `os._exit(0)`으로 끝낸다 (2.5절 래퍼). |
| gz 서버가 SIGTERM에 끝나지 않음 (1회 관찰) | 원인 [미확인] | 스크립트는 `kill -9 -- -PGID`로 자기 그룹만 강제 종료한다. |
| `Permission denied` (스크립트 실행) | `\\wsl$`에서 편집하며 실행 권한 손실 | `chmod +x <스크립트>`. |
| `land` 후 모드가 `GUIDED`로 남음 | 원인 [미확인] | `mode land`를 쓴다. |
| 일시정지 후 로거가 종료됨 | 기본 대기 10초 | `--no-data-timeout 30`을 준다. |
| `gz model --list` 타임아웃 | WSL2 터미널 간 통신 탐색 | `GZ_IP=127.0.0.1` (효과 [미확인]). B는 이 명령을 더 쓰지 않는다. |
| `ArduPilot controller has reset`, `Duplicate input frame` | SITL 재연결 시 프레임 카운터 재시작 | 한두 번은 무시한다. 반복되면 남은 SITL 프로세스를 정리한다. |

## 8. 파일 위치

`~/drone_project` 기준 (WSL: `\\wsl$\Ubuntu-24.04\home\subniin\drone_project\`).

| 경로 | 내용 |
|---|---|
| `worlds/room.sdf` | 12×12×5m 방, 북쪽 벽 문, Imu 플러그인 포함 |
| `models/floor/` | 텍스처 바닥 (`model://floor`) |
| `scripts/make_floor_texture.py` | 바닥 무늬 생성 (시드 42) |
| `docs/world_guide.md` | 구조물 추가 가이드 (12m 기준) |
| `sensors/front_camera/`, `sensors/down_camera/`, `sensors/down_range/` | 센서 모델 |
| `sensors/iris_flow_test/` | B 시험용 기체 래퍼 (A 본체 아님) |
| `docs/front_camera_spec.md` | 카메라 장착 명세 (A 전달용) |
| `docs/multi_drone.md` | 다중 드론 스폰 규칙 |
| `scripts/set_fov.sh` | 정면 카메라 FOV 변경 |
| `scripts/get_truth.py` | 실제 위치 읽기 |
| `scripts/sensor_bridge.py` | 광류·거리 MAVLink 브리지 |
| `scripts/spawn_drones.py` | 다중 드론 스폰 |
| `scripts/csv_logger.py` | CSV 로거 (10Hz, 21컬럼) |
| `scripts/visibility.py`, `scripts/test_visibility.py` | 시야 판정 참조 구현과 테스트 |
| `scripts/run_flow_check_mavproxy.sh`, `scripts/flow_accuracy_check.py` | 1대 GPS OFF 점검과 정사각형 측정 |
| `scripts/run_two_flow_check.sh`, `scripts/two_flow_check.py` | 2대 GPS OFF 점검 |
| `scripts/run_pass_check.sh`, `scripts/pass_check.py` | 2대 스침 점검 |
| `scripts/run_flow_check.sh`, `scripts/flow_flight_check.py` | 직접 TCP 연결 점검 (MAVProxy 없음) |
| `scripts/flow_probe.py` | 광류 알고리즘 단독 검증 |
| `scripts/time_sync_check.py` | 시간 정렬 점검 |
| `config/flow_test.parm` | GPS OFF 파라미터 (12줄) |
| `config/frame.parm` | `FRAME_CLASS 1` |
| `docs/versions.txt` | 버전 기록 |
| `docs/B_manual.md` | B 매뉴얼 (상세, 10장) |
| `logs/sample_logger_check.csv`, `logs/pass_check_sample.csv` | 샘플 로그 |
| `results/flow_accuracy_*.csv` | 정사각형 정확도 원자료 |
| `env.sh` | 환경 변수 (git 제외) |
