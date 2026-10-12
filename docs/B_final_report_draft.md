# 가제보 + ArduPilot SITL 시뮬레이션 환경 최종 리포트 초안 (B 담당 부분)

이 문서는 B 담당 내용만 바탕으로 한 초안이다. A 담당 부분은 [A 담당 내용 입력]으로 표시했다.

- 작성일: 2026-10-12 (기준 자료: B 작업 기록 2026-10-11, `docs/B_manual.md`, 결과 파일)
- 상태: 초안. 통합 전 검토 필요
- 표기: [미확인]은 자료에서 확인하지 못한 항목. [A 담당 내용 입력]은 A의 산출물이 필요한 자리
- 저장소: WSL `~/drone_project`, 업로드 대상 `https://github.com/Multi-Bus/capstone_design.git` (브랜치 `main`)

---

## 1. 개요

### 1.1 목적

다중 드론의 충돌 회피와 편대 제어를 시험할 수 있는 시뮬레이션 환경을 만든다. 1차 범위는 시뮬레이션 환경만이다. 실제 하드웨어, 경로 계획, 회피 알고리즘, 통신 모델, 학습은 제외한다.

### 1.2 B 담당 범위

B는 Gazebo 월드, 센서 모듈, 다중 드론 스폰, 실제 위치 제공, CSV 로거, 2대 스침 점검, B 매뉴얼을 맡았다. 광류 계산과 MAVLink 브리지도 B 담당으로 추가됐다(A 가이드 §1).

파일 소유 규칙은 다음과 같다. 본체 SDF와 ArduPilot 파라미터는 A만 수정한다. 월드와 센서 모듈은 B만 수정한다. 센서는 별도 모델로 제공하고 A가 본체 장착 링크에 붙인다.

### 1.3 이 리포트의 범위

이 리포트는 B 부분만 다룬다. 아래 A 담당 항목은 [A 담당 내용 입력]으로 남긴다.

- ArduPilot 설치와 버전 확정, 기본 1대 이륙·착륙
- 본체 기체 모델과 장착 적용, GPS OFF 파라미터 적용 확인
- 추정 위치·속도·분산 API(`get_estimate`), 속도 명령(`send_velocity`)
- 시야 판정 함수 최종본
- 정사각형 시험 소유와 그래프(일정표 A7)
- A 매뉴얼

주의: 정사각형 시험은 일정표와 A 가이드 §14에서 A 담당이다. B 작업 기록 11번에는 B가 실행한 것으로 적혀 있다. 소유 확정이 필요하다 [A 담당 내용 입력].

---

## 2. 환경과 버전

`docs/versions.txt` 기록(2026-10-11T00:24:12+09:00)과 작업 기록을 옮겼다.

| 항목 | 값 | 출처 |
|---|---|---|
| OS | Ubuntu 24.04.5 LTS (noble), 커널 6.18.40.1-microsoft-standard-WSL2 (WSL2) | `docs/versions.txt` |
| 시뮬레이터 | Gazebo Harmonic, gz sim 8.15.0 | `docs/versions.txt` |
| ArduPilot | commit `d67add0b9b65aaed713357d91c95b0d2e8568982`, 4.6.0-beta1 | `docs/versions.txt`, 작업 기록 2.11 |
| ardupilot_gazebo | commit `082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5` | `docs/versions.txt` |
| Python (시스템) | 3.12.3 | `docs/versions.txt` |
| Python (venv `~/venv-ardupilot`) | numpy 2.5.3, pymavlink 2.4.50, OpenCV 5.0.0 | `docs/python-requirements.txt`, 작업 기록 2.9 |
| 기준 기체 | 공식 Iris (`baseline_iris`로 표시) | 작업 기록 4 |

열린 항목:

- OS 불일치: A 가이드는 Ubuntu 22.04 기준이다. B 환경은 24.04다. A와 통일할지 결정이 필요하다 [A 담당 내용 입력].
- 버전 비교: A의 ArduPilot·플러그인 커밋 해시와 B의 해시가 같은지 확인이 필요하다 [A 담당 내용 입력].
- 플러그인 경로: A 가이드는 `~/gz_ws/src/ardupilot_gazebo`, B는 `~/ardupilot_gazebo`를 쓴다. 경로는 B 쪽에 두고 있다(작업 기록 2.6).
- 재현: B 두 번째 PC에서 설치 재현은 하지 않았다. 아래 5절 B0 참고.
- `docs/python-requirements.txt`에는 OpenCV가 없다. 광류 브리지는 OpenCV를 쓴다. 목록을 보완해야 한다.

---

## 3. 구현 내용

### 3.1 실내 방 월드

- 파일: `worlds/room.sdf`, `models/floor/`, `scripts/make_floor_texture.py`, `docs/world_guide.md`
- 크기: 바닥 12 m × 12 m, 벽 높이 5 m, 벽 두께 0.2 m, 벽 중심 ±6 m, 지붕 없음
- 문: 북쪽 벽(y = +6)의 가운데. 폭 1 m(x -0.5~0.5), 높이 2 m. 벽은 왼쪽 5.5 m, 오른쪽 5.5 m, 문 위 1 m × 3 m 조각 세 개로 나눴다. 구멍을 파는 기능은 쓰지 않았다.
- 바닥: 12.4 m × 12.4 m 평면에 1024 × 1024 흑백 랜덤 무늬(8 px 칸, 시드 42 고정). 12.4 m에 펼치면 칸 하나가 약 9.7 cm다. 단색 바닥은 광류를 계산할 수 없어서 텍스처를 넣었다.
- 재질 요구사항: 벽과 바닥에 `<ambient>`와 `<diffuse>`를 넣어야 한다. 재질 없는 벽과 `<pbr>`만 있는 바닥은 카메라 영상에서 검게 렌더링됐다(6절 항목 3).
- 시스템 플러그인: Physics, UserCommands, SceneBroadcaster, Imu, Sensors(ogre2). Imu가 없으면 SITL에서 `no JSON sensor message received`가 난다.
- 변경 이력: 8 m × 8 m × 3 m 버전(2.4) → 12 m × 12 m × 5 m 텍스처 바닥 버전(2.4-2). 12 m 방의 공 충돌 결과는 기록에 없다 [미확인].

### 3.2 센서 모듈

센서는 `sensors/` 아래 별도 모델이다. 링크 이름은 모두 `link`, 질량 0.01 kg, 중력 비활성이다.

| 모델 | 종류 | 해상도 | 수평 FOV | 주기 | 범위 | 토픽 | 방향 |
|---|---|---|---|---|---|---|---|
| `front_camera` | camera | 640 × 480 | 1.5708 rad (90°) | 15 Hz | clip 0.1~20 m | `front_camera/image` | 링크 +x가 정면 |
| `down_camera` | camera | 320 × 240 | 1.0472 rad (60°) | 30 Hz | clip 0.1~10 m | `down_camera/image` | 아래 (센서 pose pitch 1.5708) |
| `down_range` | gpu_lidar, 레이 1개 | 1 샘플 | - | 30 Hz | 0.1~5.0 m | `down_range/scan` | 아래 (센서 pose pitch 1.5708) |

설계 결정:

- 정면 카메라의 수평 FOV는 `horizontal_fov` 한 줄이다. `scripts/set_fov.sh <도>`로 바꾼다. 현재 값은 90°다. 이미 떠 있는 모델은 새 값을 읽지 않으므로 삭제 후 다시 스폰해야 한다.
- 본체 장착 pose(본체 기준 x y z): 정면 카메라 (0.1, 0, 0.05), 하향 센서 (0, 0, -0.05). 장착은 fixed joint로 하며, A가 본체 SDF에서 한다(`docs/front_camera_spec.md`).
- B 시험용 래퍼 `sensors/iris_flow_test/`는 기본 iris에 하향 센서를 붙인 것이다. A의 본체 SDF가 아니다.

### 3.3 실제 위치 제공 `get_truth`

- 파일: `scripts/get_truth.py`
- 원리: `/world/room/dynamic_pose/info`(Pose_V)를 `gz.transport13`과 `gz.msgs10`으로 구독한다. 시스템 `python3`로 실행한다.
- 반환 값:

| 키 | 의미 |
|---|---|
| `time_sim_s` | 메시지 헤더의 시뮬레이션 시각 |
| `position_enu_m` | 모델 원점 위치 (ENU) |
| `yaw_enu_rad` | 동쪽 0, 반시계 양수 |
| `camera_position_enu_m` | 기체 위치 + 장착 오프셋 (0.1, 0, 0.05)을 yaw로 회전한 값. roll/pitch 무시 |
| `camera_yaw_enu_rad` | 기체 yaw와 같음 |
| `valid` | 마지막 수신 후 0.5 s 이내(호스트 시계)이면 True |

- 검증: 스폰 (-2, 0), `takeoff 2` 후 위치 (-1.99, 0.003, 2.19), yaw 0. x, y 오차는 1 cm 이내.
- z 기준: 착륙한 iris의 z는 약 0.19 m다. z는 `base_link` 원점 높이다. 로그에는 truth z를 그대로 쓰고, 분석할 때 0.19 m를 뺀다. 이 기준은 A와 공유해야 한다 [A 담당 내용 입력].
- 평가 전용: 제어에는 실제 위치를 쓰지 않는다(A 가이드 §14). 시험 스크립트도 제어 입력은 추정 위치만 쓴다.

### 3.4 광류 계산과 MAVLink 브리지

- 파일: `scripts/sensor_bridge.py`, `config/flow_test.parm`, `scripts/flow_probe.py`(알고리즘 단독 시험), `scripts/flow_flight_check.py`, `scripts/run_flow_check.sh`, `scripts/run_flow_check_mavproxy.sh`, `scripts/flow_accuracy_check.py`

계산 방법:

- 하향 카메라 연속 프레임의 평균 그레이를 `cv2.phaseCorrelate`(Hanning 창)로 비교해 픽셀 이동량 (dx, dy)를 얻는다.
- 초점거리 f = (320 / 2) / tan(30°) = 277.1 px.
- 광류: `flow_rate_x = dx / f / dt`, `flow_rate_y = dy / f / dt` (rad/s). dt는 이미지 헤더의 시뮬레이션 시각 차이이며 0 초과 0.2 s 미만인 쌍만 쓴다.
- quality: 위상 상관 신뢰도 × 255. 신뢰도가 0.3 미만이면 0을 보낸다.
- 축 규약: 기체가 앞(+x)으로 가면 영상 내용이 아래(+row)로, 왼쪽(+y)으로 가면 오른쪽(+col)으로 움직인다. ArduPilot SITL 가상 광류 소스(`AP_OpticalFlow_SITL.cpp`)와 같은 규약이며, 시험으로 확인했다.
- 거리: `/down_range/scan` 값을 cm로 바꿔 `DISTANCE_SENSOR`로 보낸다. orientation 25(아래), 최소 10 cm, 최대 500 cm. 범위를 벗어나면 501 cm를 보낸다.
- 하트비트: 1 Hz, ONBOARD_CONTROLLER, source system 200.
- MAVLink 2: `flow_rate_x/y`는 확장 필드다. 코드 맨 위에서 `os.environ["MAVLINK20"] = "1"`을 설정한 뒤 pymavlink를 import해야 한다.
- 수신 루프: pymavlink `udpin`은 상대 주소를 수신한 패킷에서 알아낸다. 브리지는 `recv_match` 수신 루프를 돌아야 송신이 전달된다.

연결 경로:

| 경로 | 제어 | 브리지 | 비고 |
|---|---|---|---|
| 직접 TCP (단독 시험) | TCP 5760 | TCP 5762 | `--no-mavproxy` |
| MAVProxy (A 가이드 §12) | udpin 14550 | udpin 15550 | SITL `--no-extra-ports --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:15550` |

GPS OFF 시험 파라미터 (`config/flow_test.parm`, B가 작성한 시험용 파일):

| 파라미터 | 값 | 의미 |
|---|---|---|
| `FRAME_CLASS` | 1 | 0이면 시동 거절 |
| `GPS1_TYPE` | 0 | GPS 끔 (이 빌드는 `GPS_TYPE`이 없음) |
| `FLOW_TYPE` | 5 | 광류 = MAVLink |
| `RNGFND1_TYPE` | 10 | 거리 = MAVLink |
| `RNGFND1_ORIENT` | 25 | 아래 |
| `RNGFND1_MIN` / `MAX` | 0.1 / 5.0 | m 단위 |
| `EK3_SRC1_POSXY` / `VELXY` | 0 / 5 | 수평 위치 입력 없음 / 수평 속도 = 광류 |
| `EK3_SRC1_POSZ` / `VELZ` | 2 / 0 | 높이 = 거리센서 / 수직 속도 입력 없음 |
| `EK3_SRC1_YAW` | 1 | yaw = 컴퍼스 |

- A 가이드 §10.2의 `EK3_ENABLE`, `AHRS_EKF_TYPE`, `EK3_SRC_OPTIONS`, `GPS2_TYPE`는 이 파일에 없다. EKF3 메시지는 로그에서 확인했다. 파라미터 소유 규칙에 따라 A와 합의한다 [A 담당 내용 입력].
- 시뮬레이션 SITL 가상 광류(`FLOW_TYPE 10`)는 진단용이다. 결과는 `synthetic_flow`로 구분하며 이 리포트의 결과에는 쓰지 않았다.

### 3.5 다중 드론 스폰

- 파일: `scripts/spawn_drones.py`, `docs/multi_drone.md`
- 방법: 원본 `iris_with_ardupilot`는 고치지 않는다. 드론 i마다 복사본을 `models/generated/drone_{i}/model.sdf`(git 제외)로 만들고 모델 이름과 플러그인 포트만 바꾼다. 센서 세 개는 `sensors/*`에서 읽어 드론별 토픽으로 바꾸고, `iris_with_standoffs::base_link`에 fixed 조인트로 붙인다.

| 항목 | 규칙 |
|---|---|
| 모델 이름 | `drone_{i}` |
| 플러그인 `fdm_port_in` | 9002 + 10i |
| SITL | `-I{i} --sysid {i+1}`, TCP 5760 + 10i |
| 제어 UDP / 브리지 UDP | 14550 + 10i / 15550 + 10i |
| 센서 토픽 | `/drone_{i}/front_camera/image`, `/drone_{i}/down_camera/image`, `/drone_{i}/down_range/scan` |
| 기본 위치 (ENU, z 0.2, yaw 0) | 0:(-2, 0), 1:(2, 0), 2:(-2, 3), 3:(2, 3), 4:(-2, -3), 5:(2, -3). 6 이상은 `--pos` 필수 |
| 위치 제한 | \|x\|, \|y\| ≤ 5.5 m |

- 포트 규칙은 A 가이드 §3의 드론 0, 1 값(9002, 9012, 5760, 5770, 14550, 14560, 15550, 15560)과 일치한다.
- 검증 범위: 드론 3, 4 스폰(포트 9032, 9042, 토픽 6개, `get_truth.get(3/4)` 동작). 드론 1, 2에 SITL 2개를 붙여 동시 이륙, 드론 1만 명령했을 때 드론 2는 움직이지 않았다. 3대 이상은 시험하지 않았다 [미확인].

### 3.6 CSV 로거와 시야 판정

- 파일: `scripts/csv_logger.py`, `scripts/visibility.py`, `scripts/test_visibility.py`, `logs/sample_logger_check.csv`, `logs/pass_check_sample.csv`
- 컬럼: A 가이드 §4의 21개 컬럼을 그대로 쓴다.

```text
run_id,time_sim_s,drone_id,estimate_time_boot_ms,estimate_age_s,truth_x,truth_y,truth_z,estimate_x,estimate_y,estimate_z,estimate_vx,estimate_vy,estimate_vz,position_var_x,position_var_y,position_var_z,uncertainty_valid,uncertainty_source,estimate_valid,visible_ids
```

- 주기: 10 Hz, 시뮬레이션 시간 기준. 한 행은 한 시각의 한 드론이다.
- 빈칸 규칙: 추정이 없으면 추정 칸은 빈칸이고 `estimate_valid=false`다. 불확실성은 0으로 채우지 않는다. 공분산이 없으면 `uncertainty_valid=false`, `uncertainty_source=unavailable`, `position_var_*`는 빈칸이다.
- `visible_ids`: JSON 배열 문자열(예: `[1]`, `[]`).
- 추정 소스: `main(estimate_source=...)`로 교체한다. 기본값 `stub_estimate`는 None을 돌려준다. A의 `get_estimate`를 연결해야 추정 칸이 채워진다 [A 담당 내용 입력].
- 종료: 호스트 시간 10 s 동안 새 시뮬레이션 시각이 없으면 끝난다. 일시정지가 길면 `--no-data-timeout`을 준다. 종료는 `os._exit(0)`으로 한다(6절 항목 6).
- 시야 판정: 관측자 카메라 위치와 yaw에서 다른 드론의 중심점이 수평 FOV 90°(±45°) 안에 있고 3차원 거리 10 m 이내이면 보인다. 거리와 FOV 경계는 포함하고 자기 자신은 뺀다. 수직 FOV와 벽 가림은 판정하지 않는다. `test_visibility.py`는 `all passed`다.
- 샘플 검증 (`logs/sample_logger_check.csv`): 280행(2대 × 140 시각), 9.690~23.590 s(13.9 s, 간격 0.1 s). drone_0의 `visible_ids`는 `[]` → `[1]` → `[]`로 바뀌었다. 전환 위치 y ≈ ±3.9 m는 FOV ±45° 기하와 맞는다.

### 3.7 시간 정렬

- 파일: `scripts/time_sync_check.py`(드론 5, 지상 대기, 격리 파티션)
- 방법: 같은 호스트 순간에 받은 `ATTITUDE.time_boot_ms`와 `dynamic_pose/info`의 최신 시뮬레이션 스탬프 쌍으로 offset = sim_s − time_boot_ms / 1000을 잰다.
- 결과: 4절 4.6 참고.
- 규칙: `sim_s = time_boot_ms / 1000 + offset`. offset은 실행 시작 때 다시 재고, SITL을 재시작하면 다시 잰다. `time_boot_ms`를 호스트 시간으로 쓰지 않는다.
- 로그 시간: CSV에는 `time_sim_s`와 `estimate_time_boot_ms`를 둘 다 남긴다(계약에 포함).
- 시간 정렬 원자료는 `/tmp/agentT/`에 있었고 저장소에는 없다 [미확인: 보존 여부].

---

## 4. 검증 결과

주의: 이 시뮬레이션에는 바람, 센서 잡음, 모션 블러, 기체 기울기가 없다. 아래 정확도 수치는 실제 기체 성능이 아니다. 연결과 계산이 맞는지 확인하는 용도다.

### 4.1 광류 GPS OFF 정사각형 정확도

시험: 이륙 2 m, 정지비행, A 가이드 §14 정사각형(한 변 2 m, 0.3 m/s, 추정 위치로만 제어), 착륙. 오차 = 추정 − 실제 − offset(정지 첫 5 s 평균, 정사각형은 직전 3 s 평균). RMSE xy = sqrt(mean(err_xy²)).

| 회차 | 결과 파일 | 정사각형 RMSE xy (m) | 정사각형 최대 xy (m) | 정지비행 최대 xy (m) |
|---|---|---|---|---|
| 1회차 | `results/flow_accuracy_20261011_200631.csv` | 0.011 | 0.021 | 0.017 (60 s) |
| 2회차 | `results/flow_accuracy_20261011_201418.csv` | 0.022 | 0.040 | 0.020 (60 s) |
| 재검증 | 결과 파일 없음 (작업 기록 2.13) | 0.012 | 0.025 | 0.018 (20 s) |

- 1·2회차 CSV를 다시 계산해 같은 값을 얻었다(1회차 0.0115 / 0.0213 / 0.0171, 2회차 0.0219 / 0.0402 / 0.0198).
- 정사각형 4개 구간(sq1~sq4)은 두 회차 CSV에 모두 있다.
- 재검증에서 `EKF3 IMU0 fusing optical flow`, `started relative aiding`을 확인했다. 브리지는 광류를 2699건 보냈고 failsafe는 없었다.
- 시뮬레이션 실시간 배율은 재검증 기준 약 0.81이었다(7.5절). 자료 간 차이는 9.4절.
- 검증 주체: 정사각형 시험과 그래프는 A 담당이다 [A 담당 내용 입력].

### 4.2 2대 GPS OFF 광류 비행 (A 가이드 §12 연결)

시험: 두 기체 모두 2 m 이륙, 20 s 정지, 드론 0만 동쪽 1.5 m, 드론 1만 동쪽 1.5 m, 착륙. 제어는 추정 위치만 썼다. 드론 0은 sysid 1, 제어 14550, 브리지 15550. 드론 1은 sysid 2, 제어 14560, 브리지 15560.

| 실행 | 정지 중 최대 이동 (드론 0 / 드론 1, m) | 드론 0 명령 시: 드론 0 이동 / 드론 1 최대 (m) | 드론 1 명령 시: 드론 1 이동 / 드론 0 최대 (m) | 추정 − 실제 차 (드론 0 단계 / 드론 1 단계, m) |
|---|---|---|---|---|
| 1회 | 0.028 / 0.015 | 1.501 / 0.007 | 1.497 / 0.022 | 0.007 / 0.011 |
| 2회 | 0.015 / 0.016 | 1.489 / 0.016 | 1.491 / 0.029 | 0.020 / 0.004 |

- 두 SITL 모두 `EKF3 IMU0 fusing optical flow`, `started relative aiding`이 나왔다. 브리지는 드론마다 약 1900건을 보냈다.
- failsafe 없음, 착륙 후 disarm, 남은 프로세스 없음.
- 원본 로그는 저장소에 없다(실행 스크립트가 `/tmp`에 기록) [미확인: 보존 여부].
- A 쪽 확인(송신자 sysid, 기체별 EKF 설정)은 [A 담당 내용 입력].

### 4.3 정면 카메라 방향 확인

방법: 격리 파티션에 드론 7을 (0, 0), yaw 0으로 스폰하고 빨간 상자(0.4 × 0.4 × 1 m)를 옮겨 가며 `/drone_7/front_camera/image`의 빨간 영역 중심 열을 쟀다. 영상 중심 열은 320이다.

| 경우 | 상자 위치 | 중심 열 (기하 예상) | 판정 |
|---|---|---|---|
| yaw 0 | (+2, 0) | 319.5 | 중앙 |
| yaw 0 | (+2, +1) | 144.3 (약 152) | 왼쪽 |
| yaw 0 | (+2, -1) | 494.7 (약 488) | 오른쪽 |
| yaw 0 | (-2, 0) | 빨간 픽셀 0 | 안 보임 |
| yaw 90° | (0, +2) | 319.5 | 중앙 |
| yaw 90° | (+2, 0) | 빨간 픽셀 0 | 안 보임 |

- 기체 +x가 정면이다. 영상은 위아래가 바르다(위쪽 벽, 아래쪽 바닥 무늬).
- 미확인: 비행 중 pitch/roll이 있을 때의 영상.

### 4.4 다중 드론 스폰 확인

| 항목 | 결과 |
|---|---|
| 드론 3, 4 스폰 | 성공. 포트 9032, 9042. 토픽 6개 |
| `gz sdf -k` | 드론 3, 4 모두 Valid |
| 영상 | 드론 3, 4 down_camera 프레임 정상 (B_manual 10.1). 드론 1, 2 프레임 평균 171.02, 168.43 (`docs/multi_drone.md`) |
| 지상 거리센서 | 0.145 m (드론 3, 4) |
| `get_truth.get(3)`, `get(4)` | 동작 |
| 드론 1 북쪽 0.5 m/s 6 s 명령 | 드론 1 Δy = +1.72 m, 드론 2 0.00 m |
| 호버 중 거리센서 | 두 드론 약 2.13~2.14 m |
| 이동 전후 하향 카메라 평균 절대차 | 드론 1 34.53, 드론 2 6.73 |

- 한계: 드론 1의 이동 1.72 m는 명령 속도 × 시간(3 m)보다 작다. 원인은 분석하지 않았다 [미확인].
- 정면 카메라는 벽 앞 단색 영상이라 이동 증거로 쓰지 않았다.
- 3대 이상 동시 비행은 시험하지 않았다 [미확인].

### 4.5 2대 스침 시험 (3회)

시나리오: 첫째 드론 (-4, -0.75), yaw 0, 동쪽. 둘째 드론 (4, +0.75), yaw π, 서쪽. 고도 2 m, 0.5 m/s, 각자 8 m. GPS ON, 회피 없음. 로거는 이륙 직전부터 70 s 기록.

| 실행 | 이동 (m) | `[상대]` → `[]` 전환 시점 x (m, 간격) | 최소 거리 (m) | `visible_ids` 재계산 불일치 |
|---|---|---|---|---|
| 드론 2, 3 (`logs/pass_check_sample.csv`) | 8.30 / 8.34 | ∓0.77 (1.54) | 1.493 | 0 / 1402행 |
| 드론 0, 1 (1차) | 8.32 / 8.33 | ∓0.80 (1.60) | 1.497 | 0 / 1400행 |
| 드론 0, 1 (2차, 로거 수정 후) | 8.31 / 8.37 | ∓0.79 (1.58) | 1.494 | 0 / 1400행 |

- 예상 전환 위치: 카메라가 본체 앞 0.1 m, 수평 FOV ±45°, 차선 간격 1.5 m. 간격 약 1.6 m, 각자 x ≈ ∓0.8 m. 측정과 일치한다.
- 샘플 파일 재확인: 1402행, 최소 거리 1.4934 m, 재계산 불일치 0건(같은 `visibility.py` 함수로 재계산).
- 두 드론 최대 고도 2.23 m, 마지막 z 0.19 m(착륙). 벽 안쪽(|x| ≤ 4.38 m)에서 끝났다.
- 재계산은 로거가 함수를 올바르게 불렀는지 확인한 것이다. 판정 기준 자체를 검증한 것이 아니다.
- yaw 변화는 시험하지 않았다. 드론 2, 3 실행에서 yaw가 0, π로 유지되는 것만 확인했다.

### 4.6 Gazebo·SITL 시간 정렬

| 상황 | 결과 |
|---|---|
| 시작 60 s | offset 평균 0.0006 s, 범위 ±0.010 s, 기울기 0.9997 |
| Gazebo 15 s 일시정지 | SITL `time_boot_ms`가 37734에서 멈추고 HEARTBEAT도 끊김. 재개 후 offset 변화 +0.0018 s |
| SITL만 재시작 | `time_boot_ms`가 0으로 돌아가지 않고 43300(sim 43.297)에서 이어짐. offset 변화 −0.009 s |

- JSON 모델에서 SITL은 Gazebo가 보내는 시각으로 진행한다. 시작 시 offset 차이는 ±0.010 s 이내였다(표본 간격 수준).
- 시뮬레이션 실시간 배율: 이 PC에서 약 0.4~0.55(작업 기록 2.18, B_manual 10.5). 9.4절 참고.

### 4.7 로거 일시정지 시험

| 설정 | 결과 |
|---|---|
| 기본 (대기 10 s) | 일시정지 중 "새 시뮬 시각 없음, 종료", 37행에서 끝남 |
| `--no-data-timeout 30` | 300행, sim 12.02~41.92 s, 최대 간격 0.108 s |

- 로거 시작 직후(약 5 s 안) 일시정지하면 첫 데이터 전이라 0행이 될 수 있다. 재실행에서 확인했다.
- 원자료는 저장소에 없다 [미확인].

---

## 5. 일정표 체크리스트 대응

B 항목(일정표 B0~B7)과 체크리스트의 B 항목을 정리했다. 상태는 완료, 부분, 미완으로 표시했다.

| 항목 | 일정표 내용 | 상태 | 근거 |
|---|---|---|---|
| B0 | 같은 설치를 각자 PC에서 따라 하기 | 미완 | 작업 기록에 두 번째 PC 재현 기록이 없다. 설치는 한 PC(WSL, 24.04)에서 했다 (2.2) |
| B1 | 실내 방 월드(벽 + 문) + 구조물 설치 가이드 | 완료 | 2.4, 2.4-2, `docs/world_guide.md` (12 m 기준 갱신) |
| B2 | 센서 모듈: 정면 카메라, 옵티컬 플로우, 고도센서 | 완료 | 2.5, 2.8~2.11. GUI 프레임 주기 재측정 미완 (7절) |
| B3 | 다중 드론 스폰 자동화 (2대, 가능하면 16대) | 부분 | 2.14, 10.1. 스폰 자동화 완료. 3대 이상 미시험 |
| B4 | 연결 층 ③: 실제 위치 읽기 | 완료 | 2.7 |
| B5 | CSV 로거 (시간, 실제, 추정, 분산, 시야) | 부분 | 2.12, 2.18. 시간·실제·시야 완료. 추정·분산 칸은 stub |
| B6 | 2대 스침 점검 (시야 목록 기록 확인) | 완료 | 2.15 |
| B7 | 매뉴얼 (월드·센서, 로그 저장, 점검 결과) | 완료 (초안) | `docs/B_manual.md`, 2.16. B 영역만 |

체크리스트 B 항목:

| 체크 항목 | 상태 | 근거 |
|---|---|---|
| 두 번째 PC에서 설치 재현 (B) | 미완 | B0과 같음 |
| 실내 방(벽 + 문) + 구조물 설치 가이드 (B) | 완료 | B1 |
| 정면 카메라 (수평 FOV 설정 가능) (B) | 완료 | 2.5, 2.17, `scripts/set_fov.sh` |
| 옵티컬 플로우 (B 모델 + A 파라미터) | 부분 | 시뮬 지원 여부는 2.9로 확인. B 모델·브리지·GPS OFF 비행 확인(2.11, 2.13, 2.19). A 파라미터 합의 남음 [A 담당 내용 입력] |
| 고도센서 (B) | 완료 | 2.8, 2.11 (0.1~5 m, DISTANCE_SENSOR) |
| 실제 위치 읽기 (B) | 완료 | 2.7 |
| CSV: 시간, 실제 위치, 추정 위치, 분산, 시야 목록 (B) | 부분 | 2.12, 2.18. 추정·분산 칸은 A의 `get_estimate` 연결 필요 |
| 2대 스침 → 시야 목록 기록 확인 (B) | 완료 | 2.15 |
| 매뉴얼 (B 부분) | 완료 (초안) | B7 |

A 담당 항목 (B 리포트에서는 확인만 한다):

| 항목 | 상태 |
|---|---|
| GPS 끄기, 1대 이륙·착륙, 2대 동시 | [A 담당 내용 입력] (B 시험에서 GPS OFF 비행은 확인됨, 4.1~4.2) |
| 추정 위치·속도·분산 읽기, 속도 명령 | [A 담당 내용 입력] |
| 시야 판정 함수 최종본 | [A 담당 내용 입력] (참조 구현 `visibility.py`는 A 가이드 §13 기준으로 B가 작성) |
| 정사각형 시험, 그래프 | [A 담당 내용 입력] |
| A 매뉴얼 | [A 담당 내용 입력] |

마감: 일정표 마감은 2026-10-10이다. 작업 기록 6절은 마감이 지났다고 적었고, 연장 여부는 [미확인]이다.

---

## 6. 발생한 문제와 해결

| 번호 | 증상 (원문) | 원인 | 해결 |
|---|---|---|---|
| 1 | `Arm: Motors: Check frame class and type` | `FRAME_CLASS` 0(미설정). `FRAME_TYPE`은 1 | `param set FRAME_CLASS 1` 후 재시작. `config/frame.parm`을 `--add-param-file`로 준다. 새 실행 폴더는 파라미터가 초기화되므로 필요 |
| 2 | `no JSON sensor message received`, MAVProxy `FTP: request timed out` | `room.sdf`에 Imu 시스템 플러그인이 없어 ArduPilot 플러그인이 센서 값을 보내지 못함 | `gz-sim-imu-system` 추가. 순서는 Gazebo → 스폰 → SITL |
| 3 | 카메라 영상에서 벽과 바닥이 검게 나옴 (`down_camera` 픽셀 전부 0) | 재질 없는 벽 visual과 `<pbr>`만 있는 바닥이 카메라에서 검게 렌더링됨. `<scene>` 제거는 효과 없었음 | 벽에 `<material>`(ambient, diffuse 0.7), 바닥에 `<ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>` 추가 후 재확인 |
| 4 | `optical_flow_send() takes from 9 to 10 positional arguments but 11 were given` | 기본 MAVLink 1에는 확장 필드 `flow_rate_x/y`가 없음 | 코드 맨 위에서 `os.environ["MAVLINK20"] = "1"` 후 pymavlink import |
| 5 | MAVProxy 경로에서 브리지 송신이 SITL에 도달하지 않음 | pymavlink `udpin`은 상대 주소를 수신 패킷에서 배우므로 수신하지 않으면 송신이 버려짐 | 브리지 주 루프를 `recv_match` 수신 루프로 바꿈 |
| 6 | `csv_logger.py` 종료 시 `Segmentation fault (core dumped)` | CSV 기록 후 인터프리터 정리 중 gz.transport 구독 정리 문제로 보임 | CSV를 다 쓴 뒤 `os._exit(0)`으로 종료 |
| 7 | 시험 종료 후 `gz sim` 서버가 SIGTERM에 끝나지 않음 | 원인 미확인 (1회 관찰) | `run_pass_check.sh`에서 프로세스 그룹에 `kill -9` 추가 [미확인: 원인] |
| 8 | `EKF3 IMU0 fusing optical flow` 메시지를 스크립트가 놓침 | 이 메시지는 EKF 초기화 때 한 번만 나와서 붙기 전에 지나감 | 실시간 대기는 EKF 상태 플래그(0x08, 상대 위치)로 하고, 메시지는 끝난 뒤 MAVProxy `mav.tlog`에서 읽음 |
| 9 | `MAV_CMD_NAV_TAKEOFF` `result 4`가 6번 거절됨 | 시동 때 모드가 GUIDED가 아니라 STABILIZE였음 | EKF 준비 후 `set_mode("GUIDED")`를 보내고 `flightmode`가 바뀐 것을 확인한 뒤 시동 |

추가 메모: 정면 카메라 영상 확인(2.5)은 처음에 "정상 표시"로 기록했다. 3절 항목 3의 재질 문제를 찾은 뒤, 재질 수정 후 Image Display로 다시 확인했다(B_manual 5.3). 최종 상태만 유효하다.

---

## 7. 한계와 미확인

| 구분 | 내용 |
|---|---|
| 정확도 해석 | 시뮬레이션 결과다. 바람, 센서 잡음, 모션 블러, 기체 기울기가 없다. 실제 기체 정확도로 해석하지 않는다 |
| 이동 거리 | 직선 이동은 1.5 m 수준이고, 정사각형은 한 변 2 m다. 정지비행은 최대 60 s였다. 장기 드리프트는 측정하지 않았다 |
| 기체 기울기 | 기울기 큰 비행의 광류와 시야 판정은 시험하지 않았다. 정면 카메라 기울기 영상도 미확인 |
| 대수 | 3대 이상 동시 비행은 시험하지 않았다 |
| yaw 변화 | yaw가 바뀌는 비행의 시야 판정은 시험하지 않았다 |
| 추정 칸 | 추정·분산 칸은 stub이다. A의 `get_estimate` 연결 전까지 로그의 추정 값은 없다 |
| 공분산 | 위치 분산은 확보하지 못했다. A 가이드 §11 확인 대기 [A 담당 내용 입력] |
| 오프셋 | 실제 − 추정 offset 기록 담당을 정하지 않았다 [A 담당 내용 입력] |
| EKF 원점 | A 가이드 §10.2의 원점 37.0/127.0은 적용하지 않았다. 기본 원점으로 시험했다 |
| OS | Ubuntu 24.04 (A 가이드 22.04) 불일치 [A 담당 내용 입력] |
| 재현 | 두 번째 PC에서 재현하지 않았다 (B0 미완) |
| 프레임 주기 | 헤드리스 소프트웨어 렌더링에서 down_camera가 약 18 Hz였다(설정 30 Hz). GUI 환경 재측정은 [미확인] |
| 공 충돌 | 12 m 방의 공 충돌 결과는 기록에 없다 [미확인] |
| 원자료 | 2대 비행과 시간 정렬의 원자료는 `/tmp`에 있었고 저장소에는 없다 [미확인] |
| 기체 | 직접 제작 드론 사양은 확보하지 못했다. 공식 Iris(`baseline_iris`)를 썼다 |

---

## 8. 남은 작업과 통합 계획

| 구분 | 항목 | 담당 |
|---|---|---|
| A 연동 | `get_estimate`를 로거에 연결하고 추정·분산 칸 검증 | A + B [A 담당 내용 입력] |
| A 연동 | 실제 − 추정 offset 기록 담당과 방식 합의 | A + B [A 담당 내용 입력] |
| A 연동 | 위치 공분산 확보 여부 확인 (A 가이드 §11) | A [A 담당 내용 입력] |
| A 연동 | EKF 파라미터 합의 (`flow_test.parm`과 A 가이드 §10.2 차이) | A + B [A 담당 내용 입력] |
| A 연동 | 카메라 장착 명세(`docs/front_camera_spec.md`) 전달, 본체 SDF에 fixed joint 장착 | A [A 담당 내용 입력] |
| A 연동 | `FRAME_CLASS 1`을 A의 파라미터 파일에 반영 | A [A 담당 내용 입력] |
| A 연동 | OS 버전 통일 여부, ArduPilot·플러그인 커밋 비교 | A + B [A 담당 내용 입력] |
| 결정 | 마감 연장 여부 | 사용자 [미확인] |
| 결정 | 직접 제작 드론 사양 확보 담당과 시점 | 사용자 [미확인] |
| 결정 | 정사각형 시험과 2대 GPS OFF 시험의 소유 | A + B [A 담당 내용 입력] |
| B 작업 | 두 번째 PC에서 설치 재현 (B0) | B |
| B 작업 | GUI 환경에서 down_camera 프레임 주기 재측정 | B |
| B 작업 | 3대 이상 동시 비행 시험 | B |
| B 작업 | yaw가 바뀌는 비행의 시야 판정 시험 | B |
| B 작업 | `python-requirements.txt`에 OpenCV 추가 | B |
| B 작업 | 2대 비행과 시간 정렬 원자료를 저장소에 보관 | B |

통합 계획: A의 `get_estimate`와 `send_velocity`를 붙이고, 로거의 추정 칸을 채운 뒤 시야 판정을 A 최종본으로 교체한다. 그다음 2대 스침과 정사각형 시험을 같은 run ID로 다시 실행한다 [A 담당 내용 입력].

GitHub: 업로드 대상은 `https://github.com/Multi-Bus/capstone_design.git`이다. `main` 브랜치를 올렸다(2026-10-12). A 산출물은 같은 저장소에 커밋한다.

---

## 9. 부록

### 9.1 파일 목록

저장소 루트: WSL `~/drone_project` (Windows 경로 `\\wsl$\Ubuntu-24.04\home\subniin\drone_project`)

| 경로 | 용도 |
|---|---|
| `worlds/room.sdf` | 방 월드 (12 × 12 × 5, 문, 재질, Imu) |
| `models/floor/` | 텍스처 바닥 모델 |
| `scripts/make_floor_texture.py` | 바닥 텍스처 생성 (시드 42) |
| `docs/world_guide.md` | 구조물 설치 가이드 |
| `sensors/front_camera/` | 정면 카메라 모델 (640 × 480, 90°) |
| `sensors/down_camera/` | 하향 카메라 모델 (320 × 240, 60°) |
| `sensors/down_range/` | 하향 거리센서 모델 (0.1~5 m) |
| `sensors/iris_flow_test/` | 하향 센서를 붙인 B 시험용 기체 (A 본체 아님) |
| `scripts/set_fov.sh` | 정면 카메라 FOV 변경 |
| `docs/front_camera_spec.md` | A에게 줄 카메라 장착 명세 |
| `scripts/get_truth.py` | 실제 위치 읽기 |
| `scripts/flow_probe.py` | 광류 알고리즘 단독 시험 |
| `scripts/sensor_bridge.py` | 광류·거리 MAVLink 브리지 |
| `config/flow_test.parm` | GPS OFF 시험 파라미터 (B 시험용) |
| `config/frame.parm` | `FRAME_CLASS 1` |
| `scripts/flow_flight_check.py` | 직접 TCP GPS OFF 비행 점검 |
| `scripts/run_flow_check.sh` | 격리 실행, 직접 TCP 경로 |
| `scripts/run_flow_check_mavproxy.sh` | 격리 실행, MAVProxy 경로 (A 가이드 방식), 정확도 시험 |
| `scripts/flow_accuracy_check.py` | 정지비행·정사각형 정확도 측정 |
| `results/flow_accuracy_20261011_200631.csv` | 정확도 시험 1회차 |
| `results/flow_accuracy_20261011_201418.csv` | 정확도 시험 2회차 |
| `scripts/spawn_drones.py` | 다중 드론 스폰 (`drone_{i}`, 포트 9002 + 10i) |
| `docs/multi_drone.md` | 다중 드론 규칙 |
| `scripts/csv_logger.py` | CSV 로거 (21컬럼, 10 Hz) |
| `scripts/visibility.py` | 시야 판정 참조 구현 (A 최종본으로 교체 예정) |
| `scripts/test_visibility.py` | 시야 판정 자체 점검 |
| `logs/sample_logger_check.csv` | 로거 샘플 (2대, 140 시각) |
| `scripts/pass_check.py` | 2대 스침 비행 제어 |
| `scripts/run_pass_check.sh` | 2대 스침 시험 일괄 실행 |
| `logs/pass_check_sample.csv` | 스침 샘플 (드론 2, 3) |
| `scripts/time_sync_check.py` | Gazebo·SITL 시간 정렬 측정 |
| `scripts/two_flow_check.py`, `scripts/run_two_flow_check.sh` | 2대 GPS OFF 광류 비행 점검 |
| `docs/B_manual.md` | B 매뉴얼 초안 |
| `docs/versions.txt` | 버전 기록 |
| `docs/python-requirements.txt` | Python 패키지 목록 (OpenCV 없음) |
| `env.sh` | 환경변수 (git 제외) |

### 9.2 실행 명령 요약 (3개 점검 스크립트)

모두 `~/drone_project`에서, `env.sh`를 적용한 뒤 실행한다. 각 스크립트는 격리된 헤드리스 Gazebo를 따로 띄우며, 사용자가 켠 Gazebo와 섞이지 않는다.

```bash
cd ~/drone_project && source env.sh

# 1. 광류 GPS OFF 정지·정사각형 정확도 (MAVProxy 경로, 인자 없으면 flow_accuracy_check.py 실행)
scripts/run_flow_check_mavproxy.sh

# 2. 2대 GPS OFF 광류 비행 (A 가이드 §12 연결, 드론 0·1)
bash scripts/run_two_flow_check.sh

# 3. 2대 스침 시험 (GPS ON, 로거 70 s, 결과 CSV: $OUT, 기본 /tmp/pass_check/pass_check.csv)
scripts/run_pass_check.sh
```

환경변수: 1은 `PART`, `RUN`, `INST`, `LOGD`. 2는 `PART`(기본 agentM), `RUN`, `LOGD`. 3은 `IDS`, `PART`, `RUN`, `LOGD`, `OUT`, `DUR`.

### 9.3 커밋 목록 (작업 기록 0절 기준)

이 목록은 작업 기록의 기록이다. 이 초안 작성에서는 git을 실행하지 않았다.

| 커밋 | 내용 |
|---|---|
| `1323664`, `3d1f0ad`, `0b77738` | 카메라 장착 명세, `get_truth`, 12 m 방 월드 |
| `b581416`, `de3b27b`, `25815da`, `321d5f4` | 하향 센서, 벽·바닥 재질 수정, 광류 가능성 시험, 센서 브리지 |
| `5dc32a8`, `d33a240` | CSV 로거와 시야 판정, 매뉴얼 초안과 정면 FOV 90° 복구 |
| `1f9fa6b`, `60db88e` | 다중 드론 스폰, MAVProxy 경로 광류 정확도 측정 |
| `bb6e054`, `cee9291` | 매뉴얼 7.5·10.1·10.2, 2대 스침 시험과 로거 종료 segfault 수정 |
| `738674f`, `9d07171`, `31c6799` | 시간 정렬 점검과 `--no-data-timeout`, 2대 GPS OFF 점검, 매뉴얼 10.4·10.5 |

### 9.4 자료 간 차이

| 번호 | 내용 |
|---|---|
| 1 | 시뮬레이션 실시간 배율: 작업 기록 2.18과 B_manual 10.5는 약 0.4~0.55, B_manual 7.5 재검증은 약 0.81. 측정 시점이 다를 수 있다. 확인 필요 |
| 2 | 정면 카메라 영상: 2.5는 "정상 표시"로 적었고, 3절은 재질 수정 전 영상이 어두웠을 수 있다고 적었다. 최종 상태만 유효하다 |
| 3 | 다중 스폰 검증 위치: `docs/multi_drone.md`는 드론 1, 2를 (3, -2), (-4, 3)에 두었다고 적었다. 작업 기록 2.14와 B_manual 10.1은 드론 3, 4 스폰과 드론 1, 2 비행만 적었고 위치는 적지 않았다 |
| 4 | 정사각형 시험 소유: 일정표 A7과 A 가이드 §14는 A 담당이다. 작업 기록 11번은 B 작업으로 적었다 |
| 5 | 2대 GPS OFF 시험: A 가이드 §12와 일정표 A4 항목인데, B 기록에 포함되어 있다 |
| 6 | 파라미터: `config/flow_test.parm`에 A 가이드 §10.2의 `EK3_ENABLE`, `AHRS_EKF_TYPE`, `EK3_SRC_OPTIONS`, `GPS2_TYPE`가 없다 |
| 7 | Python 패키지: `docs/python-requirements.txt`에 OpenCV가 없으나 광류 브리지가 쓴다 |
| 8 | 플러그인 경로: A 가이드 `~/gz_ws/src/ardupilot_gazebo`와 B `~/ardupilot_gazebo`가 다르다 |
| 9 | 마감: 일정표 2026-10-10, 작업 기록은 지났다고 적었다. 연장 여부는 기록에 없다 |
