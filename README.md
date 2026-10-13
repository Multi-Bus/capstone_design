# capstone_design: Gazebo + ArduPilot SITL 시뮬레이션 환경

다중 드론 충돌 회피와 편대 제어를 시험하기 위한 시뮬레이션 환경이다. 1차 범위는 환경 구성만이다. 회피 알고리즘, 실제 하드웨어, 통신 모델, 학습은 포함하지 않는다.

- 시뮬레이터: Gazebo Harmonic (gz sim 8.15.0)
- 비행제어: ArduPilot SITL (ArduCopter 4.6.0-beta1) + `ardupilot_gazebo` 플러그인
- 환경: WSL2 Ubuntu 24.04. 자세한 버전은 `docs/versions.txt`
- 기준 기체: 공식 Iris (`baseline_iris`). 직접 제작 드론 사양은 아직 반영하지 않았다.
- 좌표계와 단위: ENU, m, m/s, rad, s. 로그는 10 Hz, 시뮬레이션 시간 기준

## 현재 상태

이 저장소에는 B 담당 산출물과 A의 Ubuntu 22.04용 통합 실행기, GPS OFF 설정, 추정·속도 API, 자동 비행 검사와 그래프 생성기가 들어 있다. 위 버전 목록은 B 환경 기준이며, A 실행 방법과 제한은 [A 매뉴얼](docs/A_manual.md)을 따른다. 실제 제작 기체 사양과 위치 공분산 확보는 아직 미완료다.

| 항목 | 상태 |
|---|---|
| 12 m × 12 m × 5 m 방 월드 (북쪽 벽 문, 텍스처 바닥) | 완료 |
| 센서 3종: `front_camera`, `down_camera`, `down_range` | 완료 |
| 실제 위치 `get_truth` | 완료 |
| 광류 계산과 MAVLink 브리지, GPS OFF 비행 | 완료 (1대, 2대) |
| 다중 드론 스폰 (`drone_{i}`) | 완료 |
| CSV 로거 (A 가이드 §4 컬럼 21개) | A 실행기의 integrated.csv에 추정값 연결. 단독 csv_logger 기본값은 stub 유지 |
| Gazebo와 SITL 시간 정렬 확인 | 완료 |
| 2대 스침 시험 | 완료 |
| 추정 위치·속도 API, 정사각형 시험 | A 실행기에 통합. 실제 위치 정확도는 결과 지표를 별도 확인 |

모든 정확도 수치는 바람과 센서 잡음이 없는 시뮬레이션 결과다. 실제 기체 성능이 아니다.

## 폴더 구조

| 경로 | 내용 |
|---|---|
| `worlds/` | `room.sdf` 방 월드 |
| `models/` | 바닥 모델 (`floor`). `models/generated/`는 스폰 스크립트가 만드는 파일이라 git 제외 |
| `sensors/` | 센서 모델 (`front_camera`, `down_camera`, `down_range`)과 시험용 기체 래퍼 `iris_flow_test` |
| `scripts/` | `get_truth.py`, `sensor_bridge.py`, `spawn_drones.py`, `csv_logger.py`, `visibility.py`, 점검 스크립트 |
| `config/` | ArduPilot 파라미터 (`frame.parm`, `flow_test.parm`) |
| `docs/` | 매뉴얼, A 인수인계, 최종 리포트 초안, 작업 기록 |
| `logs/`, `results/` | 샘플 CSV와 측정 결과 |

## 시작하기

1. Gazebo Harmonic, ArduPilot, `ardupilot_gazebo`를 설치한다. 절차와 실제 환경은 `docs/B_manual.md` 2장.
2. 저장소를 `~/drone_project`에 둔다. 스크립트는 이 경로와 `~/venv-ardupilot`, `~/ardupilot`, `~/ardupilot_gazebo`를 쓴다.
3. 환경 변수 파일을 만든다. `env.sh`는 git 제외 대상이다.

```bash
cp env.sh.example env.sh      # 경로가 다르면 고친다
source ~/drone_project/env.sh # 터미널마다 한 번
```

4. Python 패키지는 `docs/python-requirements.txt`를 참고한다. 센서 브리지에는 OpenCV가 필요하다.

## 점검 스크립트

모두 격리된 Gazebo 파티션(`GZ_PARTITION`)에서 헤드리스로 돌고, 끝나면 자기가 띄운 프로세스 그룹만 종료한다. `pkill`은 쓰지 않는다. 실행 권한이 없으면 `chmod +x scripts/*.sh`.

| 스크립트 | 확인하는 것 |
|---|---|
| `scripts/run_flow_check_mavproxy.sh` | 1대 GPS OFF 광류 비행. 정지, A 가이드 §14 정사각형 정확도 |
| `scripts/run_two_flow_check.sh` | 2대 GPS OFF 광류 비행. 드론별 명령 분리 |
| `scripts/run_pass_check.sh` | 2대 스침 (GPS ON). CSV의 `visible_ids` 전환 |

```bash
cd ~/drone_project
PART=check1 RUN=check1 scripts/run_flow_check_mavproxy.sh
PART=check2 RUN=check2 bash scripts/run_two_flow_check.sh
PART=check3 RUN=check3 scripts/run_pass_check.sh
```

## 문서

| 문서 | 내용 |
|---|---|
| `docs/B_manual.md` | B 영역 매뉴얼: 설치, 월드, 센서, `get_truth`, 광류 브리지, 격리 시험, 문제 해결, 다중 드론, 로거, 시험 결과 |
| `docs/A_handoff.md` | A에게 전달하는 문서: B 산출물, 파라미터 검토 항목, 요청 사항, 재현 방법 |
| `docs/B_final_report_draft.md` | 최종 리포트 초안 (B 내용만) |
| `docs/B_작업기록.md` | B 작업 기록 (진행 과정, 문제와 해결) |
| `docs/world_guide.md`, `docs/front_camera_spec.md`, `docs/multi_drone.md` | 월드 구조물 추가, 카메라 장착 명세, 다중 드론 규칙 |

## 알려진 한계

- 3대 이상 동시 비행, 기체 기울기가 큰 비행, yaw가 바뀌는 비행의 시야 판정은 시험하지 않았다.
- 위치 공분산 확보는 미완료다. A 실행기는 실제−추정 offset과 EKF 원점 응답을 기록한다.
- B는 Ubuntu 24.04, A 통합기는 Ubuntu 22.04에서 시험했다. Python 경로와 ArduPilot 커밋은 서로 다르므로 각 매뉴얼의 환경을 따른다.
