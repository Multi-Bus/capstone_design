# B 담당 작업 기록 (Gazebo · 센서 · 로그)

이 문서는 B 담당이 진행한 작업의 내용과 과정을 기록한다. 지금까지의 작업을 정리했고, 앞으로의 작업은 맨 아래 "작업 기록 양식"에 따라 계속 추가한다.

- 프로젝트: Gazebo + ArduPilot SITL 시뮬레이션 환경 구성
- 담당: B (Gazebo 월드, 센서, 다중 스폰, 실제 위치 제공, CSV 로거, 스침 시험, B 매뉴얼)
- 기준 문서: `회의록/가제보_환경_구성_일정표.md`, A 담당 작업 가이드(`Gazebo_A담당_작업가이드.md`)
- 작성 시점: 2026-10-11 (이후 항목은 작업할 때마다 날짜와 함께 추가)
- 최종 갱신: 2026-10-11 (B 작업 전체 정리, 2.17~2.19 추가)
- 표기: [미확인]은 아직 확인하지 못한 항목, [결과 미수신]은 명령을 안내했으나 실행 결과를 받지 못한 항목

## 0. 한눈에 보기 (2026-10-11 정리)

A 없이 할 수 있는 B 작업은 모두 끝났다. 남은 일은 A의 산출물이나 사용자 결정이 필요하다.

| 영역 | 결과 | 기록 |
|---|---|---|
| 방 월드 | 12m × 12m × 5m, 북쪽 벽 문, 텍스처 바닥. 벽·바닥에 ambient/diffuse를 넣어야 카메라에 보인다 | 2.4, 2.4-2, 2.8 |
| 센서 3종 | `front_camera`(640×480, 90°), `down_camera`(320×240, 60°), `down_range`(0.1~5m). 정면은 기체 +x를 보고 영상이 바르다 | 2.5, 2.8, 2.17 |
| 실제 위치 | `get_truth`: ENU 위치·yaw, 카메라 위치·yaw, 시뮬 시각 | 2.7 |
| 광류 | `sensor_bridge.py`가 영상에서 광류를 계산해 MAVLink로 주입. GPS OFF 정사각형 RMSE xy 0.01~0.02m (바람·잡음 없는 시뮬레이션) | 2.9~2.11, 2.13 |
| 다중 드론 | `spawn_drones.py`: `drone_{i}`, 포트 9002+10i, 드론별 센서 토픽 | 2.14 |
| 로거 | `csv_logger.py`: A 가이드 §4 컬럼 21개, 10Hz 시뮬 시간. 추정 칸은 stub. 일시정지는 `--no-data-timeout` | 2.12, 2.18 |
| 시간 정렬 | SITL `time_boot_ms`가 Gazebo 시뮬 시각과 10ms 이내로 같다. 일시정지, SITL 재시작에서도 유지 | 2.18 |
| 2대 스침 | 평행 차선 반대 방향 3회 통과. `visible_ids`가 `[상대]` → `[]`로 한 번 바뀜, 전환 위치가 기하 계산과 일치 | 2.15 |
| 2대 GPS OFF | A 가이드 §12 연결(14550/14560, 15550/15560)에서 2회 통과. 명령받은 드론만 움직임 | 2.19 |
| 매뉴얼 | `docs/B_manual.md` (10.1~10.5절 포함) | 2.16 |

커밋 (WSL `~/drone_project`, 원격 저장소 없음):

| 커밋 | 내용 |
|---|---|
| `1323664`, `3d1f0ad`, `0b77738` | 카메라 장착 명세, `get_truth`, 12m 방 월드 |
| `b581416`, `de3b27b`, `25815da`, `321d5f4` | 하향 센서, 벽·바닥 재질 수정, 광류 가능성 시험, 센서 브리지 |
| `5dc32a8`, `d33a240` | CSV 로거와 시야 판정, 매뉴얼 초안과 정면 FOV 90° 복구 |
| `1f9fa6b`, `60db88e` | 다중 드론 스폰, MAVProxy 경로 광류 정확도 측정 |
| `bb6e054`, `cee9291` | 매뉴얼 7.5·10.1·10.2, 2대 스침 시험과 로거 종료 segfault 수정 |
| `738674f`, `9d07171`, `31c6799` | 시간 정렬 점검과 `--no-data-timeout`, 2대 GPS OFF 점검, 매뉴얼 10.4·10.5 |

검증 방식: 서브 에이전트(Sonnet, Haiku)가 작성·실행하고, 메인이 원본 로그와 스크립트를 읽거나 직접 다시 실행해 확인했다. 에이전트 결과를 그대로 받아들이지 않은 사례는 로거 일시정지 시험(에이전트 0행, 메인 재실행으로 정상 확인), 스침 시험(메인 2회 재실행에서 로거 종료 segfault 발견·수정)이다.

남은 일: 7장.

## 1. 현황 요약

### 환경 구성

| # | 작업 | 상태 |
|---|---|---|
| 1 | WSL2 + Ubuntu 설치 | 완료. 실제 설치된 배포판은 **Ubuntu 24.04** (2.2 참고) |
| 2 | ArduPilot 설치, SITL 실행 확인 | 완료 |
| 3 | Gazebo Harmonic 설치, `gz sim` GUI 확인 | 완료 |
| 4 | ardupilot_gazebo 플러그인 빌드, 환경변수 | 완료 (`libArduPilotPlugin.so` 확인) |
| 5 | iris 이륙·착륙 통합 확인 | 완료 |
| 6 | 버전 기록 | 완료 (`docs/versions.txt`: Ubuntu 24.04.5, gz sim 8.15.0, ArduPilot d67add0, ardupilot_gazebo 082a0fe) |
| 7 | 작업 저장소 구성, 첫 커밋 | 완료 |
| 정렬 | A 가이드에 맞춘 폴더·환경·실행 명령 정렬 | 완료 (2.6) |

### 본 작업

| # | 작업 | 상태 |
|---|---|---|
| 8 | 실내 방 월드(벽 + 문) + 구조물 설치 가이드 | 완료 (8m 버전). 이후 12m × 12m × 5m + 바닥 텍스처로 갱신(8-2)하고 방 안 iris 이륙·착륙 확인 |
| 9 | 정면 카메라 모듈 | 단독 시험 완료 (토픽, Image Display, FOV 변경). 고정 조인트(2.14), 정면 방향(2.17) 확인. 남은 일: A에게 장착 명세 전달 |
| 10 | 시뮬레이터 실제 위치 읽기 (`get_truth`) | 완료 (2.7 참고) |
| 11 | 하향 센서(거리센서, 광류용 카메라) + 광류 계산 + MAVLink 브리지 | 완료. 직접 TCP 연결(2.11)과 MAVProxy 경로(2.13) 모두 GPS OFF 비행 확인, 정사각형 정확도 측정, 2대 GPS OFF 비행(2.19). 남은 일: A와 EKF 설정 합의 |
| 12 | 다중 드론 스폰 | 완료 (2.14) |
| 13 | 로거 뼈대 | 완료 (2.12) |
| 14 | 로거 완성 | 시간 정렬 검증, 일시정지 대응 완료(2.18). 추정 칸은 A의 `get_estimate`를 받아야 채움. 지금은 stub |
| 15 | 2대 스침 점검 | 완료 (2.15) |
| 16 | B 매뉴얼 | 초안 완료 (`docs/B_manual.md`, 2.16) |
| 17 | 통합, 최종 리포트 | 시작 전 (A 산출물 필요) |

## 2. 진행 과정

### 2.1 작업 분담과 범위 확정
- 1차 범위: Gazebo + ArduPilot SITL 시뮬레이션 환경만 구성한다. 실제 하드웨어, 경로 계획, 회피, 통신 모델, 학습은 제외한다.
- 일정표의 담당을 B(Gazebo·센서·로그)로 정했다.
- 파일 소유 규칙: 본체 SDF와 ArduPilot 파라미터는 A만 수정, 월드와 센서 모듈은 B만 수정한다. 센서는 별도 모델로 제공하고 A가 본체 장착 링크에 붙인다.
- A 산출물에 의존하는 작업과 의존하지 않는 작업을 나눴다.
  - A 없이 가능: 월드, 카메라 모듈(단독), 실제 위치 읽기, 로거 뼈대
  - A 필요: 광류·고도 동작 확인(A 파라미터), 다중 스폰(A 포트 규칙, 모델), 로거 완성(A의 `get_estimate`, 시야 판정), 스침 점검(A의 2대 비행)

### 2.2 환경 구성 (WSL2 + Ubuntu 24.04)

> 정정 (2026-10-11): 처음에는 Ubuntu 22.04로 안내하고 기록했으나, `wsl -l -v`로 확인한 실제 배포판은 `Ubuntu-24.04`(24.04.5 LTS)다. A 가이드는 22.04 기준이라 두 PC의 OS 버전이 다르다. A와 버전을 맞출지, 차이를 기록하고 갈지 정해야 한다. 아래 명령은 22.04 기준으로 안내한 것이며 24.04에서 `~/venv-ardupilot`이 자동 생성된 점이 다르다.

구성 값:

| 항목 | 값 |
|---|---|
| 개발 환경 | WSL2, Ubuntu 24.04 (A 가이드는 22.04. 확인 필요) |
| 시뮬레이터 | Gazebo Harmonic |
| 비행제어기 | ArduCopter SITL |
| 연동 | ArduPilot/ardupilot_gazebo 플러그인 (JSON 모델) |
| 기준 기체 | 공식 Iris |

수행 순서와 명령:

1. WSL2 + Ubuntu 설치 (Windows PowerShell, 관리자 권한). 아래는 22.04로 안내한 명령이고 실제 배포판은 24.04다.
   ```powershell
   wsl --install -d Ubuntu-22.04
   wsl --update
   ```
2. 기본 패키지 (Ubuntu)
   ```bash
   sudo apt update && sudo apt upgrade -y
   sudo apt install -y git curl lsb-release gnupg
   ```
3. ArduPilot 설치 (`~/ardupilot`, WSL 홈 안에 설치. `/mnt/c`는 느림)
   ```bash
   cd ~
   git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
   cd ardupilot
   Tools/environment_install/install-prereqs-ubuntu.sh -y
   . ~/.profile
   ```
   - Ubuntu 22.04에서는 설치 스크립트가 venv를 만들지 않고 `pip --user`로 설치한다. Ubuntu 24.04에서는 스크립트가 `~/venv-ardupilot`을 자동으로 만든다 (실제로 생성됨). 어느 쪽이든 직접 venv를 만들 필요는 없다.
4. Gazebo Harmonic 설치 후 `gz sim -v4 -r shapes.sdf`로 GUI 확인
5. ardupilot_gazebo 설치 (`~/ardupilot_gazebo`)
   ```bash
   sudo apt install libgz-sim8-dev rapidjson-dev
   sudo apt install libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
   cd ~
   git clone https://github.com/ArduPilot/ardupilot_gazebo
   cd ardupilot_gazebo && mkdir build && cd build
   cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
   make -j4
   ```
   - 빌드 확인: `build/`에 `libArduPilotPlugin.so`, `libCameraZoomPlugin.so`, `libGstCameraPlugin.so`, `libParachutePlugin.so`가 생성됨.
   - 환경변수: `GZ_SIM_SYSTEM_PLUGIN_PATH`는 `~/ardupilot_gazebo/build`, `GZ_SIM_RESOURCE_PATH`는 `~/ardupilot_gazebo/models`와 `~/ardupilot_gazebo/worlds`를 가리킨다.
6. 통합 확인 (터미널 2개)
   ```bash
   gz sim -v4 -r iris_runway.sdf
   sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
   ```
   MAVProxy에서 `mode guided` → `arm throttle` → `takeoff 5` → `land`. iris 이륙과 착륙을 확인했다.

### 2.3 작업 저장소 구성
- 처음에는 `~/capstone_sim`에 만들고 git 초기화, 폴더(`worlds`, `models`, `sensors`, `params`, `scripts`, `logs`, `docs`), `.gitignore`(`.venv/`, `eeprom.bin`, `mav.parm`, `mav.tlog*`, `logs/*.BIN`, `terrain/` 등), Python venv(`pymavlink`, `matplotlib`)를 구성했다.
- 첫 커밋 완료 (`init: folder layout and room world`).
- `git commit`에서 `not a git repository`가 한 번 났다. 작업 폴더로 이동하지 않았거나 `git init`이 실행되지 않은 것이 원인이었고, `cd ~/capstone_sim && git init` 후 해결했다.

### 2.4 방 월드 (8번)
- 파일: `worlds/room.sdf` (8m × 8m × 3m, 벽 두께 0.2m, 지붕 없음)
- 북쪽 벽(y=+4)에 문(폭 1m, 높이 2m). 벽을 box 3개(왼쪽, 오른쪽, 문 위)로 나눠서 구멍을 만들었다.
- 시스템 플러그인: Physics, UserCommands, SceneBroadcaster, Sensors(ogre2). 이후 **Imu 추가**(아래 3절 참고).
- 확인한 것
  - `gz sim`으로 월드 열기
  - 공 두 개 충돌 확인: `ball1`은 바닥에, `ball2`는 동쪽 벽 위(높이 3m)에 멈춤
  - 방 안에서 iris 이륙 (`takeoff 2`)
- 구조물 설치 가이드: `docs/world_guide.md`
- 참고: Windows 프로젝트의 `sim/worlds/room.sdf`는 시작용 복사본이라 Imu 수정 전 8m 버전이다. WSL의 저장소 파일이 기준이다.

### 2.4-2 방 월드 갱신 (8-2, A 가이드 기준)
- 목적: A 가이드의 월드 목표(바닥 12m × 12m, 천장 5m)와 광류용 텍스처 바닥 반영
- 변경 내용 (`worlds/room.sdf` 전체 재작성)
  - 크기 12m × 12m × 5m, 벽 중심 ±6m, 높이 5m
  - 문: 북쪽 벽(y=+6) 가운데, x -0.5~0.5, 높이 2m. 벽 조각은 왼쪽 5.5m, 오른쪽 5.5m, 문 위 1m × 3m
  - 바닥: `models/floor` 모델을 `model://floor`로 include. 12.4m × 12.4m 평면에 랜덤 흑백 무늬 텍스처(PBR `albedo_map`)
  - 텍스처 생성: `scripts/make_floor_texture.py` (1024 × 1024, 8px 칸, 시드 42 고정). 12.4m에 펼치면 칸 하나가 약 9.7cm
  - 단색 바닥은 영상 변화가 없어서 광류를 계산할 수 없으므로 텍스처를 넣었다.
  - Imu 플러그인 포함, `scene`(ambient 0.6, 배경색) 추가, 그림자 끔(드론 그림자가 광류에 섞이는 것을 막기 위한 초기 설정)
- 확인: 방 안 iris 스폰 (-2, 0), `FRAME_CLASS` 설정 후 `takeoff 2` → `mode land` 이륙·착륙 성공
- [결과 미수신]: 바닥 무늬 육안 확인, 공 두 개 충돌 (`ball2`는 `6 0 7` 기준) 결과
- 소유: `worlds/room.sdf`와 `models/floor`는 B가 수정한다.

### 2.5 정면 카메라 모듈 (9번)
- 위치: `sensors/front_camera/` (`model.config`, `model.sdf`)
- 센서 타입 `camera`, 640×480, 15Hz, 토픽 `front_camera/image`
- 수평 FOV는 `horizontal_fov`(라디안) 한 줄로 설정한다. 기본 1.5708(90°)이며 A 가이드의 기준과 일치한다.
- `scripts/set_fov.sh <도>`로 FOV를 도 단위로 바꾼다. 출력 예: `horizontal_fov = 60 deg (1.0472 rad)`
- 링크 +x 방향이 정면이다. 기본 iris 몸체의 앞쪽도 +x라서 정면이 된다. 스폰한 드론에서 영상으로 확인함(2.17).
- 단독 시험 (2026-10-11): 스폰 위치 `0 -5 1.5`, yaw 1.5708(북쪽을 봄)
  - 토픽 확인: `gz topic -e -t /front_camera/image -n 1`에서 `width: 640`, `height: 480` 확인
  - Gazebo의 Image Display(오른쪽 위 `⋮` 메뉴)에서 `/front_camera/image`를 선택해 영상 확인. 영상이 정상 표시됨 → WSL2에서 ogre2 카메라 렌더링이 동작함
  - FOV 변경: `set_fov.sh 60` 후 모델을 삭제하고 다시 스폰해서 화면 폭이 달라지는 것 확인. `model.sdf`의 값도 반영됨
  - 이미 떠 있는 모델은 FOV 변경을 새로 읽지 않으므로 삭제 후 다시 스폰해야 한다.
- 장착 명세: `docs/front_camera_spec.md` (링크 이름, +x 정면, 권장 장착 위치 (0.1, 0, 0.05) [미확인: 실제 기체 사양으로 조정], 고정 조인트 필요)
- 남은 일
  - A에게 명세를 전달한다. 장착은 A가 본체 SDF에서 한다 (소유 규칙).
  - (해결, 2.14) 고정 조인트: `spawn_drones.py`가 `iris_with_standoffs::base_link`에 fixed 조인트로 붙인다. 별도 래퍼(`sensors/iris_cam_test`)는 만들지 않았다.
  - 장착 위치 상수는 4번(`get_truth`)에서 카메라 위치를 계산할 때 쓴다.

### 2.7 실제 위치 읽기 (10번, `get_truth`)
- 파일: `scripts/get_truth.py`. Gazebo의 `/world/room/dynamic_pose/info`(Pose_V)를 Python 바인딩(`gz.transport13`, `gz.msgs10`)으로 구독한다. 시스템 `python3`로 실행하고 venv는 끈다.
- 반환 값: `time_sim_s`(메시지 헤더의 시뮬레이션 시각), `position_enu_m`, `yaw_enu_rad`, `camera_position_enu_m`, `camera_yaw_enu_rad`, `valid`(마지막 수신 후 0.5초 이내, 호스트 시계 기준)
- 카메라 위치 = 기체 위치 + 장착 오프셋(0.1, 0, 0.05)을 yaw로 회전한 값. roll/pitch는 무시한다. 오프셋은 `docs/front_camera_spec.md`와 같게 유지한다.
- 시험 결과 (모델 `iris_with_ardupilot`, 스폰 (-2, 0), `takeoff 2`)
  - 이륙 후 위치 (-1.99, 0.003, 2.19), yaw 0
  - 착륙 후 z = 0.19. z는 모델 원점(`base_link`)의 높이라서 착륙 시에도 약 0.19m이고, 상승분은 2.19 - 0.19 = 2.0m로 `takeoff 2`와 일치한다.
  - x, y 오차는 1cm 이내. yaw 0은 동쪽을 향한다는 뜻이고 스폰 방향과 일치한다.
- 정해 둘 것: 로그에는 truth z를 그대로 기록하고, 분석 시 착륙 상태의 z(약 0.19)를 빼서 지면 대비 높이로 쓴다. A와 기준을 공유한다.
- 12번(스폰 스크립트)에서 모델 이름을 `drone_{i}`로 맞추면 `Truth.get(drone_id)`가 그대로 동작한다.

### 2.8 하향 센서 모델과 카메라 렌더링 문제 (11번 앞부분, 2026-10-11)
- 파일: `sensors/down_camera`(320×240, 30Hz, 수평 FOV 60°, 토픽 `down_camera/image`), `sensors/down_range`(`gpu_lidar` 레이 1개, 0.1~5m, 30Hz, 토픽 `down_range/scan`). 센서 안의 pose에 pitch 1.5708을 넣어서 본체에 회전 없이 붙이면 아래를 본다.
- 문법 검사: `gz sdf -k` Valid (두 센서 모델과 `room.sdf`. `room.sdf`는 `SDF_PATH`로 `models`를 지정해야 `model://floor`를 찾음)
- 시험 방법: 사용자의 Gazebo와 분리된 구역(`GZ_PARTITION=probe`)에서 서버만(`gz sim -s -r --headless-rendering`) 띄우고, 센서를 스폰한 뒤 Python 바인딩(`gz.msgs10.image_pb2`)으로 프레임을 받아 픽셀 통계를 내고 PNG로 저장해서 확인했다.
- 결과
  - `down_range`: `ranges` 약 2.0001 (센서 z=2m에서 바닥까지). 샘플 1개 설정이 동작함.
  - `down_camera`: 수정 후 영상에 바닥의 랜덤 무늬가 선명하게 보임 (최소 93, 최대 203, 평균 157.8). 바닥 무늬 칸은 2m 높이에서 약 13픽셀.
  - 프레임 수: 5초 동안 약 89프레임(약 18Hz, 헤드리스 소프트웨어 렌더링 환경). 30Hz 설정과 차이가 있어 실제 GUI 환경에서 다시 측정해야 한다.
- 발견한 문제: 처음에는 정면·하향 카메라 영상에서 벽과 바닥이 검게 나왔다 (하향 카메라는 전체 픽셀 값이 0). 원인과 해결은 3절 표 참고.
- 변경: `worlds/room.sdf`의 벽 6개에 `<material>`(ambient, diffuse 0.7) 추가, `models/floor/model.sdf`에 `<ambient>`, `<diffuse>` 추가, `docs/world_guide.md`를 12m 기준으로 갱신
- 커밋: `b581416` (하향 센서), `de3b27b` (재질 수정)
- 이미 켜 둔 Gazebo는 수정 전 월드를 쓰고 있으므로 재시작해야 반영된다.

### 2.9 광류 계산 가능 여부 시험 (11번, 2026-10-11)
- 목적: 하향 카메라 영상만으로 이동량을 정확히 측정할 수 있는지, 알고리즘을 확정하기 전에 검증
- 환경: 시스템 Python(`python3`)에는 OpenCV가 없고 `~/venv-ardupilot`에 numpy 2.5.3, OpenCV 5.0.0, pymavlink가 있다. 이 venv는 시스템 패키지(`gz` 바인딩)도 보이므로 광류 작업은 `~/venv-ardupilot/bin/python`으로 실행한다.
- 방법 (`scripts/flow_probe.py`): `down_camera`를 z=2m에서 `/world/room/set_pose`로 5cm씩 옮기고, 연속 프레임을 `cv2.phaseCorrelate`(Hanning 창)로 비교해 픽셀 이동량을 측정한 뒤 이론값과 비교한다.
  - 이론 이동량 = 이동 거리 × 초점거리 / 높이. 초점거리 = (320/2) / tan(30°) = 277.1px → 5cm 이동 시 6.93px
- 결과
  - 정지 상태 가짜 이동량: 약 0.03px 이하
  - 세계 +x 방향 5cm × 5회: dy = +6.96 ~ +6.98 (이론 6.93), dx ≈ 0
  - 세계 +y 방향 5cm × 5회: dx = +6.97 ~ +7.02 (이론 6.93), dy ≈ 0
  - 오차 약 0.5~1%, 신뢰도 0.87~0.98
- 축 규약 확인: 기체가 앞(+x)으로 움직이면 영상 내용은 아래(+row)로, 왼쪽(+y)으로 움직이면 오른쪽(+col)으로 이동한다.
- 속도 환산: 이동량 [px] / 초점거리 / 시간차 = 각속도 [rad/s]. 정지 오차 0.03px/프레임을 30Hz, 높이 2m에 적용하면 속도 오차 약 0.007 m/s
- 한계 (이번 시험이 확인하지 못한 것): 순간 이동 방식이라 연속 이동·모션 블러 없음, 회전(roll/pitch) 없음, 고도 일정, 평평한 바닥, 기체 진동 없음, 속도 0.3m/s에서 프레임당 이동량 약 1.4px(5cm 시험의 6.9px보다 훨씬 작음)
- ArduPilot 소스 확인 (`libraries/AP_OpticalFlow/AP_OpticalFlow_MAV.cpp`)
  - `FLOW_TYPE` 5 = MAVLink, 10 = SITL(가상 광류)
  - `OPTICAL_FLOW` 메시지의 `flow_rate_x/y`(rad/s, 확장 필드)가 0이 아닌 값으로 한 번이라도 들어오면 이 필드를 쓰고, 아니면 `flow_x/y`(deprecated)를 쓴다. → 브리지는 `flow_rate_x/y`를 보낸다.
  - 드라이버가 AHRS 자이로 평균을 몸체 각속도로 같이 기록하므로, 센서가 보낸 광류에는 회전 성분이 포함돼도 EKF가 보정한다.
  - `quality`는 0~255. `sensor_id`는 메시지 값을 그대로 쓴다.
  - 보정 파라미터: `FLOW_FXSCALER`, `FLOW_FYSCALER`(0.1% 단위), `FLOW_ORIENT_YAW`(센티도), `FLOW_POS_X/Y/Z`
  - 수신 지점: `GCS_Common.cpp`의 `MAVLINK_MSG_ID_OPTICAL_FLOW` 처리. 어느 GCS 채널로 들어와도 처리된다.
- (당시 미확인, 이후 해결) 광류 x/y축과 부호는 2.11에서, MAVProxy 출력 포트(15550) 경유 전달은 2.13에서 확인했다.
- 변경: `scripts/flow_probe.py` 추가

### 2.10 센서 래퍼와 기본 비행 확인 (11번, 2026-10-11)
- `sensors/iris_flow_test`: 기본 iris(`iris_with_ardupilot`)에 `down_camera`, `down_range`를 고정 조인트로 붙인 B 쪽 시험용 래퍼. A의 본체 SDF가 아니다.
  - 구조: `iris_with_ardupilot`(안에 `iris_with_standoffs` 모델, 링크 `base_link`, `imu_link`, `rotor_0~3`)을 `iris`라는 이름으로 include하고, 센서를 `0 0 -0.05` 위치에 include한 뒤 조인트 부모를 `iris::iris_with_standoffs::base_link`로 지정한다.
  - `gz sdf -k` Valid (`gz_frame_id` 경고는 기본 iris 모델에 원래 있던 것)
- 시험 방법: 격리된 헤드리스 Gazebo(`GZ_PARTITION=probe`) + SITL(`--no-mavproxy`, TCP 5760) + pymavlink 스크립트. 사용자의 GUI Gazebo와 섞이지 않는다.
- 결과: 센서를 붙여도 GPS ON 기본 비행은 정상. `takeoff 2` 후 D=-2.19(상승 2.0m + 지면 0.19m)
- 발견한 문제와 해결
  - 이륙 명령 `MAV_CMD_NAV_TAKEOFF`가 `result 4`(실패)로 6번 거절됨. 원인은 시동을 걸 때 모드가 `GUIDED`가 아니라 `STABILIZE`였던 것. EKF가 준비된 뒤에 `set_mode("GUIDED")`를 보내고 `flightmode`가 바뀐 것을 확인한 다음 시동을 걸어야 한다.
  - MAVProxy 없이 pymavlink만 쓰면 `LOCAL_POSITION_NED`가 오지 않는다. `request_data_stream_send`로 스트림을 요청해야 한다.
  - `param_request_read_send(..., index=-1)` 뒤 첫 `PARAM_VALUE`는 다른 파라미터일 수 있다. `param_id`를 비교해서 골라야 한다.
  - 시동까지 `Arm: Accels inconsistent`가 약 1분 반복된 뒤 풀린다. 시동 명령을 재시도하면 된다.
- `--add-param-file`로 `config/frame.parm`을 주면 새 실행 폴더에서도 `FRAME_CLASS`가 1로 시작한다 (확인됨).
- 이 빌드(ArduPilot 4.6.0-beta1)의 파라미터 이름: `GPS1_TYPE`(GPS_TYPE 없음), `RNGFND1_MIN`/`RNGFND1_MAX`(미터 단위, 이전 `_CM` 아님)
- 참고: 지면에서 EKF 원점이 GPS로 설정될 때 LOCAL_POSITION_NED는 NED(N=y, E=x)로 Gazebo 월드 좌표와 일치했다 (스폰 (-2,0) → N=0, E=-2).

### 2.11 센서 브리지와 GPS OFF 광류 비행 (11번, 2026-10-11)
- 파일: `scripts/sensor_bridge.py`, `config/flow_test.parm`, 점검 스크립트 `scripts/flow_flight_check.py` + `scripts/run_flow_check.sh`
- 브리지 동작
  - `/down_camera/image` 연속 프레임을 `cv2.phaseCorrelate`로 비교해 픽셀 이동량 → 광류(rad/s) 변환 (`dx/f/dt`, f = 초점거리 277.1px, dt는 이미지 헤더의 시뮬레이션 시각 차이)
  - `OPTICAL_FLOW`: `flow_rate_x = dx/f/dt`, `flow_rate_y = dy/f/dt`, `quality`는 위상 상관 신뢰도(0.3 미만이면 0), `ground_distance`는 거리센서 값
  - `DISTANCE_SENSOR`: `/down_range/scan`의 거리(cm), orientation 25(아래), 최소 10cm, 최대 500cm
  - 1Hz로 하트비트 송신(ONBOARD_CONTROLLER)
- 축 규약: ArduPilot의 SITL 가상 광류 소스(`AP_OpticalFlow_SITL.cpp`)가 `flowRate.x = -v_y(오른쪽)/h`, `flowRate.y = +v_x(앞)/h`(rad/s)이다. 우리 영상 규약(앞으로 가면 내용이 아래로 이동, 왼쪽으로 가면 오른쪽으로 이동)과 대응하면 변환 없이 `flow_rate_x = dx/f/dt`, `flow_rate_y = dy/f/dt`가 된다. 시험으로 맞음을 확인했다.
- 문제와 해결
  - `optical_flow_send() takes from 9 to 10 positional arguments but 11 were given`: 기본 연결이 MAVLink 1이라 확장 필드(`flow_rate_x/y`)가 없다. 코드 맨 위에서 `os.environ["MAVLINK20"] = "1"`을 설정하고 `pymavlink`를 import하면 해결된다 (pymavlink 2.4.50 v2 dialect에는 필드가 있음).
  - 첫 시험에서는 이 오류로 광류가 한 번도 전송되지 않았다(`flow n=0`). 그런데도 정지 비행에서는 위치 변화가 없어서 광류 효과를 구분할 수 없었다. 그래서 움직임 구간을 추가했다.
- 파라미터 (`config/flow_test.parm`): `FRAME_CLASS 1`, `GPS1_TYPE 0`, `FLOW_TYPE 5`, `RNGFND1_TYPE 10`, `RNGFND1_ORIENT 25`, `RNGFND1_MIN 0.1`, `RNGFND1_MAX 5.0`, `EK3_SRC1_POSXY 0`, `EK3_SRC1_VELXY 5`, `EK3_SRC1_POSZ 2`, `EK3_SRC1_VELZ 0`, `EK3_SRC1_YAW 1`
- 시험 절차 (`run_flow_check.sh`): 격리된 헤드리스 Gazebo에 `iris_flow_test` 스폰 → SITL(`-w`, `--no-mavproxy`) → 브리지(TCP 5762) → pymavlink로 GUIDED 모드 확인 후 시동, `takeoff 2`, 12초 정지, 북쪽 0.5m/s로 8초, 정지, 동쪽 0.5m/s로 8초, 정지
- 결과 (GPS 끔)
  - EKF 메시지: `EKF3 IMU0 fusing optical flow`, `started relative aiding`
  - 이륙: 추정 D = -2.17, 실제 z = 2.17 (일치)
  - 북쪽 이동 후: 추정 N = +1.57, 실제 y = +1.57
  - 동쪽 이동 후: 추정 E = -0.34, 실제 x = -0.34 (이동량 1.66m 일치)
  - 추정 속도 약 0.47~0.53 m/s (명령 0.5)
  - 브리지: 광류와 거리 약 25Hz로 송신, 광류 신뢰도 q ≈ 200~220
- 해석 주의: 이동 거리가 약 1.6m라서 장기 드리프트와 정확도는 아직 측정하지 못했다. 이동 시작이 이륙 완료 전이어서 목표 4m가 아닌 1.6m만 이동했다. 시험 중 이동량이 짧아서 정사각형 같은 긴 경로에서 오차를 다시 재야 한다. GPS를 끈 상태에서는 시뮬레이션에 바람과 센서 잡음이 없다.
- 아직 확인하지 못한 것
  - A 가이드의 연결 경로(MAVProxy `--out=udp:...:15550`을 통한 주입)는 시험하지 않았다. 이번에는 SITL의 TCP 5762에 브리지를 직접 연결했다. MAVProxy가 출력 포트로 들어온 메시지를 SITL로 전달하는지는 당시 [미확인]이었고 2.13에서 확인했다.
  - 2대 구성(포트 15550/15560, 센서 토픽 이름 충돌 방지)
  - 센서 잡음, 모션 블러, 기체 기울기가 있는 경우의 정확도
  - 위치 불확실성(분산) 메시지 확보 (A 담당 11절)
- 커밋: `321d5f4`

### 2.12 CSV 로거와 시야 판정 (13번, 2026-10-11)
- 작업 방식: 서브 에이전트(Sonnet)가 작성하고 메인이 다시 실행해 검증
- 파일: `scripts/csv_logger.py`, `scripts/visibility.py`(A 가이드 §13 참조 구현), `scripts/test_visibility.py`, `logs/sample_logger_check.csv`
- 내용: A 가이드 §4 컬럼 21개, 10Hz(시뮬 시간), 한 행 = 한 시각·한 드론. 값 없음은 빈칸, 불확실성은 0으로 채우지 않음(`uncertainty_valid=false`, `uncertainty_source=unavailable`). `visible_ids`는 JSON 배열. 추정 소스는 `main(estimate_source=...)`로 교체 가능, 기본은 stub(None)
- 결과: 헤더가 계약과 일치, 140개 시각(13.9초, 간격 0.1초), drone_0의 `visible_ids`가 `[]` → `[1]` → `[]`(전환 y ≈ ±3.9m, FOV ±45° 기하와 일치), `test_visibility: all passed`
- 커밋: `5dc32a8`

### 2.13 MAVProxy 경로 광류 비행과 정확도 측정 (11번, 2026-10-11)
- 파일: `scripts/run_flow_check_mavproxy.sh`, `scripts/flow_accuracy_check.py`, `results/flow_accuracy_*.csv`
- 연결: SITL `--no-extra-ports --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:15550 --mavproxy-args=--daemon`. 제어는 `udpin:14550`, 브리지는 `udpin:15550`
- 문제와 해결: pymavlink `udpin`은 상대 주소를 수신한 패킷에서 배우므로, 브리지가 읽지 않으면 송신이 전부 버려졌다. 브리지 주 루프를 `time.sleep(1)` 대신 `recv_match` 수신 루프로 바꿈. 스크립트 실행 권한(`Permission denied`)은 `chmod +x`
- 시험: 이륙 2m → 8초 안정화 → 정지비행 → A 가이드 §14 정사각형(2m, 0.3m/s, 추정 위치로만 제어) → 착륙. 오차 = 추정 − 실제 − 정지 상태 평균 offset
- 결과 (GPS 끔)

| 측정 | 정사각형 RMSE xy | 정사각형 최대 xy | 정지비행 최대 xy |
|---|---|---|---|
| 1회차 | 0.011 m | 0.021 m | 0.017 m (60초) |
| 2회차 | 0.022 m | 0.040 m | 0.020 m (60초) |
| 메인 재검증 | 0.012 m | 0.025 m | 0.018 m (20초) |

- 재검증에서 `EKF3 IMU0 fusing optical flow`, `started relative aiding` 확인, 브리지 `flow n=2699`, failsafe 없음
- 해석 주의: 바람·센서 잡음이 없는 시뮬레이션이라 실제보다 훨씬 좋게 나온다. 연결과 계산이 맞는지 확인하는 용도
- 커밋: `60db88e`

### 2.14 다중 드론 스폰 (12번, 2026-10-11)
- 파일: `scripts/spawn_drones.py`, `docs/multi_drone.md`, `.gitignore`(`models/generated/` 추가)
- 내용: 원본 iris SDF를 복사해 모델 이름 `drone_{i}`, `fdm_port_in` 9002+10i로 바꾸고, `sensors/*` 링크를 드론별 토픽(`/drone_{i}/...`)으로 붙여 `models/generated/drone_{i}/model.sdf`를 만든 뒤 `/world/room/create`로 스폰. 기본 위치 0:(-2,0) 1:(2,0) 2:(-2,3) 3:(2,3) 4:(-2,-3) 5:(2,-3), 위치 제한 |x|,|y| ≤ 5.5. `--pos ID=x,y[,z[,yaw]]`, `--gen-only`
- 결과: 드론 3, 4 스폰(포트 9032/9042, 토픽 6개, 영상 정상, 지상 거리 0.145m, `get_truth.get(3/4)` 동작). 드론 1, 2에 SITL 2개를 붙여 동시 이륙, 드론 1만 이동 명령 시 드론 2는 정지(명령 분리 확인)
- 커밋: `1f9fa6b`

### 2.15 2대 스침 점검 (15번, 2026-10-11)
- 파일: `scripts/pass_check.py`, `scripts/run_pass_check.sh`, `logs/pass_check_sample.csv`
- 시나리오: 첫째 드론 (-4,-0.75) yaw 0에서 동쪽, 둘째 드론 (4,+0.75) yaw π에서 서쪽으로 고도 2m, 0.5m/s, 각자 8m. GPS ON, 회피 없음. 로거는 이륙 직전부터 70초 기록
- 실행: `scripts/run_pass_check.sh` (환경변수 `IDS`, `PART`, `RUN`, `LOGD`, `OUT`, `DUR`)
- 결과

| 실행 | 이동 | `[상대]` → `[]` 전환 x | 최소 거리 | `visible_ids` 재계산 불일치 |
|---|---|---|---|---|
| 드론 2, 3 (에이전트) | 8.30m, 8.34m | ∓0.77 | 1.493m | 0 / 1402행 |
| 드론 0, 1 (메인 1차) | 8.32m, 8.33m | ∓0.80 | 1.497m | 0 / 1400행 |
| 드론 0, 1 (메인 2차, 수정 후) | 8.31m, 8.37m | ∓0.79 | 1.494m | 0 / 1400행 |

- 예상 전환(카메라 앞 0.1m, 차선 간격 1.5m, FOV ±45°): 두 드론 x 간격 약 1.6m, 각자 x ≈ ∓0.8m. 측정과 일치
- 문제와 해결
  - `csv_logger.py`가 CSV를 다 쓴 뒤 종료할 때 `Segmentation fault (core dumped)`. gz.transport 정리 문제로 보여 `os._exit(0)`으로 종료하게 바꿈
  - 종료 후 gz 서버가 SIGTERM에 끝나지 않고 남음. `run_pass_check.sh`에 `kill -9` 추가
  - Windows에서 `\\wsl$`로 스크립트를 편집하자 실행 권한이 빠져 `Permission denied`. `chmod +x`
  - 수정 후 2차 실행: segfault 없음, 남은 프로세스 없음
- 커밋: `cee9291` (매뉴얼 10.3 포함)

### 2.16 B 매뉴얼 (16번, 2026-10-11)
- 파일: `docs/B_manual.md`
- 내용: 환경 구성, 월드, 센서, `get_truth`, 광류 브리지(직접 TCP, MAVProxy 경로), 격리 시험 방법, 문제 해결 표, 다중 드론·로거·스침 시험(10장), [미확인] 항목 모음
- 작업 방식: 초안은 서브 에이전트(Sonnet), 보강은 Haiku. 메인이 명령과 수치를 실제 파일과 대조
- 커밋: `d33a240`(초안, 정면 카메라 FOV 90° 복구), `bb6e054`(7.5, 10.1, 10.2)

### 2.17 정면 카메라 방향 확인 (9번, 2026-10-11)
- 작업 방식: 서브 에이전트(Haiku)가 실행, 메인이 영상과 수치를 검토
- 방법: 격리 파티션에 드론 7을 (0,0) yaw 0으로 스폰하고, 빨간 상자(0.4×0.4×1m)를 옮겨 가며 `/drone_7/front_camera/image`를 저장. 빨간 영역의 중심 열을 계산
- 결과 (640×480, 수평 FOV 90°, 영상 중심 열 320)

| 경우 | 상자 위치 | 중심 열 | 판정 |
|---|---|---|---|
| yaw 0 | (+2, 0) 정면 | 319.5 | 중앙 |
| yaw 0 | (+2, +1) 왼쪽 | 144.3 (기하 예상 약 152) | 왼쪽 |
| yaw 0 | (+2, -1) 오른쪽 | 494.7 (기하 예상 약 488) | 오른쪽 |
| yaw 0 | (-2, 0) 뒤 | 빨간 픽셀 0 | 안 보임 |
| yaw 90° | (0, +2) | 319.5 | 중앙 |
| yaw 90° | (+2, 0) | 빨간 픽셀 0 | 안 보임 |

- 영상은 위아래가 바르다(위쪽 회색 벽, 아래쪽 바닥 무늬). 메인이 PNG를 직접 열어 확인함
- 참고: 빨간 상자는 조명 때문에 RGB 약 (121~137, 0, 0)으로 렌더링된다. 색 판정은 상대 기준(R>60, G·B < R/2)으로 했다. `set_pose` 서비스는 없는 엔티티에도 true를 돌려주므로 응답만으로 성공을 판단하지 않는다
- 미확인: 비행 중 기울기(pitch/roll)가 있을 때의 영상. 시험 스크립트는 저장소에 넣지 않았다(일회성 확인)

### 2.18 Gazebo·SITL 시간 정렬 (14번, 2026-10-11)
- 목적: A 가이드 §4 "시간 정렬은 B의 로거에서 담당". Gazebo 시뮬 시각과 SITL `time_boot_ms`의 관계를 시작, 일시정지, 재시작 때 확인
- 작업 방식: 서브 에이전트(Haiku)가 측정, 로거 일시정지 시험은 메인이 다시 실행
- 파일: `scripts/time_sync_check.py`(드론 5, 지상), `scripts/csv_logger.py`에 `--no-data-timeout` 추가
- 결과
  - 시작: offset = sim − boot_ms/1000 평균 0.0006 s, 범위 ±0.010 s(표본 간격 수준), 기울기 0.9997. SITL 시각은 Gazebo 시뮬 시각을 그대로 따른다(JSON 모델이 Gazebo 시각으로 SITL을 진행시킴)
  - Gazebo 15초 일시정지: SITL `boot_ms`가 37734에서 멈추고 HEARTBEAT도 끊김. 재개 후 offset 변화 +0.0018 s
  - SITL만 재시작: `boot_ms`가 0으로 돌아가지 않고 43300(sim 43.297)에서 이어짐. offset 변화 −0.009 s(잡음 수준)
  - 시뮬/실시간 배율은 이 PC에서 약 0.4~0.55
- 규칙: `sim_s = time_boot_ms/1000 + offset`, offset은 실행마다 시작 시 재고 SITL 재시작 후 다시 잰다. `boot_ms`를 호스트 시간으로 쓰지 않는다
- 로거 일시정지 시험(메인, 파티션 agentV, SITL 실행 중, 15초 일시정지)
  - 기본(대기 10초): 일시정지 중 "새 시뮬 시각 없음, 종료", 37행에서 끝남 (예상대로)
  - `--no-data-timeout 30`: 300행, sim 12.02~41.92 s, 최대 간격 0.108 s. 일시정지 동안 시뮬 시각도 멈추므로 기록에 빈 구간이 없다
  - 에이전트의 첫 시험은 0행이었는데, 로거 시작 5초 만에 일시정지해 첫 데이터를 받기 전이었던 것으로 보인다. 메인 시험에서는 로거 시작 10초 뒤에 일시정지함
- 커밋: `738674f`

### 2.19 2대 GPS OFF 광류 비행, A 가이드 §12 연결 (11·12번, 2026-10-11)
- 작업 방식: 서브 에이전트(Haiku)가 작성·실행, 메인이 원본 로그와 스크립트를 검토
- 파일: `scripts/run_two_flow_check.sh`, `scripts/two_flow_check.py`
- 연결: 드론 0 = sysid 1, 제어 14550, 브리지 15550 / 드론 1 = sysid 2, 제어 14560, 브리지 15560. 두 기체 모두 `flow_test.parm`(GPS 끔), 브리지는 `/drone_{i}/down_*` 토픽 사용
- 시험: 둘 다 2m 이륙 → 20초 정지 → 드론 0만 동쪽 1.5m → 드론 1만 동쪽 1.5m → 착륙. 제어는 추정 위치만 사용
- 결과 (2회 모두 통과)

| 실행 | B 드론 0 이동 / 드론 1 최대 | C 드론 1 이동 / 드론 0 최대 | 추정−실제 차 |
|---|---|---|---|
| 1회 | 1.501 / 0.007 m | 1.497 / 0.022 m | 0.007 / 0.011 m |
| 2회 | 1.489 / 0.016 m | 1.491 / 0.029 m | 0.020 / 0.004 m |

- 두 SITL 모두 `EKF3 IMU0 fusing optical flow`, `started relative aiding`. 브리지 드론마다 약 1900건 송신. failsafe 없음, 착륙 후 disarm, 남은 프로세스 없음
- 문제와 해결: `fusing optical flow` 메시지는 EKF 초기화 때 한 번만 나와서 스크립트가 붙기 전에 지나간다. 실시간 대기는 EKF 상태 플래그(0x08, 상대 위치)로 하고, 메시지는 끝난 뒤 MAVProxy `mav.tlog`에서 읽는다
- 커밋: `9d07171`, 매뉴얼 10.4·10.5절 `31c6799`

### 2.6 A 가이드와의 정렬
A 가이드를 받아 B의 환경과 비교했고, 아래 방향으로 조정하기로 했다.

| 항목 | A 가이드 | B 기존 | 조정 |
|---|---|---|---|
| 프로젝트 폴더 | `~/drone_project` | `~/capstone_sim` | `drone_project`로 이름 변경, `config`, `results`, `runs` 추가 |
| 환경 설정 | `env.sh`를 터미널마다 `source` | `~/.bashrc` | `env.sh`로 옮김. `.bashrc`의 옛 줄 제거 |
| 플러그인 위치 | `~/gz_ws/src/ardupilot_gazebo` | `~/ardupilot_gazebo` | 위치는 유지. 커밋 해시는 A와 비교 예정 (B: `082a0fe`) |
| SITL 실행 | `-I0 --sysid 1 --no-extra-ports --out=udp:127.0.0.1:14550`, `runs/drone0` | 옵션 없음 | 문서 명령으로 통일 |
| 월드 크기 | 12m × 12m × 5m | 8m × 8m × 3m | 완료 (2.4-2) |
| 바닥 | 텍스처 있는 평평한 바닥 | 단색 | 완료 (2.4-2) |
| 모델 이름 | `drone_0`, `drone_1` | 기본 이름 | 완료 (2.14) |

정렬 절차(폴더 이동, `env.sh`, venv 재생성, `.gitignore`, SITL 실행 명령, 버전 기록, 커밋)는 안내했고, 정렬 완료.

## 3. 발생한 문제와 해결

| 증상 | 원인 | 해결 |
|---|---|---|
| `Arm: Motors: Check frame class and type` | `FRAME_CLASS`가 0(미설정). `FRAME_TYPE`은 1 | MAVProxy에서 `param set FRAME_CLASS 1` 후 SITL을 재시작해 이륙 성공. 이후 `-w` 없이 실행해야 설정이 유지됨 |
| `param set FRAME_CLASS 1` 후 `reboot` 입력 시 `EOF on TCP socket`, `Connection refused` 반복 | SITL 프로세스가 종료되고 자동으로 다시 뜨지 않음 | `Ctrl+C` 후 같은 명령으로 SITL 직접 재시작 |
| `sim_vehicle.py: not found` | PATH에 `Tools/autotest`가 없음 (새 터미널에 `~/.profile` 미적용) | `. ~/.profile`. 영구 해결은 PATH 추가 (`env.sh`에 포함) |
| `no JSON sensor message received`, MAVProxy `FTP: request timed out` | `room.sdf`에 Imu 시스템 플러그인이 빠져서 ArduPilot 플러그인이 센서 값을 보내지 못했고, SITL이 대기 상태에 멈춤. FTP 메시지는 그 증상 | `room.sdf`에 `gz-sim-imu-system` 추가. 이후 방 안 이륙 성공 |
| `gz model --list` 타임아웃 | 터미널 간 Gazebo 통신 탐색 문제 (WSL2에서 흔함) | `export GZ_IP=127.0.0.1`을 안내했고 `env.sh`에 포함. 효과는 [미확인] |
| `ArduPilot controller has reset`, `Duplicate input frame` 경고 | SITL 연결·재연결 시 프레임 카운터 재시작 | 한두 번은 무시. 반복되면 `pkill -f arducopter`로 남은 SITL 정리 |
| 카메라 영상에서 벽과 바닥이 검게 나옴 (`down_camera`는 픽셀 값 전부 0, `front_camera`는 문 구멍과 맨 위 띠만 밝음) | 재질이 없는 벽 visual과 `<pbr>`만 있는 바닥(`<ambient>`, `<diffuse>` 없음)이 카메라 센서에서 검게 렌더링됨. `<scene>` 요소 제거는 효과 없었음. 같은 월드로 비교한 결과: 벽에 재질을 넣으면 벽이 회색, 바닥 PBR만 쓰면 검은색, 바닥에 `<ambient>`/`<diffuse>`를 1로 같이 넣으면 텍스처가 보임 | 벽 visual에 `<material>`(ambient/diffuse 0.7), 바닥 material에 `<ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>` 추가. 수정 후 `front_camera`에서 회색 벽·문·바닥 무늬, `down_camera`에서 바닥 무늬가 확인됨. 참고: 이전에 "Image Display 영상이 보인다"고 확인한 것은 이 문제를 알아채기 전이라 영상이 어두웠을 수 있음 |
| `gz topic -e` 한 번의 출력만으로는 영상이 검은지 판단하기 어려움 | 첫 프레임이 비어 있거나 바이트 출력이 길어서 구분이 안 됨 | Python 바인딩으로 프레임을 받아 `min/max/mean`을 계산하고 PNG로 저장해서 확인 |
| 새 실행 폴더(`runs/drone0`)에서 `param show FRAME_CLASS`가 다시 0 | 폴더가 새것이면 파라미터가 초기화되어 `gazebo-iris` 기본값에 `FRAME_CLASS`가 없음. 2대 구성에서는 `runs/drone1`에서도 반복될 것 | `param set FRAME_CLASS 1` 후 SITL 재시작(`-w` 없이)해서 이륙·착륙 성공. 값을 `config/frame.parm`에 저장해 두는 방법을 안내함. A의 파라미터 파일에도 들어가야 하므로 A에게 전달 필요 |
| `[Err] [UserCommands.cc:927] ... Unable to find uri[model://front_camera]` | `sensors/front_camera` 파일이 만들어지지 않은 상태였음 (`ls`로 확인). 환경변수가 원인이 될 수도 있으니, Gazebo는 `source env.sh` 후에 시작해야 `sensors` 경로를 읽음 | `model.config`, `model.sdf`, `set_fov.sh`를 `~/drone_project` 기준으로 다시 생성하고 Gazebo를 `env.sh` 적용 후 재시작. 이후 스폰 성공 |
| `gz topic -e -t /front_camera/image`가 `\000`으로 도배됨 | 이미지 픽셀 데이터를 바이트로 그대로 출력해서 원래 그렇게 보임. 값이 전부 0이면 검은 영상일 수 있어 확인이 필요했음 | `width/height` 필드만 확인(`grep -E "width|height|pixel_format|step"`)하고 영상은 Gazebo의 Image Display로 확인. 영상이 정상 표시됨 |
| MAVProxy 연결 관련 `mav` 에러가 반복됨 | 남은 SITL·Gazebo 프로세스, 또는 실행 순서가 어긋남 | 전체 종료(`pkill -f arducopter; pkill -f sim_vehicle; pkill -f mavproxy; pkill -f "gz sim"`) 후 순서대로 재시작: Gazebo → iris 스폰 → SITL. SITL을 스폰보다 먼저 켜면 `no JSON sensor message received`가 난다 |
| `land` 입력 후 모드 표시가 `GUIDED`로 남음 | 착륙 명령이 모드 변경이 아닌 명령으로 전달되는 것으로 보임 [미확인] | 착륙은 정상 동작. 모드까지 바꾸려면 `mode land` 사용 (A 가이드도 `mode land` 사용) |
| `udpin` 연결에서 브리지 송신이 SITL에 도달하지 않음 | pymavlink `udpin`은 상대 주소를 수신한 패킷에서 배움. 브리지가 수신하지 않아 송신이 버려짐 | 브리지 주 루프에서 `recv_match`로 계속 수신 (2.13) |
| `csv_logger.py` 종료 시 `Segmentation fault (core dumped)` | CSV 기록은 끝난 뒤 인터프리터 정리 중 발생. gz.transport 구독 정리 문제로 보임 | `os._exit(0)`으로 종료 (2.15) |
| 시험 종료 후 `gz sim` 서버가 SIGTERM에 끝나지 않음 | 원인 미확인 (1회 관찰) | `kill -9 -- -<PGID>` (2.15) |
| `\\wsl$` 경로에서 편집한 스크립트가 `Permission denied` | Windows 편집기가 파일을 다시 쓰면서 실행 권한이 빠짐 | `chmod +x <스크립트>` |

## 4. 결정 사항
- 개발 환경은 WSL2 + Ubuntu 24.04(실제 설치 기준, A 가이드의 22.04와 다름), 시뮬레이터는 Gazebo Harmonic, 기준 기체는 공식 iris로 한다. 직접 제작 드론의 사양이 확인되기 전에는 `baseline_iris`로 표시한다.
- 좌표는 ENU, 단위는 m·m/s·rad·s, 로그는 10Hz, simulation time 기준으로 한다.
- 센서 브리지와 광류 계산은 B 담당으로 한다. 원래 계획의 "광류 + 고도센서 모델 3h"보다 범위가 훨씬 크므로 가능 여부를 먼저 검증한다.
- SITL 자체 가상 광류(`SIM_FLOW_ENABLE`)는 진단용으로만 쓰고 `synthetic_flow`로 표시한다.

## 5. 산출물 (WSL `~/drone_project` 기준)

| 경로 | 내용 | 비고 |
|---|---|---|
| `worlds/room.sdf` | 실내 방 월드 (12×12×5, 텍스처 바닥, Imu 포함) | 2.4-2 참고. 구조물 설명은 `docs/world_guide.md` |
| `models/floor/` | 텍스처 바닥 모델 | `model://floor` |
| `scripts/make_floor_texture.py` | 바닥 텍스처 생성 | 시드 42 고정 |
| `docs/world_guide.md` | 구조물 설치 가이드 | 12m 기준으로 갱신 완료 (커밋 `de3b27b`) |
| `sensors/front_camera/` | 정면 카메라 모델 | 단독 시험 완료. 고정 조인트 2.14, 정면(+x) 방향 2.17에서 확인 |
| `docs/front_camera_spec.md` | A에게 넘길 카메라 장착 명세 | |
| `config/frame.parm` | `FRAME_CLASS 1` | 새 실행 폴더에서 `param load`로 사용 |
| `scripts/set_fov.sh` | FOV 변경 스크립트 | |
| `env.sh` | 환경변수 | git 제외 |
| `docs/versions.txt` | 버전 기록 | 완료 (2026-10-11, WSL 확인) |
| `docs/python-requirements.txt` | Python 패키지 목록 | |
| `scripts/get_truth.py` | 실제 위치 읽기 (ENU 위치·yaw, 카메라 위치·yaw) | 2.7 |
| `sensors/down_camera/`, `sensors/down_range/` | 하향 카메라(320×240, 30Hz), 하향 거리센서(0.1~5m) | 2.8 |
| `scripts/flow_probe.py` | 광류 계산 가능 여부 시험 | 2.9 |
| `sensors/iris_flow_test/` | 기본 iris에 하향 센서를 붙인 B 시험용 래퍼 (A 본체 SDF 아님) | 2.10 |
| `scripts/sensor_bridge.py`, `config/flow_test.parm` | 광류·거리 MAVLink 브리지, 시험용 파라미터 | 2.11, 2.13 |
| `scripts/flow_flight_check.py`, `scripts/run_flow_check.sh` | GPS OFF 광류 비행 점검 (직접 TCP 연결) | 2.11 |
| `scripts/run_flow_check_mavproxy.sh`, `scripts/flow_accuracy_check.py`, `results/flow_accuracy_*.csv` | MAVProxy 경로 광류 비행, 정사각형 정확도 측정 | 2.13 |
| `scripts/csv_logger.py`, `scripts/visibility.py`, `scripts/test_visibility.py`, `logs/sample_logger_check.csv` | CSV 로거(10Hz, 21컬럼), 시야 판정, 샘플 로그 | 2.12 |
| `scripts/spawn_drones.py`, `docs/multi_drone.md` | 다중 드론 스폰 (`drone_{i}`, 포트 9002+10i), 정면 카메라 고정 조인트 | 2.14 |
| `scripts/pass_check.py`, `scripts/run_pass_check.sh`, `logs/pass_check_sample.csv` | 2대 스침 점검 | 2.15 |
| `docs/B_manual.md` | B 매뉴얼 초안 | 2.16 |

## 6. 미확인 · 확인 필요
- OS 버전 불일치: B는 Ubuntu 24.04, A 가이드는 22.04. A의 실제 OS와 `versions.txt`를 받아 비교하고 통일 여부를 결정해야 한다.
- 사용자의 GUI Gazebo에서 `down_camera` 영상 확인, 프레임 주기(약 18Hz → 30Hz 목표) 재측정 (헤드리스 시험은 2.8)
- A의 ArduPilot·플러그인 커밋 해시와 B의 해시 일치 여부
- 바닥 텍스처가 육안으로 보이는지, 새 월드의 공 두 개 충돌 결과
- `GZ_IP=127.0.0.1`이 `gz model --list` 문제를 해결하는지 [미확인]. `gz model --list`는 타임아웃이 났었고 이후 사용하지 않음
- 확인 완료(2026-10-11): WSL2에서 카메라 센서(ogre2) 렌더링 정상 (Image Display 영상 확인), `front_camera` 토픽·이미지 크기·FOV 변경
- 일정: A 가이드 작성일이 10/10이라 기존 마감(2026-10-10)이 지났다. 연장 여부와 새 마감 확인 필요
- 직접 제작 드론의 사양 확보 담당과 시점
- 해결됨: Gazebo↔SITL 시간 정렬(2.18), 2대 GPS OFF MAVProxy 경로(2.19), 정면 카메라 +x 방향(2.17)
- 실제−추정 offset(A 가이드 §3) 기록을 누가 할지 A와 합의 필요. 지금 로거에는 없음
- EKF 원점 37.0/127.0(A 가이드 §10) 미적용. 우리 시험은 기본 원점으로 돌렸다
- 위치 분산 확보 여부(A 가이드 §11)는 A 확인 대기. 로거 분산 칸은 비어 있음
- 기체 기울기가 큰 비행, 3대 이상 동시 비행, yaw가 바뀌는 비행의 시야 판정은 시험하지 않았다
- (해결, 2026-10-12) GitHub `https://github.com/Multi-Bus/capstone_design.git` `main`에 업로드. README 추가

## 7. 다음 작업 (순서)
1. (완료) 정렬. A의 해시와 비교는 남음
2. (완료) `room.sdf` 갱신: 12m × 12m × 5m, 바닥 텍스처, 문 위치 재계산. `docs/world_guide.md`도 갱신(`de3b27b`)
3. (완료) `front_camera` 마무리: 고정 조인트(2.14), 정면 방향(2.17). 남은 것은 장착 명세(`docs/front_camera_spec.md`)를 A에게 전달
4. (완료) `get_truth` 구현 (2.7)
5. (완료) 하향 거리센서(0.1~5m)와 하향 카메라
6. (완료) 광류 계산 + MAVLink 브리지, MAVProxy 경로, 정확도 측정
7. (완료) 다중 드론 스폰 (`drone_i`, 포트 9002+10i)
8. (완료) CSV 로거와 시간 정렬. 추정 칸은 A의 `get_estimate`를 받으면 `main(estimate_source=...)`로 연결
9. (완료) 2대 스침 시험, 2대 GPS OFF 광류 비행, B 매뉴얼
10. 남은 일 (A 산출물 또는 사용자 결정이 필요)
    - A의 `get_estimate` 연결과 오프셋 기록 담당 합의
    - 장착 명세를 A에게 전달, A와 OS 버전·커밋 해시 비교
    - 마감 연장 확인
    - (완료, 2026-10-12) GitHub 업로드: 원격 `origin`, 브랜치 `main`, `README.md`, `env.sh.example` 추가
    - 통합과 최종 리포트

## 8. 작업 기록 양식 (앞으로 항목을 이 형식으로 추가)

```
### YYYY-MM-DD 작업 제목
- 목적:
- 수행 내용 / 명령:
- 결과 (성공/실패, 확인한 값):
- 문제와 해결 (오류 원문 포함):
- 변경한 파일:
- 다음 할 일:
```

작업 기록 작성 규칙:
- 오류 메시지는 원문 그대로 적는다.
- 확인하지 못한 내용은 [미확인]으로 표시하고, 확인되면 갱신한다.
- 명령은 실제로 실행한 것만 적는다.
- 설정값이나 구조를 바꾸면 A 가이드와 달라지는 점을 4절에 추가한다.
