# B 담당 매뉴얼 (Gazebo · 센서 · 실제 위치 · 광류 브리지)

이 문서는 B 담당 영역을 처음 하는 사람이 순서대로 따라 할 수 있게 정리한 초안이다. 작업 기록(`B_작업기록.md`)과 `~/drone_project`의 실제 파일에서 확인된 내용만 적었다.

- 표기: [미확인]은 아직 확인하지 못한 항목이다. 명령 앞의 `PowerShell`, `Ubuntu`, `MAVProxy`는 입력하는 곳이다.
- 작성 시점: 2026-10-11. 다중 드론 스폰, CSV 로거, 2대 스침 시험은 10장에 자리만 있다.

## 1. 개요

### 1.1 B 담당 범위

| 항목 | 상태 |
|---|---|
| 실내 방 월드(12m × 12m × 5m, 북쪽 벽에 문, 텍스처 바닥)와 구조물 추가 | 완료 |
| 센서 모듈: `front_camera`, `down_camera`, `down_range` | 완료 |
| 실제 위치 읽기 `get_truth` | 완료 |
| 광류 계산 + MAVLink 브리지(`sensor_bridge.py`), GPS OFF 광류 비행 점검 | 직접 TCP 연결 구성에서 확인 |
| 다중 드론 스폰, CSV 로거, 2대 스침 시험 | 작성 예정(10장) |

파일 소유 규칙은 다음과 같다. 본체 SDF와 ArduPilot 파라미터는 A만 수정하고, 월드와 센서 모듈은 B만 수정한다. 센서는 별도 모델로 제공하고 A가 본체에 붙인다.

### 1.2 폴더 구조

```
~/ardupilot            ArduPilot (SITL, sim_vehicle.py)
~/ardupilot_gazebo     Gazebo 플러그인 (build/ 안에 libArduPilotPlugin.so)
~/venv-ardupilot       ArduPilot 설치 스크립트가 만든 Python venv (OpenCV, pymavlink 있음)
~/drone_project        B 작업 저장소
  env.sh               환경변수 (git 제외, 2.5절 내용으로 직접 만든다)
  worlds/room.sdf      방 월드
  models/floor/        텍스처 바닥 모델
  sensors/             front_camera, down_camera, down_range, iris_flow_test
  scripts/             get_truth.py, sensor_bridge.py, run_flow_check.sh, set_fov.sh 등
  config/              frame.parm, flow_test.parm
  docs/                world_guide.md, front_camera_spec.md, versions.txt, 이 문서
  runs/  logs/  results/   실행 폴더와 결과 (runs/는 git 제외)
```

`~/drone_project`의 파일을 다른 PC로 옮기는 방법(원격 저장소 등)은 기록에 없다 [미확인].

## 2. 설치

모든 설치는 WSL 홈 디렉터리 안에 한다. `/mnt/c` 아래는 느리다.

### 2.1 실제 환경과 A 가이드의 차이

| 항목 | A 가이드 | B의 실제 환경 |
|---|---|---|
| Ubuntu | 22.04 | **24.04.5 LTS** (`wsl -l -v`의 배포판 이름 `Ubuntu-24.04`) |
| ArduPilot Python | 22.04는 `pip --user` | 24.04는 설치 스크립트가 `~/venv-ardupilot`을 자동 생성 |
| 플러그인 위치 | `~/gz_ws/src/ardupilot_gazebo` | `~/ardupilot_gazebo` (경로만 다름) |
| env.sh | 플러그인 경로가 `gz_ws` | 플러그인 경로가 `~/ardupilot_gazebo`, `sensors` 경로와 `GZ_IP` 추가 |
| SITL 옵션 | `-I0 --sysid 1 --no-extra-ports --out=...` | 같은 명령으로 통일 |

두 PC의 OS 버전을 통일할지는 정해지지 않았다. A의 `versions.txt`를 받아 비교해야 한다 [미확인].

### 2.2 WSL2 + Ubuntu

```powershell
# PowerShell (관리자)
wsl --install -d Ubuntu-24.04
wsl --update
```

배포판 이름은 `wsl -l -v`로 확인한 값이다. 설치 후 재부팅이 필요할 수 있다. 이미 Ubuntu가 있으면 건너뛴다.

```bash
# Ubuntu
sudo apt update && sudo apt upgrade -y
sudo apt install -y git curl wget lsb-release gnupg build-essential cmake pkg-config python3-pip python3-venv mesa-utils
mkdir -p ~/drone_project/{config,scripts,models,worlds,sensors,logs,results,docs,runs}
```

### 2.3 Gazebo Harmonic

```bash
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install -y gz-harmonic
gz sim -v4 -r shapes.sdf
```

창이 뜨고 시간이 진행되면 정상이다.

### 2.4 ArduPilot

```bash
cd ~
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

Ubuntu 24.04에서는 스크립트가 `~/venv-ardupilot`을 만든다. 직접 만들 필요가 없다. 이 venv에는 numpy, OpenCV, pymavlink가 있고 시스템 패키지(`gz` Python 바인딩)도 보인다. 광류 관련 스크립트는 이 venv의 Python으로 실행한다.

### 2.5 Gazebo 플러그인 빌드와 env.sh

```bash
sudo apt install -y libgz-sim8-dev rapidjson-dev libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
cd ~
git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

`build/`에 `libArduPilotPlugin.so`, `libCameraZoomPlugin.so`, `libGstCameraPlugin.so`, `libParachutePlugin.so`가 생기면 성공이다. (A 가이드는 빌드 전에 `export GZ_VERSION=harmonic`을 한다.)

`~/drone_project/env.sh`를 아래 내용으로 만든다. 이 파일은 git에 올라가지 않으므로 직접 만들어야 한다.

```bash
export GZ_VERSION=harmonic
export GZ_IP=127.0.0.1
export PATH="$HOME/ardupilot/Tools/autotest:$PATH"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$HOME/drone_project/models:$HOME/drone_project/worlds:$HOME/drone_project/sensors:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}"
```

**새 Ubuntu 터미널마다 `source ~/drone_project/env.sh`를 먼저 실행한다.** `GZ_SIM_RESOURCE_PATH`에 `sensors`가 있어야 `model://front_camera` 등을 찾는다. `GZ_IP=127.0.0.1`이 터미널 간 통신 문제를 실제로 해결하는지는 [미확인]이다.

### 2.6 프로젝트 Python 환경과 버전 기록

```bash
python3 -m venv ~/drone_project/.venv
source ~/drone_project/.venv/bin/activate
python -m pip install --upgrade pip
python -m pip install pymavlink numpy pandas matplotlib
python -m pip freeze > ~/drone_project/docs/python-requirements.txt
deactivate
```

첫 정상 비행에 성공한 시점의 버전을 기록한다. 아래는 `docs/versions.txt`를 만든 항목과 같다.

```bash
{
  date -Is
  lsb_release -a
  uname -a
  gz sim --versions
  git -C ~/ardupilot rev-parse HEAD
  git -C ~/ardupilot_gazebo rev-parse HEAD
  python --version
} > ~/drone_project/docs/versions.txt 2>&1
```

현재 기록된 값(2026-10-11): Ubuntu 24.04.5, gz sim 8.15.0, Python 3.12.3, ArduPilot 커밋 `d67add0b9b65aaed713357d91c95b0d2e8568982`(작업 기록상 ArduCopter 4.6.0-beta1), ardupilot_gazebo 커밋 `082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5`. A의 커밋과 일치하는지는 [미확인]이다. 임의로 `git pull`하지 않는다.

### 2.7 설치 확인

3장 절차를 월드만 `iris_runway.sdf`로 바꿔 실행한다. 이 월드에는 iris가 이미 들어 있어서 스폰 단계가 없다.

```bash
gz sim -v4 -r iris_runway.sdf
```

MAVProxy에서 `mode guided` → `arm throttle` → `takeoff 5` → `land`로 이륙과 착륙이 되면 설치가 끝난 것이다.

## 3. 실행 순서

**반드시 Gazebo → 스폰 → SITL 순서로 한다.** SITL을 먼저 켜면 `no JSON sensor message received`가 난다.

### 3.1 터미널 1: Gazebo

```bash
source ~/drone_project/env.sh
gz sim -v4 -r ~/drone_project/worlds/room.sdf
```

### 3.2 터미널 2: iris 스폰

`run_flow_check.sh`가 센서 달린 iris를 스폰하는 명령과 같은 형식이다. 모델만 기본 iris로 바꿨다. 이 문자열 그대로 실행한 기록은 없다 [미확인].

```bash
source ~/drone_project/env.sh
gz service -s /world/room/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 8000 \
  --req 'sdf: "<sdf version=\"1.9\"><include><uri>model://iris_with_ardupilot</uri><name>iris_with_ardupilot</name><pose>-2 0 0.2 0 0 0</pose></include></sdf>"'
```

위치 `(-2, 0)`은 A 가이드의 드론 0 초기 위치이고 yaw 0은 동쪽이다. 센서가 달린 시험용 기체는 `model://iris_flow_test`를 쓴다(5.4절).

### 3.3 터미널 3: SITL

```bash
source ~/drone_project/env.sh
mkdir -p ~/drone_project/runs/drone0
cd ~/drone_project/runs/drone0
~/venv-ardupilot/bin/python ~/ardupilot/Tools/autotest/sim_vehicle.py \
  -v ArduCopter -f gazebo-iris --model JSON \
  -I0 --sysid 1 --no-extra-ports \
  --out=udp:127.0.0.1:14550 \
  --add-param-file=$HOME/drone_project/config/frame.parm \
  --console
```

- `~/venv-ardupilot/bin/python`은 `run_flow_check.sh`가 쓰는 방식이다. A 가이드는 `python3`로 쓴다. 어느 쪽이 되는지는 환경에 따라 다를 수 있다 [미확인].
- `--no-extra-ports`는 기본 UDP 14550/14551 출력을 끄는 옵션이라서 `--out=udp:127.0.0.1:14550`을 직접 준다.
- 이 조합(위 전체 명령)을 그대로 실행한 결과는 작업 기록에 없다. `-I0 --sysid 1 ...` 정렬 결과가 [결과 미수신]이다.

### 3.4 FRAME_CLASS

새 실행 폴더에서는 `FRAME_CLASS`가 0으로 시작해서 시동 시 `Arm: Motors: Check frame class and type`가 난다. `config/frame.parm`(내용은 `FRAME_CLASS 1` 한 줄)을 `--add-param-file`로 주면 새 폴더에서도 1로 시작한다(확인됨). 2대 구성에서 `runs/drone1`에도 같은 옵션이 필요할 것이다.

수동으로 고칠 때는 MAVProxy에서 `param set FRAME_CLASS 1` 후 SITL을 `Ctrl+C`로 끄고 같은 명령으로 다시 켠다. `reboot`는 입력하지 않는다(SITL이 종료만 되고 다시 뜨지 않는다, 9장).

### 3.5 MAVProxy 기본 명령

SITL이 뜬 터미널(또는 `--console` 창)에서 한 줄씩 입력한다.

```text
mode guided
arm throttle
takeoff 2
mode land
```

- `arm throttle`이 거절되면 메시지를 읽고 기다린다. EKF가 준비될 때까지 `Arm: Accels inconsistent`가 약 1분 반복된 뒤 풀린 적이 있다.
- `takeoff`는 반드시 GUIDED 모드에서 보낸다. STABILIZE에서 시동하면 이륙 명령이 거절된다(9장).
- 이륙 높이는 방 높이 5m 안으로 한다.
- `land`만 입력하면 모드 표시가 GUIDED로 남을 수 있다(착륙은 정상). 모드까지 바꾸려면 `mode land`를 쓴다.

### 3.6 종료

각 터미널에서 `Ctrl+C`로 끈다. 프로세스가 남아 `mav` 연결 오류가 반복되면 9장의 전체 종료 후 순서대로 다시 시작한다. 단, 전체 종료 명령은 내 Gazebo 창도 같이 닫으므로 격리 시험(8장)에는 쓰지 않는다.

## 4. 월드와 구조물 추가

### 4.1 방 월드 (`worlds/room.sdf`)

| 항목 | 값 |
|---|---|
| 크기 | 12m × 12m × 5m, 벽 두께 0.2m, 지붕 없음 |
| 벽 위치 | 남 y=-6, 동 x=+6, 서 x=-6, 북 y=+6 (중심 기준, 높이 5m) |
| 문 | 북쪽 벽 가운데, x -0.5~0.5, 높이 2m. 벽을 box 3개(왼쪽 5.5m, 오른쪽 5.5m, 문 위 1m × 3m)로 나눠 구멍을 만든다 |
| 바닥 | `models/floor`(`model://floor`), 12.4m × 12.4m 평면, 흑백 랜덤 무늬 |
| 시스템 플러그인 | Physics, UserCommands, SceneBroadcaster, **Imu**, Sensors(ogre2) |
| 기타 | ambient 0.6, 그림자 끔(드론 그림자가 광류에 섞이는 것을 막기 위한 초기 설정) |

**Imu 플러그인(`gz-sim-imu-system`)을 빼면 안 된다.** 빠지면 ArduPilot 플러그인이 센서 값을 보내지 못해 `no JSON sensor message received`가 나고 SITL이 대기 상태에 멈춘다.

바닥 무늬는 광류 계산에 필요하다(단색이면 영상 변화가 없다). 텍스처는 `python3 scripts/make_floor_texture.py <출력.png>`로 만든다(1024 × 1024, 8px 칸, 시드 42 고정). 12.4m에 펼치면 칸 하나가 약 9.7cm다.

### 4.2 구조물 추가

`room.sdf`의 `</world>` 바로 위에 붙여 넣는다.

```xml
<model name="box_1">
  <static>true</static>
  <pose>1 1 0.5 0 0 0</pose>
  <link name="link">
    <collision name="collision"><geometry><box><size>1 1 1</size></box></geometry></collision>
    <visual name="visual"><geometry><box><size>1 1 1</size></box></geometry>
      <material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material>
    </visual>
  </link>
</model>
```

- `<pose>`는 `x y z roll pitch yaw`, x y z는 상자 중심이다. 바닥에 놓으려면 z = 높이/2.
- `<size>`는 가로(x) 세로(y) 높이(z)다.
- `<collision>`과 `<visual>`을 둘 다 넣는다. visual만 있으면 보이기만 하고 통과된다.
- 모델 이름은 월드 안에서 겹치면 안 된다.
- **`<visual>`에 `<material>`의 `<ambient>`와 `<diffuse>`를 반드시 넣는다.** 재질이 없는 visual과, `<pbr>`만 있고 ambient/diffuse가 없는 바닥은 카메라 센서 영상에서 검게 렌더링된다(`down_camera` 영상은 전체 픽셀 값이 0이었다). 바닥은 `<ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>`, 벽은 0.7을 쓴다. 수정은 Gazebo를 재시작해야 반영된다.

### 4.3 문법·충돌 확인

```bash
SDF_PATH=~/drone_project/models gz sdf -k ~/drone_project/worlds/room.sdf
```

`model://floor`를 찾으려면 `SDF_PATH`를 `models` 폴더로 지정해야 한다. Gazebo를 켠 상태에서 공을 떨어뜨려 충돌을 본다(바닥이나 벽 위에서 멈추면 정상).

```bash
gz service -s /world/room/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 3000 --req 'sdf: "<sdf version=\"1.9\"><model name=\"ball\"><pose>0 0 2 0 0 0</pose><link name=\"l\"><collision name=\"c\"><geometry><sphere><radius>0.2</radius></sphere></geometry></collision><visual name=\"v\"><geometry><sphere><radius>0.2</radius></sphere></geometry></visual></link></model></sdf>"'
```

12m 방 기준으로 바꾼 뒤 공 충돌 결과는 받지 못했다 [미확인]. 바닥 무늬는 카메라 영상과 GUI에서 확인됐다.

## 5. 센서 모듈

센서는 `sensors/` 아래 별도 모델이다. 링크 이름은 모두 `link`이고 중력은 끄고 질량 0.01kg이다.

| 모델 | 종류 | 해상도 / 범위 | 주기 | 토픽 | 방향 |
|---|---|---|---|---|---|
| `front_camera` | camera | 640 × 480, clip 0.1~20m | 15Hz | `/front_camera/image` | 링크 +x가 정면 |
| `down_camera` | camera | 320 × 240, FOV 60°(1.0472 rad), clip 0.1~10m | 30Hz | `/down_camera/image` | 아래(센서 pose pitch 1.5708) |
| `down_range` | gpu_lidar, 레이 1개 | 0.1~5.0m | 30Hz | `/down_range/scan` | 아래(센서 pose pitch 1.5708) |

하향 센서는 센서 안의 pose에 pitch를 넣어 두었으므로 본체에 회전 없이 붙이면 아래를 본다.

### 5.1 front_camera와 FOV 변경

수평 FOV는 `sensors/front_camera/model.sdf`의 `horizontal_fov`(라디안) 한 줄이다. 도 단위로 바꾸려면:

```bash
~/drone_project/scripts/set_fov.sh 90
# 출력: horizontal_fov = 90 deg (1.5708 rad)
```

- A 가이드의 기준은 90°다. 이 스크립트를 쓰고 난 뒤 파일에는 마지막으로 준 값이 남는다. 시험으로 값을 바꿨으면 끝나고 `set_fov.sh 90`으로 되돌린다(2026-10-11에 60° 시험 값이 커밋돼 있던 것을 90°로 되돌렸다).
- 이미 떠 있는 모델은 FOV 변경을 다시 읽지 않는다. 모델을 삭제하고 다시 스폰한다.
- 기체에는 fixed joint로 붙여야 한다. 장착 위치 권장값 (0.1, 0, 0.05) m와 규칙은 `docs/front_camera_spec.md`에 있다. 장착은 A가 한다. 기본 iris에 붙이는 고정 조인트 시험은 아직 하지 않았다 [미확인].

단독 시험은 다음처럼 한다(스폰 위치 `0 -5 1.5`, yaw 1.5708, 북쪽을 봄).

```bash
gz service -s /world/room/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'sdf: "<sdf version=\"1.9\"><include><uri>model://front_camera</uri><name>front_camera</name><pose>0 -5 1.5 0 0 1.5708</pose></include></sdf>"'
```

이 스폰 문자열은 2026-10-11에 그대로 실행해 `data: true`를 확인했다. 같은 위치·방향에서 영상이 나오고 FOV가 바뀌는 것도 확인했다.

### 5.2 down_camera와 down_range

- `down_range`: 센서 z=2m에서 `ranges`가 약 2.0001로 나왔다.
- `down_camera`: 2m 높이에서 바닥 무늬 칸 하나가 약 13픽셀로 보인다. 프레임은 헤드리스 소프트웨어 렌더링 환경에서 5초에 약 89장(약 18Hz)이었고 설정값 30Hz와 차이가 있다. GUI 환경에서 다시 재야 한다 [미확인].

### 5.3 확인 방법

**① 토픽 목록과 이미지 크기**

```bash
gz topic -l | grep -E "camera|range"
gz topic -e -t /front_camera/image -n 1 | grep -E "width|height|pixel_format|step"
```

`gz topic -e`로 이미지를 그대로 찍으면 픽셀 바이트가 `\000`으로 도배된다. 그것만으로 영상이 검은지 알 수 없으므로 위처럼 필드만 본다. `width: 640`, `height: 480`이 나오면 토픽은 정상이다.

**② Image Display**

Gazebo GUI 오른쪽 위 `⋮` 메뉴에서 Image Display를 열고 `/front_camera/image`를 선택한다. 영상이 보이면 WSL2에서 ogre2 카메라 렌더링이 동작하는 것이다. 영상이 어두우면 4.2절의 재질 문제를 먼저 의심한다. 재질 수정 후 GUI의 Image Display에서 `/front_camera/image`(회색 벽, 문, 바닥 무늬)와 `/down_camera/image`(바닥 무늬)가 정상으로 보이는 것을 확인했다(2026-10-11). Gazebo를 새로 켜면 서비스로 스폰한 센서는 남지 않으므로 다시 스폰해야 한다.

**③ Python으로 프레임 통계**

영상이 실제로 검은지 아닌지는 `min/max/mean`으로 판단한다. 아래는 `sensor_bridge.py`와 같은 방식(`gz.transport13`, `gz.msgs10`)으로 쓴 예시이며 이 형태 그대로 실행한 기록은 없다 [미확인]. 시스템 `python3`로 실행한다.

```python
import time
import numpy as np
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

got = []
node = Node()
node.subscribe(Image, "/down_camera/image", lambda m: got.append(m))
time.sleep(3)
m = got[-1]
g = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
print(m.width, m.height, "프레임", len(got), "min", g.min(), "max", g.max(), "mean", round(float(g.mean()), 1))
```

정상 예: `down_camera`가 2m 높이에서 최소 93, 최대 203, 평균 157.8. 전부 0이면 렌더링 문제다.

### 5.4 센서 달린 시험용 기체

`sensors/iris_flow_test`는 기본 `iris_with_ardupilot`를 `iris`라는 이름으로 include하고 `down_camera`, `down_range`를 `0 0 -0.05` 위치에 고정 조인트(부모 `iris::iris_with_standoffs::base_link`)로 붙인 B 쪽 시험용 래퍼다. A의 본체 SDF가 아니다. 센서를 붙여도 GPS ON 기본 비행은 정상이었다(`takeoff 2` 후 D=-2.19).

## 6. 실제 위치 읽기: `get_truth`

`scripts/get_truth.py`는 `/world/room/dynamic_pose/info`(Pose_V)를 구독한다. 시스템 `python3`로 실행한다.

### 6.1 단독 확인

```bash
python3 ~/drone_project/scripts/get_truth.py iris_with_ardupilot
```

0.5초마다 해당 모델의 값을 출력한다. 인자는 Gazebo 모델 이름이다(생략하면 `iris_with_ardupilot`).

### 6.2 코드에서 사용

```python
import os, sys; sys.path.insert(0, os.path.expanduser("~/drone_project/scripts"))
from get_truth import Truth
tr = Truth()                                     # world="room", model_fmt="drone_{}"
tr.get(0)                                        # 모델 이름 drone_0
tr.get(0, model_name="iris_flow_test")           # 이름을 직접 지정
```

| 반환 키 | 의미 |
|---|---|
| `time_sim_s` | 메시지 헤더의 시뮬레이션 시각(s) |
| `position_enu_m` | 모델 원점 위치 [x, y, z], ENU(x=동, y=북, z=위) |
| `yaw_enu_rad` | yaw, 동쪽 0, 반시계 양수 |
| `camera_position_enu_m` | 기체 위치 + 장착 오프셋(0.1, 0, 0.05)을 yaw로 회전한 값. roll/pitch는 무시 |
| `camera_yaw_enu_rad` | 기체 yaw와 같음 |
| `valid` | 마지막 수신 후 0.5초 이내(호스트 시계)면 True |

모델을 아직 받지 못했으면 `{"valid": False}`만 돌아온다. 오프셋은 `docs/front_camera_spec.md`와 같게 유지한다. 스폰 스크립트가 만들어지면 모델 이름을 `drone_{i}`로 맞춰서 `get(i)`가 그대로 동작하게 한다(작성 예정).

### 6.3 z 기준 (0.19)

z는 모델 원점(`base_link`)의 높이다. iris는 착륙해도 z가 약 **0.19m**다. 시험 결과(스폰 (-2, 0), `takeoff 2`): 이륙 후 (-1.99, 0.003, 2.19), yaw 0, 착륙 후 z = 0.19. 상승분 2.19 - 0.19 = 2.0m가 `takeoff 2`와 일치하고, x, y 오차는 1cm 이내다. 로그에는 truth z를 그대로 기록하고 분석할 때 착륙 상태 z(약 0.19)를 빼서 지면 대비 높이로 쓴다. 이 기준은 A와 공유해야 한다.

## 7. 광류 브리지와 GPS OFF 점검

### 7.1 sensor_bridge.py 원리

`scripts/sensor_bridge.py`는 Gazebo 하향 센서 값을 MAVLink로 바꿔 SITL에 보낸다.

- **광류**: `/down_camera/image`의 연속 프레임(평균 그레이)을 `cv2.phaseCorrelate`(Hanning 창)로 비교해 픽셀 이동량 (dx, dy)를 얻는다. `OPTICAL_FLOW`의 `flow_rate_x = dx/f/dt`, `flow_rate_y = dy/f/dt`(rad/s)로 보낸다.
  - f = (320/2)/tan(FOV/2) = 277.1px(FOV 60°). 브리지의 `--fov`(기본 1.0472)를 `down_camera/model.sdf`와 같게 유지한다.
  - dt는 이미지 헤더의 시뮬레이션 시각 차이다. 0 초과 0.2초 미만인 쌍만 쓴다.
  - `quality`는 위상 상관 신뢰도에 255를 곱한 값이고 0.3 미만이면 0이다. `ground_distance`는 거리센서 값이다.
- **거리**: `/down_range/scan`의 첫 값을 cm로 바꿔 `DISTANCE_SENSOR`로 보낸다. orientation 25(아래), 최소 10cm, 최대 500cm. 범위를 벗어나면 501cm를 보낸다.
- **하트비트**: 1Hz로 ONBOARD_CONTROLLER 하트비트를 보낸다. source_system 200.
- **MAVLink 2 필수**: `flow_rate_x/y`는 확장 필드라서 코드 맨 위에서 `os.environ["MAVLINK20"] = "1"`을 설정한 뒤 `pymavlink`를 import한다. 빠지면 `optical_flow_send() takes from 9 to 10 positional arguments but 11 were given`이 나고 광류가 전송되지 않는다. ArduPilot은 `flow_rate_x/y`가 0이 아니면 이 필드를 쓰고, 아니면 deprecated인 `flow_x/y`를 쓴다.

**축 규약.** 영상은 기체가 앞(+x)으로 가면 내용이 아래(+row)로, 왼쪽(+y)으로 가면 오른쪽(+col)으로 움직인다. ArduPilot의 SITL 가상 광류(`AP_OpticalFlow_SITL.cpp`)는 `flowRate.x = -v_y(오른쪽)/h`, `flowRate.y = +v_x(앞)/h`이므로 변환 없이 `dx`, `dy`를 그대로 넣으면 대응한다. 시험으로 맞음을 확인했다. 알고리즘 단독 검증(`scripts/flow_probe.py`, 카메라를 5cm씩 옮김)에서는 이론 6.93px 대비 6.96~7.02px(오차 약 0.5~1%), 정지 상태 가짜 이동량은 0.03px 이하였다.

실행(SITL이 떠 있고 Gazebo에 `iris_flow_test`가 스폰된 상태):

```bash
source ~/drone_project/env.sh
~/venv-ardupilot/bin/python ~/drone_project/scripts/sensor_bridge.py --mav tcp:127.0.0.1:5762
```

5762는 `--no-mavproxy`로 띄운 SITL의 TCP 포트다(비행 제어 스크립트는 5760). A 가이드의 연결 경로인 MAVProxy `--out=udp:127.0.0.1:15550`을 통한 주입은 시험하지 않았고, MAVProxy가 그 포트로 들어온 메시지를 SITL로 전달하는지도 [미확인]이다. MAVProxy를 같이 쓸 때 5762가 비어 있는지도 [미확인]이다.

### 7.2 파라미터 (`config/flow_test.parm`)

| 파라미터 | 값 | 의미 |
|---|---|---|
| `FRAME_CLASS` | 1 | 프레임 클래스(0이면 시동 거절) |
| `GPS1_TYPE` | 0 | GPS 끔 (이 빌드에는 `GPS_TYPE`이 없고 `GPS1_TYPE`) |
| `FLOW_TYPE` | 5 | 광류 소스 = MAVLink (10은 SITL 가상 광류) |
| `RNGFND1_TYPE` | 10 | 거리센서 = MAVLink |
| `RNGFND1_ORIENT` | 25 | 아래 방향 |
| `RNGFND1_MIN` | 0.1 | 최소 거리(m) |
| `RNGFND1_MAX` | 5.0 | 최대 거리(m). 이 빌드는 m 단위이고 `_CM`이 아니다 |
| `EK3_SRC1_POSXY` | 0 | 수평 위치 직접 입력 없음 |
| `EK3_SRC1_VELXY` | 5 | 수평 속도 = 광류 |
| `EK3_SRC1_POSZ` | 2 | 높이 = 거리센서 |
| `EK3_SRC1_VELZ` | 0 | 수직 속도 입력 없음 |
| `EK3_SRC1_YAW` | 1 | yaw = 컴퍼스 |

기준 빌드는 ArduPilot 4.6.0-beta1이다. 다른 버전이면 `param show GPS*`, `param show RNGFND1*`, `param show FLOW*`로 이름과 단위를 먼저 확인한다. 보정이 필요하면 `FLOW_FXSCALER`, `FLOW_FYSCALER`(0.1% 단위), `FLOW_ORIENT_YAW`(센티도), `FLOW_POS_X/Y/Z`가 있다. SITL 자체 가상 광류(`SIM_FLOW_ENABLE`, `FLOW_TYPE 10`)는 진단용이며 결과는 `synthetic_flow`로 구분한다.

### 7.3 run_flow_check.sh

격리된 헤드리스 Gazebo, SITL, 브리지, 비행 점검(`scripts/flow_flight_check.py`)을 한 번에 돌린다. 내 Gazebo 창과 섞이지 않는다(8장).

```bash
cd ~/drone_project
scripts/run_flow_check.sh flow_test.parm bridge     # 브리지 포함
scripts/run_flow_check.sh flow_test.parm            # 브리지 없이 (비교용)
```

- 첫 인자는 `config/` 안의 파라미터 파일 이름, 둘째 인자 `bridge`는 브리지를 켠다.
- 순서: 헤드리스 Gazebo(room.sdf) → `iris_flow_test` 스폰 (-2, 0, 0.2) → SITL(`--no-mavproxy -w --add-param-file`) → 브리지(TCP 5762) → 비행 점검.
- 비행 점검: GUIDED 확인 후 시동, `takeoff 2`, 12초 정지, 북쪽 0.5m/s로 8초, 정지, 동쪽 0.5m/s로 8초, 정지, 착륙. 비행 점검 부분의 제한 시간은 330초다. 시동까지 `Arm: Accels inconsistent`가 약 1분 반복될 수 있다.
- 매번 `runs/probe`를 지우고 새로 만든다(`-w`로 파라미터도 초기화되고 `--add-param-file`로 다시 들어간다).
- 로그: `/tmp/probe_gz.log`, `/tmp/probe_sitl.log`, `/tmp/probe_bridge.log`. 스크립트가 끝에 브리지 로그 끝 6줄과 Gazebo 오류 줄을 보여 준다.
- 실행 후 `== leftover`에 `arducopter`가 남아 있으면 정리한다.

### 7.4 검증 결과 (GPS 끔, 2026-10-11)

| 항목 | 결과 |
|---|---|
| EKF 메시지 | `EKF3 IMU0 fusing optical flow`, `started relative aiding` |
| 이륙 후 높이 | 추정 D = -2.17, 실제 z = 2.17 |
| 북쪽 이동 후 | 추정 N = +1.57, 실제 y = +1.57 |
| 동쪽 이동 후 | 추정 E = -0.34, 실제 x = -0.34 (이동량 1.66m 일치) |
| 추정 속도 | 약 0.47~0.53 m/s (명령 0.5) |
| 브리지 | 광류와 거리 약 25Hz 송신, 광류 신뢰도 q ≈ 200~220 |

해석할 때 주의할 점은 다음과 같다.

- 이동 시작이 이륙 완료 전이어서 목표 4m가 아니라 약 1.6m만 이동했다. 장기 드리프트와 정확도는 아직 측정하지 못했다.
- 시뮬레이션에 바람과 센서 잡음이 없다. 모션 블러, 기체 기울기가 있을 때의 정확도도 측정하지 않았다.
- 첫 시험은 `optical_flow_send` 오류로 광류가 한 번도 전송되지 않았는데도 정지 비행에서 위치 변화가 없어서 구분이 안 되었다. 그래서 이동 구간을 넣었다. 브리지 로그의 `flow n=`이 증가하는지 꼭 확인한다.

## 8. 시험을 격리해서 돌리는 방법

자동 시험이 내가 켜 둔 Gazebo GUI와 SITL을 건드리지 않게 하는 방법이다. `run_flow_check.sh`가 이 방식을 쓴다.

1. **Gazebo 구역 분리**: `export GZ_PARTITION=probe`. 같은 구역끼리만 토픽과 서비스가 보인다. 이 시험에 쓰는 모든 `gz` 명령, `get_truth.py`, 브리지는 같은 값을 export한 셸에서 실행해야 한다. 내 GUI(기본 구역)에서는 이 시험이 보이지 않는다.
2. **서버만 헤드리스로**: GUI 없이 `gz sim -s -r -v4 --headless-rendering worlds/room.sdf`. 소프트웨어 렌더링이라 프레임이 느릴 수 있다(약 18Hz).
3. **프로세스 그룹으로 종료**: 각 프로세스를 `setsid`로 따로 띄우고 PID를 저장해 두었다가 그 그룹만 끈다. `pkill -f "gz sim"` 같은 이름 기반 종료는 내 GUI Gazebo도 같이 죽이므로 쓰지 않는다.

```bash
export GZ_PARTITION=probe
setsid gz sim -s -r -v4 --headless-rendering worlds/room.sdf > /tmp/probe_gz.log 2>&1 &
GZ=$!
# ... 시험 ...
kill -- -$GZ      # 그 프로세스 그룹만 종료
```

SITL은 `--no-mavproxy`로 띄우고 pymavlink가 TCP 5760에 붙는다(MAVProxy 창이 필요 없다). pymavlink만 쓸 때 `LOCAL_POSITION_NED`가 오게 하려면 `request_data_stream_send`로 스트림을 요청해야 한다(`flow_flight_check.py` 참고).

## 9. 막힌 부분과 해결법

| 증상 (오류 원문) | 원인 | 해결 |
|---|---|---|
| `Arm: Motors: Check frame class and type` | `FRAME_CLASS`가 0(미설정) | `--add-param-file=$HOME/drone_project/config/frame.parm`을 준다. 수동이면 MAVProxy `param set FRAME_CLASS 1` 후 SITL 재시작 |
| `param set FRAME_CLASS 1` 후 `reboot` 입력 시 `EOF on TCP socket`, `Connection refused` 반복 | SITL 프로세스가 종료되고 자동으로 다시 뜨지 않음 | `Ctrl+C` 후 같은 명령으로 SITL을 직접 재시작 |
| 새 실행 폴더(`runs/drone0`)에서 `param show FRAME_CLASS`가 다시 0 | 새 폴더는 파라미터가 초기화됨. `gazebo-iris` 기본값에 `FRAME_CLASS`가 없음 | `config/frame.parm`을 `--add-param-file`로 준다. A의 파라미터 파일에도 들어가야 하므로 A에게 전달 필요 |
| `sim_vehicle.py: not found` | PATH에 `Tools/autotest`가 없음(새 터미널에 `~/.profile` 미적용) | `. ~/.profile`, 영구 해결은 `env.sh`의 PATH |
| `no JSON sensor message received`, MAVProxy `FTP: request timed out` | (1) `room.sdf`에 Imu 시스템 플러그인이 없었음. FTP 메시지는 그 증상. (2) SITL을 Gazebo 스폰보다 먼저 켬 | (1) `room.sdf`에 `gz-sim-imu-system` 추가 (2) Gazebo → 스폰 → SITL 순서로 재시작 |
| `[Err] [UserCommands.cc:927] ... Unable to find uri[model://front_camera]` | `sensors/front_camera` 파일이 없었음. 또는 `env.sh`를 적용하지 않고 Gazebo를 켜서 `sensors` 경로를 못 읽음 | 파일을 확인하고, `source env.sh` 후 Gazebo를 다시 시작 |
| `gz model --list` 타임아웃 | 터미널 간 Gazebo 통신 탐색 문제(WSL2에서 흔함) | `export GZ_IP=127.0.0.1`(`env.sh`에 포함). 효과는 [미확인]. 이후 이 명령은 쓰지 않음 |
| `ArduPilot controller has reset`, `Duplicate input frame` 경고 | SITL 연결·재연결 시 프레임 카운터 재시작 | 한두 번은 무시. 반복되면 `pkill -f arducopter`로 남은 SITL 정리 |
| MAVProxy 연결 관련 `mav` 에러가 반복됨 | 남은 SITL·Gazebo 프로세스, 또는 실행 순서가 어긋남 | 전체 종료 `pkill -f arducopter; pkill -f sim_vehicle; pkill -f mavproxy; pkill -f "gz sim"` 후 Gazebo → 스폰 → SITL 순서로 재시작 (내 Gazebo 창도 닫힘) |
| 카메라 영상에서 벽과 바닥이 검게 나옴 (`down_camera`는 픽셀 값 전부 0) | 재질 없는 벽 visual과 `<pbr>`만 있는 바닥이 카메라 센서에서 검게 렌더링됨. `<scene>` 제거는 효과 없었음 | 벽 `<material>`에 ambient/diffuse 0.7, 바닥에 `<ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>` 추가 후 Gazebo 재시작 |
| `gz topic -e -t /front_camera/image`가 `\000`으로 도배됨 | 픽셀 바이트를 그대로 출력하는 것이라 원래 그렇게 보임 | `grep -E "width|height|pixel_format|step"`로 필드만 보고, 영상은 Image Display나 Python 프레임 통계(5.3절)로 확인 |
| `land` 입력 후 모드 표시가 `GUIDED`로 남음 | 착륙이 모드 변경이 아닌 명령으로 전달되는 것으로 보임 [미확인] | 착륙은 정상. 모드까지 바꾸려면 `mode land` |
| 이륙 명령 `MAV_CMD_NAV_TAKEOFF`가 `result 4`로 6번 거절됨 | 시동을 걸 때 모드가 GUIDED가 아니라 STABILIZE였음 | EKF 준비 후 `set_mode("GUIDED")`를 보내고 `flightmode`가 바뀐 것을 확인한 다음 시동 |
| pymavlink만 쓸 때 `LOCAL_POSITION_NED`가 오지 않음 | MAVProxy가 없어서 스트림 요청이 없음 | `request_data_stream_send`로 스트림 요청 |
| `param_request_read_send(..., index=-1)` 뒤 첫 `PARAM_VALUE`가 다른 파라미터 | 응답이 섞여 옴 | `param_id`를 비교해서 고른다 |
| `Arm: Accels inconsistent`가 약 1분 반복 | 시동 직후 센서 안정화 | 약 1분 뒤 풀린다. 시동 명령을 재시도 |
| `optical_flow_send() takes from 9 to 10 positional arguments but 11 were given` | 기본 MAVLink 1에는 확장 필드(`flow_rate_x/y`)가 없음 | 코드 맨 위에서 `os.environ["MAVLINK20"] = "1"` 후 `pymavlink` import |
| `fatal: not a git repository` (`git commit`) | 작업 폴더로 이동하지 않았거나 `git init`이 안 됨 | 작업 폴더로 이동하고 `git init` |

참고: 세션 종류에 따라 `-w`(파라미터 초기화) 사용이 다르다. 작업 기록 3장은 FRAME_CLASS를 수동으로 설정하던 때 "`-w` 없이 실행해야 설정이 유지됨"이라고 적었고, `run_flow_check.sh`는 `-w`와 `--add-param-file`을 같이 쓴다(파라미터가 매번 파일에서 들어가므로 문제없다). 평소 실행 명령(3.3절)에는 `-w`를 넣지 않았다.

## 10. 작성 예정

- 다중 드론 스폰: 작성 예정
- CSV 로거: 작성 예정
- 2대 스침 시험: 작성 예정

## 부록: 이 문서에서 [미확인]으로 남긴 항목 모음

- A의 OS·`versions.txt`·ArduPilot/플러그인 커밋과의 일치 여부, OS 통일 여부
- 저장소를 다른 PC로 옮기는 방법
- 3.3절 전체 SITL 명령(`-I0 --sysid 1 ... --add-param-file ... --console`)의 단일 실행 결과, `python3`로 `sim_vehicle.py` 실행 가능 여부
- 3.2절 기본 iris 스폰 문자열(`<name>` 포함 형태)의 그대로 실행 여부
- `front_camera`를 기본 iris에 붙이는 고정 조인트 시험, 정면이 +x인지 확인
- `GZ_IP=127.0.0.1`의 효과
- 12m 방의 공 충돌 결과 (바닥 무늬는 카메라 영상과 GUI에서 확인됨)
- GUI 환경에서 `down_camera` 프레임 주기 (헤드리스 약 18Hz, 설정 30Hz)
- MAVProxy `--out=udp:127.0.0.1:15550`으로 센서 메시지를 SITL에 주입하는 경로, MAVProxy 사용 시 TCP 5762 이용 가능 여부
- 광류 장기 드리프트·정확도, 센서 잡음과 기체 기울기 상황
- `land` 입력 후 모드 표시가 GUIDED로 남는 이유
