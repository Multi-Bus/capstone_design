# N6 드론 FC 세팅 가이드

DAKEFPV H743 FC(ArduCopter 4.7.x)에 N6 드론 공통 파라미터를 넣고, 보정하고, 확인하고, 기록하는 순서입니다.
기체 한 대에 40분~1시간 정도 걸립니다. **순서대로** 하세요.

이 폴더에 든 것:

| 파일 | 하는 일 | FC에 쓰는 것 |
|---|---|---|
| `setup.sh` | 처음 한 번: 파이썬 환경 만들기, 권한 확인 | 없음 |
| `n6_base.param` | 모든 N6 기체에 공통으로 넣는 값 (47개) | - |
| `drones/drone<N>.param` | 기체마다 다른 값 (모터 방향 등) | - |
| `apply_params.py` | 백업 → 값 넣기 → 재부팅 → 다시 읽어 확인 | 파라미터, 재부팅 |
| `orient_check.py` | FC 장착 방향이 맞는지 확인 | 없음 (읽기만) |
| `sensor_check.py` | 재부팅 후 경고 메시지, 거리센서/광류/배터리 확인 | 재부팅만 |
| `save_params.py` | 최종 점검 + 최종 파라미터 파일 저장 | 없음 (읽기만) |

> 스크립트는 **시동(ARM), 모터 테스트, 이륙 명령을 보내지 않습니다.** FC가 시동 상태로 보이면 바로 멈춥니다.

> Claude Code를 쓸 때는 이 폴더에서 실행하세요. `CLAUDE.md`(안전 규칙, Windows에서 고칠 곳)를 자동으로 읽습니다.

---

## 0. 안전 수칙 — 먼저 읽기

1. **프롭은 세팅이 끝날 때까지 떼어 둡니다.** 모터 테스트할 때도 프롭은 없어야 합니다.
2. 이 기체는 조종기(RC) 없이도 시동이 걸리게 설정됩니다. 배터리가 꽂혀 있으면 **언제든 모터가 돌 수 있다**고 생각하세요.
3. 배터리는 필요한 단계(4·5·6단계)에서만 꽂고, 끝나면 바로 뺍니다. 1~3단계는 **USB 전원만** 씁니다.
4. QGroundControl(QGC)과 스크립트는 FC 포트를 동시에 못 씁니다. **스크립트를 돌리기 전에 QGC를 끄세요.**
5. `MOT_SPIN_ARM`, `MOT_SPIN_MIN`은 건드리지 않습니다. 담당자가 정합니다. 스크립트도 이 두 값은 쓰지 않습니다.
6. 이상한 메시지가 나오거나 모르는 게 있으면 **멈추고** 담당자에게 화면을 보여 주세요.

---

## 1. 준비

### 준비물
- Ubuntu 22.04 노트북 (Windows는 Claude Code에게 `CLAUDE.md`의 "Windows 에서 쓸 때"대로 고쳐 달라고 하세요)
- 데이터용 USB-C 케이블 (충전 전용 케이블은 안 됨)
- QGroundControl, 멀티미터, 충전된 4S 배터리

### 처음 한 번만
```bash
cd n6_fc_setup
./setup.sh
```
- `dialout 그룹이 아니라서` 경고가 나오면: `sudo usermod -aG dialout $USER` 실행 후 **로그아웃/로그인**.
- `ModemManager 가 켜져 있습니다` 경고가 나오면: `sudo apt remove modemmanager`. FC를 꽂을 때 포트를 건드려서 연결을 끊습니다.
- 마지막에 `준비 끝.`이 나오면 됩니다.

이 문서의 명령은 모두 `n6_fc_setup` 폴더 안에서 실행합니다.

### 배선 확인

| 장치 | FC 커넥터 | FC 포트 | 설정값 |
|---|---|---|---|
| MTF-01P (광류 + 거리센서) | GPS 커넥터 | SERIAL1 | MAVLink, 115200 |
| NUCLEO (노트북 BLE 링크) | HD VTX 커넥터 | SERIAL4 | MAVLink2, 921600 |

- **HD VTX 커넥터의 RX4는 FC의 DJI 패드를 납땜해야 연결됩니다.** drone1은 납땜 전에 노트북 → FC 방향 신호가 하나도 안 들어왔습니다.
- FC 장착 방향: drone1은 FC가 **뒤집힌 채(윗면이 아래), 화살표가 기체 뒤쪽**을 보게 달려 있습니다. 이 방향은 `AHRS_ORIENTATION = 12 (PITCH_180)`로 `n6_base.param`에 들어 있습니다. 다르게 달았으면 3단계에서 잡힙니다.

---

## 2. 전체 순서

| 단계 | 할 일 | 배터리 | 프롭 | QGC |
|---|---|---|---|---|
| 1 | 기체 번호 정하고 기체별 값 파일 만들기 | - | - | - |
| 2 | 파라미터 넣기 (`apply_params.py`) | **빼기** | 떼기 | 끄기 |
| 3 | FC 장착 방향 확인 (`orient_check.py`) | 빼기 | 떼기 | 끄기 |
| 4 | QGC: 가속도계 6방향 + 수평 보정 | 빼기 | 떼기 | **켜기** |
| 5 | QGC: 모터 방향 확인 → 모터 방향 값 넣기 | **꽂기** | 떼기 | 켜기 → 끄기 |
| 6 | 센서·전압 확인 (`sensor_check.py`) | 꽂기 | 떼기 | 끄기 |
| 7 | 최종 점검 + 저장 (`save_params.py`) → 배터리 빼기 | 꽂기 → 빼기 | 떼기 | 끄기 |
| 8 | 기록표 채우기 | - | - | - |

---

## 3. 단계별 방법

아래 예시는 **3번 기체**입니다. `3`을 자기 기체 번호로 바꾸세요.

### 1단계 — 기체 번호, 기체별 값 파일

```bash
cp drones/droneN.param.example drones/drone3.param
```
지금은 파일을 그대로 두세요. 모터 방향 값은 5단계에서 채웁니다. 이 파일은 `n6_base.param`보다 우선합니다.

### 2단계 — 파라미터 넣기 (USB만, 배터리 X)

1. 배터리를 빼고 USB만 꽂습니다. QGC를 끕니다.
2. 먼저 **읽기만** 해서 무엇이 바뀔지 봅니다.
   ```bash
   .venv/bin/python apply_params.py --dry-run --drone 3
   ```
   - 표에 `[변경예정]`으로 바뀔 값이 나옵니다. 아무것도 쓰지 않습니다.
   - 새 FC는 `지금 없는 이름`에 `MOT_...` 같은 값이 나올 수 있습니다. 프레임 설정 뒤에 생기는 값이라 정상입니다.
3. 실제로 넣습니다. 재부팅을 1~3번 하고 2~4분쯤 걸립니다. **끝날 때까지 USB를 뽑지 마세요.**
   ```bash
   .venv/bin/python apply_params.py --drone 3
   ```
4. 끝에 이렇게 나와야 합니다.
   ```
   이름이 없어서 못 넣은 값: 없음
   최종 확인 불일치: 없음
   ```
5. 이 단계에서 나오는 PreArm 메시지는 아래와 같으면 정상입니다 (아직 보정 전이고 배터리가 없어서 나옴).
   - `PreArm: 3D Accel calibration needed` → 4단계에서 해결
   - `PreArm: Battery 1 low voltage failsafe` → USB만 꽂아서 나옴
   - `PreArm: Rangefinder 1: No Data` → 배터리가 없어 MTF-01P가 꺼져 있어서 나옴
   - `PreArm: RC not found`, `Compass 1 not healthy` → 값을 넣기 전 메시지. 값을 넣고 나면 사라집니다.

   이 밖의 메시지가 나오면 담당자에게 보여 주세요.

자동으로 백업 파일(`backup_<날짜시간>.param`)이 먼저 저장되므로, 잘못돼도 원래대로 되돌릴 수 있습니다.

### 3단계 — FC 장착 방향 확인 (USB만)

```bash
.venv/bin/python orient_check.py --drone 3
```
화면 안내대로 세 자세를 하나씩 잡고 Enter를 누릅니다. 자세마다 2초 동안 **가만히** 있어야 합니다.

1. 평평한 책상에 바로 놓기
2. **기수(앞쪽)를 아래로** 30~60° 숙여서 잡기
3. **오른쪽을 아래로** 30~60° 기울여서 잡기 (뒤에서 봤을 때 오른쪽)

결과:
- `지금 값 12 이 맞아요. 바꿀 필요 없음.` → 4단계로 갑니다.
- `AHRS_ORIENTATION 을 12 -> X 로 바꿔야 해요.` → 화면 안내대로 합니다.
  1. `drones/drone3.param`에 `AHRS_ORIENTATION,X`를 적습니다.
  2. `apply_params.py --drone 3`을 다시 돌립니다.
  3. `orient_check.py`를 다시 돌려 `바꿀 필요 없음`이 나오는지 확인합니다.
- `측정이 애매해요` → 기수/오른쪽을 정말 아래로 내렸는지 확인하고 다시 합니다.

> 앞/뒤를 헷갈리면 잘못된 값이 나옵니다. 기체 앞쪽이 어디인지 먼저 확인하세요.

### 4단계 — QGC 가속도계 보정 + 수평 보정 (USB만)

1. QGC를 켜고 FC 연결을 기다립니다.
2. **Vehicle Setup → Sensors → Accelerometer**
   - 창에 나오는 **Autopilot Orientation**이 `Pitch180`(QGC 버전에 따라 `ROTATION_PITCH_180`처럼 보임. 3단계에서 다른 값으로 바꿨으면 그 값)인지 확인하고 OK.
   - QGC 그림대로 6방향(바로, 왼쪽, 오른쪽, 기수 아래, 기수 위, 뒤집기)으로 놓습니다. 방향마다 움직이지 않게 둡니다.
3. **Sensors → Level Horizon (수평 맞추기)**: 평평한 곳에 바로 놓고 실행합니다.
4. Compass(나침반)는 이 보드에 없으니 **건너뜁니다**.
5. 확인: 기체를 기울인 쪽으로 QGC 화면의 자세(수평계)도 기울어야 합니다. 기수를 숙이면 기수 숙임, 오른쪽을 내리면 오른쪽 기울임.

> `AHRS_ORIENTATION`이 바뀌면 이 보정을 **다시** 해야 합니다.

### 5단계 — 모터 방향 (배터리 꽂기, 프롭 X)

**프롭이 없는지 다시 확인하세요.**

1. 배터리를 꽂습니다. QGC에서 **Vehicle Setup → Motors**를 엽니다.
2. 낮은 출력(5~10%)으로 모터를 하나씩 돌리면서 두 가지를 적습니다.
   - **어느 위치의 모터가 도는지**: ArduPilot은 1번부터 차례로 앞-오른쪽 → 뒤-오른쪽 → 뒤-왼쪽 → 앞-왼쪽(시계방향) 순서로 돕니다. 위치가 이 순서와 다르면 **멈추고 담당자에게** 알리세요 (배선/순서 문제).
   - **도는 방향** (위에서 볼 때). 모터 축에 테이프 조각을 붙이면 잘 보입니다.
3. 맞는 방향 (BetaFlightX, `FRAME_TYPE=12`):
   ```
                앞 (기수)
     앞-왼쪽: 시계(CW)     앞-오른쪽: 반시계(CCW)
                  \       /
                    [FC]
                  /       \
     뒤-왼쪽: 반시계(CCW)  뒤-오른쪽: 시계(CW)
                  뒤
   ```
4. **반대로 도는 모터**의 숫자를 더합니다.

   | 위치 | 뒤-오른쪽 | 앞-오른쪽 | 뒤-왼쪽 | 앞-왼쪽 |
   |---|---|---|---|---|
   | 숫자 | 1 | 2 | 4 | 8 |

   예) 네 개 다 반대 → 1+2+4+8 = **15**, 앞-왼쪽만 반대 → **8**, 다 맞음 → **0**

   - drone1은 네 개 모두 반대여서 **15**였습니다. 같은 방식으로 만든 기체는 15가 나올 가능성이 높지만, **반드시 직접 확인**하세요.
   - 이미 값을 한 번 넣은 뒤에 다시 테스트해서 또 반대인 모터가 있으면: 새 값 = 지금 값 XOR 이번 합계. 계산은 `python3 -c "print(15 ^ 4)"`처럼 합니다.
5. `drones/drone3.param`에 적습니다 (`#`을 지워서 켜기).
   ```
   SERVO_BLH_RVMASK,15
   ```
6. **QGC를 끄고** 다시 넣습니다. 이번에는 모터 방향 값만 바뀌고 재부팅을 한 번 합니다.
   ```bash
   .venv/bin/python apply_params.py --drone 3
   ```
7. QGC를 다시 켜고 모터 테스트를 한 번 더 해서 **네 개 모두 맞는 방향**인지 확인합니다. 그다음 QGC를 끕니다.

### 6단계 — 센서·전압 확인 (배터리 꽂기, 프롭 X)

1. **재부팅 후 경고 메시지와 센서 상태 확인** (약 1분)
   ```bash
   .venv/bin/python sensor_check.py reboot
   ```
   정상 결과 (drone1):
   ```
   == 재부팅 후 PreArm / Config ==
     (없음)
     광류 (OPTICAL_FLOW)        예  예  예
     거리 (LASER_POSITION)      예  예  예
     배터리 (BATTERY)           예  예  예
     배터리 전압 (SYS_STATUS): 16.63 V
   ```
   - `PreArm / Config`가 `(없음)`이어야 합니다. 뭔가 나오면 담당자에게 보여 주세요.
   - `PreArm`이 없다는 것은 **시동이 걸릴 수 있는 상태**라는 뜻입니다. 프롭을 끼우지 마세요.
2. **전압 확인**: 멀티미터로 배터리 전압을 재서 화면 값과 비교합니다.
   - 0.1 V 넘게 다르면: 새 `BATT_VOLT_MULT` = 16.0 × 멀티미터 값 ÷ 화면 값
     - 예) 멀티미터 16.80 V, 화면 16.50 V → 16.0 × 16.80 ÷ 16.50 = **16.29**
   - `drones/drone3.param`에 `BATT_VOLT_MULT,16.29`를 적고 `apply_params.py --drone 3`을 다시 돌린 뒤, 이 단계를 다시 합니다.
3. **거리센서·광류 확인** (15초)
   ```bash
   .venv/bin/python sensor_check.py watch
   ```
   - 시작하면 바로 기체를 바닥에서 **천천히 50 cm쯤 들어 올렸다가** 다시 내려놓습니다. 바닥 무늬가 있는 곳이 좋습니다.
   - 정상 결과 (drone1): `DISTANCE_SENSOR 거리: 최소 2 cm, 최대 48 cm`, 광류 quality 최대 100 이상.
   - 거리가 계속 같은 값이면 → 문제 해결의 "거리센서 값이 안 들어옴"을 보세요.
4. **광류 센서 방향** (`FLOW_ORIENT_YAW`): drone1은 MTF-01P가 기체 기준 180° 돌아가 있어서 `18000`(단위 0.01°)입니다.
   - 같은 방식으로 달았으면 같을 가능성이 높지만, 방향 시험(기체를 들고 롤/피치로 흔들어 광류와 자이로 비교)은 **담당자와 같이** 하세요.
   - 정해진 값은 `drones/drone3.param`에 `FLOW_ORIENT_YAW,18000`처럼 적고 `apply_params.py --drone 3`으로 넣습니다.

### 7단계 — 최종 점검 + 저장

보정이 끝난 뒤에 저장해야 보정 값까지 파일에 남습니다.
```bash
.venv/bin/python save_params.py --drone 3
```
- 마지막 `== 결론 ==`이 `문제 없음`이어야 합니다.
- `drone3_final_<날짜>.param` 파일이 생깁니다. 이 파일을 담당자에게 보냅니다 (같은 날 또 저장하면 이름 뒤에 시각이 붙습니다).
- 끝나면 **배터리를 뺍니다.**

### 8단계 — 기록표

| 기체 | 날짜 | 이름 | SERVO_BLH_RVMASK | AHRS_ORIENTATION | FLOW_ORIENT_YAW | BATT_VOLT_MULT | 최종 파일 | 비고 |
|---|---|---|---|---|---|---|---|---|
| 1 | 2026-11-19 | | 15 | 12 | 18000 | 16.0 | drone1_final_20261119.param | 멀티미터 비교 기록 없음. EKF 높이 값 확정 뒤 다시 저장 |
| | | | | | | | | |

`drones/drone<N>.param`도 같이 보관하세요. 나중에 FC를 바꾸거나 초기화하면 `apply_params.py --drone <N>` 한 번으로 같은 값이 다시 들어갑니다.

---

## 4. 넣는 값 설명 (`n6_base.param`)

| 구분 | 값 | 이유 |
|---|---|---|
| 프레임 | `FRAME_CLASS=1`, `FRAME_TYPE=12` | 쿼드, BetaFlightX 모터 순서. 이 값을 먼저 넣고 재부팅해야 `MOT_*`가 생깁니다. |
| 장착 방향 | `AHRS_ORIENTATION=12` | PITCH_180. drone1 측정 결과. 기체마다 다르면 `drones/`에 적습니다. |
| ESC | `MOT_PWM_TYPE=5` | DShot300 |
| MTF-01P | `SERIAL1_PROTOCOL=1`, `SERIAL1_BAUD=115`, `MAV2_OPTIONS=2`, `FLOW_TYPE=5`, `RNGFND1_TYPE=10`, `RNGFND1_ORIENT=25`, `RNGFND1_MIN=0.01`, `RNGFND1_MAX=8` | GPS 포트로 MAVLink 수신. `MAV2_OPTIONS=2`는 센서 메시지를 다른 링크로 퍼뜨리지 않게 합니다. 거리센서는 아래 방향, 0.01~8 m. |
| 착지 높이 | `RNGFND1_GNDCLR=0.05` | 바닥에 놓였을 때 거리값 (실측 0.02 m, EKF가 0.05 m 아래는 0.05로 씀) |
| NUCLEO | `SERIAL4_PROTOCOL=2`, `SERIAL4_BAUD=921`, `OSD_TYPE2=0` | HD VTX 포트를 노트북 BLE 링크(MAVLink2, 921600)로. 그 포트의 OSD 기능은 끔. |
| EKF (광류) | `EK3_SRC1_POSXY=0`, `EK3_SRC1_VELXY=5`, `EK3_SRC1_POSZ=1`, `EK3_SRC1_VELZ=0`, `EK3_SRC_OPTIONS=0` | GPS 없이 광류로 수평 속도, 기압계로 높이 |
| EKF 높이 | `EK3_RNG_USE_HGT=70` | 낮게(약 3.9 m 아래) 천천히 날 때는 높이를 거리센서로 잡음. **시험 중, 바뀔 수 있음** (아래 5절) |
| 나침반 | `COMPASS_ENABLE=0` | 보드에 나침반 없음 |
| 링크 끊김 | `MAV_GCS_SYSID=250`, `FS_GCS_ENABLE=5`, `FS_GCS_TIMEOUT=3`, `FS_OPTIONS=0` | 노트북 프로그램(시스템 ID 250)의 신호가 3초 끊기면 자동 착륙 |
| RC 없이 비행 | `FS_THR_ENABLE=0`, `ARMING_SKIPCHK=64` | 조종기가 없으므로 RC 페일세이프 끄고 RC 시동 검사 건너뜀 |
| 배터리 | `BATT_MONITOR=4`, `BATT_VOLT_PIN=11`, `BATT_CURR_PIN=10`, `BATT_VOLT_MULT=16.0`, `BATT_AMP_PERVLT=83.3`, `BATT_LOW_VOLT=14.0`, `BATT_FS_LOW_ACT=1` | 전압/전류 측정. 14.0 V 아래면 착륙. |
| 초기 튜닝 | `MOT_BAT_VOLT_MAX=16.8`, `MOT_BAT_VOLT_MIN=13.2`, `MOT_THST_HOVER=0.25`, `INS_ACCEL_FILTER=10`, `INS_GYRO_FILTER=80`, `ATC_RAT_RLL/PIT_FLTD·FLTT=40`, `ATC_RAT_YAW_FLTT=40`, `ATC_RAT_YAW_FLTE=2` | 3인치 덕트 4S 초기값 (ArduPilot "Setting up for Tuning") |

펌웨어 버전에 따라 이름이 다른 값(`MAV_GCS_SYSID`↔`SYSID_MYGCS`, `ARMING_SKIPCHK`↔`ARMING_CHECK`, `RNGFND1_MIN/MAX/GNDCLR`↔`..._CM`/`GNDCLEAR`, `MAV2_OPTIONS`↔`SERIAL1_OPTIONS`)은 `apply_params.py`가 알아서 바꿔 넣습니다. 그래도 4.7.x 펌웨어를 쓰세요.

`n6_base.param`은 **모든 기체 공통**입니다. 한 기체만 다른 값은 여기 말고 `drones/drone<N>.param`에 적으세요.

---

## 5. 아직 정해지지 않은 것

- **EKF 높이 설정**: drone1 저고도 비행 시험 중입니다. 이륙할 때 EKF 높이가 실제보다 크게 올라가는 현상이 있었고, `EK3_SRC1_POSZ=2`(높이를 항상 거리센서로)로 바꾸는 안을 검토하고 있습니다. 정해지면 새 `n6_base.param`을 받아서 `apply_params.py --drone <N>`을 다시 돌리세요.
- **`MOT_SPIN_ARM` / `MOT_SPIN_MIN`**: 담당자가 모터 테스트로 정합니다.
- **튜닝 값**: 지금은 초기값입니다. 비행하면서 바뀝니다.

---

## 6. 결과 파일

스크립트를 돌리면 이 폴더에 생깁니다. 지우지 말고 담당자에게 같이 보내세요.

| 파일 | 내용 |
|---|---|
| `backup_<날짜시간>.param` | 값을 넣기 **전** 전체 파라미터 (되돌리기용) |
| `drone<N>_final_<날짜>.param` | 최종 전체 파라미터 |
| `apply_log_*.txt`, `apply_result_*.json` | `apply_params.py` 화면 기록 / 결과 |
| `orient_drone<N>_*.txt/json` | 장착 방향 측정 기록 |
| `sensor_reboot_*`, `sensor_watch_*` | 센서 확인 기록 |
| `save_log_drone<N>_*`, `save_result_drone<N>_*` | 최종 점검 기록 |

파라미터 파일 형식은 `이름,값`(Mission Planner 형식)입니다.

---

## 7. 문제 해결

| 증상 | 해결 |
|---|---|
| `포트 권한 오류` | `sudo usermod -aG dialout $USER` 후 로그아웃/로그인 |
| `FC 포트를 다른 프로그램이 잡고 있음` / `포트 사용 중` | QGC를 끄세요. 그래도 나오면 화면에 나온 프로그램 이름을 담당자에게. |
| `FC 포트를 찾지 못함` | USB 케이블이 데이터용인지, FC LED가 켜졌는지 확인. `ls /dev/serial/by-id/`에 `usb-ArduPilot_DAKEFPVH743_...`이 보여야 합니다. |
| 연결이 자꾸 끊김 | ModemManager 제거 (`sudo apt remove modemmanager`) |
| `FC가 시동(ARMED) 상태` | **즉시 배터리를 빼고** 담당자에게. |
| `이름이 없어서 못 넣은 값`이 있음 | 펌웨어 버전이 다를 수 있습니다. 화면 맨 위의 펌웨어 줄과 함께 담당자에게. |
| `최종 확인 불일치`가 있음 | `apply_params.py`를 한 번 더 돌리고, 그래도 나오면 담당자에게. |
| 배터리를 꽂아 두면 10분쯤 뒤 **삐삐** 소리 | ESC의 "기체 찾기" 비콘입니다. 오래 가만히 있으면 납니다. 배터리를 빼면 멈춥니다. |
| 거리센서 값이 안 들어옴 / 계속 같은 값 | 배터리가 꽂혀 있는지(MTF-01P 전원), GPS 커넥터 배선 확인. 그래도 안 되면 MicoAssistant로 MTF-01P 설정을 확인합니다 (MAVLink 출력, 115200, `mav_id` = 200). |
| 바닥에 있는데 EKF 높이가 0이 아님 | FC 전원을 완전히 껐다 켭니다 (USB와 배터리 모두 뽑았다가 다시). |
| 노트북 BLE 링크로 FC가 명령을 안 받음 | DJI 패드 납땜(1절 배선)과 `SERIAL4_*` 값 확인. NUCLEO/BLE 펌웨어는 담당자에게. |
| 처음 상태로 되돌리고 싶음 | `backup_*.param` 파일을 담당자에게 주세요 (Mission Planner의 Full Parameter List → Load from file로 되돌릴 수 있음). |
