# 다중 드론 스폰 (`scripts/spawn_drones.py`)

## 사용
```bash
source ~/drone_project/env.sh
gz sim -s -r worlds/room.sdf &                       # (또는 GUI)
python3 scripts/spawn_drones.py 0 1                   # 기본 위치
python3 scripts/spawn_drones.py 1 2 --pos 1=3,-2 --pos 2=-4,3,0.2,1.57   # ID=x,y[,z[,yaw]]  ENU m / rad
python3 scripts/spawn_drones.py 0 1 --world room --gen-only              # SDF만 생성
```
python 은 gz 바인딩이 있는 것(`gz.transport13`, `gz.msgs10`)을 쓴다. 격리 시험은 `GZ_PARTITION`을 서버/스크립트에 똑같이 준다.

## 규칙
| 항목 | 값 |
|---|---|
| 모델 이름 | `drone_{i}` |
| 플러그인 `fdm_port_in` | `9002+10i` |
| SITL | `-I{i} --sysid {i+1}`, TCP `5760+10i` |
| 센서 토픽 | `/drone_{i}/front_camera/image`, `/drone_{i}/down_camera/image`, `/drone_{i}/down_range/scan` |
| 기본 위치 (ENU, z=0.2, yaw 0) | 0:(-2,0) 1:(2,0) (A 가이드) / 2:(-2,3) 3:(2,3) / 4:(-2,-3) 5:(2,-3). 6 이상은 `--pos` 필수 |
| 위치 제한 | 스폰 x,y 는 벽 ±6m 안쪽 0.5m(|x|,|y|≤5.5). 기본 배치 간격은 최소 4m(x), 3m(y) |

## 생성물
- `models/generated/drone_{i}/model.sdf` (git 제외). 원본 `iris_with_ardupilot`는 수정하지 않고 복사해 모델 이름·포트만 바꾼다.
- 센서는 `sensors/{front_camera,down_camera,down_range}/model.sdf`의 링크를 읽어 와 토픽만 드론별로 바꿔 넣고, 각각 `iris_with_standoffs::base_link`에 **fixed 조인트**로 고정한다(링크 `*_link`, 조인트 `*_joint`).
  장착 pose(본체 기준): front_camera (0.1, 0, 0.05), down_camera/down_range (0, 0, -0.05). 해상도·FOV·주기는 sensors/* 값 그대로(정면 640x480 15Hz FOV 1.5708, 하향 320x240 30Hz FOV 1.0472, 거리 30Hz 0.1~5m).
- 스폰은 `/world/<world>/create` (EntityFactory, `sdf_filename`).
- 모델 `drone_{i}` 안에 `iris_with_standoffs`가 중첩되므로 imu/조인트 이름은 원본과 같은 `iris_with_standoffs::...`.

## 검증 결과 (파티션 agentB, 드론 1·2, 위치 (3,-2), (-4,3))
- `gz sdf -k` (SDF_PATH=$GZ_SIM_RESOURCE_PATH): drone_1, drone_2 모두 `Valid.` (imu `gz_frame_id` 경고는 원본 iris 자체의 것).
- 스폰 `OK`, 토픽 6개 존재, down_camera 프레임 drone_1 min 128/max 189/mean 171.02, drone_2 min 105/max 198/mean 168.43 (320x240 RGB).
- SITL 인스턴스 1·2: heartbeat sysid 2·3, 로그에 `no JSON sensor message received` 0건, GUIDED -> arm -> 이륙 2m (z 0.19 -> 2.19, 둘 다).
- drone_1 북쪽 0.5 m/s 6초 명령 후 ENU 변화 drone_1 (0.00, +1.72, -0.01), drone_2 (0.00, 0.00, 0.00). 이후 LAND 로 둘 다 z 0.19.
- 센서 고정: 호버 중 down_range 가 둘 다 약 2.13~2.14 m (지상에서는 0.145)로 드론 고도를 따라감. 이동 전/후 down_camera 프레임 평균절대차 drone_1 34.53, drone_2 6.73. 정면 카메라는 벽 앞의 단색에 가까운 영상이라 차이 0.00/0.01로 이동 증거로는 쓸 수 없음.
- 한계: 이동 거리 1.72m(명령 속도 0.5m/s x 6s = 3m 보다 작음, 가속/감속 구간 포함 추정, 원인 미분석). 드론 0과 3대 이상 동시 실행은 시험하지 않음.
