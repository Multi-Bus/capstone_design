# 방 월드와 구조물 설치 가이드

## 월드 파일
- `worlds/room.sdf` (8m x 8m x 3m, 벽 두께 0.2m, 지붕 없음)
- 북쪽 벽(y=+4)에 문: x -0.5~0.5, 높이 2m
- 실행: `gz sim -v4 -r ~/capstone_sim/worlds/room.sdf`

## 구조물 추가 방법
1. `room.sdf`의 `</world>` 바로 위에 아래 모델을 붙여 넣는다.

        <model name="box_1">
          <static>true</static>
          <pose>1 1 0.5 0 0 0</pose>
          <link name="link">
            <collision name="collision"><geometry><box><size>1 1 1</size></box></geometry></collision>
            <visual name="visual"><geometry><box><size>1 1 1</size></box></geometry></visual>
          </link>
        </model>

2. `<pose>`는 `x y z roll pitch yaw`이고, x y z는 상자 중심이다. 바닥에 놓으려면 z = 높이/2.
3. `<size>`는 `가로(x) 세로(y) 높이(z)`이다.
4. `<collision>`과 `<visual>`을 둘 다 넣는다. visual만 있으면 보이기만 하고 통과된다.
5. 모델 이름(`name`)은 월드 안에서 겹치면 안 된다.

## 문이 있는 벽
- 벽 하나를 box 3개로 나눈다 (왼쪽 3.5m, 오른쪽 3.5m, 문 위 1m x 1m). 구멍을 파는 기능은 쓰지 않는다.

## 충돌 확인
- Gazebo를 켠 상태에서 공을 떨어뜨려 본다.

        gz service -s /world/room/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 3000 --req 'sdf: "<sdf version=\"1.9\"><model name=\"ball\"><pose>0 0 2 0 0 0</pose><link name=\"l\"><collision name=\"c\"><geometry><sphere><radius>0.2</radius></sphere></geometry></collision><visual name=\"v\"><geometry><sphere><radius>0.2</radius></sphere></geometry></visual></link></model></sdf>"'

- 바닥이나 벽 위에서 멈추면 충돌이 정상이다.

## 주의
- 월드에 시스템 플러그인(Physics, UserCommands, SceneBroadcaster, Imu, Sensors)이 모두 있어야 한다. Imu가 없으면 SITL에서 `no JSON sensor message received`가 난다.
- iris 높이: 이륙 높이는 방 높이 3m 안으로 한다 (예: `takeoff 2`).
