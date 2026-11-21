# swarm — 트랙 A (우리 방식 baseline, laptop 파이썬)

계획: `충돌회피_편대제어_구현계획` v2 (2026-11-20) 5절 1단계 "파이썬 모듈".

| 파일 | 내용 |
| --- | --- |
| `path_planning.py` | A* (0.1 m 격자, 8방향), 장애물 부풀림(편대 반경 + D), 지그재그 다듬기 |
| `formation_control.py` | consensus 편대 속도, 편대 중심 웨이포인트 추종 |
| `collision_avoidance.py` | CBF-QP(활성 제약 전수 확인 QP), APF, 교착 탐지 · 완화, 안전거리 식 |
| `event_trigger.py` | 이벤트 트리거 송신 (등속 예측 오차 > δ, 경로 변경, 최소 1 Hz), D_eff = D + 2δ |
| `simulator.py` | 2D 다기체 시뮬. 1차 지연 τ, 위치 잡음, 통신 지연, 주기 / 이벤트 송신 |
| `../tests/test_swarm.py` | 모듈 점검 (문서 2.2 계산 예 포함) |

레포 루트에서 실행한다.

```powershell
python tests/test_swarm.py
```

```powershell
python -m swarm.simulator --scenario swap --avoid cbf --comm event --delta 0.1
```

시나리오: `swap`(6대 원 위 정반대편 이동), `formation`(육각 대형이 가운데 기둥을 A*로 돌아 15 m 방 횡단).
출력: 최소 이격, 충돌 수(0.3 m 미만 진입), QP 해 없음 횟수, 교착 횟수, 도달 시간, 초당 메시지 수, 편대 오차.

아직 없는 것: trajectory repair와 통신 최적화(논문 CBF-Link로 마지막에 합침), Gazebo SITL(2단계), A* 재실행 조건(맵에 없던 장애물 발견).
시뮬 값은 이상 모델 결과다. 실측 τ, 지연, 잡음으로 다시 돌려야 한다.
