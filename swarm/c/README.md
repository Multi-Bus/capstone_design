# swarm/c — 편대 제어 알고리즘 C99 포팅

`swarm/*.py`의 알고리즘을 STM32N657(Cortex-M55)에 올릴 수 있게 C99로 옮겼다. malloc과 재귀가 없고, 반복 횟수 상한이 정해져 있다. 파이썬 원본은 그대로 두며, 대시보드 가상 시뮬은 계속 파이썬을 쓴다.

| 파이썬 | C | 내용 |
| --- | --- | --- |
| `collision_avoidance.py` | `src/collision_avoidance.c` | QP(활성 제약 전수 확인), CBF-QP, APF, 교착 완화, 안전거리 |
| `formation_control.py`, `event_trigger.py` | `src/formation_event.c` | consensus 편대 속도, 웨이포인트 추종, 이벤트 트리거 |
| `path_planning.py` | `src/path_planning.c` | 장애물 부풀리기, A*, 지름길, 계획 |

API는 `include/swarm.h` 한 파일에 있다.

## 빌드와 시험

```powershell
gcc -std=c99 -Wall -Wextra -O2 -Iswarm/c/include -o swarm_test.exe swarm/c/test/test_swarm.c swarm/c/src/*.c -lm; ./swarm_test.exe
```

`-DSWARM_DOUBLE`을 붙이면 `real_t`가 double이 된다. 기본은 float다.

```powershell
python tests/test_swarm_c.py
```

위 파이썬 시험은 double 공유 라이브러리를 만들어 무작위 입력 수백 개로 파이썬 원본과 결과를 비교한다(QP, CBF, APF, 교착, 편대, 트리거, 부풀리기, A*, 계획). A*는 같은 비용의 다른 경로를 허용한다. 동점 처리 순서가 다르기 때문이다.

## 정적 메모리 (기본 상한)

| 항목 | 크기 | 비고 |
| --- | --- | --- |
| A* 작업 공간 | 칸 수 22,801개(151 x 151, 15 m / 0.1 m) x 20 B 정도 | `SWARM_MAX_CELLS`로 조절 |
| 부풀린 격자와 경로 | 약 114 KB | `swarm_plan` 안의 정적 배열 |
| QP 제약 | 32줄 | `SWARM_MAX_ROWS`. 이웃 + 장애물 + 상자 제약 4줄 |

합쳐도 약 1 MB 미만이라 N6의 SRAM(약 4.2 MB) 안에 들어간다.

## 쓸 때 지킬 것

- A*(`swarm_plan`)는 정적 작업 공간을 쓰므로 한 번에 한 태스크에서만 부른다. 제어 주기 안에서 부르지 말고, 목표나 장애물이 바뀔 때만 낮은 우선순위 태스크에서 돌린다.
- QP는 2차원(x, y)만 푼다. 파이썬 `solve_qp`는 d = 2 ~ 3을 받지만 기체 속도 제어는 2차원이다.
- float 빌드는 QP 허용치를 파이썬(double)보다 넓게 잡았다(`DET_TOL`, `FEAS_TOL`). float 결과와 double 결과가 가장자리 경우에서 다를 수 있다.
- CBF 이웃의 `d`가 0 이하이면 기본 `D`를 쓴다. APF에서는 `d`를 표면까지 거리를 재는 반경으로 쓴다(파이썬의 선택 인자 `D_j`와 같다).
- 이 PC에서 잰 시간은 CBF-QP(제약 12줄) 1 us 안팎, 계획(151 x 151) 1 ms 안팎이다. **N6 시간이 아니다.** N6에서는 `docs/진행현황.md`의 Task 3 Lv3로 따로 잰다.

## N6에서 시간 재기

`firmware/nucleo/n6_algo_bench/`가 이 라이브러리를 링크해서 NUCLEO-N657 보드에서 시간을 잰다. FSBL RAM이 256 KB라서 A* 작업 공간을 `SWARM_USE_AXISRAM1`로 AXISRAM1에 둔다(`SWARM_BIG`).

## 아직 안 한 것

- `n6_algo_bench`를 보드에서 실제로 돌려 시간을 재는 일.
- 기체 코드(`firmware/nucleo/n6_bridge`)와의 연결. 지금 브리지는 BLE와 FC 사이를 잇기만 한다.
