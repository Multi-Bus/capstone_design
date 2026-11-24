# n6_algo_bench — N6에서 알고리즘 시간 재기 (Task 3 Lv3)

`swarm/c`(편대 제어, CBF-QP, A* 경로 계획의 C 포팅)를 NUCLEO-N657X0-Q에서 돌려 시간을 잰다. 연산량이 모자라 제어 주기가 밀리는지 확인하는 것이 목적이다. 결과는 ST-LINK 가상 COM 포트로 한 줄씩 나온다.

**상태: 헤드리스 빌드 성공(오류 0, 경고 0). 보드에서는 아직 돌려 보지 않았다.**

## 무엇을 재는가

| 줄 | 내용 |
| --- | --- |
| `BENCH,cbf_qp_random` | CBF-QP 1회. 이웃 드론 5 + 기둥 3 + 상자 제약 4 = 제약 12줄, 무작위 입력 10,000회 |
| `BENCH,cbf_qp_surrounded` | 해가 없는 최악 입력(8방향에서 다가옴). 모든 조합을 확인하므로 가장 오래 걸린다 |
| `BENCH,control_step` | 기체 한 대의 제어 1주기: 편대 속도 → 교착 완화 → CBF-QP → 송신 판단 |
| `BENCH,plan_151x151` | 경로 계획 1회. 15 m 방, 0.1 m 격자, 기둥 3개, 부풀리기 1.2 m, 시작과 목표는 무작위 30회 |
| `LOAD,tick1k_idle` | SysTick 1 kHz 인터럽트에서 제어 1주기를 5초 돌림. 메인 루프는 놀고 있음 (비교 기준) |
| `LOAD,tick1k_with_plan` | 같은 조건에서 메인 루프가 경로 계획을 계속 돌림. **주기가 밀리는지 보는 핵심 시험** |
| `BENCH,*_nodcache` | D-Cache를 끄고 CBF-QP와 경로 계획을 다시 잼 |

시간은 DWT 사이클 카운터로 잰다. `BENCH` 측정 중에는 인터럽트를 잠깐 막아 순수 연산 시간만 잰다.

## 출력 예와 읽는 법

```
BOOT,fw=n6_algo_bench-v1,cpu_hz=600000000,...
CHECK,cyccnt_per_10ms=6000000
BENCH,cbf_qp_random,n=10000,min_ns=...,p50_ns=...,p99_ns=...,max_ns=...,avg_ns=...,max_cyc=...,a=<해 있음>,b=<해 없음>
LOAD,tick1k_with_plan,ticks=...,period_avg_ns=...,period_max_ns=...,jitter_max_ns=...,late=...,step_avg_ns=...,step_max_ns=...,plans=...,plan_fail=...
DONE
```

- `CHECK`가 `cpu_hz / 100` 근처가 아니면 사이클 카운터나 클럭 설정이 잘못된 것이다. 이때는 다른 숫자를 믿지 않는다.
- `BENCH`의 `a`, `b`: CBF-QP는 해 있음과 해 없음 횟수, 경로 계획은 성공과 실패 횟수다. 경로 계획 실패는 시작점이나 목표가 부풀린 기둥 안에 떨어진 경우다.
- 판정 기준:
  - `control_step`의 `max_ns`가 제어 주기(50 Hz면 20 ms, 1 kHz면 1 ms)보다 충분히 작아야 한다.
  - `LOAD,tick1k_with_plan`의 `late`(1.5 ms 넘게 밀린 주기)가 0이어야 한다.
  - `jitter_max_ns`가 `tick1k_idle`과 비슷해야 한다. 경로 계획이 제어 인터럽트를 밀어내지 않는다는 뜻이다.
- 결과는 `docs/진행현황.md` Task 3 Lv3 항목에 옮겨 적는다.

## 실행 (개발 모드, 외부 플래시는 건드리지 않음)

1. NUCLEO 점퍼를 개발 모드로 둔다: `BOOT1` 2–3 (`docs/호버링_시험_절차.md` 2-1절과 같음).
2. STM32CubeIDE 2.x에서 **File → Import → Existing Projects into Workspace**로 `nucleo/n6_algo_bench/STM32CubeIDE/FSBL`을 연다. 프로젝트 이름은 `n6_algo_bench_FSBL`이다.
3. Build(Debug 설정, `-O2`) 후 **Debug As → STM32 C/C++ Application**으로 올린다. 펌웨어는 RAM(AXISRAM2)에 올라가 실행되고 외부 플래시의 브리지(`n6_bridge`)는 그대로 남는다.
4. 터미널(115200 8N1)로 ST-LINK 가상 COM 포트를 연다. 모든 측정이 끝나기까지 약 15초 걸리고 마지막에 `DONE`이 나온다. LED1이 깜빡이면 끝난 것이고, LED2가 켜지면 오류다.
5. 끝나면 `BOOT1`을 1–2로 돌리고 전원을 다시 넣는다. 브리지로 부팅한다.

드론에 달린 NUCLEO로 해도 된다. 외부 플래시에 쓰지 않기 때문이다. 다만 FC와 XIAO 배선은 이 펌웨어가 쓰지 않으므로 비행 시험과 같이 하지 않는다.

헤드리스 빌드:

```powershell
& 'C:\ST\STM32CubeIDE_2.3.0\STM32CubeIDE\stm32cubeidec.exe' --launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data <빈 작업 폴더> -import '<레포>\nucleo\n6_algo_bench\STM32CubeIDE\FSBL' -build 'n6_algo_bench_FSBL/Debug'
```

## 구성

| 경로 | 내용 |
| --- | --- |
| `FSBL/Src/main.c` | 측정 본체. 클럭 설정은 `n6_bridge`와 같다 |
| `FSBL/Src/stm32n6xx_hal_msp.c` | USART1(ST-LINK VCP, PE5/PE6, AF7) 클럭과 핀 |
| `FSBL/Src/stm32n6xx_it.c` | `n6_bridge`와 같고, SysTick에서 `Bench_Tick()`을 부른다 |
| `FSBL/Inc/*`, `FSBL/Src/system_stm32n6xx_fsbl.c` | `n6_bridge`에서 복사 |
| `STM32CubeIDE/FSBL/` | CubeIDE 프로젝트. 링커 스크립트에 `AXISRAM1` 구역을 추가했고 스택을 8 KB로 늘렸다 |

- 드라이버(HAL, CMSIS, BSP)는 복사하지 않고 `nucleo/n6_bridge/Drivers`를 같이 쓴다.
- 알고리즘 소스는 `swarm/c/src`를 링크로 가져온다(`Application/Swarm`).
- `SWARM_USE_AXISRAM1`을 정의해서 A* 작업 공간을 `.axisram1` 구역에 둔다.

## 알아 둘 것

- **CPU 클럭은 600 MHz다.** `n6_bridge`와 같은 클럭 설정(PLL1 1200 MHz, IC1 /2)이다. 800 MHz로 올리는 설정은 이번 범위 밖이다. 부팅 줄의 `cpu_hz`를 확인한다.
- **AXISRAM1 사용은 보드에서 확인하지 않았다.** FSBL용 RAM은 256 KB뿐이다. 그래서 A* 작업 공간 등 약 800 KB를 `0x34000000`(AXISRAM1, 보안 별칭)에 두었다. 처음 실행에서 HardFault나 멈춤이 나면 이 구역부터 의심한다. 이때는 `swarm.h`의 `SWARM_MAX_CELLS`를 줄여(예: 76 x 76, 0.2 m 격자) 256 KB 안에 넣고 다시 잰다.
- 펌웨어는 기체와 통신하지 않는다. FC, BLE, UWB 핀을 건드리지 않는다.
