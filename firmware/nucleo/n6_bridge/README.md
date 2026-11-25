# n6_bridge — NUCLEO-N657X0-Q 투명 UART 브리지 (FSBL 단독)

```
XIAO nRF52840 --USART2 115200 (PA2 TX / PF6 RX, AF7)--> [NUCLEO] --UART7 921600 (PH4 TX / PE7 RX, AF8)--> FC SERIAL4
```

바이트를 양방향으로 그대로 넘깁니다(인터럽트·DMA 없음, 폴링). MAVLink는 링크 끊김 강제 DISARM을 위해 HEARTBEAT만 읽습니다.
빌드·플래시·시험 절차는 저장소 루트의 **`README_KO.md`** 를 보세요.

## 출처

STM32CubeN6 **V1.4.1** 의 `Projects/NUCLEO-N657X0-Q/Examples/UART/UART_TwoBoards_ComPolling` 을 복사해 만들었습니다.
원래 예제는 `PARENT-7-PROJECT_LOC` 로 STM32CubeN6 루트의 `Drivers/` 를 참조하므로,
여기서는 필요한 드라이버만 `Drivers/` 에 동봉하고 경로를 `PARENT-2-PROJECT_LOC` / `../../../Drivers` 로 바꿨습니다.

| 경로 | 내용 |
|---|---|
| `FSBL/Src/main.c` | 브리지 본체 (링버퍼 2×4 KB, 폴링 루프, 보드레이트 오차 검사, LED) |
| `FSBL/Inc/link_failsafe.h` | 링크 끊김 강제 DISARM. 양방향 HEARTBEAT만 읽는다. 노트북 하트비트가 5초 넘게 없고 FC가 LAND + 시동 ON을 9초 이어 가면 FC에 `COMMAND_LONG 400`(param2 21196)을 넣는다 |
| `test/test_link_failsafe.py` | 위 헤더의 호스트 시험 (gcc + pymavlink). `python firmware/nucleo/n6_bridge/test/test_link_failsafe.py` |
| `FSBL/Src/stm32n6xx_hal_msp.c` | USART2/UART7 커널 클럭(PCLK1)·핀 설정 |
| `FSBL/Inc/main.h` | 링버퍼 크기, 핀 정의 |
| `FSBL/Src/stm32n6xx_it.c`, `system_stm32n6xx_fsbl.c`, 그 외 `Inc/*` | 예제 그대로 |
| `STM32CubeIDE/FSBL/` | CubeIDE 프로젝트 `n6_bridge_FSBL` (경로·이름만 수정) |
| `Drivers/` | HAL Inc 전체 + 쓰는 Src 12개, CMSIS 코어, N657 디바이스 헤더, Nucleo BSP (V1.4.1 그대로) |

EWARM/MDK-ARM 프로젝트와 `.ioc` 는 뺐습니다. **CubeMX 로 코드를 다시 생성하지 마세요**
(예제의 `.ioc` 는 USART1 설정이라 브리지 코드를 덮어씁니다).

## 서명 (trusted.bin 만들기)

Release 빌드 뒤 아래처럼 서명한다. **`-align`을 빼면 페이로드가 0x400에 오지 않아 부팅하지 않는다**(2026-11-25 6호기에서 겪음).

```powershell
$p = "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin"
& "$p\STM32_SigningTool_CLI.exe" -bin n6_bridge_FSBL.bin -nk -of 0x80000000 -t fsbl -hv 2.3 -align -o n6_bridge_FSBL-trusted.bin -s
```

## 디버거로 볼 변수 (Live Expressions)

- D-Cache 때문에 디버거는 RAM 의 옛 값을 볼 수 있어서, 펌웨어가 0.5 s 마다(LED 토글 때) 아래 변수들을
  캐시에서 RAM 으로 내보냅니다(`Bridge_PublishDebug`). 값은 0.5 s 단위로 갱신됩니다.
- Debug 세션은 **리셋부터 계속 연결한 채로** 보세요. 이미 돌고 있는 보드에 attach 하면 AP1 접속이 실패하는 것을 봤습니다.
- UART `RDR`(오프셋 0x24)은 디버거로 읽지 마세요 — 읽으면 RX FIFO 에서 바이트가 빠집니다.

| 변수 | 정상값 |
|---|---|
| `g_pclk1_hz` | 100000000 |
| `g_stat_ble.baud_err_ppm` (USART2) | 약 60 (0.006 %) |
| `g_stat_fc.baud_err_ppm` (UART7) | 약 4523 (0.45 %) |
| `g_stat_*.rx_bytes / tx_bytes` | 링크가 살아 있으면 계속 증가 |
| `g_stat_*.err_ore / err_fe` | 0 근처 (FE 가 계속 늘면 보드레이트/배선 문제) |
| `rb_fc2ble.dropped` | FC 가 115200 보다 많이 보내면 증가 (스크립트가 스트림을 끄면 멈춰야 함) |
| `g_xiao_link_up` | PD11 상태 (1 = XIAO 가 BLE 연결됨이라고 표시) |
