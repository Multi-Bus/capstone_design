/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    n6_bridge/FSBL/Src/main.c
  * @brief   N6 드론 "투명 UART 브리지" 펌웨어 (NUCLEO-N657X0-Q, FSBL 단독)
  *
  *   노트북 --BLE(NUS)--> XIAO nRF52840 --USART2 115200--> [이 보드] --UART7 921600--> FC
  *
  *   - 바이트를 양방향으로 그대로 넘기기만 한다. MAVLink 해석 없음.
  *   - 인터럽트/DMA 없이 메인 루프에서 레지스터를 폴링한다.
  *   - 방향별 4 KB 링버퍼 2개. 넘치면 "가장 오래된 바이트"를 버린다
  *     (FC 921600 -> BLE 115200 구간은 속도 차가 8배라 넘칠 수 있음.
  *      MAVLink 쪽은 CRC로 깨진 프레임을 버리고 다음 프레임에서 다시 동기화한다).
  *   - 링크 끊김 안전장치는 이 보드가 아니라 FC의 GCS 페일세이프가 담당한다
  *     (노트북이 0.2 s 마다 HEARTBEAT, 끊기면 FS_GCS_TIMEOUT 3 s 뒤 LAND).
  *     -> 예전 설계(이 보드가 PD11을 보고 하트비트를 대신 보내던 방식)는 폐기.
  *   - 예외 하나 (2026-11-25, link_failsafe.h): HEARTBEAT 만 읽어서, 노트북 하트비트가
  *     12 s 없는데 FC 가 LAND + 시동 ON 이면 (착지 감지 실패) 강제 DISARM 을 FC 로 넣는다.
  *     GUIDED 등 다른 모드에서는 아무것도 안 한다. 그 밖의 바이트는 지금처럼 그대로 넘긴다.
  *
  *   베이스: STM32CubeN6 V1.4.1
  *           Projects/NUCLEO-N657X0-Q/Examples/UART/UART_TwoBoards_ComPolling
  *   예제에서 바뀐 점:
  *     - 버튼 대기, 송수신 비교 로직 제거. SystemClock_Config 는 손대지 않음.
  *     - USART1(PE5/PE6, ST-LINK VCP) 대신 USART2(PA2/PF6) + UART7(PH4/PE7).
  *     - FIFO 모드 켬 (예제는 DisableFifoMode).
  *     - LED 초기화를 UART 초기화보다 앞으로 옮김 (UART 초기화 실패 시에도
  *       Error_Handler 의 LED2 가 실제로 켜지도록).
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include "link_failsafe.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* 단방향 링버퍼. head/tail 은 계속 증가하는 카운터(free-running)이고
 * (head - tail) 이 저장된 바이트 수. 인터럽트를 안 쓰므로 잠금이 필요 없다. */
typedef struct
{
  uint8_t  data[BRIDGE_BUF_SIZE];
  uint32_t head;      /* 다음에 쓸 위치 */
  uint32_t tail;      /* 다음에 읽을 위치 */
  uint32_t dropped;   /* 넘쳐서 버린 바이트 수 (디버거 확인용) */
} Ring_t;

/* UART 별 통계 (디버거 Live Expressions 로 확인용) */
typedef struct
{
  uint32_t rx_bytes;
  uint32_t tx_bytes;
  uint32_t err_ore;   /* overrun : 폴링이 RX FIFO 를 못 따라감 */
  uint32_t err_fe;    /* framing : 보드레이트 불일치/배선 */
  uint32_t err_ne;    /* noise */
  uint32_t err_pe;    /* parity (8N1 이라 원래 안 생김) */
  uint32_t baud_actual;   /* PCLK1 / BRR 로 계산한 실제 보드레이트 */
  uint32_t baud_err_ppm;  /* 설정값 대비 오차 (ppm, 10000 ppm = 1 %) */
} UartStats_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BLE_UART_BAUD          115200U   /* USART2 <-> XIAO nRF52840 */
#define FC_UART_BAUD           921600U   /* UART7  <-> FC SERIAL4 (SERIAL4_BAUD 921) */

#define BAUD_MAX_ERR_PPM       20000U    /* 2 %. 넘으면 Error_Handler (LED2) */

#define LED_PERIOD_MS          1000U     /* LED1 깜빡임 주기 = 1 Hz */

/* 한 번 폴링에서 RX FIFO 를 비우는 최대 바이트 수 (무한 루프 방지용 안전 상한) */
#define RX_DRAIN_MAX           32U

/* PD11(XIAO_INT) 로 LED1 깜빡임 모양을 바꿀지. 0 이면 단순 1 Hz 토글. */
#define BRIDGE_USE_XIAO_INT    1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define USART_ERR_FLAGS   (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)
#define USART_ERR_CLEAR   (USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart2;   /* USART2 : BLE(XIAO) 쪽, 115200 */
UART_HandleTypeDef huart7;   /* UART7  : FC 쪽, 921600 */

/* USER CODE BEGIN PV */

/* FC -> BLE 방향, BLE -> FC 방향 */
static Ring_t rb_fc2ble;
static Ring_t rb_ble2fc;

/* 디버거에서 보려고 static 을 안 붙임 */
UartStats_t g_stat_ble;   /* USART2 */
UartStats_t g_stat_fc;    /* UART7  */
uint32_t    g_pclk1_hz;
volatile uint8_t g_xiao_link_up;   /* PD11 상태 (1 = XIAO 가 BLE 연결됨이라고 표시) */
LinkFs_t g_linkfs;                 /* 링크 끊김 강제 DISARM 상태 (g_linkfs.cuts = 보낸 횟수) */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART7_Init(void);
/* USER CODE BEGIN PFP */
static void Bridge_CheckBaud(const UART_HandleTypeDef *huart, UartStats_t *st);
static void Bridge_PollUart(USART_TypeDef *uart, Ring_t *rx_to, Ring_t *tx_from, UartStats_t *st, uint8_t from_fc);
static void Bridge_LinkFailsafe(void);
static void Bridge_Led(void);
static void Bridge_PublishDebug(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static inline void Ring_Push(Ring_t *rb, uint8_t b)
{
  if ((rb->head - rb->tail) >= BRIDGE_BUF_SIZE)
  {
    /* 가득 참 -> 가장 오래된 바이트를 버리고 자리를 만든다 */
    rb->tail++;
    rb->dropped++;
  }
  rb->data[rb->head & (BRIDGE_BUF_SIZE - 1U)] = b;
  rb->head++;
}

static inline uint32_t Ring_Count(const Ring_t *rb)
{
  return rb->head - rb->tail;
}

static inline uint8_t Ring_Pop(Ring_t *rb)
{
  uint8_t b = rb->data[rb->tail & (BRIDGE_BUF_SIZE - 1U)];
  rb->tail++;
  return b;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* LED 를 먼저 켜 둬야 아래 UART 초기화가 실패했을 때 Error_Handler 의 LED2 가 보인다 */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_UART7_Init();
  /* USER CODE BEGIN 2 */

  /*
   * 보드레이트 오차 확인 (2 % 이하인지)
   * ----------------------------------------------------------------------
   * SystemClock_Config (예제 그대로):
   *   HSI 64 MHz / PLLM 4 * PLLN 75 = PLL1 1200 MHz
   *   IC2 = 1200 / 3 = 400 MHz (sys bus) -> AHB /4 = HCLK 100 MHz -> APB1 /1
   *   => PCLK1 = 100 MHz  (USART2, UART7 커널 클럭 = PCLK1)
   *
   *   OVER16, PRESC=DIV1 이면 BRR = round(PCLK1 / baud), 실제 baud = PCLK1 / BRR
   *   USART2 115200 : BRR = 868 -> 115 207 baud, 오차 +0.006 %
   *   UART7  921600 : BRR = 109 -> 917 431 baud, 오차 -0.45 %
   *   => 둘 다 2 % 이하. FC(H743) 쪽 자체 오차는 여기서 확인 못 함
   *      (양쪽 오차의 합이 대략 2~3 % 안이면 정상 동작).
   *
   * 아래 함수가 실행 중에 HAL_RCC_GetPCLK1Freq() 와 BRR 로 같은 계산을 다시 해서
   * 2 % 를 넘으면 Error_Handler(LED2) 로 멈춘다. 결과는 g_stat_*.baud_err_ppm.
   */
  g_pclk1_hz = HAL_RCC_GetPCLK1Freq();
  Bridge_CheckBaud(&huart2, &g_stat_ble);
  Bridge_CheckBaud(&huart7, &g_stat_fc);
  Bridge_PublishDebug();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 레지스터 접근은 huartX.Instance 로 한다. 이 프로젝트는 -mcmse(보안) 빌드라
     * USART2/UART7 매크로가 *_S 별칭으로 풀리는데, Instance 를 쓰면 HAL 초기화에
     * 쓴 것과 같은 주소를 쓰게 되어 보안/비보안 별칭이 섞일 일이 없다. */
    Bridge_PollUart(huart7.Instance, &rb_fc2ble, &rb_ble2fc, &g_stat_fc, 1U);   /* FC  쪽 */
    Bridge_PollUart(huart2.Instance, &rb_ble2fc, &rb_fc2ble, &g_stat_ble, 0U);  /* BLE 쪽 */
    Bridge_LinkFailsafe();
    Bridge_Led();
  }
  /* USER CODE END 3 */
}
/* USER CODE BEGIN CLK 1 */
/* USER CODE END CLK 1 */

/**
  * @brief System Clock Configuration
  * @note  예제(UART_TwoBoards_ComPolling) 그대로. 이 보드에서 검증된 설정.
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the System Power Supply
  */
  if (HAL_PWREx_ConfigSupply(PWR_EXTERNAL_SOURCE_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Enable HSI */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Get current CPU/System buses clocks configuration and if necessary switch
 to intermediate HSI clock to ensure target clock can be set
  */
  HAL_RCC_GetClockConfig(&RCC_ClkInitStruct);
  if ((RCC_ClkInitStruct.CPUCLKSource == RCC_CPUCLKSOURCE_IC1) ||
     (RCC_ClkInitStruct.SYSCLKSource == RCC_SYSCLKSOURCE_IC2_IC6_IC11))
  {
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_SYSCLK);
    RCC_ClkInitStruct.CPUCLKSource = RCC_CPUCLKSOURCE_HSI;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
    {
      /* Initialization Error */
      Error_Handler();
    }
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
  RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL1.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL1.PLLM = 4;
  RCC_OscInitStruct.PLL1.PLLN = 75;
  RCC_OscInitStruct.PLL1.PLLFractional = 0;
  RCC_OscInitStruct.PLL1.PLLP1 = 1;
  RCC_OscInitStruct.PLL1.PLLP2 = 1;
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_CPUCLK|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2|RCC_CLOCKTYPE_PCLK5
                              |RCC_CLOCKTYPE_PCLK4;
  RCC_ClkInitStruct.CPUCLKSource = RCC_CPUCLKSOURCE_IC1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_IC2_IC6_IC11;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;
  RCC_ClkInitStruct.APB5CLKDivider = RCC_APB5_DIV1;
  RCC_ClkInitStruct.IC1Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
  RCC_ClkInitStruct.IC1Selection.ClockDivider = 2;
  RCC_ClkInitStruct.IC2Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
  RCC_ClkInitStruct.IC2Selection.ClockDivider = 3;
  RCC_ClkInitStruct.IC6Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
  RCC_ClkInitStruct.IC6Selection.ClockDivider = 4;
  RCC_ClkInitStruct.IC11Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
  RCC_ClkInitStruct.IC11Selection.ClockDivider = 3;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function (BLE / XIAO 쪽, 115200 8N1)
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = BLE_UART_BAUD;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  /* 예제는 DisableFifoMode. 브리지는 폴링 사이 여유를 벌기 위해 FIFO 를 켠다. */
  if (HAL_UARTEx_EnableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief UART7 Initialization Function (FC 쪽, 921600 8N1)
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = FC_UART_BAUD;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable (UART 핀 포트는 HAL_UART_MspInit 에서도 켬) */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
#if BRIDGE_USE_XIAO_INT
  /* PD11 = XIAO_INT (XIAO D0, "BLE 링크 살아 있음"). 입력 + 내부 풀다운.
   * 이번 설계에서는 LED 표시에만 쓴다 (안전 로직에는 안 씀). */
  {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = XIAO_INT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(XIAO_INT_GPIO_Port, &GPIO_InitStruct);
  }
#endif
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  BRR 레지스터에서 실제 보드레이트를 거꾸로 계산해 오차를 확인한다.
  *         OVER16, PRESC=DIV1, 커널 클럭 = PCLK1 전제 (HAL_UART_MspInit 참고).
  *         오차가 BAUD_MAX_ERR_PPM(2 %)를 넘으면 Error_Handler.
  */
static void Bridge_CheckBaud(const UART_HandleTypeDef *huart, UartStats_t *st)
{
  uint32_t pclk = HAL_RCC_GetPCLK1Freq();
  uint32_t brr  = huart->Instance->BRR & 0xFFFFU;
  uint32_t want = huart->Init.BaudRate;

  if ((brr == 0U) || (want == 0U))
  {
    Error_Handler();
  }

  st->baud_actual  = pclk / brr;
  st->baud_err_ppm = (uint32_t)(((uint64_t)abs((int32_t)st->baud_actual - (int32_t)want) * 1000000ULL) / want);

  if (st->baud_err_ppm > BAUD_MAX_ERR_PPM)
  {
    Error_Handler();
  }
}

/**
  * @brief  UART 하나를 한 번 폴링한다.
  *         1) 오류 플래그가 있으면 ICR 로 지운다 (안 지우면 ORE 에서 수신이 막힐 수 있음)
  *         2) RXFNE 인 동안 RDR 을 읽어 rx_to 링버퍼에 넣는다
  *         3) TXFNF 이고 tx_from 에 보낼 게 있으면 TDR 에 쓴다
  */
static void Bridge_PollUart(USART_TypeDef *uart, Ring_t *rx_to, Ring_t *tx_from, UartStats_t *st, uint8_t from_fc)
{
  uint32_t isr = uart->ISR;
  uint32_t n;
  uint32_t now = HAL_GetTick();

  if ((isr & USART_ERR_FLAGS) != 0U)
  {
    if ((isr & USART_ISR_ORE) != 0U) { st->err_ore++; }
    if ((isr & USART_ISR_FE)  != 0U) { st->err_fe++;  }
    if ((isr & USART_ISR_NE)  != 0U) { st->err_ne++;  }
    if ((isr & USART_ISR_PE)  != 0U) { st->err_pe++;  }
    uart->ICR = USART_ERR_CLEAR;
  }

  for (n = 0U; (n < RX_DRAIN_MAX) && ((uart->ISR & USART_ISR_RXNE_RXFNE) != 0U); n++)
  {
    uint8_t b = (uint8_t)(uart->RDR & 0xFFU);
    Ring_Push(rx_to, b);
    st->rx_bytes++;
    if (from_fc != 0U) { LinkFs_OnFcByte(&g_linkfs, b, now); }
    else               { LinkFs_OnGcsByte(&g_linkfs, b, now); }
  }

  while ((Ring_Count(tx_from) != 0U) && ((uart->ISR & USART_ISR_TXE_TXFNF) != 0U))
  {
    uart->TDR = Ring_Pop(tx_from);
    st->tx_bytes++;
  }
}

/**
  * @brief  노트북 하트비트가 끊긴 채 FC 가 LAND + 시동 ON 이면 강제 DISARM 프레임을 FC 쪽으로 넣는다.
  *         노트북 바이트가 섞이지 않게 BLE->FC 링버퍼가 비어 있을 때만 넣는다 (끊긴 뒤라 보통 비어 있음).
  */
static void Bridge_LinkFailsafe(void)
{
  uint8_t frame[LINKFS_FRAME_LEN];
  uint32_t i;
  if (Ring_Count(&rb_ble2fc) != 0U) { return; }
  if (LinkFs_Poll(&g_linkfs, HAL_GetTick(), frame) == 0U) { return; }
  for (i = 0; i < LINKFS_FRAME_LEN; i++) { Ring_Push(&rb_ble2fc, frame[i]); }
}

/**
  * @brief  LED1 1 Hz 깜빡임 = 펌웨어 살아 있음. HAL_GetTick 기준, 루프를 막지 않음.
  *         BRIDGE_USE_XIAO_INT=1 이면 PD11 상태에 따라 모양만 바꾼다 (주기는 둘 다 1 s):
  *           PD11 high (XIAO: BLE 연결 + 최근 1 s 안에 데이터 수신) : 0.5 s 켜짐 / 0.5 s 꺼짐
  *           PD11 low  (BLE 미연결, 1 s 넘게 데이터 없음, 배선 없음)   : 0.05 s 짧게 깜빡 / 0.95 s 꺼짐
  */
static void Bridge_Led(void)
{
  static uint32_t period_start;
  static uint8_t  led_on;
  uint32_t elapsed = HAL_GetTick() - period_start;
  uint32_t on_ms = LED_PERIOD_MS / 2U;
  uint8_t  want_on;

#if BRIDGE_USE_XIAO_INT
  g_xiao_link_up = (HAL_GPIO_ReadPin(XIAO_INT_GPIO_Port, XIAO_INT_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  if (g_xiao_link_up == 0U)
  {
    on_ms = 50U;
  }
#endif

  if (elapsed >= LED_PERIOD_MS)
  {
    period_start += LED_PERIOD_MS * (elapsed / LED_PERIOD_MS);
    elapsed = HAL_GetTick() - period_start;
  }

  want_on = (elapsed < on_ms) ? 1U : 0U;
  if (want_on != led_on)
  {
    BSP_LED_Toggle(LED1);
    led_on = want_on;
    Bridge_PublishDebug();   /* 0.5 s 마다 (LED 상태가 바뀔 때) */
  }
}

/**
  * @brief  디버거가 볼 변수들을 D-Cache 에서 RAM 으로 내보낸다 (clean, 무효화 아님).
  *         D-Cache 가 켜져 있으면 CPU 가 쓴 값이 캐시에만 있어서, 디버거(Live Expressions,
  *         CubeProgrammer 메모리 읽기)는 RAM 의 옛 값을 보게 된다. 브리지 동작과는 무관.
  */
static void Bridge_PublishDebug(void)
{
  SCB_CleanDCache_by_Addr(&g_stat_ble, (int32_t)sizeof(g_stat_ble));
  SCB_CleanDCache_by_Addr(&g_stat_fc, (int32_t)sizeof(g_stat_fc));
  SCB_CleanDCache_by_Addr(&g_pclk1_hz, (int32_t)sizeof(g_pclk1_hz));
  SCB_CleanDCache_by_Addr(&g_xiao_link_up, (int32_t)sizeof(g_xiao_link_up));
  SCB_CleanDCache_by_Addr(&rb_fc2ble.head, (int32_t)(3U * sizeof(uint32_t)));  /* head, tail, dropped */
  SCB_CleanDCache_by_Addr(&rb_ble2fc.head, (int32_t)(3U * sizeof(uint32_t)));
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* Turn LED2 on */
  BSP_LED_On(LED2);
  while (1);
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
