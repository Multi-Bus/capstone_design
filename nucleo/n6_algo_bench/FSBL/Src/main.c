/**
  ******************************************************************************
  * @file    n6_algo_bench/FSBL/Src/main.c
  * @brief   N6 알고리즘 시간 측정 (Task 3 Lv3). swarm/c 를 NUCLEO-N657X0-Q 에서 돌려 시간을 잰다.
  *
  *   결과는 ST-LINK 가상 COM 포트(USART1, 115200 8N1)로 한 줄씩 나온다. 형식은 README.md.
  *   잰 것:
  *     1. CBF-QP 1회 (이웃 5 + 기둥 3 + 상자 4 = 제약 12줄): 무작위 입력, 해가 없는 최악 입력
  *     2. 제어 1주기 (편대 속도 + 교착 완화 + CBF-QP + 이벤트 트리거)
  *     3. 경로 계획 1회 (15 m 방, 0.1 m 격자 151 x 151, 기둥 3개, 부풀리기 1.2 m)
  *     4. 동시 부하: SysTick 1 kHz 인터럽트에서 제어 1주기를 돌리면서 메인 루프에서 경로 계획을 계속 돌린다.
  *        인터럽트 주기가 밀리는지(지터), 제어 1주기가 얼마나 걸리는지 잰다. 비교용으로 메인 루프가 놀 때도 잰다.
  *     5. D-Cache 를 끄고 1, 3 을 다시 잰다.
  *
  *   시간은 DWT 사이클 카운터로 잰다. 클럭 설정은 n6_bridge 와 같다(CPU = IC1 = PLL1 1200 MHz / 2 = 600 MHz).
  *   부팅 줄에 실제 CPU 클럭을 찍는다.
  *
  *   베이스: nucleo/n6_bridge (STM32CubeN6 V1.4.1 UART_TwoBoards_ComPolling). SystemClock_Config 는 그대로.
  ******************************************************************************
  * Copyright (c) 2024 STMicroelectronics. 원래 파일의 라이선스는 n6_bridge/LICENSE_ST.md.
  ******************************************************************************
  */
#include "main.h"
#include "swarm.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 측정 횟수 */
#define N_QP            10000U
#define N_STEP          10000U
#define N_PLAN          30U
#define N_QP_NOCACHE    2000U
#define N_PLAN_NOCACHE  5U
#define LOAD_MS         5000U   /* 동시 부하 측정 시간 */

/* 큰 배열은 AXISRAM1 에 둔다 (FSBL RAM 은 256 KB). 링커 스크립트의 .axisram1 구역. */
#define BIG __attribute__((section(".axisram1")))

UART_HandleTypeDef huart1;

static uint32_t g_samples[N_QP > N_STEP ? N_QP : N_STEP] BIG;
static uint8_t  g_grid[SWARM_MAX_CELLS] BIG;
static vec2_t   g_wp[SWARM_MAX_CELLS] BIG;
static uint32_t g_cpu_hz;

#define GRID_N   151
#define GRID_RES 0.1f
#define ROOM_HALF 7.5f

void SystemClock_Config(void);
static void MX_USART1_UART_Init(void);

/* ------------------------------------------------------------------ 출력 */

static void out(const char *fmt, ...)
{
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf) - 2U, fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if (n > (int)sizeof(buf) - 3) n = (int)sizeof(buf) - 3;
  buf[n++] = '\r';
  buf[n++] = '\n';
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)n, 1000U);
}

/* ------------------------------------------------------------------ 시간 */

static void cycles_init(void)
{
  DCB->DEMCR |= DCB_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t cycles(void) { return DWT->CYCCNT; }

static uint32_t cyc_to_ns(uint32_t c)
{
  return (uint32_t)(((uint64_t)c * 1000000000ULL) / g_cpu_hz);
}

static int cmp_u32(const void *a, const void *b)
{
  uint32_t x = *(const uint32_t *)a, y = *(const uint32_t *)b;
  return (x > y) - (x < y);
}

/* 표본을 정렬해서 한 줄로 낸다. 단위: 사이클과 ns */
static void report(const char *name, uint32_t *s, uint32_t n, uint32_t extra_a, uint32_t extra_b)
{
  uint64_t sum = 0;
  for (uint32_t i = 0; i < n; i++) sum += s[i];
  qsort(s, n, sizeof(uint32_t), cmp_u32);
  uint32_t p50 = s[n / 2U], p99 = s[(n * 99U) / 100U], mx = s[n - 1U], mn = s[0];
  uint32_t avg = (uint32_t)(sum / n);
  out("BENCH,%s,n=%lu,min_ns=%lu,p50_ns=%lu,p99_ns=%lu,max_ns=%lu,avg_ns=%lu,max_cyc=%lu,a=%lu,b=%lu",
      name, (unsigned long)n, (unsigned long)cyc_to_ns(mn), (unsigned long)cyc_to_ns(p50),
      (unsigned long)cyc_to_ns(p99), (unsigned long)cyc_to_ns(mx), (unsigned long)cyc_to_ns(avg),
      (unsigned long)mx, (unsigned long)extra_a, (unsigned long)extra_b);
}

/* ------------------------------------------------------------------ 입력 만들기 */

static uint32_t g_rng = 12345U;
static float frand(float lo, float hi)
{
  g_rng = g_rng * 1664525U + 1013904223U;
  return lo + (hi - lo) * ((float)(g_rng >> 8) * (1.0f / 16777216.0f));
}

/* 이웃 드론 5대 + 기둥 3개 (기둥은 반경 0.5 + D 0.4 = 0.9 m 를 안전거리로) */
static void make_random_nbrs(swarm_nbr_t *nb, vec2_t p_i)
{
  for (int k = 0; k < 5; k++)
  {
    nb[k].p = (vec2_t){ p_i.x + frand(-1.0f, 1.0f), p_i.y + frand(-1.0f, 1.0f) };
    nb[k].v = (vec2_t){ frand(-0.5f, 0.5f), frand(-0.5f, 0.5f) };
    nb[k].d = 0.0f;
  }
  for (int k = 5; k < 8; k++)
  {
    nb[k].p = (vec2_t){ p_i.x + frand(-2.0f, 2.0f), p_i.y + frand(-2.0f, 2.0f) };
    nb[k].v = (vec2_t){ 0.0f, 0.0f };
    nb[k].d = 0.9f;
  }
}

/* 8 방향에서 0.25 m 거리로 다가오는 이웃: 해가 없어 조합을 모두 확인하는 최악 입력 */
static void make_surrounded_nbrs(swarm_nbr_t *nb)
{
  static const float c8[8][2] = {
    {1.0f, 0.0f}, {0.7071f, 0.7071f}, {0.0f, 1.0f}, {-0.7071f, 0.7071f},
    {-1.0f, 0.0f}, {-0.7071f, -0.7071f}, {0.0f, -1.0f}, {0.7071f, -0.7071f}};
  for (int k = 0; k < 8; k++)
  {
    nb[k].p = (vec2_t){ 0.25f * c8[k][0], 0.25f * c8[k][1] };
    nb[k].v = (vec2_t){ -0.5f * c8[k][0], -0.5f * c8[k][1] };
    nb[k].d = 0.0f;
  }
}

/* ------------------------------------------------------------------ 제어 1주기 */

static swarm_deadlock_t g_dead;
static swarm_trigger_t  g_trig;
static swarm_nbr_t      g_ctrl_nb[8];
static swarm_fnbr_t     g_ctrl_fn[5];
static vec2_t           g_ctrl_u;
static float            g_ctrl_t;

static void control_setup(void)
{
  swarm_deadlock_init(&g_dead, 1.0f, 1.5f, 60.0f, 0.05f, 0.5f);
  swarm_trigger_init(&g_trig, 0.1f, 1.0f);
  make_random_nbrs(g_ctrl_nb, (vec2_t){ 0.0f, 0.0f });
  for (int k = 0; k < 5; k++)
  {
    g_ctrl_fn[k].p = g_ctrl_nb[k].p;
    g_ctrl_fn[k].d = (vec2_t){ frand(-1.0f, 1.0f), frand(-1.0f, 1.0f) };
  }
  g_ctrl_u = (vec2_t){ 0.0f, 0.0f };
  g_ctrl_t = 0.0f;
}

/* 기체 한 대의 제어 1주기: 편대 속도 -> 교착 완화 -> CBF-QP -> 송신 판단 */
static void control_step(void)
{
  vec2_t p_i = { 0.0f, 0.0f };
  vec2_t v_des = swarm_formation_vel(p_i, (vec2_t){ 0.2f, 0.1f }, g_ctrl_fn, 5, (vec2_t){ 3.0f, 1.0f },
                                     0.5f, 0.6f, 1.0f);
  v_des = swarm_deadlock_adjust(&g_dead, v_des, g_ctrl_u, 3.0f, g_ctrl_t, 0.001f);
  vec2_t u;
  (void)swarm_cbf_qp(v_des, p_i, g_ctrl_nb, 8, 0.4f, 1.0f, 1.0f, &u);
  if (swarm_trigger_should_send(&g_trig, p_i, u, g_ctrl_t, 0))
  {
    swarm_trigger_sent(&g_trig, p_i, u, g_ctrl_t);
  }
  g_ctrl_u = u;
  g_ctrl_t += 0.001f;
}

/* ------------------------------------------------------------------ 동시 부하 (SysTick) */

static volatile uint32_t g_load_on;
static volatile uint32_t g_tick_last, g_tick_n, g_per_min, g_per_max, g_late, g_step_max;
static volatile uint64_t g_per_sum, g_step_sum;

void Bench_Tick(void)
{
  if (g_load_on == 0U) return;
  uint32_t now = cycles();
  if (g_tick_last != 0U)
  {
    uint32_t per = now - g_tick_last;
    if (per < g_per_min) g_per_min = per;
    if (per > g_per_max) g_per_max = per;
    g_per_sum += per;
    if (per > (g_cpu_hz / 1000U) * 3U / 2U) g_late++;   /* 1.5 ms 넘게 밀린 주기 */
    g_tick_n++;
  }
  g_tick_last = now;
  uint32_t t0 = cycles();
  control_step();
  uint32_t d = cycles() - t0;
  if (d > g_step_max) g_step_max = d;
  g_step_sum += d;
}

static void load_reset(void)
{
  g_tick_last = 0U; g_tick_n = 0U; g_per_min = 0xFFFFFFFFU; g_per_max = 0U; g_late = 0U;
  g_step_max = 0U; g_per_sum = 0U; g_step_sum = 0U;
}

static void load_report(const char *name, uint32_t plans, uint32_t plan_fail)
{
  uint32_t n = g_tick_n ? g_tick_n : 1U;
  uint32_t expect = g_cpu_hz / 1000U;
  uint32_t per_avg = (uint32_t)(g_per_sum / n);
  uint32_t jitter = (g_per_max > expect ? g_per_max - expect : 0U);
  uint32_t jitter2 = (expect > g_per_min ? expect - g_per_min : 0U);
  if (jitter2 > jitter) jitter = jitter2;
  out("LOAD,%s,ticks=%lu,period_avg_ns=%lu,period_min_ns=%lu,period_max_ns=%lu,jitter_max_ns=%lu,late=%lu,"
      "step_avg_ns=%lu,step_max_ns=%lu,plans=%lu,plan_fail=%lu",
      name, (unsigned long)g_tick_n, (unsigned long)cyc_to_ns(per_avg), (unsigned long)cyc_to_ns(g_per_min),
      (unsigned long)cyc_to_ns(g_per_max), (unsigned long)cyc_to_ns(jitter), (unsigned long)g_late,
      (unsigned long)cyc_to_ns((uint32_t)(g_step_sum / n)), (unsigned long)cyc_to_ns(g_step_max),
      (unsigned long)plans, (unsigned long)plan_fail);
}

/* ------------------------------------------------------------------ 측정 */

static void bench_qp(const char *name, uint32_t n, int surrounded)
{
  swarm_nbr_t nb[8];
  uint32_t ok = 0, infeasible = 0;
  for (uint32_t i = 0; i < n; i++)
  {
    vec2_t p_i = { 0.0f, 0.0f };
    if (surrounded) make_surrounded_nbrs(nb); else make_random_nbrs(nb, p_i);
    vec2_t v_des = { frand(-1.0f, 1.0f), frand(-1.0f, 1.0f) };
    vec2_t u;
    __disable_irq();
    uint32_t t0 = cycles();
    int rc = swarm_cbf_qp(v_des, p_i, nb, 8, 0.4f, 1.0f, 1.0f, &u);
    g_samples[i] = cycles() - t0;
    __enable_irq();
    if (rc == SWARM_OK) ok++; else infeasible++;
  }
  report(name, g_samples, n, ok, infeasible);
}

static void bench_step(void)
{
  control_setup();
  for (uint32_t i = 0; i < N_STEP; i++)
  {
    __disable_irq();
    uint32_t t0 = cycles();
    control_step();
    g_samples[i] = cycles() - t0;
    __enable_irq();
  }
  report("control_step", g_samples, N_STEP, 0U, 0U);
}

static void build_grid(void)
{
  static const float pillars[3][2] = { {-3.5f, 2.5f}, {3.0f, -3.5f}, {1.0f, 4.5f} };
  for (int y = 0; y < GRID_N; y++)
  {
    for (int x = 0; x < GRID_N; x++)
    {
      float wx = x * GRID_RES - ROOM_HALF, wy = y * GRID_RES - ROOM_HALF;
      uint8_t occ = 0U;
      for (int k = 0; k < 3; k++)
      {
        float dx = wx - pillars[k][0], dy = wy - pillars[k][1];
        if (dx * dx + dy * dy <= 0.25f) occ = 1U;   /* 반경 0.5 m */
      }
      g_grid[y * GRID_N + x] = occ;
    }
  }
}

static int plan_once(uint32_t *cyc)
{
  vec2_t s = { frand(1.0f, 14.0f), frand(1.0f, 14.0f) };
  vec2_t g = { frand(1.0f, 14.0f), frand(1.0f, 14.0f) };
  int len = 0;
  uint32_t t0 = cycles();
  int rc = swarm_plan(g_grid, GRID_N, GRID_N, GRID_RES, s, g, 1.2f, g_wp, SWARM_MAX_CELLS, &len);
  if (cyc) *cyc = cycles() - t0;
  return rc;
}

static void bench_plan(const char *name, uint32_t n)
{
  uint32_t ok = 0, fail = 0;
  for (uint32_t i = 0; i < n; i++)
  {
    __disable_irq();
    int rc = plan_once(&g_samples[i]);
    __enable_irq();
    if (rc == SWARM_OK) ok++; else fail++;
  }
  report(name, g_samples, n, ok, fail);
}

static void bench_load(int with_plan)
{
  control_setup();
  load_reset();
  uint32_t plans = 0, fail = 0;
  g_load_on = 1U;
  uint32_t t0 = HAL_GetTick();
  while (HAL_GetTick() - t0 < LOAD_MS)
  {
    if (with_plan)
    {
      if (plan_once(NULL) != SWARM_OK) fail++;
      plans++;
    }
  }
  g_load_on = 0U;
  load_report(with_plan ? "tick1k_with_plan" : "tick1k_idle", plans, fail);
}

/* ------------------------------------------------------------------ main */

int main(void)
{
  SCB_EnableICache();
  SCB_EnableDCache();

  HAL_Init();
  SystemClock_Config();

  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  MX_USART1_UART_Init();

  cycles_init();
  g_cpu_hz = HAL_RCC_GetCpuClockFreq();
  out("");
  out("BOOT,fw=n6_algo_bench-v1,cpu_hz=%lu,sysclk_hz=%lu,hclk_hz=%lu,real=%s,max_cells=%lu,max_rows=%lu",
      (unsigned long)g_cpu_hz, (unsigned long)HAL_RCC_GetSysClockFreq(), (unsigned long)HAL_RCC_GetHCLKFreq(),
      sizeof(real_t) == 4U ? "float" : "double", (unsigned long)SWARM_MAX_CELLS, (unsigned long)SWARM_MAX_ROWS);

  /* 사이클 카운터가 도는지 먼저 확인: 10 ms 동안 늘어난 값 */
  {
    uint32_t c0 = cycles();
    HAL_Delay(10);
    out("CHECK,cyccnt_per_10ms=%lu", (unsigned long)(cycles() - c0));
  }

  build_grid();

  bench_qp("cbf_qp_random", N_QP, 0);
  bench_qp("cbf_qp_surrounded", N_QP, 1);
  bench_step();
  bench_plan("plan_151x151", N_PLAN);
  bench_load(0);
  bench_load(1);

  SCB_DisableDCache();
  bench_qp("cbf_qp_random_nodcache", N_QP_NOCACHE, 0);
  bench_plan("plan_151x151_nodcache", N_PLAN_NOCACHE);
  SCB_EnableDCache();

  out("DONE");

  while (1)
  {
    BSP_LED_Toggle(LED1);
    HAL_Delay(500);
  }
}

/**
  * @brief System Clock Configuration. n6_bridge 와 같다.
  *        HSI 64 MHz / 4 * 75 = PLL1 1200 MHz, CPU = IC1 = 600 MHz, 시스템 버스 IC2 = 400 MHz,
  *        HCLK = 100 MHz, PCLK1/2/4/5 = 100 MHz.
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ConfigSupply(PWR_EXTERNAL_SOURCE_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

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

  HAL_RCC_GetClockConfig(&RCC_ClkInitStruct);
  if ((RCC_ClkInitStruct.CPUCLKSource == RCC_CPUCLKSOURCE_IC1) ||
      (RCC_ClkInitStruct.SYSCLKSource == RCC_SYSCLKSOURCE_IC2_IC6_IC11))
  {
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_SYSCLK);
    RCC_ClkInitStruct.CPUCLKSource = RCC_CPUCLKSOURCE_HSI;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }
  }

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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_HCLK
                              | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
                              | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK5
                              | RCC_CLOCKTYPE_PCLK4;
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

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = VCP_UART_BAUD;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  BSP_LED_On(LED2);
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
  while (1)
  {
  }
}
#endif
