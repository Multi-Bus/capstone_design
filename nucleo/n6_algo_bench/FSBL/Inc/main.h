/**
  ******************************************************************************
  * @file    n6_algo_bench/FSBL/Inc/main.h
  * @brief   N6 알고리즘 시간 측정 (swarm/c) 헤더
  ******************************************************************************
  * 베이스: nucleo/n6_bridge (STM32CubeN6 V1.4.1 UART_TwoBoards_ComPolling).
  * Copyright (c) 2024 STMicroelectronics. 원래 파일의 라이선스는 n6_bridge/LICENSE_ST.md.
  ******************************************************************************
  */
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined ( __ICCARM__ )
#  define CMSE_NS_CALL  __cmse_nonsecure_call
#  define CMSE_NS_ENTRY __cmse_nonsecure_entry
#else
#  define CMSE_NS_CALL  __attribute((cmse_nonsecure_call))
#  define CMSE_NS_ENTRY __attribute((cmse_nonsecure_entry))
#endif

#include "stm32n6xx_hal.h"
#include "stm32n6xx_nucleo.h"

#if defined ( __ICCARM__ )
typedef void (CMSE_NS_CALL *funcptr)(void);
#else
typedef void CMSE_NS_CALL (*funcptr)(void);
#endif
typedef funcptr funcptr_NS;

void Error_Handler(void);

/* SysTick(1 kHz) 인터럽트에서 부른다. 동시 부하 측정 중에만 제어 1주기를 흉내 낸다. */
void Bench_Tick(void);

/* ST-LINK 가상 COM 포트 (USART1, PE5 TX / PE6 RX, AF7) */
#define VCP_UART_TX_Pin        GPIO_PIN_5
#define VCP_UART_TX_GPIO_Port  GPIOE
#define VCP_UART_RX_Pin        GPIO_PIN_6
#define VCP_UART_RX_GPIO_Port  GPIOE
#define VCP_UART_BAUD          115200U

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
