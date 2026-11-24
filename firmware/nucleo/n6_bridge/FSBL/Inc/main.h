/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    n6_bridge/FSBL/Inc/main.h
  * @author  MCD Application Team (base: UART_TwoBoards_ComPolling)
  * @brief   Header for main.c module (N6 UART 브리지)
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

/* Define to prevent recursive inclusion -------------------------------------*/
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

/* Includes ------------------------------------------------------------------*/
#include "stm32n6xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32n6xx_nucleo.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* Function pointer declaration in non-secure*/
#if defined ( __ICCARM__ )
typedef void (CMSE_NS_CALL *funcptr)(void);
#else
typedef void CMSE_NS_CALL (*funcptr)(void);
#endif

/* typedef for non-secure callback functions */
typedef funcptr funcptr_NS;

/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
/* 예제의 TXBUFFERSIZE/RXBUFFERSIZE/COUNTOF 는 브리지에서 안 써서 제거함 */

/* 방향별 링버퍼 크기. 인덱스를 마스크로 감싸므로 반드시 2의 거듭제곱. */
#define BRIDGE_BUF_SIZE                   4096U
#if (BRIDGE_BUF_SIZE & (BRIDGE_BUF_SIZE - 1U)) != 0U
#error "BRIDGE_BUF_SIZE must be a power of two"
#endif

/* 사용 핀 (carrier_pinmap.csv / ST open pin data 로 확인된 값)
 *   USART2_TX PA2 (AF7) -> XIAO D7      USART2_RX PF6 (AF7) <- XIAO D6
 *   UART7_TX  PH4 (AF8) -> FC RX4 (J2-2) UART7_RX PE7 (AF8) <- FC TX4 (J2-3)
 *   XIAO_INT  PD11 입력(풀다운) <- XIAO D0
 * 그 외 핀(SD PC0~PC5/PE4, I2C1 PH9/PC1, MUX_RSTn PE1, LED PE13/PE14, PD7 UWB WAKEUP 등)은
 * 건드리지 않는다. NUCLEO 보드 LED1=PG8, LED2=PG10 은 BSP 가 사용. */
#define BLE_UART_TX_Pin                   GPIO_PIN_2
#define BLE_UART_TX_GPIO_Port             GPIOA
#define BLE_UART_RX_Pin                   GPIO_PIN_6
#define BLE_UART_RX_GPIO_Port             GPIOF
#define FC_UART_TX_Pin                    GPIO_PIN_4
#define FC_UART_TX_GPIO_Port              GPIOH
#define FC_UART_RX_Pin                    GPIO_PIN_7
#define FC_UART_RX_GPIO_Port              GPIOE
#define XIAO_INT_Pin                      GPIO_PIN_11
#define XIAO_INT_GPIO_Port                GPIOD
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
