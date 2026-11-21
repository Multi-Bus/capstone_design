/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    n6_bridge/FSBL/Src/stm32n6xx_hal_msp.c
  * @author  MCD Application Team (base: UART_TwoBoards_ComPolling)
  * @brief   HAL MSP module.
  *          예제의 USART1(PE5/PE6) 대신 브리지용 USART2(PA2/PF6), UART7(PH4/PE7)의
  *          커널 클럭(PCLK1), 버스 클럭, 핀(AF_PP, 풀업, Very high speed)을 설정한다.
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
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(huart->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */

  /** Initializes the peripherals clock
  *   커널 클럭 = PCLK1 (100 MHz). main.c 의 보드레이트 오차 계산이 이 전제를 쓴다.
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX  -> XIAO D7
    PF6     ------> USART2_RX  <- XIAO D6
    */
    GPIO_InitStruct.Pin = BLE_UART_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(BLE_UART_TX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BLE_UART_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(BLE_UART_RX_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(huart->Instance==UART7)
  {
  /* USER CODE BEGIN UART7_MspInit 0 */

  /* USER CODE END UART7_MspInit 0 */

  /** Initializes the peripherals clock
  *   커널 클럭 = PCLK1 (100 MHz). 921600 -> BRR 109, 오차 -0.45 % (main.c 참고)
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART7;
    PeriphClkInitStruct.Uart7ClockSelection = RCC_UART7CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_UART7_CLK_ENABLE();

    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**UART7 GPIO Configuration
    PH4     ------> UART7_TX  -> FC RX4 (J2 2번)
    PE7     ------> UART7_RX  <- FC TX4 (J2 3번)
    */
    GPIO_InitStruct.Pin = FC_UART_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART7;
    HAL_GPIO_Init(FC_UART_TX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = FC_UART_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART7;
    HAL_GPIO_Init(FC_UART_RX_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN UART7_MspInit 1 */

  /* USER CODE END UART7_MspInit 1 */
  }

}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */
    __HAL_RCC_USART2_FORCE_RESET();
    __HAL_RCC_USART2_RELEASE_RESET();

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    HAL_GPIO_DeInit(BLE_UART_TX_GPIO_Port, BLE_UART_TX_Pin);
    HAL_GPIO_DeInit(BLE_UART_RX_GPIO_Port, BLE_UART_RX_Pin);

  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(huart->Instance==UART7)
  {
  /* USER CODE BEGIN UART7_MspDeInit 0 */
    __HAL_RCC_UART7_FORCE_RESET();
    __HAL_RCC_UART7_RELEASE_RESET();

  /* USER CODE END UART7_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART7_CLK_DISABLE();

    HAL_GPIO_DeInit(FC_UART_TX_GPIO_Port, FC_UART_TX_Pin);
    HAL_GPIO_DeInit(FC_UART_RX_GPIO_Port, FC_UART_RX_Pin);

  /* USER CODE BEGIN UART7_MspDeInit 1 */

  /* USER CODE END UART7_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
