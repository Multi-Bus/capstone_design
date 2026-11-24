/**
  ******************************************************************************
  * @file    n6_algo_bench/FSBL/Src/stm32n6xx_hal_msp.c
  * @brief   USART1(ST-LINK VCP) 클럭과 핀 설정
  ******************************************************************************
  * 베이스: nucleo/n6_bridge 의 같은 파일. USART2/UART7 대신 USART1 하나만 쓴다.
  * Copyright (c) 2024 STMicroelectronics. 원래 파일의 라이선스는 n6_bridge/LICENSE_ST.md.
  ******************************************************************************
  */
#include "main.h"

void HAL_MspInit(void)
{
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  if (huart->Instance == USART1)
  {
    /* USART1 커널 클럭 = PCLK2 (APB2 /1 -> 100 MHz) */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitStruct.Pin = VCP_UART_TX_Pin | VCP_UART_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(VCP_UART_TX_GPIO_Port, &GPIO_InitStruct);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_GPIO_DeInit(VCP_UART_TX_GPIO_Port, VCP_UART_TX_Pin | VCP_UART_RX_Pin);
  }
}
