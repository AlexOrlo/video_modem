/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32f3xx_ll_usart.h"
#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_cortex.h"
#include "stm32f3xx_ll_system.h"
#include "stm32f3xx_ll_utils.h"
#include "stm32f3xx_ll_pwr.h"
#include "stm32f3xx_ll_gpio.h"
#include "stm32f3xx_ll_dma.h"

#include "stm32f3xx_ll_exti.h"

#include "stdbool.h"

void Error_Handler(void);

#define LED_OK_Pin GPIO_PIN_13
#define LED_OK_GPIO_Port GPIOC
#define LED_ERROR_Pin GPIO_PIN_14
#define LED_ERROR_GPIO_Port GPIOC
#define VIDEO_BUF_OUT_AMP1_VOUT_Pin GPIO_PIN_2
#define VIDEO_BUF_OUT_AMP1_VOUT_GPIO_Port GPIOA
#define VIDEO_IN_AMP1_VNP_Pin GPIO_PIN_3
#define VIDEO_IN_AMP1_VNP_GPIO_Port GPIOA
#define VREF_250mv_DAC_1_Pin GPIO_PIN_4
#define VREF_250mv_DAC_1_GPIO_Port GPIOA
#define VREF_1000mV_Pin GPIO_PIN_5
#define VREF_1000mV_GPIO_Port GPIOA
#define UART_3_RX_Pin GPIO_PIN_10
#define UART_3_RX_GPIO_Port GPIOB
#define DATA_OUT_Pin GPIO_PIN_9
#define DATA_OUT_GPIO_Port GPIOA
#define TO_UART3_Pin GPIO_PIN_12
#define TO_UART3_GPIO_Port GPIOA


#define LED_OK_TURN_ON       LED_OK_GPIO_Port->BRR     = (uint32_t)LED_OK_Pin
#define LED_OK_TURN_OFF      LED_OK_GPIO_Port->BSRR    = (uint32_t)LED_OK_Pin
#define LED_OK_TOGGLE        LED_OK_GPIO_Port->BSRR = ((LED_OK_GPIO_Port->ODR & LED_OK_Pin) << 16U) | (~LED_OK_GPIO_Port->ODR & LED_OK_Pin)

#define LED_ERROR_TURN_ON    LED_ERROR_GPIO_Port->BRR  = (uint32_t)LED_ERROR_Pin
#define LED_ERROR_TURN_OFF   LED_ERROR_GPIO_Port->BSRR = (uint32_t)LED_ERROR_Pin
#define LED_ERROR_TOGGLE     LED_ERROR_GPIO_Port->BSRR = ((LED_ERROR_GPIO_Port->ODR & LED_ERROR_Pin) << 16U) | (~LED_ERROR_GPIO_Port->ODR & LED_ERROR_Pin)


#define __DEBUG

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
