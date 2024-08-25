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
#include "stdbool.h"
void Error_Handler(void);

#define CLI_NAME video_tx_cli
#define __DEBUG

#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOB
#define VIDEO_BUF_OUT_AMP1_VOUT_Pin GPIO_PIN_2
#define VIDEO_BUF_OUT_AMP1_VOUT_GPIO_Port GPIOA
#define NEG_FEED_AMP1_VNM_Pin GPIO_PIN_3
#define NEG_FEED_AMP1_VNM_GPIO_Port GPIOA
#define VREF_55mv_DAC_1_Pin GPIO_PIN_4
#define VREF_55mv_DAC_1_GPIO_Port GPIOA
#define VIDEO_IN_AMP1_VNP_Pin GPIO_PIN_5
#define VIDEO_IN_AMP1_VNP_GPIO_Port GPIOA
#define SUM_IN_AMP3_VNP_Pin GPIO_PIN_0
#define SUM_IN_AMP3_VNP_GPIO_Port GPIOB
#define VIDEO_OUT_AMP3_VOUT_Pin GPIO_PIN_1
#define VIDEO_OUT_AMP3_VOUT_GPIO_Port GPIOB
#define NEG_FEED_AMP3_VNM_Pin GPIO_PIN_2
#define NEG_FEED_AMP3_VNM_GPIO_Port GPIOB
#define DATA_FOR_VIDEO_USART3_TX_Pin GPIO_PIN_10
#define DATA_FOR_VIDEO_USART3_TX_GPIO_Port GPIOB
#define UART1_MAV_TX_Pin GPIO_PIN_9
#define UART1_MAV_TX_GPIO_Port GPIOA
#define UART1_MAV_RX_Pin GPIO_PIN_10
#define UART1_MAV_RX_GPIO_Port GPIOA


#define DMA_BUF_SIZE_U1 1024
#define DMA_TIMEOUT_MS_U1      2

typedef struct
{
    volatile uint32_t  flag;     /* Timeout event flag */
    volatile uint32_t prevCNDTR;         /* Holds previous value of DMA_CNDTR */
} DMA_Event_t;


#define LED_Off   LED_GPIO_Port->BRR = (uint32_t)LED_Pin
#define LED_On  LED_GPIO_Port->BSRR = (uint32_t)LED_Pin
#define LED_Toggle LED_GPIO_Port->BSRR = ((LED_GPIO_Port->ODR & LED_Pin) << 16U) | (~(LED_GPIO_Port->ODR) & LED_Pin)




#endif /* __MAIN_H */
