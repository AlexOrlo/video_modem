
#include "main.h"

extern DMA_HandleTypeDef hdma_usart1_rx;


void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}


void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance==ADC1)
  {
    __HAL_RCC_ADC12_CLK_ENABLE();
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance==ADC1)
  {
    __HAL_RCC_ADC12_CLK_DISABLE();
  }
}


void HAL_COMP_MspInit(COMP_HandleTypeDef* hcomp)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hcomp->Instance==COMP1)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**COMP1 GPIO Configuration
    PA1     ------> COMP1_INP
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }

}


void HAL_COMP_MspDeInit(COMP_HandleTypeDef* hcomp)
{
  if(hcomp->Instance==COMP1)
  {
    //PA1     ------> COMP1_INP
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);
  }

}


void HAL_DAC_MspInit(DAC_HandleTypeDef* hdac)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hdac->Instance==DAC)
  {
    __HAL_RCC_DAC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    //PA4     ------> DAC_OUT1
    GPIO_InitStruct.Pin = VREF_55mv_DAC_1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(VREF_55mv_DAC_1_GPIO_Port, &GPIO_InitStruct);
  }

}


void HAL_DAC_MspDeInit(DAC_HandleTypeDef* hdac)
{
  if(hdac->Instance==DAC)
  {
    __HAL_RCC_DAC1_CLK_DISABLE();
    //PA4     ------> DAC_OUT1
    HAL_GPIO_DeInit(VREF_55mv_DAC_1_GPIO_Port, VREF_55mv_DAC_1_Pin);
  }

}


void HAL_OPAMP_MspInit(OPAMP_HandleTypeDef* hopamp)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    //PA2     ------> OPAMP1_VOUT
    //PA3     ------> OPAMP1_VINM
    //PA5     ------> OPAMP1_VINP
    GPIO_InitStruct.Pin = VIDEO_BUF_OUT_AMP1_VOUT_Pin|NEG_FEED_AMP1_VNM_Pin|VIDEO_IN_AMP1_VNP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


void HAL_OPAMP_MspDeInit(OPAMP_HandleTypeDef* hopamp)
{
    //PA2     ------> OPAMP1_VOUT
    //PA3     ------> OPAMP1_VINM
    //PA5     ------> OPAMP1_VINP
    HAL_GPIO_DeInit(GPIOA, VIDEO_BUF_OUT_AMP1_VOUT_Pin|NEG_FEED_AMP1_VNM_Pin|VIDEO_IN_AMP1_VNP_Pin);
}


void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USART1)
  {
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    //PA9     ------> USART1_TX
    //PA10     ------> USART1_RX
    GPIO_InitStruct.Pin = UART1_MAV_TX_Pin|UART1_MAV_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Channel5;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(huart,hdmarx,hdma_usart1_rx);
    
    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

  }
  else if(huart->Instance==USART3)
  {
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    //PB10     ------> USART3_TX
    GPIO_InitStruct.Pin = DATA_FOR_VIDEO_USART3_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(DATA_FOR_VIDEO_USART3_TX_GPIO_Port, &GPIO_InitStruct);
  }

}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART1)
  {
    __HAL_RCC_USART1_CLK_DISABLE();

    //PA9     ------> USART1_TX
    //PA10     ------> USART1_RX
    HAL_GPIO_DeInit(GPIOA, UART1_MAV_TX_Pin|UART1_MAV_RX_Pin);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(huart->hdmarx);
  }
  else if(huart->Instance==USART3)
  {
    __HAL_RCC_USART3_CLK_DISABLE();
    //PB10     ------> USART3_TX
    HAL_GPIO_DeInit(DATA_FOR_VIDEO_USART3_TX_GPIO_Port, DATA_FOR_VIDEO_USART3_TX_Pin);
  }

}





void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim_base->Instance==TIM3)
  {
    __HAL_RCC_TIM3_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  }

}


void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM3)
  {
    __HAL_RCC_TIM3_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
  }

}