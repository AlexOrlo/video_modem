
#include "main.h"
#include <string.h>
#include "settings.h" 
#include "shell.h"

#define COMP_OUT COMP_OUTPUTPOL_INVERTED

uint32_t vect_table_ram[98] __attribute__ ((section("V_DATA"))); 

ADC_HandleTypeDef hadc1;
COMP_HandleTypeDef hcomp1;
DAC_HandleTypeDef hdac;
OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp3;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;

volatile DMA_Event_t dma_uart_rxu1 = {0, DMA_BUF_SIZE_U1};
volatile uint8_t dma_rx_bufu1[DMA_BUF_SIZE_U1] = {0};       /* Buffer for DMA */
volatile bool initDone = false;
volatile bool CLI_Mode = false;

extern volatile settingT setting;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_COMP1_Init(void);
static void MX_DAC_Init(void);
static void MX_OPAMP1_Init(void);
static void MX_OPAMP3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);

void vblankInit(void);
void startVideoSystemDetection(void);

void parsMavlink(char c);
void crsf_telemetry_push_byte(uint8_t data);
void addBytesToQueue(uint8_t* c, int len);

int main(void)
{
    memcpy(&vect_table_ram, (uint32_t*)SCB->VTOR, sizeof(vect_table_ram));
    SCB->VTOR = (uint32_t)&vect_table_ram;

    HAL_Init();
    SystemClock_Config();
    
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_DAC_Init();
    MX_COMP1_Init();
    MX_OPAMP1_Init();
    MX_TIM3_Init();
    MX_ADC1_Init();
    MX_USART3_UART_Init();
    flash_init();
    
    HAL_ADC_Start(&hadc1); 
    HAL_ADC_PollForConversion(&hadc1, 100); 
    uint32_t a = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    if(a>700)
    {
        CLI_Mode = true;
        shell_init(&huart1);
    }
    
    MX_USART1_UART_Init();
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    __HAL_DMA_ENABLE_IT (&hdma_usart1_rx, DMA_IT_TC);
    HAL_UART_Receive_DMA(&huart1, (uint8_t*)dma_rx_bufu1, DMA_BUF_SIZE_U1);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE); 
    
 
    //OPAMP's self callibration ~50ms
    HAL_OPAMP_SelfCalibrate(&hopamp1);
    //Start OPAMP's
    HAL_OPAMP_Start(&hopamp1);
    //VBLANK detection module init
    HAL_Delay(1000);
    vblankInit();
    
    //Check video system
    startVideoSystemDetection();
    
    initDone=true;


    #ifndef __DEBUG
    //Disable SysTick
    HAL_SuspendTick();
    //Go to sleep mode
    SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPONEXIT_Msk));
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    #endif /* __DEBUG */
    
    while (1)
    {
    }
}



__attribute__ ((section("CCM_DATA")))
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    volatile uint32_t start, length;
    volatile uint32_t currCNDTR = __HAL_DMA_GET_COUNTER(huart->hdmarx);
    static volatile uint32_t oldCounter, si=0;
    

        if(oldCounter==currCNDTR){
            si++;
            if(si>10){
                HAL_UART_Receive_DMA(&huart1, (uint8_t*)dma_rx_bufu1, DMA_BUF_SIZE_U1);
                si=0;
                currCNDTR=0;
                dma_uart_rxu1.prevCNDTR=0;
            }
        }
        oldCounter=currCNDTR;
    
  
        // Ignore IDLE Timeout when the received characters exactly filled up the DMA buffer and DMA Rx Complete IT is generated, but there is no new character during timeout //
        if(dma_uart_rxu1.flag && currCNDTR == DMA_BUF_SIZE_U1)
        { 
            dma_uart_rxu1.flag = 0;
            return;
        }
    
        // Determine start position in DMA buffer based on previous CNDTR value//
        start = (dma_uart_rxu1.prevCNDTR < DMA_BUF_SIZE_U1) ? (DMA_BUF_SIZE_U1 - dma_uart_rxu1.prevCNDTR) : 0;
    
        if(dma_uart_rxu1.flag)    // Timeout event //
        {
            length = (dma_uart_rxu1.prevCNDTR < DMA_BUF_SIZE_U1) ? (dma_uart_rxu1.prevCNDTR - currCNDTR) : (DMA_BUF_SIZE_U1 - currCNDTR);
            dma_uart_rxu1.prevCNDTR = currCNDTR;
            dma_uart_rxu1.flag = 0;
        }else                // DMA Rx Complete event //
        {
            length = DMA_BUF_SIZE_U1 - start;
            dma_uart_rxu1.prevCNDTR = DMA_BUF_SIZE_U1;
        }
    
        
        if(CLI_Mode)
        {
            for(int tmpFor=start; tmpFor<length+start; tmpFor++)
                shell_encode(dma_rx_bufu1[tmpFor&0x3FF]);
            return;
        }
            
        if(setting.telemetry_type != Raw)
            for(int tmpFor=start; tmpFor<length+start; tmpFor++)
            {
                if(setting.telemetry_type == MavLink_Telemetry)
                    parsMavlink( dma_rx_bufu1[tmpFor&0x3FF] );
                else if(setting.telemetry_type == Crossfier_Telemetry)
                    crsf_telemetry_push_byte(dma_rx_bufu1[tmpFor&0x3FF]);
            }
        else
        {            
            if(start+length <= DMA_BUF_SIZE_U1)
                addBytesToQueue((uint8_t*)&dma_rx_bufu1[start], length);
            else
            {
                addBytesToQueue((uint8_t*)&dma_rx_bufu1[start], DMA_BUF_SIZE_U1-start);
                addBytesToQueue((uint8_t*)&dma_rx_bufu1[0], length-(DMA_BUF_SIZE_U1-start));
            }
        }
}




void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3
                            |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}



static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4;//5;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}



static void MX_COMP1_Init(void)
{
  hcomp1.Instance = COMP1;
  hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1_CH1;
  hcomp1.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
  hcomp1.Init.Output = COMP_OUTPUT_TIM3IC1;
  hcomp1.Init.OutputPol = COMP_OUT;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRCE_NONE;
  hcomp1.Init.Mode = COMP_MODE_HIGHSPEED;
  hcomp1.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }

}


static void MX_DAC_Init(void)
{
  DAC_ChannelConfTypeDef sConfig = {0};
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}


static void MX_OPAMP1_Init(void)
{
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.Mode = OPAMP_STANDALONE_MODE;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO3;
  hopamp1.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO1;
  hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
}


static void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}


static void MX_USART1_UART_Init(void)
{
  uint32_t uSpeed;
  if(CLI_Mode) uSpeed = 115200;
  else uSpeed = setting.uart_speed;
  huart1.Instance = USART1;
  huart1.Init.BaudRate = uSpeed;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart1.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}


static void MX_USART3_UART_Init(void)
{
  
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 230400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT|UART_ADVFEATURE_TXINVERT_INIT;
  huart3.AdvancedInit.TxPinLevelInvert = UART_ADVFEATURE_TXINV_ENABLE;
  huart3.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart3.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_HalfDuplex_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}


static void MX_DMA_Init(void)
{

  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}


static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins :  PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA6 PA7 PA8
                           PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB11 PB12 PB13 PB14
                           PB15 PB4 PB5 PB6
                           PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  //IRQ for DMA_UART
  EXTI->IMR |= EXTI_IMR_MR1;
  EXTI->EMR |= EXTI_IMR_MR1;
  NVIC_SetPriority(EXTI1_IRQn, 11);
  NVIC_EnableIRQ(EXTI1_IRQn);

}


void Error_Handler(void)
{
  __disable_irq();
}
