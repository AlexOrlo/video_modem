
#include "main.h"
#include "stm32f3xx_it.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

extern volatile bool initDone;
extern volatile DMA_Event_t dma_uart_rxu1;
extern volatile uint8_t dma_rx_bufu1[DMA_BUF_SIZE_U1];       /* Buffer for DMA */


/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
void NMI_Handler(void){while (1){ }}
void HardFault_Handler(void){
    
    #ifndef __DEBUG
        //Core power cycle
        __set_FAULTMASK (1);
        NVIC_SystemReset();
    #endif /* __DEBUG */
    
    while (1)
    {
        HAL_Delay(1);
    }

}
void MemManage_Handler(void){while (1){ }}
void BusFault_Handler(void){while (1){ }}
void UsageFault_Handler(void){while (1){ }}
void SVC_Handler(void){}
void DebugMon_Handler(void){}
void PendSV_Handler(void){}
    


__attribute__ ((section("CCM_DATA")))
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

//__attribute__ ((section("CCM_DATA")))
void USART1_IRQHandler(void)
{
    volatile uint32_t tsr = huart1.Instance->ISR;
    volatile uint32_t tdr = huart1.Instance->RDR;
    
    if(__HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_IDLE)){
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
       
        if(initDone)
        {
            dma_uart_rxu1.flag = 1;
            EXTI->SWIER |= EXTI_IMR_MR1;   
        }
    }
}



__attribute__ ((section("CCM_DATA")))
void DMA1_Channel5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
}



__attribute__ ((section("CCM_DATA")))
void EXTI1_IRQHandler (void){
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
    
    hdma_usart1_rx.XferCpltCallback(&hdma_usart1_rx);
}

