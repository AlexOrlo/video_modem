
#include "main.h"


#define DMA_BUFER_SIZE          100 

#define TIMER_27_USEC_INTERVAL    300   
#define TIMER_2_5_USEC_INTERVAL   15 

#define V_REFERECE (uint32_t)(((float)4096/(float)3300)*(float)372)        //290
#define V_REFERECE_UART (uint32_t)(((float)4096/(float)3300)*(float)(944*2))        //1176

static void UART3_Receive_One_Byte(void);
static void raisingVblank(void);
static void resetCommunication(void);
static void stopVideoSystemDetection();

extern TIM_HandleTypeDef htim3; //timer for vblanc periods calculation
extern COMP_HandleTypeDef hcomp1, hcomp2;
extern DAC_HandleTypeDef hdac;


extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
volatile uint8_t dataBuffer[DMA_BUFER_SIZE]={0};
volatile int uartBufCounter=0;


volatile uint32_t counter;

volatile uint32_t longLowDetectedCount=0, 
                  shortLowDetectedCount=0; 

volatile uint32_t LONG_DETECTIONS = 6, 
                  SHORT_DETECTIONS = 6,
                  MAX_SHORT_DETECTIONS = 19;


volatile bool inTest=false;
volatile uint32_t maxDetections=0;


void vblankInit(){
    
    //reset all values
    resetCommunication(); 
    
    //start dac for vblank: ref voltage to comparator 260mv recalculated
    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, V_REFERECE);
    
    //start dac for uart3: ref voltage to comparator 1000mv
    HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, V_REFERECE_UART);
    
    //start comparator
    HAL_COMP_Start(&hcomp1);  /*for vblank*/
    HAL_COMP_Start(&hcomp2);  /*for uart3*/
    
    //start timer 3
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
    
}


//__attribute__ ((section("CCM_DATA")))
//void signalFail(){
//    resetCommunication(); 
//}

__attribute__ ((section("CCM_DATA")))
static void resetCommunication(){
    uartBufCounter=0;
    longLowDetectedCount=0;
    shortLowDetectedCount=0;
}


void startVideoSystemDetection(){
    uint32_t adc_val = 0;
    uint32_t adc_val_min = 999999;
    uint32_t adc_val_max = 0;
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    for(int i=0; i<299999; i++)
    {
        HAL_ADC_Start(&hadc1); 
        HAL_ADC_PollForConversion(&hadc1, 100); 
        adc_val = HAL_ADC_GetValue(&hadc1);
        if(adc_val<adc_val_min)
            adc_val_min = adc_val;
        if(adc_val>adc_val_max)
            adc_val_max = adc_val;
    }        
    HAL_ADC_Stop(&hadc1);
    
    HAL_ADC_MspDeInit(&hadc1);
    
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, adc_val_min*1.75);
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, adc_val_min+((adc_val_max-adc_val_min)*0.6));
    
    maxDetections=0;
    inTest=true;
    
    HAL_Delay(1500);
    stopVideoSystemDetection();
}

static void stopVideoSystemDetection(){
    if(maxDetections==6)//NTSC
    {
        LONG_DETECTIONS = 6;
        SHORT_DETECTIONS = 6;
        MAX_SHORT_DETECTIONS = 19; //12 bytes per frame
    }
    else if(maxDetections==5)//PAL
    {
        LONG_DETECTIONS = 5;
        SHORT_DETECTIONS = 5;
        MAX_SHORT_DETECTIONS = 22; //16 bytes per frame
    }
    else
    {
        //error
    }
    inTest=false;
}

void videoSystemDetection(){
    static uint32_t detections=0;
    
    if(counter>TIMER_27_USEC_INTERVAL){
        detections++;
    }
    else if(counter > TIMER_2_5_USEC_INTERVAL){
        if(maxDetections<detections){
            maxDetections=detections;
        }
         detections=0;
    }
}


/**
  * @brief This function handles TIM3 global interrupt.
  * Handler for timer 3 that detect (low) voltage (rise and drop)  
  */
__attribute__ ((section("CCM_DATA")))
void TIM3_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_CC1) != RESET){        /*FAILING*/
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC1);
    }else if(__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_CC2) != RESET){   /*RAISING*/
        counter = TIM3->CCR2;
        if(!inTest)
            raisingVblank();
        else
            videoSystemDetection();
        __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC2);
    }else{
        __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
        __HAL_TIM_SET_COUNTER(&htim3, 0);
    }
}


__attribute__ ((section("CCM_DATA")))
static void raisingVblank(){
    if(counter>TIMER_27_USEC_INTERVAL){                                                                                             //Detect long low pulse
        longLowDetectedCount++;
        shortLowDetectedCount=0;
        if(longLowDetectedCount>LONG_DETECTIONS){                                                                                   //Check how many long pulses, if more that LONG_DETECTIONS => signal if fail
            resetCommunication();
        }
    }else if( counter > TIMER_2_5_USEC_INTERVAL && longLowDetectedCount==LONG_DETECTIONS){          //detect short low pulses
        shortLowDetectedCount++;
        if(shortLowDetectedCount>SHORT_DETECTIONS && shortLowDetectedCount<MAX_SHORT_DETECTIONS){                                   //if counter of short pulses in limit => receive one byte 
            UART3_Receive_One_Byte();
            
        }else if(shortLowDetectedCount==MAX_SHORT_DETECTIONS){                                                                      //after max short pulses, reset all counters and transmit bytes (if count of bytes !=0)
            if(uartBufCounter)
                HAL_UART_Transmit_DMA(&huart1, (void*)dataBuffer, uartBufCounter);
            resetCommunication();
        }
    }else{                                                                                                                          
        resetCommunication();
    }
}


__attribute__ ((section("CCM_DATA")))
static void UART3_Receive_One_Byte(void){
      LL_USART_Enable(USART3);
      LL_USART_EnableIT_RXNE(USART3);
      LL_USART_EnableIT_ERROR(USART3);
}


__attribute__ ((section("CCM_DATA")))
void USART3_IRQHandler(void)
{
  if(   (LL_USART_IsActiveFlag_RXNE(USART3) &&   LL_USART_IsEnabledIT_RXNE(USART3) &&
       !LL_USART_IsActiveFlag_FE(USART3)   &&  !LL_USART_IsActiveFlag_NE(USART3) && 
       !LL_USART_IsActiveFlag_ORE(USART3)  &&  uartBufCounter)  || (!uartBufCounter && longLowDetectedCount==LONG_DETECTIONS) ||
    (LL_USART_IsEnabledIT_RXNE(USART3) && LL_USART_IsActiveFlag_NE(USART3)) )
  {
     dataBuffer[uartBufCounter] = LL_USART_ReceiveData8(USART3);
     uartBufCounter++;
  }
  else
  {
    (void) USART3->RDR;
  }
  
  LL_USART_Disable(USART3);
}


