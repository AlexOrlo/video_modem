
#include "main.h"

/*
    Total byte rate (NTSC) is 60*12 = 720 bytes per second.
    Total byte rate (PAL) is 50*16 = 800 bytes per second.
    For example one mavlink gps package is 34 bytes
*/



#define TIMER_27_USEC_INTERVAL    300   
#define TIMER_2_5_USEC_INTERVAL   15     

void init_cli_mode();
    
static void stopVideoSystemDetection();
static void raisingVblank(void);
static void signalFail(void);
void transmiteNextByte(void);

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3; //timer for vblanc periods calculation
extern COMP_HandleTypeDef hcomp1;
extern DAC_HandleTypeDef hdac;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart1;

volatile uint32_t counter;

volatile uint32_t longLowDetectedCount=0, 
                  shortLowDetectedCount=0; 

volatile uint32_t LONG_DETECTIONS = 6, 
                  SHORT_DETECTIONS = 6,
                  MAX_SHORT_DETECTIONS = 19;


volatile bool inTest=false;
volatile uint32_t maxDetections=0;


#ifdef __DEBUG
volatile uint16_t pulseW[8192]={0};

void saveValueOfpulseW(uint32_t newN){
    static uint16_t index=0;
    index++;
    index&=0x1FFF;
    pulseW[index]=(uint16_t)newN;
}
#endif /* __DEBUG */


/*
    1) Need detect 5/6 (low) intervals ~27[uSec]
    2) Need detect 7/8 (low) intervals ~2.5[uSec]
    3) After ~6[uSec], start transmit 1 Byte (44.5[uSec] )
    4) Detect (low) intervals ~2.5[uSec]  
    5) After ~6[uSec], start transmit 1 Byte (44.5[uSec] ) (#4 and #5 repite for otherbytes)
*/

void vblankInit(){
    
    //reset all values
    signalFail();
    
    //start dac: ref voltage to comparator
    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)(((float)4096/(float)3300)*(float)300));
    
    //start comparator
    HAL_COMP_Start(&hcomp1);
    
    //start timer 3
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
    
}

//If vblank fail, reset all values
__attribute__ ((section("CCM_DATA")))
static void signalFail(){
    longLowDetectedCount=0;
    shortLowDetectedCount=0; 
}

void startVideoSystemDetection(){
    uint32_t adc_val = 0;
    uint32_t adc_val_min = 999999;
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    for(int i=0; i<299999; i++)
    {
        HAL_ADC_Start(&hadc1); 
        HAL_ADC_PollForConversion(&hadc1, 100); 
        adc_val = HAL_ADC_GetValue(&hadc1);
        if(adc_val<adc_val_min)
            adc_val_min = adc_val;
    }        
    HAL_ADC_Stop(&hadc1);
    HAL_ADC_MspDeInit(&hadc1);
    
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, adc_val_min*1.75f);
    
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
        //Video signal is disconnected, go to CLI mode
        init_cli_mode();
    }
    inTest=false;
}

static void videoSystemDetection(){
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
    if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_CC1) != RESET)
    {        /*FAILING*/
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC1);
    }
    else if(__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_CC2) != RESET)
    {   /*RAISING*/
        counter = TIM3->CCR2;
        
        #ifdef __DEBUG
            saveValueOfpulseW(counter);
        #endif /* __DEBUG */
        
        if(!inTest)
            raisingVblank();
        else
            videoSystemDetection();
        
        __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC2);
    }
    else
    {
        __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
        __HAL_TIM_SET_COUNTER(&htim3, 0);
    }
}


__attribute__ ((section("CCM_DATA")))
static void raisingVblank(){
    if(counter>TIMER_27_USEC_INTERVAL){
        longLowDetectedCount++;
        shortLowDetectedCount=0;
        if(longLowDetectedCount>LONG_DETECTIONS){
            signalFail();
        }
    }
    else if(counter > TIMER_2_5_USEC_INTERVAL && longLowDetectedCount==LONG_DETECTIONS)
    { //detect LONG_DETECTIONS long (low) pulses
        shortLowDetectedCount++;
        if(shortLowDetectedCount>SHORT_DETECTIONS && shortLowDetectedCount<MAX_SHORT_DETECTIONS){   //  here transmite bytes after short (low)pulses
            int i;
            for(i=0;i<600;i++)__nop();
            transmiteNextByte();
        }else if(shortLowDetectedCount>=MAX_SHORT_DETECTIONS){ //started line syncro impulse, reset all counters
            signalFail();
        }
    }else{
        signalFail();
    }
}









