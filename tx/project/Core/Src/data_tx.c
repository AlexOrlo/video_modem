
#include "main.h"
#include <string.h>

#define CIRC_BUFFER_SIZE 8192

static int getSize(void);
void transmiteNextByte(void);

extern UART_HandleTypeDef huart3;

volatile int pointerAdd = 0, pointerTransmite=0;
uint8_t circBuffer[CIRC_BUFFER_SIZE] = {0};



//comand to send next byte from Queue
__attribute__ ((section("CCM_DATA")))
void transmiteNextByte(){
    if(pointerAdd != pointerTransmite){ //Check if we have some bytes to transmite
        
        huart3.Instance->TDR = (uint8_t)(circBuffer[pointerTransmite] & 0xFFU);
        
        pointerTransmite++;
        pointerTransmite &= (CIRC_BUFFER_SIZE-1);
    }
}





__attribute__ ((section("CCM_DATA")))
void addBytesToQueue(uint8_t* c, int len){
    
    if(CIRC_BUFFER_SIZE-getSize()<len)
    {
        //overflow
        //Core power cycle
        __set_FAULTMASK (1);
        NVIC_SystemReset();
    }
    
    if((CIRC_BUFFER_SIZE-pointerAdd)>len){
        memcpy(circBuffer+pointerAdd, c, len);
        pointerAdd += len;
    }else{
        memcpy(circBuffer+pointerAdd, c, (CIRC_BUFFER_SIZE-pointerAdd));
        memcpy(circBuffer, c+(CIRC_BUFFER_SIZE-pointerAdd), len-(CIRC_BUFFER_SIZE-pointerAdd));
        pointerAdd=len-(CIRC_BUFFER_SIZE-pointerAdd);
    }
}




__attribute__ ((section("CCM_DATA")))
static int getSize(){
    if(pointerAdd>=pointerTransmite)
        return (pointerAdd-pointerTransmite);
    else 
        return (CIRC_BUFFER_SIZE-pointerTransmite+pointerAdd);
}

















