#include "settings.h"

volatile settingT setting;


static void myFlashErase(uint32_t PageAddress){
  HAL_FLASH_Unlock(); 
  FLASH_PageErase(PageAddress);
  CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
  HAL_FLASH_Lock();
}

void FLASH_WriteSettings(void) {
    myFlashErase(MY_FLASH_PAGE_ADDR);
    HAL_FLASH_Unlock(); 
    uint32_t *source_addr = (void *)&setting;
    uint32_t *dest_addr = (uint32_t *) MY_FLASH_PAGE_ADDR;
    for (uint16_t i=0; i<SETTINGS_WORDS; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)dest_addr, *source_addr);
        source_addr++;
        dest_addr++;
    }
    HAL_FLASH_Lock();
}


void FLASH_ReadSettings(settingT *sett) {
    //Read settings
    uint32_t *source_addr = (uint32_t *)MY_FLASH_PAGE_ADDR;
    uint32_t *dest_addr = (void *)sett;
    for (uint16_t i=0; i<SETTINGS_WORDS; i++) {
        *dest_addr = *(__IO uint32_t*)source_addr;
        source_addr++;
        dest_addr++;
    }
}

void flash_init(){
    memset((void*)&setting, 0, sizeof(settingT));
    FLASH_ReadSettings((void*)&setting);
    if(setting.uart_speed<9600 || setting.uart_speed>1000000)
    {
        setting.uart_speed = 115200;
        setting.telemetry_type = MavLink_Telemetry;
        
        setting.mav_msges.sys_status = false;
        setting.mav_msges.scaled_imu = false;
        setting.mav_msges.attitude = false;
        setting.mav_msges.global_position_int = false;
        setting.mav_msges.vfr_hud = false;
        setting.mav_msges.altitude = false;
        setting.mav_msges.high_latency2 = true;
        
        setting.crsf_frames.vario = true;
        setting.crsf_frames.baro_alt = false;
        setting.crsf_frames.gps = true;
        setting.crsf_frames.link = false;
        setting.crsf_frames.battery = true;
        setting.crsf_frames.attitude = true;
        setting.crsf_frames.flight_mode = false;
        
        FLASH_WriteSettings();
    }
}
    
    
    
    
    
