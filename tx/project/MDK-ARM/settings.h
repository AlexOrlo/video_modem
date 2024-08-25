#ifndef SETTINGS_H
#define SETTINGS_H

#include "main.h"
#include <string.h>

#define SETTINGS_WORDS  (sizeof(settingT))/4
#define MY_FLASH_PAGE_ADDR    0x801F800

extern void FLASH_PageErase(uint32_t PageAddress);

enum Telemetry_Type {
    MavLink_Telemetry,
    Crossfier_Telemetry,
    Raw
};


typedef struct
{
    bool sys_status;
    bool scaled_imu;
    bool attitude;
    bool global_position_int;
    bool vfr_hud;
    bool altitude;
    bool high_latency2;
} mav_msg_Type;

typedef struct
{
    bool vario;
    bool baro_alt;
    bool gps;
    bool link;
    bool battery;
    bool attitude;
    bool flight_mode;
    
} crsf_frame_Type;


typedef struct
{
    uint32_t                uart_speed;
    enum Telemetry_Type     telemetry_type;
    mav_msg_Type            mav_msges;
    crsf_frame_Type         crsf_frames;
    
} settingT;



void FLASH_WriteSettings(void);
void FLASH_ReadSettings(settingT *sett);
void flash_init(void);





#endif //SETTINGS_H