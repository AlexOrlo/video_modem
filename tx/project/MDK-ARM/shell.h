#ifndef __SHELL_H
#define __SHELL_H

#include "main.h"

//comands definition
#define SET_PROTOCOL                    "set_protocol"
#define MAVLINK_MSG_ON                  "mav_msg_on"
#define MAVLINK_MSG_OFF                 "mav_msg_off"
#define CRSF_MSG_ON                     "crsf_msg_on"
#define CRSF_MSG_OFF                    "crsf_msg_off"
#define SET_UART_SPEED                  "set_uart_speed"
#define GET_STATUS                      "get_status"
#define HELP                            "help"


//arguments definition
#define ON "on"
#define OFF "off"
#define MAVLINK "mavlink"
#define CRSF "crsf"
#define RAW "raw"

//arguments for mav_on/off
/*
    SYS_STATUS(1)                43
    SCALED_IMU (26)              24
    ATTITUDE (30)                28
    GLOBAL_POSITION_INT (33)     28
    VFR_HUD (74)                 20 
    ALTITUDE (141)               32 bytes
    HIGH_LATENCY2 (235)          42 bytes
*/

#define     SYS_STATUS              "sys_status"
#define     SCALED_IMU              "scaled_imu"
#define     ATTITUDE                "attitude"
#define     GLOBAL_POSITION_INT     "global_position_int"
#define     VFR_HUD                 "vfr_hud"
#define     ALTITUDE                "altitude"
#define     HIGH_LATENCY2           "high_latency2"




/*  arguments for crsf_on/off

    CRSF_FRAMETYPE_VARIO        6
    CRSF_FRAMETYPE_BARO_ALT     6
    CRSF_FRAMETYPE_GPS          19 bytes
    CRSF_FRAMETYPE_LINK         14
    CRSF_FRAMETYPE_BATTERY      11
    CRSF_FRAMETYPE_ATTITUDE     10
    CRSF_FRAMETYPE_FLIGHT_MODE  <=20

*/

#define    CRSF_VARIO               "vario"
#define    CRSF_BARO_ALT            "baro_alt"
#define    CRSF_GPS                 "gps"
#define    CRSF_LINK                "link"
#define    CRSF_BATTERY             "battery"
#define    CRSF_ATTITUDE            "attitude"
#define    CRSF_FLIGHT_MODE         "flight_mode"


void shell_init(UART_HandleTypeDef *huart);
void shell_encode(char cIn);



#endif /* __SHELL_H */