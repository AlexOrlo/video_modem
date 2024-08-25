#include "main.h"
#include "settings.h" 
#include "mavlink.h"

void addBytesToQueue(uint8_t* c, int len);

extern volatile settingT setting;

volatile mavlink_message_t r_message;
volatile mavlink_status_t r_mavlink_status;
volatile uint8_t buf[256] = {0};


__attribute__ ((section("CCM_DATA")))
void parsMavlink(char c){
    if(mavlink_parse_char(0, c, (void*)&r_message, (void*)&r_mavlink_status)){
        switch(r_message.msgid){
            
                case MAVLINK_MSG_ID_SYS_STATUS: 
                {
                    if(setting.mav_msges.sys_status)
                    {
                        int len = mavlink_msg_to_send_buffer((void*)buf, (void*)&r_message); 
                        addBytesToQueue((void*)buf, len);
                    }
                }
                break;
                
                case MAVLINK_MSG_ID_SCALED_IMU: 
                {
                    if(setting.mav_msges.scaled_imu)
                    {
                        int len = mavlink_msg_to_send_buffer((void*)buf, (void*)&r_message); 
                        addBytesToQueue((void*)buf, len);
                    }
                }
                break;
                
                case MAVLINK_MSG_ID_ATTITUDE: 
                {
                    if(setting.mav_msges.attitude)
                    {
                        int len = mavlink_msg_to_send_buffer((void*)buf, (void*)&r_message); 
                        addBytesToQueue((void*)buf, len);
                    }
                }
                break;
            
                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 
                {
                    if(setting.mav_msges.global_position_int)
                    {
                        int len = mavlink_msg_to_send_buffer((void*)buf, (void*)&r_message); 
                        addBytesToQueue((void*)buf, len);
                    }
                }
                break;
                
                case MAVLINK_MSG_ID_VFR_HUD: 
                {
                    if(setting.mav_msges.vfr_hud)
                    {
                        int len = mavlink_msg_to_send_buffer((void*)buf, (void*)&r_message); 
                        addBytesToQueue((void*)buf, len);
                    }
                }
                break;
                
                case MAVLINK_MSG_ID_ALTITUDE: 
                {
                    if(setting.mav_msges.altitude)
                    {
                        int len = mavlink_msg_to_send_buffer((void*)buf, (void*)&r_message); 
                        addBytesToQueue((void*)buf, len);
                    }
                }
                break;
                
                case MAVLINK_MSG_ID_HIGH_LATENCY2: 
                {
                    if(setting.mav_msges.high_latency2)
                    {
                        int len = mavlink_msg_to_send_buffer((void*)buf, (void*)&r_message); 
                        addBytesToQueue((void*)buf, len);
                    }
                }
                break;
                
                
                default:
                   break;
                   
        }
    }
}