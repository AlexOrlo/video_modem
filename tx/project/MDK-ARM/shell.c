
#include "main.h"
#include <string.h>
#include <stdio.h>
#include "shell.h" 
#include "settings.h"  

UART_HandleTypeDef *shell_huart;

volatile char shell_command_string[15] = {0};
volatile char shell_number_argument_string[15] = {0};
volatile char shell_text_argument_string[15] = {0};

extern volatile settingT setting;

static void shell_take_action();


static void sendStringToCom(char s[]){
    HAL_UART_Transmit(shell_huart, (unsigned char*)s, strlen(s), 1000);  
}

static void sendWStringToCom(char s[], char s2[]){
    char com[50] = {0};
	sprintf((void *)com, "%s %s\r", s, s2);
    HAL_UART_Transmit(shell_huart, (unsigned char*)com, strlen(com), 1000);  
}

static void sendIntToCom (char* command, int data){
    char com[50] = {0};
	sprintf((void *)com, "%s %i\r", command, data);
    HAL_UART_Transmit(shell_huart, (unsigned char*)com, strlen(com), 1000);  
}

static float string_to_float() {
  int  rl = 0;
  float rr = 0.0;
  float rb = 0.1;
  bool dec = false;
  int i = 0;

    
  if ((shell_number_argument_string[i] == '-') || (shell_number_argument_string[i] == '+')) { i++; }
  while (shell_number_argument_string[i] != 0) {
    if (shell_number_argument_string[i] == '.') {
      dec = true;
    }
    else{
      if (!dec) {
        rl = (10 * rl) + (shell_number_argument_string[i] - 48);
      }
      else {
        rr += rb * (float)(shell_number_argument_string[i] - 48);
        rb /= 10.0;
      }
    }
    i++;
		if ( i > 16 ) return 0;
  }
  rr += (float)rl;
  if (shell_number_argument_string[0] == '-') {
    rr = 0.0 - rr;
  }
  return rr;
}


void shell_init(UART_HandleTypeDef *huart){
    shell_huart = huart;
}

void shell_encode(char cIn){
    
    static int staticSpasersCounter=0;
    static int commandPtr=0, numberPtr, textPtr=0;
    
    if (cIn == ' '){  //start parsing number
       staticSpasersCounter++;
       return;
    }
    
    if(staticSpasersCounter){
       if(cIn > 0x2D && cIn < 0x3A){
           shell_number_argument_string[numberPtr] = cIn;
           numberPtr++;
           return;
       }else if(cIn != '\r'){
           shell_text_argument_string[textPtr] = cIn;
           textPtr++;
           return;
       }
       
    }else{
        if(cIn != '\r'){
           shell_command_string[commandPtr] = cIn;
           commandPtr++;
           return;
        }
    }
    
    if(cIn == '\r'){
        commandPtr = 0;
        textPtr = 0;
        numberPtr = 0;
        staticSpasersCounter = 0;
        shell_take_action();
    }
}

static void shell_take_action(){
  if(strncmp(SET_PROTOCOL, (void *)shell_command_string, 6) == 0){
      if(strncmp(MAVLINK, (void *)shell_text_argument_string, 6) == 0){
          setting.telemetry_type = MavLink_Telemetry;
          FLASH_WriteSettings();
          sendWStringToCom("Protocol choised ", MAVLINK);
      }else if(strncmp(CRSF, (void *)shell_text_argument_string, 4) == 0){
          setting.telemetry_type = Crossfier_Telemetry;
          FLASH_WriteSettings();
          sendWStringToCom("Protocol choised ", CRSF);
      }else if(strncmp(RAW, (void *)shell_text_argument_string, 3) == 0){
          setting.telemetry_type = Raw;
          FLASH_WriteSettings();
          sendWStringToCom("Protocol choised ", RAW);
      }
  }
  
  else if(strncmp(MAVLINK_MSG_ON, (void *)shell_command_string, 10) == 0){  
      if(strncmp(SYS_STATUS, (void *)shell_text_argument_string, 6) == 0){
          setting.mav_msges.sys_status=true;
          FLASH_WriteSettings();
          sendStringToCom("ok \r");
      }else if(strncmp(SCALED_IMU, (void *)shell_text_argument_string, 6) == 0){
          setting.mav_msges.scaled_imu=true;
          FLASH_WriteSettings();
          sendStringToCom("ok \r");
      }else if(strncmp(ATTITUDE, (void *)shell_text_argument_string, 6) == 0){
          setting.mav_msges.attitude=true;
          FLASH_WriteSettings();
          sendStringToCom("ok \r");
      }else if(strncmp(GLOBAL_POSITION_INT, (void *)shell_text_argument_string, 6) == 0){
          setting.mav_msges.global_position_int=true;
          FLASH_WriteSettings();
          sendStringToCom("ok \r");
      }else if(strncmp(VFR_HUD, (void *)shell_text_argument_string, 6) == 0){
          setting.mav_msges.vfr_hud=true;
          FLASH_WriteSettings();
          sendStringToCom("ok \r");
      }else if(strncmp(ALTITUDE, (void *)shell_text_argument_string, 6) == 0){
          setting.mav_msges.altitude=true;
          FLASH_WriteSettings();
          sendStringToCom("ok \r");
      }else if(strncmp(HIGH_LATENCY2, (void *)shell_text_argument_string, 6) == 0){
          setting.mav_msges.high_latency2=true;
          FLASH_WriteSettings();
          sendStringToCom("ok \r");
      }
  }
  
  
  else if(strncmp(MAVLINK_MSG_OFF, (void *)shell_command_string, 10) == 0){  
      if(strncmp(SYS_STATUS, (void *)shell_text_argument_string, 6) == 0){
          setting.mav_msges.sys_status=false;
          FLASH_WriteSettings();
          sendStringToCom("ok \r");
      }else if(strncmp(SCALED_IMU, (void *)shell_text_argument_string, 6) == 0){
          setting.mav_msges.scaled_imu=false;
          FLASH_WriteSettings();
          sendStringToCom("ok \r");
      }else if(strncmp(ATTITUDE, (void *)shell_text_argument_string, 6) == 0){
          setting.mav_msges.attitude=false;
          FLASH_WriteSettings();
          sendStringToCom("ok \r");
      }else if(strncmp(GLOBAL_POSITION_INT, (void *)shell_text_argument_string, 6) == 0){
          setting.mav_msges.global_position_int=false;
          FLASH_WriteSettings();
          sendStringToCom("ok \r");
      }else if(strncmp(VFR_HUD, (void *)shell_text_argument_string, 6) == 0){
          setting.mav_msges.vfr_hud=false;
          FLASH_WriteSettings();
          sendStringToCom("ok \r");
      }else if(strncmp(ALTITUDE, (void *)shell_text_argument_string, 6) == 0){
          setting.mav_msges.altitude=false;
          FLASH_WriteSettings();
          sendStringToCom("ok \r");
      }else if(strncmp(HIGH_LATENCY2, (void *)shell_text_argument_string, 6) == 0){
          setting.mav_msges.high_latency2=false;
          FLASH_WriteSettings();
          sendStringToCom("ok \r");
      }
  }
  
  
  
  
  
  else if(strncmp(CRSF_MSG_ON, (void *)shell_command_string, 11) == 0){  
      if(strncmp(CRSF_VARIO, (void *)shell_text_argument_string, 3) == 0){
          setting.crsf_frames.vario=true;
          FLASH_WriteSettings();
          sendStringToCom("ok \r");
      }else if(strncmp(CRSF_BARO_ALT, (void *)shell_text_argument_string, 3) == 0){
          setting.crsf_frames.baro_alt=true;
          FLASH_WriteSettings();
          sendStringToCom("ok \r");
      }else if(strncmp(CRSF_GPS, (void *)shell_text_argument_string, 3) == 0){
          setting.crsf_frames.gps=true;
          FLASH_WriteSettings();
          sendStringToCom("ok \r");
      }else if(strncmp(CRSF_LINK, (void *)shell_text_argument_string, 3) == 0){
          setting.crsf_frames.link=true;
          FLASH_WriteSettings();
          sendStringToCom("ok \r");
      }else if(strncmp(CRSF_BATTERY, (void *)shell_text_argument_string, 3) == 0){
          setting.crsf_frames.battery=true;
          FLASH_WriteSettings();
          sendStringToCom("ok \r");
      }else if(strncmp(CRSF_ATTITUDE, (void *)shell_text_argument_string, 3) == 0){
          setting.crsf_frames.attitude=true;
          FLASH_WriteSettings();
          sendStringToCom("ok \r");
      }else if(strncmp(CRSF_FLIGHT_MODE, (void *)shell_text_argument_string, 3) == 0){
          setting.crsf_frames.flight_mode=true;
          FLASH_WriteSettings();
          sendStringToCom("ok \r");
      }
  }
  
  
  else if(strncmp(CRSF_MSG_OFF, (void *)shell_command_string, 11) == 0){  
      if(strncmp(CRSF_VARIO, (void *)shell_text_argument_string, 3) == 0){
          setting.crsf_frames.vario=false;
          FLASH_WriteSettings();
          sendStringToCom("ok \r");
      }else if(strncmp(CRSF_BARO_ALT, (void *)shell_text_argument_string, 3) == 0){
          setting.crsf_frames.baro_alt=false;
          FLASH_WriteSettings();
          sendStringToCom("ok \r");
      }else if(strncmp(CRSF_GPS, (void *)shell_text_argument_string, 3) == 0){
          setting.crsf_frames.gps=false;
          FLASH_WriteSettings();
          sendStringToCom("ok \r");
      }else if(strncmp(CRSF_LINK, (void *)shell_text_argument_string, 3) == 0){
          setting.crsf_frames.link=false;
          FLASH_WriteSettings();
          sendStringToCom("ok \r");
      }else if(strncmp(CRSF_BATTERY, (void *)shell_text_argument_string, 3) == 0){
          setting.crsf_frames.battery=false;
          FLASH_WriteSettings();
          sendStringToCom("ok \r");
      }else if(strncmp(CRSF_ATTITUDE, (void *)shell_text_argument_string, 3) == 0){
          setting.crsf_frames.attitude=false;
          FLASH_WriteSettings();
          sendStringToCom("ok \r");
      }else if(strncmp(CRSF_FLIGHT_MODE, (void *)shell_text_argument_string, 3) == 0){
          setting.crsf_frames.flight_mode=false;
          FLASH_WriteSettings();
          sendStringToCom("ok \r");
      }
  }
  
  else if(strncmp(SET_UART_SPEED, (void *)shell_command_string, 6) == 0){  
      setting.uart_speed = string_to_float();
      FLASH_WriteSettings();
      sendStringToCom("Uart speed changed \r");
  }
  
  
  else if(strncmp(GET_STATUS, (void *)shell_command_string, 10) == 0){  
      sendIntToCom ("uart speed is", setting.uart_speed);
      if(setting.telemetry_type == MavLink_Telemetry)
      {
          sendWStringToCom("Protocol ", MAVLINK);
          sendIntToCom (SYS_STATUS, setting.mav_msges.sys_status);
          sendIntToCom (SCALED_IMU, setting.mav_msges.scaled_imu);
          sendIntToCom (ATTITUDE, setting.mav_msges.attitude);
          sendIntToCom (GLOBAL_POSITION_INT, setting.mav_msges.global_position_int);
          sendIntToCom (VFR_HUD, setting.mav_msges.vfr_hud);
          sendIntToCom (ALTITUDE, setting.mav_msges.altitude);
          sendIntToCom (HIGH_LATENCY2, setting.mav_msges.high_latency2);
      }
      else if(setting.telemetry_type == Crossfier_Telemetry)
      {
          sendWStringToCom("Protocol ", CRSF);
          sendIntToCom (CRSF_VARIO, setting.crsf_frames.vario);
          sendIntToCom (CRSF_BARO_ALT, setting.crsf_frames.baro_alt);
          sendIntToCom (CRSF_GPS, setting.crsf_frames.gps);
          sendIntToCom (CRSF_LINK, setting.crsf_frames.link);
          sendIntToCom (CRSF_BATTERY, setting.crsf_frames.battery);
          sendIntToCom (CRSF_ATTITUDE, setting.crsf_frames.attitude);
          sendIntToCom (CRSF_FLIGHT_MODE, setting.crsf_frames.flight_mode);
      }    
      else  sendWStringToCom("Protocol ", RAW);
  }
  
  
  else if(strncmp(HELP, (void *)shell_command_string, 4) == 0){  
      sendStringToCom("set_protocol argument \r");
      sendStringToCom("argument avalible: mavlink, crsf, raw\r");
      sendStringToCom("\r");
      sendStringToCom("mav_msg_on /mav_msg_off argument \r");
      sendStringToCom("argument avalible: sys_status, scaled_imu, attitude, global_position_int, vfr_hud, altitude, high_latency2\r");
      sendStringToCom("\r");
      sendStringToCom("crsf_msg_on /crsf_msg_off argument \r");
      sendStringToCom("argument avalible: vario, baro_alt, gps, link, battery, attitude, flight_mode\r");
      sendStringToCom("\r");
      sendStringToCom("set_uart_speed argument      telemetry uart speed\r");
  }
  
  
  memset((void *)shell_command_string, 0, 15);
  memset((void *)shell_number_argument_string, 0, 15);
  memset((void *)shell_text_argument_string, 0, 15);
  
}

