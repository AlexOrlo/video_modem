

#ifndef LIB_CROSSFIRE_H
#define LIB_CROSSFIRE_H

#include "main.h"

#define CRSF_BAUDRATE           115200

enum {
	CRSF_SYNC_BYTE = 0xC8,
	CRSF_FRAME_SIZE_MAX = 64, // 62 bytes frame plus 2 bytes frame header(<length><type>)
	CRSF_PAYLOAD_SIZE_MAX = CRSF_FRAME_SIZE_MAX - 6,
	CRSF_FRAME_TX_MSP_FRAME_SIZE = 58,
	CRSF_FRAME_RX_MSP_FRAME_SIZE = 8,
	CRSF_FRAME_ORIGIN_DEST_SIZE = 2,
};

typedef enum {
	CRSF_ADDRESS_BROADCAST = 0x00,
	CRSF_ADDRESS_USB = 0x10,
	CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,
	CRSF_ADDRESS_RESERVED1 = 0x8A,
	CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,
	CRSF_ADDRESS_GPS = 0xC2,
	CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
	CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
	CRSF_ADDRESS_RESERVED2 = 0xCA,
	CRSF_ADDRESS_RACE_TAG = 0xCC,
	CRSF_ADDRESS_RADIO_RADIO = 0xEA,
	CRSF_ADDRESS_CRSF_MODULE = 0xEE,
	CRSF_ADDRESS_CRSF_RECEIVER = 0xEC
} crossfire_address_t;

typedef enum {
	CRSF_FRAMETYPE_GPS = 0x02,
    CRSF_FRAMETYPE_BARO_ALT = 0x09,
	CRSF_FRAMETYPE_VARIO = 0x07,
	CRSF_FRAMETYPE_BATTERY = 0x08,
	CRSF_FRAMETYPE_LINK = 0x14,
	CRSF_FRAMETYPE_RC_CHANNELS = 0x16,
	CRSF_FRAMETYPE_ATTITUDE = 0x1E,
	CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,

	// Extended Header Frames, range: 0x28 to 0x96
	CRSF_FRAMETYPE_DEVICE_PING = 0x28,
	CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
	CRSF_FRAMETYPE_SETTINGS_REQUEST = 0x2A,
	CRSF_FRAMETYPE_SETTINGS_RESPONSE = 0x2B,
	CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
	CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,
	CRSF_FRAMETYPE_COMMAND = 0x32,
    CRSF_FRAMETYPE_RADIO_ID = 0x3A,

	// MSP commands
	CRSF_FRAMETYPE_MSP_REQ = 0x7A,   // response request using msp sequence as command
	CRSF_FRAMETYPE_MSP_RESP = 0x7B,  // reply with 58 byte chunked binary
	CRSF_FRAMETYPE_MSP_WRITE = 0x7C,  // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
	CRSF_FRAMETYPE_DISPLAYPORT_CMD = 0x7D, // displayport control command
} crossfire_frame_type_t;


void crsf_telemetry_push_byte(uint8_t data);

#endif //LIB_CROSSFIRE_H
