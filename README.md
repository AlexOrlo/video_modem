Telemetry Transmission via Analog Video

This is a small project that allows for receiving digital data (telemetry) through analog video. 
Although most control systems today have a feedback channel, it is not always possible to use it. 
For instance, it might be prohibited in competitions or if the power and range are limited.

Personally, I use it as a receiving modem on my ground-based rotating antenna stations with analog video.

The device allows for data transmission at speeds of up to 800 bytes per second in PAL system and up to 720 bytes per second in NTSC system. 
An 8KB buffer helps handle short-term bursts in speed.

To optimize the use of the narrow channel, message type filtering for MAVLink and Crossfire protocols is supported. 
For example, I only use the MAVLink HIGH_LATENCY2 message, which contains all the necessary information and occupies 42 bytes. 
This allows setting the maximum update frequency of such a message to 800/42 = 19 times per second.

MAVLink Message List:

    SYS_STATUS 		43 bytes
    SCALED_IMU  	24 bytes
    ATTITUDE 		28 bytes
    GLOBAL_POSITION_INT 28 bytes
    VFR_HUD 		20 bytes
    ALTITUDE  		32 bytes
    HIGH_LATENCY2  	42 bytes

Crossfire Message List:

    CRSF_FRAMETYPE_VARIO 	6 bytes
    CRSF_FRAMETYPE_BARO_ALT 	6 bytes
    CRSF_FRAMETYPE_GPS 		19 bytes
    CRSF_FRAMETYPE_LINK 	14 bytes
    CRSF_FRAMETYPE_BATTERY 	11 bytes
    CRSF_FRAMETYPE_ATTITUDE 	10 bytes
    CRSF_FRAMETYPE_FLIGHT_MODE 	<=20 bytes

It is also possible to transmit any raw data without any filtering.
For configuring the filter, selecting the protocol, and setting the UART port speed, commands are used via the serial terminal.
Both the transmitting and receiving devices are built on an STM32F303CBT6 microcontroller. 
The GitHub repository contains ready-made projects for Keil MDK as well as schematics and PCBs in DipTrace.
Ready binaries and Gerber files for the PCBs are also available.

To assemble the device, you need the following components: two STM32F303CBTx, one dual operational amplifier MAX4017ESA+, 
four MC33375ST-3.3T3 regulators, two 8MHz crystal oscillators, and a selection of SMD capacitors and resistors in 0805 package. 
The transmitting device has a form factor of 38x38 mm and a standard mounting hole distance of 30.5 mm.

If anyone is interested, let me know, and I can provide more detailed information on its configuration. 
Additionally, when using a full MAVLink setup (PX4/ArduPilot), I can add the capability to configure the message transmission frequency 
(the flight controller must support the MAV_CMD_SET_MESSAGE_INTERVAL command).

Interestingly, this device does not introduce any distortions in the video, works stably in interference conditions, 
and only fails in cases of total video degradation (when nothing is visible on the screen).
