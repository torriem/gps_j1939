/*
    This is a fork of my gps_j1939 project, but without any CAN
    support for ESP32. It's purpose is to take in GPS data from
    NMEA (single GPS with IMU or dual GPS), correct it for roll,
    and output new GGA and VTG NMEA messages over a serial port.
    
    This could be fed into AgOpenGPS directly (without need for
    IMU support in AOG), or into an external device through an
    RS232 transceiver.

    /Copyright 2018-2022 Michael Torrie
    torriem@gmail.com
 */

#include <math.h>
#include "globals.h"
#include "circle_generator.h"
#include "static_position.h"
#include "px1172rh.h"
#include "nmeaimu.h"
#include "shared_nmea_buffer.h"
#include "nmea_checksum.h"
#include <BluetoothSerial.h> //ESP32 only

uint8_t serial_buffer[1024]; //overkill hopefully

#define RADIANS(deg) deg * M_PI / 180.0
#define DEGREES(rad) rad * 180.0 / M_PI
#define INCHES 0.0254
#define FEET 0.3048

//types of GPS
#define GPS_PX1172RH 1 //dual gps from px1172rh
//#define GPS_DUAL_F9P //FUTURE: dual F9P support
#define GPS_NMEA_BNO 3  //single GPS GGA+VTG and the BNO08x

//types of virtual position generators
#define VIRTUAL_NONE 0 //no virtual position generation
#define VIRTUAL_CIRCLE 10 //simulate moving in a perfect circle
#define VIRTUAL_STATIC 11 //arbitrary static position.

#define GPS_TIMEOUT 400 //after 200ms of no GPS position, show "No GPS" on monitor

uint8_t gps_source = GPS_NMEA_BNO;
uint8_t virtual_source = VIRTUAL_NONE;

//int8_t monitor_can = -1;
int8_t monitor_can = 0;

//external GPS source variables
double autosteer_orig_lat=0;
double autosteer_orig_lon=0;
double autosteer_orig_altitude = 0;

int autosteer_source=0; //0 = inadequate, 1=WAAS, 2 = SF1 or higher, 3 = External
long autosteer_lastext=0;

unsigned long last_61184 = millis();

long can_messages_received = 0; // a sort of heartbeat on the CAN bus.
uint8_t spinner_state = 0;

BluetoothSerial SerialBT;
HardwareSerial SerialIMU(1);
HardwareSerial SerialGPS(2);


void send_serial_data(char *buffer, int buffer_len) {
	/* send generated NMEA messages to RS232 */
	//TODO
	//rs232.write(buffer,buffer_len);
	Serial.write((uint8_t *)buffer,buffer_len);

	// and send to bluetooth as well
	SerialBT.write((uint8_t *)buffer, buffer_len);
}

void send_config(void) {
	/* send a proprietary NMEA string to relavant
	   serial ports that contains the current
	   configuration

	   antenna height
	   antenna lateral offset (positive is to the right)
	   antenna forward of axle

	   current roll
	   virtual_source
	   gps_source
	   roll offset
	   imu heading offset, if known
	   imu lookback in ms
	  */

	char checksum[6];

	snprintf(nmea_buffer, NMEA_BUFFER_SIZE,
	         "$PTGPS,%.2f,%.2f,%.2f,%.2f,%d,%d,%.2f,%s,%s,%.2f,%d",
		 antenna_height,
		 antenna_right,
		 antenna_forward,
		 gps_roll,
		 virtual_source,
		 gps_source,
		 imu_roll_offset,
		 (imu_use_pitch ? "P" : "R"),
		 (imu_reverse ? "R" : "F"),
		 (imu_heading_offset_set ? imu_heading_offset : 400),
		 get_imu_lookback());

	compute_nmea_checksum(nmea_buffer, checksum);
	strncat(nmea_buffer,checksum, NMEA_BUFFER_SIZE);
	strncat(nmea_buffer,"\r\n", NMEA_BUFFER_SIZE);

	Serial.write((uint8_t *)nmea_buffer,strnlen(nmea_buffer, NMEA_BUFFER_SIZE));
	SerialBT.write((uint8_t *)nmea_buffer,strnlen(nmea_buffer, NMEA_BUFFER_SIZE));
}

void setup()
{
	Serial.begin(115200);
	SerialGPS.begin(460800,SERIAL_8N1,25,14);
	SerialIMU.begin(115200,SERIAL_8N1,27,16);
	SerialBT.begin("rovertest");

	delay(2000);
	Serial.println("gps_j1939 on ESP32");
	autosteer_source = 0;
	gps_latitude = 0;
	gps_longitude = 0;

	switch(gps_source) {
	case GPS_PX1172RH:
		setup_psti(NULL);
		break;
	case GPS_NMEA_BNO:
		//TODO: IMU serial port
		//setup_nmea_parser(send_gps_messages, (Stream *) NULL);
		setup_nmea_parser(NULL, &SerialIMU, send_serial_data);
		break;
	}
}

void loop()
{
	char c;
	
	unsigned long last_time = millis();
	unsigned long t;

	// virtual position generators
	CircleGenerator circlegen;
        StaticPosition staticpos;

	//example of setup
	//circlegen.set_circle(latitude, longitude, diameter, altitude);
        //circlegen.start(millis(), 120, 22); //22 kph for 120 seconds
	//virtual_source = VIRTUAL_CIRCLE;

	//staticpos.set_position(latitude, longitude, altitude, heading);
        //staticpos.start(millis(), 120); //static position for 2 minutes
	//virtual_source = VIRTUAL_STATIC;

	while(1) {
		t = millis();

		if (t - last_time > 1000) {
			//once a second.
			send_config();
			last_time = t;
		}

		switch(virtual_source) {
		case VIRTUAL_CIRCLE:
			if (circlegen.circle_position(t, 200)) {
				autosteer_source = 3;
				//send_gps_messages();
				autosteer_lastext = t;
			}
			if (circlegen.finished(t))
				virtual_source = VIRTUAL_NONE;
			break;
		case VIRTUAL_STATIC:
			if (staticpos.static_position(t, 200)) {
				autosteer_source = 3;
				//send_gps_messages();
				autosteer_lastext = t;
			}
			if (staticpos.finished(t))
				virtual_source = VIRTUAL_NONE;
			break;
		}

		//process bluetooth ntrip
		while (SerialBT.available())
		{
			SerialGPS.write(SerialBT.read());
		}

		//process IMU data if equipped and configured
		read_imu();

		//now process serial bytes that have accumulated
		while(SerialGPS.available()) {
			c = SerialGPS.read();

			if (virtual_source) {
				/* ignore real GPS while virtual positions are
				   being used. */
				continue;
			}

			switch(gps_source) {
			case GPS_PX1172RH:
				psti_process(c);
				break;
			case GPS_NMEA_BNO:
				nmea_process(c);
				break;
			}
		}
	}
}
